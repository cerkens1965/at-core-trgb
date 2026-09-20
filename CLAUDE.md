# AT-VIEW — at-core-trgb — CLAUDE.md

## Matériel cible

| Composant | Détail |
|-----------|--------|
| Board | LilyGo T-RGB circular — **2.1" full-circle** ou **2.8"** (variantes interchangeables) |
| SoC | ESP32-S3 (WiFi 2.4GHz intégré) |
| Écran | 480×480 px circulaire, RGB panel (ST7701) |
| Touch | Capacitif intégré — auto-détecté : CST820 (2.1"FC) / GT911 (2.8") / FT3267 (2.1"HC) |
| SD | Slot natif SD_MMC (SDIO 1-bit) — EN=7, SCK=39, CMD=40, DAT=38 |
| BLE | Client — se connecte à AT-CORE (NimBLE) |

## ⚠️⚠️ HEAP Bluedroid WS-241 — ROOT CAUSE de quasi TOUS les bugs BLE écran (fix v138, 2026-07-15)

**Le WS-241 tourne en Bluedroid** (Arduino BLE lib, ≠ NimBLE), qui **EXIGE de la RAM INTERNE** (il ne peut PAS
utiliser la PSRAM). Les buffers de dessin LVGL (`ws241_esplcd.h`) étaient en **DOUBLE buffer 1/10 d'écran, en RAM
INTERNE DMA = ~108 Ko** → il ne restait presque rien pour Bluedroid → **`E BLE_INIT: Malloc failed` EN BOUCLE**.

**Symptômes que ça causait (TOUS la même racine — on a perdu une journée à traiter les symptômes)** :
- **writes écran→boîtier qui échouent** (`{"cmd":"portal"}`, setreg immat, cloud, upload) : la commande « part »
  (`[BLE] CTRL ...`) mais **pas de `[BLE] OK`**, le boîtier ne reçoit rien (« Portal requested » vert mais AP
  jamais levée ; immat inchangée). ⚠️ Ce n'est **NI** `response=false` (ignoré) **NI** `response=true` (fige) — c'est
  le **heap**.
- **crash `connectBLE` / boot-loop** à la connexion (surtout AP GDL90 up) : `FreeRTOS::Semaphore`→`xQueueGenericSend`
  = même OOM Bluedroid.
- **figes**, `Malloc failed`, écran « stuck ».
- ⚠️ **Le monitoring série USB-CDC de l'écran AGGRAVE le heap** → déclenche `Malloc failed` à lui seul → **fausse
  tous les diagnostics** (bug apparaît sous monitoring, disparaît sur powerbank).

**FIX v138 = `ws241_esplcd.h` : buffer SIMPLE 1/20 (~26 Ko)** au lieu du double 1/10 → **libère ~82 Ko de RAM
interne** pour Bluedroid. Rendu imperceptiblement + lent. ✅ Validé : `Malloc failed = 0` **même sous monitoring USB**,
bouton « Open portal » + AP OK, tout écran→boîtier réparé. **Si un bug BLE écran réapparaît → penser HEAP d'abord**
(vrai filet définitif = migrer Bluedroid→NimBLE, mais le fix buffer suffit).

### Variantes T-RGB supportées

Pinout identique entre toutes les variantes (`src/utilities.h`). `panel.begin()` sans
arguments fait l'auto-détection via le contrôleur tactile (testé 2026-05-20 : flash
2.8" → 2.1" full-circle sans changement de code, run nominal).

| Variante | Touch chip | Init panneau | Auto-détection |
|---|---|---|---|
| 2.1" half-circle | FT3267 | `st7701_2_1_inches` | ✅ |
| 2.1" full-circle | CST820 | `st7701_2_1_inches` | ✅ |
| 2.8" full-circle | GT911  | `st7701_2_8_inches` | ✅ |
| 2.1" half-circle V2 | (idem FT3267) | `st7701_2_1_inches_rev2` | ❌ requiert `panel.begin(LILYGO_T_RGB_2_1_INCHES_HALF_CIRCLE_V2)` |
| 2.1" full-circle V2 | (idem CST820) | `st7701_2_1_inches_rev2` | ❌ requiert `panel.begin(LILYGO_T_RGB_2_1_INCHES_FULL_CIRCLE_V2)` |

Si écran blanc / couleurs cassées au boot après changement matériel → c'est probablement une révision V2,
passer le type explicite dans l'appel `panel.begin()` au setup. Si rouge↔bleu inversés → ajouter
`LILYGO_T_RGB_ORDER_BGR` en second argument.

Note ergonomique : à résolution identique (480×480), le 2.1" a un DPI plus élevé. L'UI reste lisible
mais les cibles tactiles (boutons `<` / `>` 30×22 px, dots PIN Ø22) deviennent plus petites en mm.

## Fichier principal

`examples/at_core_debug/at_core_debug.ino` — firmware AT-VIEW v0.6

## Build — PlatformIO

```bash
pio run -e T-RGB -t upload
```

Le `platformio.ini` à la racine pointe sur `src_dir = examples/at_core_debug`.

### CI — GitHub Actions

`.github/workflows/pio.yml` : build env **`T-RGB`** à chaque push touchant `src/**`,
`examples/at_core_debug/**`, `platformio.ini` ou le workflow. **✅ vert** (commit `4a8b708`).
Plateforme pinnée `espressif32@6.9.0` (reproductibilité CI/local). Pas de `secrets.h`
requis côté AT-VIEW. Couvre aussi `arduino_ci.yml`.

## Variante T4-S3 AMOLED (2026-06-04)

Second hardware supporté : **LilyGo T4-S3 AMOLED 2.41"** (600×450 paysage, RM690B0
QSPI, touch CST226SE, PMU SY6970, SD en SPI). Même firmware, env PlatformIO dédié :

```bash
pio run -e T4-S3            # build (flag -DBOARD_T4S3 active les shims)
```

- **Lib** : LilyGo-AMOLED-Series (git) — son `LV_Helper`/`lv_conf` (`LV_COLOR_16_SWAP=1`),
  PAS le `src/` T-RGB local (clash). `boards/T-Display-AMOLED.json` copié du repo LilyGo.
- **Shims** (`#ifdef BOARD_T4S3` dans le .ino) : `LilyGo_Class amoled` + `#define panel`,
  `SD_MMC`→`SD`, `panelBright()` (0-16 → 0-255), `beginAMOLED_241()`, `montserrat_10`→12.
- **UI dédiée** : page radar **plein écran** (radar Ø440 à droite, annotations en colonne
  gauche agrandies, arc CO bas-gauche, cluster scale/GS/zoom bas-gauche, clavier
  Maintenance 480×240). Tout passe par macros de variante (`RAD_*`, `PILL_*`, `HDG_*`,
  `ZOOM_SZ`, `CHIP_*`, `RAD_FONT`, `CO_*`) — autres pages : canvas 480×480 centré
  (`UI_OX=60, UI_OY=-15`), scroll/scrollbar écran désactivés (sinon scrollbar fantôme).
- **Assets** : logos OK via variantes `#if LV_COLOR_16_SWAP` (png2lvgl_logos.py) ; icônes
  radar = blanc+alpha recolorées → swap-invariantes, rien à faire.
- **⚠ Flash** : l'upload pio échoue (USB-CDC instable) → mode download manuel (BOOT
  maintenu + replug) puis esptool **no-stub** :
  `esptool.py --chip esp32s3 --port /dev/cu.usbmodem* --baud 115200 --no-stub --before no_reset --after hard_reset write_flash 0x10000 /tmp/pio_build_atview/T4-S3/firmware.bin`
  Après flash : **RST physique** (le reset RTS laisse la carte en bootloader). Vérifier la
  MAC avant d'écrire si plusieurs cartes branchées : T4 = `64:e8:33:7a:80:68`,
  AT-CORE 7600 S3 = `20:6e:f1:ce:27:6c` (⚠ sonder un port fige la carte → RST).
- Page #01 affiche ATV (version locale) + ATC (version AT-CORE live BLE).

## Variante Waveshare AMOLED 2.16" (2026-06-22)

Troisième hardware supporté : **Waveshare ESP32-S3-Touch-AMOLED-2.16** — AMOLED
**carré 480×480** (≠ T-RGB circulaire, ≠ T4 paysage). Même firmware visé, env
PlatformIO dédié :

```bash
pio run -e WS-216 -t upload        # build (flag -DBOARD_WS216)
```

| Composant | Détail |
|-----------|--------|
| SoC | ESP32-S3R8 — 8 MB PSRAM **OPI** / 16 MB flash (`board_build.arduino.memory_type = qio_opi`) |
| Écran | 480×480 AMOLED **carré**, driver **CO5300** (QSPI) |
| Touch | **CST9220** (famille CST92xx, I²C @0x5A) |
| PMIC | AXP2101 (I²C @0x34) — rail écran **ON au POR** (pas requis pour bring-up) |
| Autres | RTC PCF85063, IMU QMI8658, codec ES8311 + ES7210 (audio, non utilisé), SD **SPI** |

- **⚠ Pile spécifique WS-216 = ESP32 Arduino core 3.x (pioarduino) + GFX 1.6.4.** Contrairement
  au T-RGB/T4 (core 2.x `espressif32@6.9.0` + GFX 1.5.0), l'env WS-216 utilise
  `platform = https://github.com/pioarduino/platform-espressif32/.../53.03.13` + `GFX @ 1.6.4`.
  **Obligatoire pour le CO5300** : en GFX 1.5.0 (driver CO5300 basé `Arduino_TFT`) le **noir
  sortait verdâtre** ; la 1.6.4 (driver réécrit `Arduino_OLED`) rend un noir profond. 1.6.x
  exige core 3.x (`esp32-hal-periman.h`). Test de référence isolé : `examples/ws216_blacktest`.
  Le `.ino` reste cross-core (helper `bleStr()` pour l'API BLE String↔std::string selon
  `ESP_ARDUINO_VERSION_MAJOR`).
- **Pile d'affichage** : `Arduino_GFX` (`Arduino_CO5300`) — **PAS** le `src/` T-RGB
  (ST7701 RGB parallèle) ni la lib LilyGo-AMOLED (RM690B0). Touch via **SensorLib 0.4.1** umbrella `TouchDrvCSTXXX`
  (`#include <TouchDrvCSTXXX.hpp>` — le header racine forwarde `src/touch/`, sinon
  `TouchDrvCST92xx.h` est introuvable car les sous-dossiers ne sont pas sur l'include path).
- **Pinout** (source = **BSP ESP-IDF officiel**, fait autorité) : QSPI CS=12, SCK=38,
  D0-3=4/5/6/7, **display RST=39**, **touch RST=40 / INT=11**, I²C SDA=15/SCL=14,
  SD SPI MOSI=1/SCK=2/MISO=3/CS=41. Centralisé dans `examples/ws216_bringup/pin_config_ws216.h`.
- **⚠ Piège** : le `pin_config.h` Arduino livré par Waveshare est un **copier-coller
  buggé de la variante ronde 1.75C** (annonce 466×466 + display RST=GPIO2 qui collisionne
  SD_CLK). Ne pas s'y fier — utiliser les valeurs BSP ci-dessus. Le HelloWorld vendeur
  "marche" quand même car 466 sur 480 = image juste rognée.
- **Bring-up** : `examples/ws216_bringup/ws216_bringup.ino` (autonome). **✅ Validé
  hardware (2026-06-22)** : `gfx->begin()` OK, `touch.begin(0x5A)` OK, écran droit
  (rouge en haut-gauche, texte lisible → MADCTL `0xA0` + rotation 0 = orientation
  correcte, ordre couleur RGB OK), tactile linéaire pleine échelle. Sélection via la
  ligne `src_dir = examples/ws216_bringup` commentée dans `platformio.ini`.
- **🧭 Mapping tactile → écran (calibré, à réutiliser dans l'UI)** : la dalle est
  **tournée 90°** vs l'affichage. Transform validé sur les 4 coins :
  `screen_x = (480-1) - touch_y` ; `screen_y = touch_x`. (Alternative SensorLib :
  `setSwapXY(true)` + miroir X.)
- **Portage UI — ✅ VALIDÉ HARDWARE (2026-06-22)** : firmware AT-VIEW complet sur la WS-216,
  rendu propre (noirs profonds, couleurs justes, texte net) + tactile OK. Shim
  `examples/at_core_debug/ws216_shim.h` : flush `draw16bitRGBBitmap` (swap=0), buffer LVGL
  partiel 40 lignes en RAM **interne** (`MALLOC_CAP_INTERNAL`, pas DMA), indev mapping 90°.
  lv_conf dédié `include/lv_conf_ws216.h` (swap=0 → assets logos réutilisés). Écran **carré**
  → branches UI `#else` du T-RGB. ⚠ `LV_CONF_PATH=lv_conf_ws216.h` exige `-Iinclude`.
- **🩺 3 pièges WS-216 résolus (à NE PAS réintroduire) — cf. [[ws216_co5300_green_black]]** :
  1. **Noir verdâtre** ← GFX 1.5.0 (driver CO5300 `Arduino_TFT`). Fix = **GFX 1.6.4 + core 3.x**
     (driver `Arduino_OLED`). Aucun registre vendeur (page 0x20 etc.) ne corrige sur 1.5.0.
  2. **Couleurs corrompues au boot** ← `installSD()` faisait `SPI.begin()` sur **FSPI/SPI2**,
     l'hôte de l'écran QSPI → réinit du bus écran. Fix = **SD sur HSPI** (`SPIClass{HSPI}` dédié).
  3. **Texte live baveux/dédoublé** ← pas d'alignement 2 px des zones de flush partielles.
     Fix = **`rounder_cb`** (x/y début pairs, fin impairs), repris du BSP d'usine.
- **À faire** : init AXP2101 via XPowersLib (optionnel — rail écran ON au POR) ;
  exploiter les coins du format carré (perdus sur le cercle T-RGB). Cf. [[ws216_third_target]].

## Architecture — Pages LVGL

| Page | Accès | Contenu |
|------|-------|---------|
| #01 — Boot/Status | Démarrage | Logos AT-VIEW + AEROTRACE bicolores (A bleu #7393B4 + reste noir), sablier (silhouette 6 segments), 6 dots progressifs (gris → brand-blue), 6 check rows live latches (BLE / Bluetooth / GPS / LTE / ADS-B / OGN), Battery AT-CORE + version |
| #02 — Auth code pilote | Auto après BLE+STATUS+2s (fallback 10s) | Page style fond blanc, logo A, prompt, 4 ronds brand-blue (touch=backspace), keypad 7-8-9 / 4-5-6 / 1-2-3 / 0 ENTER, 3 états (default / wrong rouge / OK vert), diag DB Firebase, swipe bloqué |
| #03 — Have a nice flight | Auto après auth OK (3s) | Logo A, "Have a nice flight !", bandeau bleu plein avec Nom Prénom blanc, ligne rouge instructeur si student, "Status: PILOT - Owner / PILOT - Renter / STUDENT - Renter" |
| 1 — Radar | Swipe → | Trafic SafeSky, position relative, alertes, GS en bas sous scale |
| 2 — Settings | Swipe → | Échelle radar, filtre sol, debug |
| Debug (caché) | Long press version | Logs sysLog BLE |

Navigation : swipe gauche/droite entre pages (bloqué tant que #02 est ouverte).

### Identité visuelle

- **🌐 LANGUE UI = ANGLAIS** : tout le texte affiché à l'utilisateur (labels, boutons,
  messages) est en **anglais** (préférence utilisateur 2026-06-03). Les commentaires de
  code et cette doc restent en français. Écrans Maintenance + Flights déjà traduits ;
  auth/radar/settings à migrer.
- Couleur brand AeroTrace : **#7393B4** (provisoire — peut évoluer)
- Logos bicolores : A bleu #7393B4 + reste noir (sources `public/logo/*.png` → converter `tools/png2lvgl_logos.py`)
- Pages #01/#02/#03 forcent fond blanc (lisibilité logo noir)

### Auth flow détaillé

1. Page #01 (boot + progression checks) — au moins 2s après BLE+STATUS pour laisser voir la progression
2. Page #02 (encodage code) :
   - Code pilote en DB → ronds verts + "Welcome back \<TRG\> !" → page #03
   - Code non trouvé → ronds rouges + "Wrong code - not recognised / Please try again" (1.8s) → reset
   - Code student → ronds verts → bascule prompt "Encode your Instructor Code" → 2ᵉ saisie
3. Page #03 (welcome 3s) → bascule auto vers Radar

## BLE — Client NimBLE

AT-VIEW est **client BLE** — se connecte à AT-CORE serveur.

Scan filtre actuellement sur nom `"AT-CORE NimBLE"`.
**Migration prévue** : filtrer sur préfixe `ATCORE-` (nommage structuré).

| Caractéristique souscrite | UUID | Direction | Contenu |
|--------------------------|------|-----------|---------|
| STATUS | `6E400002-...` | notify | mode, GPS, LTE, BLE, batterie, **flt_ph + up_pct** (V1) |
| FLIGHT | `6E400004-...` | notify | lat, lon, alt, spd, hdg |
| TRAFFIC | `6E400005-...` | notify | tableau trafic (5 max) |
| ALERTS | `6E400006-...` | notify | CO ppm, trafic <500 m |
| DEBUG | `6E400003-...` | notify | logs sysLog |
| AUTH | `6E400007-...` | **write** | codes pilote/instructeur (V2 popup) |
| PILOTS | `6E400008-...` | notify | liste pilotes JSON chunké (Firestore). Format : `[{c,n,r,t,i}, ...]` ou `{"_date":"YYYY-MM-DD","pilots":[{...}]}` (wrapper recommandé pour traçabilité). Protocole chunks : `0x01`=start, `0x02`=data, `0x03`=end (déclenche parse). Résilient : DB préservée si JSON invalide / array vide. |
| CONFIG | `6E400009-...` | **write** | identité aéronef `{r,t,h}` — auto-push depuis `acSave()` (V1) |
| CONTROL | `6E40000A-...` | **write** | `{"cmd":"bind"\|"unpair"}` (appairage) + `{"cmd":"wifi","s","p"}` / `{"cmd":"upload"}` (Maintenance — Modèle 1) + `{"cmd":"vfilt","ft"}` + **`{"cmd":"cloud","on":0\|1}` (v119, toggle upload cloud)**. Helpers `sendCtl()` / `sendWifiCreds()` / `sendVfilt()` / `sendCloud()` |

Service UUID AT-CORE : `4FAFC201-1FB5-459E-8FCC-C5C9C331914B`

## Radar — Logique d'affichage

- Rayon radar (`RAD_R`) : 175 px
- Échelle configurable (`scale_nm`) — stockée en `Preferences`
- Avions hors-échelle : **cachés** (pas clampés à 125px — bug corrigé)
- Icônes : `getAircraftIcon(type)` — 17 types FlyADSL → LVGL image
- Couleur icône : blanc (fond sombre) / noir (fond clair) / orange (<3 km) / rouge (<1 km)
- `show_grnd` : filtre les aéronefs sol (spd < 20 kt)

## Nommage BLE — Convention (à implémenter)

Format : `ATVIEW-<OACI><N>-<SEQ>`
Exemple : `ATVIEW-EBBY1-01`

AT-CORE correspondant : `ATCORE-EBBY1-01`
Liaison sécurisée : MAC AT-CORE stockée en NVS → reconnexion auto.

## Appairage AT-CORE — cérémonie de binding (Phase 3 — 2026-06-01)

Implémenté dans `examples/at_core_debug/at_core_debug.ino`. Empêche AT-VIEW de se
connecter au mauvais boîtier (parking dense). **Plus d'auto-bind silencieux** : tant
qu'aucun MAC n'est lié, on ne se connecte pas au premier `ATCORE-` venu.

| Étape | Comportement AT-VIEW |
|-------|----------------------|
| Non lié (`paired_mac` vide) | Overlay modal `pairOverlayShow()` (LVGL `lv_layer_top`) bloque la navigation |
| Découverte | `ATCAdv::onResult` ne **collecte** que les boîtiers en mode pairing (manuf-data `FF FF 01`, `advPairable()`) dans `g_pcand[]` — pas de connexion |
| Liste | `pairListRefresh()` affiche nom + RSSI des candidats (rafraîchi 1 Hz, TTL 15 s) |
| Sélection (`cbPairPick`) | Mémorise MAC/nom, `g_binding=true`, relance le scan → connexion à CE boîtier **sans figer le MAC** (timeout 10 s → retour liste) |
| Confirmation | Connecté → AT-CORE passe LED **fixe** ; overlay demande « LED fixe ? » (`pairShowConfirm`) |
| Bind (`cbPairConfirm`) | Write `{"cmd":"bind"}` sur CHR_CONTROL + `unitSaveMac()` → fige NVS `unit/paired_mac`, ferme l'overlay |
| Annuler (`cbPairCancel`) | `disconnect()` + retour liste |

- **Déjà lié** : `ATCAdv` ne se connecte qu'au `paired_mac` (inchangé). L'AT-CORE rejette
  de son côté tout peer ≠ `paired_view` (enforcement réciproque).
- **Oublier** : long-press logo AT-VIEW → `_cbForgetPair` envoie `{"cmd":"unpair"}` à
  l'AT-CORE (ré-arme son pairing) puis efface `paired_mac` + reboot.
- Le mutex `g_pcand_mx` protège `g_pcand[]` (rempli dans le cb scan, lu par la loop UI).

⚠️ **À valider hardware** : que la manuf-data `FF FF 01` de l'AT-CORE n'évince pas le nom
du paquet d'advertising (le nom doit rester dans la scan response pour le filtre `ATCORE-`).

## Configuration aéronef (écran Settings)

| Champ | Exemple | Notes |
|-------|---------|-------|
| Immatriculation | `FJFVB` | 2-6 chars, sans préfixe pays |
| Type OACI | `VL3`, `MCR01`, `FK9` | Liste déroulante — codes à fournir |
| Hex transpondeur | `38EDC5` | 6 digits hex |

Stockage NVS :
- namespace `unit` : `name` (nom BLE complet), `paired_mac` (MAC AT-CORE lié)
- namespace `aircraft` : `reg` / `type` / `hex24` (identité aéronef, pushée via CHR_CONFIG)

## Alertes

| Condition | Action |
|-----------|--------|
| CO ≥ 35 ppm | Page radar + indicateur rouge |
| Trafic < 500 m | Page radar + indicateur orange |
| Fin alerte | Paquet clear reçu (`tfc=0`) → retour page précédente |

## Persistance NVS actuelle

Namespace `atview` (`Preferences`) :
- `scale`, `vfilt`, `dist_nm`, `alt_ft`, `bright`, `trf_src`, `show_grnd`, `icon_sz`, `aip_en`, `ad_heli`, `wifi_en`, `dark`
- `spd_kt` — unité vitesse kt/km/h (V1, toggle Settings page 0)

Namespace `aircraft` (V1 — saisi via écran Aircraft, auto-pushé vers AT-CORE via BLE CHR_CONFIG `6E400009`) :
- `reg` — immatriculation (ex `FJFVB`)
- `type` — code OACI (ex `VL3`)
- `hex24` — hex transpondeur (ex `38EDC5`)

Namespace `unit` :
- `name` — nom BLE AT-VIEW (`ATVIEW-EBBY1-01`)
- `paired_mac` — MAC AT-CORE choisi (reconnexion auto)
- `wifi_pass` — mot de passe AP **propre** d'AT-VIEW (maj AIP)
- `hs_ssid` / `hs_pass` — credentials du **hotspot téléphone** à pousser vers AT-CORE
  (écran Maintenance → BLE `{"cmd":"wifi"}`). Distincts de `wifi_pass`.

## Upload progress overlay (V1)

Modal LVGL full-screen (`mkUploadOverlay()`) qui s'affiche sur transition de phase :

| `flt_phase` reçu via BLE STATUS | Affichage |
|---|---|
| 0 (FLYING) | Caché |
| 1 (ENDED) | "Vol terminé — fermeture CSV" |
| 2 (CLOSED) | "CSV fermé — attente upload" |
| 3 (UPLOADING) | "Upload Firebase en cours..." + barre `up_pct` amber |
| 4 (UPLOADED) | "Transfert réussi ✓" vert, auto-hide 5s |
| 5 (UPLOAD_FAIL) | "Échec — nouvelle tentative..." rouge, persiste |

Hook : `updUploadOverlay()` appelé depuis `updateAllPages()` (1s).

## Écran Maintenance (Modèle 1 — 2026-06-02)

Overlay plein écran ouvert via le bouton **MAINTENANCE** (Settings sous-page 1).
`mkMaintenanceOverlay()` (fullscreen `lv_scr_act()`, fermé par `lv_obj_del`).

| Élément | Action |
|---------|--------|
| Bouton « Transferer le dernier vol » | `sendCtl("upload")` → AT-CORE connecte son hotspot + upload Firebase. L'**overlay de progression `up_pct` existant** s'affiche tout seul (STATUS `flt_ph≥1`). |
| Bouton « Tester le hotspot » | `sendCtl("wifitest")` → l'AT-CORE se connecte au hotspot et logue `[WIFI] IP=...`/`FAIL` en série. Diagnostic **au sol sans vol** (l'upload réel exige un vol finalisé). |
| Ligne « Dernier vol » / « Liste des vols » | Le 1er transfère le vol courant ; le 2nd ouvre l'**écran Vols** (WP8). |

### Écran Vols (WP8) — `mkVolsOverlay`

Lit **CHR_FLIGHTS `6E40000B`** (READ) après `{"cmd":"flights"}` (attend `STATUS.flt_rdy==1`
+ ≥1,5 s, lecture seulement si connecté → pas de freeze loop). Liste scrollable taillée
pour le cercle : `MM-DD HH:MM>HH:MM` + `[ ]` cochable ; vols **transférés en gris**
(non sélectionnables, label « envoye »). Boutons : **Transferer (N)** → `{"cmd":"uploadlist"}`
(≤8 fids), **Suppr. transferes** (double-tap de confirmation) → `{"cmd":"delflights"}`,
**Fermer**. Le vol de la session courante n'apparaît pas (→ bouton « Dernier vol »).
| Champ SSID + bouton **Scan** | `lv_textarea` + scan WiFi 2.4 GHz (`WiFi.scanNetworks`, restauré `WIFI_OFF` après, refusé si AP active) → liste cliquable qui remplit le SSID. Un hotspot absent du scan = il est en 5 GHz (diag intégré). |
| Champ mot de passe | `lv_textarea` password. Les deux partagent un `lv_keyboard` **taillé pour le cercle** (320×175 centré — le plein-largeur avait sa rangée du bas hors disque), caché par défaut, suit le focus, masqué sur ✓/✕. Max 32 / 63. |
| Bouton « Enregistrer » | `unitSaveHotspot()` (NVS `hs_ssid`/`hs_pass`) **+** `sendWifiCreds()` → BLE `{"cmd":"wifi","s","p"}`. Feedback « Envoye » (poussé BLE) ou « Sauve (hors ligne) ». |
| Aide MAJ firmware | Texte 2 lignes : **AT-CORE** = BOOT 6 s → WiFi `ATCORE-SETUP` ; **AT-VIEW** = WIFI ON (Settings) → `192.168.4.1`. |

`sendWifiCreds()` échappe `"`/`\` (JSON) et respecte la limite write AT-CORE 200 B.

Écran rond 480×480 : clavier 320×175 centré → coins dans le disque (cf.
[[trgb_round_screen_geometry]]). Le scan WiFi bloque ~2-4 s (coex BLE) → si le lien
BLE tombe, `_maint_save_cb` ne fait que la sauvegarde NVS locale (« Sauve hors ligne »)
et le portail AT-CORE reste le plan B confortable pour saisir le hotspot.

### OTA firmware AT-VIEW (WP7 — 2026-06-02)

Le T-RGB se met à jour **sans câble** via son propre AP (l'infra existait déjà pour
l'upload AIP). Partition `default_16MB.csv` = **2 slots OTA** → aucune migration.

1. **WIFI ON** (Settings) → `wifiStart()` lève l'AP (SSID = nom BLE, pass `wifi_pass`)
   + `WebServer` sur `192.168.4.1`.
2. Téléphone sur l'AP → page web → section **« Firmware (OTA) »** → choisir le
   `firmware.bin` AT-VIEW (`/tmp/pio_build_atview/T-RGB/firmware.bin`) → Flasher.
3. Route `/update` (distincte du `/upload` AIP→SD) : `handleOtaData` → `Update.write`
   sur le slot inactif → `Update.end(true)` → reboot différé (`g_ota_reboot_ms`, loop).

**Garde anti-brick** : `handleOtaData` vérifie l'en-tête image (magic `0xE9` + chip_id
`9` = ESP32-S3 à l'offset 12) sur le 1er chunk → un `.bin` étranger (ex : firmware
**AT-CORE** qui est ESP32 chip_id 0) est refusé (`Update.abort`) avant tout flash.
`yield()` dans la boucle d'écriture (respiration WiFi + WDT éventuel).

## Roadmap

### Court terme (post-V1)
- Popup auth pilote/instructeur (write CHR_AUTH `6E400007` — backend AT-CORE déjà prêt)
- Affichage progression upload basé sur `up_pct` réel (actuellement step 5→50→100 indicatif)

### Moyen terme
- Auto-découverte hex via OpenSky Network (WiFi AT-VIEW hotspot smartphone)
- Display conversion km/h ↔ kt cohérent avec `cfg/spd_kt` (UI seulement)

### Long terme
- Affichage AIP/CTR sur radar : contours CTR + aérodromes (OpenAIP Belgique) — **partiellement en place** (overlay AIP, données via SD)
- Mise à jour AIP via WiFi → hotspot iPhone (SSID/pass en NVS)

## État du projet — V1 (2026-05-17)

**V1 livrée + pushée** — commit `cb18af5` :

- ✅ **A** — BLE CHR_CONFIG WRITE : `acPushBLE()` auto-push aircraft depuis `acSave()`
- ✅ **F** — Upload progress overlay + Speed unit toggle + StatusData étendu (flt_phase/upload_pct)
- ✅ Build local 521s (iCloud lent) + Build GitHub Actions CI success
- ✅ `build_dir = /tmp/pio_build_atview` pour contourner iCloud LDF slowdown

**Stats build** : Flash 29.9% (1.96 MB / 6.55), RAM 27.4% (89.8 KB / 320)

**Validation visuelle requise** (pas de hardware T-RGB sous main pour test) :
- Overlay upload progress quand AT-CORE envoie `flt_ph >= 1`
- Bouton Speed dans Settings page 0 (entre Alt et Bright)
- Push CHR_CONFIG vers AT-CORE après édition Aircraft

## État du projet — V2 (2026-05-18)

**V2 livrée + pushée** — commit `ac0f683` :

### Refonte UI complète (maquettes AeroTrace)
- ✅ Page #01 : logos bicolores (A bleu #7393B4 + noir), sablier en lignes primitives,
  6 dots progressifs (gris → brand selon checks), V latchés (restent affichés jusqu'au disconnect BLE),
  footer compact (Battery + Version font 12 y=418/438)
- ✅ Page #02 (auth) : style page plein écran, keypad `7-8-9 / 4-5-6 / 1-2-3 / 0 ENTER`,
  pas de bouton backspace (tap sur rond rempli efface), 3 états visuels brand/rouge/vert,
  délai 2s minimum après BLE+STATUS (laisser voir progression page #01)
- ✅ Page #03 (Have a nice flight) : bandeau bleu plein avec Nom Prénom blanc, status souligné,
  ligne rouge instructeur si student-renter (cas 2 codes)
- ✅ Radar : GS déplacée du haut (sous heading) vers le bas, sous le scale 4nm

### Robustesse BLE pilotes
- ✅ `_parsePilotJSON` résilient : préserve DB précédente si JSON invalide / array vide
- ✅ Accepte format wrapper `{"_date":"YYYY-MM-DD","pilots":[...]}` (recommandé)
- ✅ Logs `notifyP` verbeux pour debug push depuis AT-CORE (size + byte0 + end-of-stream)
- ✅ Diagnostic DB live sur page #02 : "DB Firebase non chargée" (rouge) → "DB: N pilots (date)" (gris) refresh auto

### Assets / outils
- ✅ Nouveau `img_logo_a` (56×56) extrait de `AerotrAce_A-AeroTrace.png` pour pages #02 et #03
- ✅ AT-VIEW redimensionné à 110×22 (≈ moitié AEROTRACE comme maquette)
- ✅ Converter `tools/png2lvgl_logos.py` préserve les couleurs RGB565 source (plus de force-en-blanc)

### platformio.ini
- `upload_speed` baissé à **230400** (fiabilité USB ESP32-S3 sur macOS)
- `-DARDUINO_USB_CDC_ON_BOOT=1` activé (logs Serial via USB-CDC pour debug)

**Stats build** : Flash 29.6% (1.94 MB / 6.55), RAM 27.4% (89.8 KB / 320)

### Bloqueur en cours
- ⚠ Côté AT-CORE : la DB pilotes Firebase n'est pas encore poussée via BLE CHR_PILOTS.
  Côté AT-VIEW tout est prêt. Travail en cours sur le repo AT-CORE pour activer le push.

## Toggle upload cloud — Settings → SYSTEM → Diagnostic (ATV v119, 2026-07-13)

Bouton **« Cloud: ON/OFF »** ajouté à la page **DIAGNOSTIC** (à côté de Test WiFi / Reboot box /
Unpair box). Pilote l'upload Firebase des CSV **côté boîtier** (miroir de l'AT-CORE v66) :

- **Tap** → `sendCloud(!cup)` = BLE `{"cmd":"cloud","on":0|1}` sur CHR_CONTROL → l'AT-CORE persiste
  `unit/cloud_up` et coupe/arme tout l'upload auto (Phase A atterrissage + §B uploader sol).
- **État réel** lu dans STATUS **`cup`** (0/1) → champ `StatusData.cup` : le label reflète l'état
  boîtier (**vert = ON**, gris = OFF), refresh 1 Hz (`diagCloudBtn()` dans le hook périodique) +
  màj optimiste au tap. Défaut boîtier = **OFF**.
- **Motif** : tant que les **antennes WiFi ne sont pas actives**, l'upload OFF évite que le boîtier
  tente le WiFi STA en boucle (blocage / kill-BLE + reboot). Les CSV restent sur la SD.
- Le bouton `volBtn` retourne l'objet → stocké dans `g_diag_cloud` (remis à `nullptr` à la
  fermeture de la page). Layout : Diagnostic passe à 5 boutons (Close descendu d'une rangée),
  branches T4-S3 (2 colonnes) et rond/carré (1 colonne) mises à jour.

⚠️ Build **`-dev`** non publié Storage → pas d'OTA flotte tant que non béni en canal client (vert).

## Toggle GDL90 ON/OFF — Settings → SYSTEM → Diagnostic (ATV v139, 2026-07-15)

Bouton **« GDL90: ON/OFF »** à côté de « Cloud » dans DIAGNOSTIC — le pilote bascule le mode **VOL/SOL**
au doigt, sans console. Calqué exactement sur le toggle Cloud (v119) :

- **Tap** → `sendGdl(!gdl)` = BLE `{"cmd":"gdl90","on":0|1}` sur CHR_CONTROL → le boîtier lève/ferme
  l'AP `ATCORE-GDL90-<box>` (SkyDemon) et persiste NVS `unit/gdl90`.
- **État réel** lu dans STATUS **`gdl`** (0/1) → `StatusData.gdl` : label **vert = ON (mode VOL)**,
  gris = OFF (mode SOL), refresh via `diagGdlBtn()` dans le hook périodique + màj optimiste au tap.
- **Usage** : **GDL90 ON avant de voler** (SkyDemon reçoit le trafic, reste ON persisté) · **OFF au sol**
  pour libérer le WiFi (OTA/upload — AP GDL90 ↔ STA mutuellement exclusifs).
- Layout Diagnostic passe à 6 boutons (GDL90 + Close en bas ; T4-S3 2 colonnes, rond/carré 1 colonne).

⚠️ **Prérequis = le fix heap v138** : avant, les writes CHR_CONTROL de l'écran (dont ce toggle) échouaient
silencieusement (Bluedroid `Malloc failed`). Depuis v138 (buffers LVGL réduits → +82 Ko), tout écran→boîtier passe.

## Page de saisie hotspot WiFi — Settings → SYSTEM → WIFI (ATV v130→v133, 2026-07-14)

**But** : que **n'importe quel opérateur** provisionne **son** hotspot téléphone pour l'OTA cloud
**depuis l'écran**, sans USB ni portail (l'AP portail est instable si un écran est connecté en BLE,
cf saga AT-CORE v70-v76). La tuile **SYSTEM → WIFI** (`_open_wifisetup_cb`) ouvre **`showHotspotEntry()`** :

- Champs **SSID** + **Password** au **clavier LVGL** (toujours visible sur la page dédiée ; `_maint_ta_cb`
  bascule la cible SSID↔password au tap) → bouton **« Save & send »** = `_maint_save_cb` →
  `unitSaveHotspot()` (NVS `hs_ssid`/`hs_pass`) **+ `sendWifiCreds()`** = BLE **`{"cmd":"wifi","s","p"}`**
  sur CHR_CONTROL → le boîtier persiste en NVS. Feedback honnête **« Sent »** (poussé BLE) / « Saved (offline) ».
- Repli **« Web portal »** (`_hotspot_useportal_cb` → `showWifiSetupInfo`) : boîtier sans écran / SSID à espaces.
- **Réutilise 100%** des widgets/callbacks du (feu) Maintenance (`g_maint_ov` + `_maint_save_cb`/`_maint_ta_cb`,
  restés présents). Le Save est un **bouton séparé** (ne détruit pas le clavier dans son event) → **PAS** le
  freeze use-after-free de l'éditeur immat clavier (v120-127, abandonné). ✅ **Validé hardware WS-241 2026-07-14**
  (« Sent », fluide). Miroir boîtier = **FW v78** (console `wifi <SSID> <PASS>`).
- ⚠️ **`Scan` RETIRÉ** (v131) : `WiFi.scanNetworks()` allume le **WiFi STA sur un écran déjà connecté en BLE**
  (Bluedroid) → coexistence WiFi+BLE fragile sur WS-241 → **HANG**. On saisit le SSID au clavier.

### ⚠️ GOTCHA `lv_keyboard` invisible (la « bêtise » v130-132, corrigée v133)

`lv_keyboard_create()` **s'auto-aligne `LV_ALIGN_BOTTOM_MID`** en interne. Positionner le clavier avec
**`lv_obj_set_pos()` NE VIDE PAS cet align** → à la passe de layout l'align interne gagne → clavier
mal placé / **hors zone visible** (les champs/boutons, eux, s'affichaient → fausse piste « flush WS-241 »).
**Fix : `lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0)`** (comme la branche ronde `#else` qui marchait déjà)
+ overlay hauteur **`SCR_H`** (450) et non 480 (sinon 30 px sous l'écran). Réflexe : **toujours `lv_obj_align`
pour un `lv_keyboard`, jamais `set_pos`**.

### GDL90 (AP WiFi boîtier) ↔ écran BLE — coexistence DURCIE côté boîtier (FW v80, 2026-07-14)

Découvert en testant la page : **quand GDL90 est ON sur le boîtier** (WiFi AP + BLE + LTE), l'écran **boot-loopait à la
connexion BLE** — crash heap Bluedroid dans `connectBLE()` (`FreeRTOS::Semaphore` → `xQueueGenericSend` sur queue nulle =
création sémaphore KO faute de heap ; l'AP GDL90 affame le BLE → **découverte GATT** dégradée). **Le crash = la DÉCOUVERTE
pendant que l'AP est up ; l'AP up pendant écran DÉJÀ connecté = OK** (le steady-state notify n'alloue pas de sémaphore par
caractéristique).
- **Fix côté BOÎTIER (FW v80, PAS côté écran)** : `TaskGDL90` **diffère la 1ʳᵉ levée de l'AP** au boot tant que l'écran ne
  s'est pas connecté + stabilisé (6 s) → l'écran fait sa découverte SANS AP, puis l'AP monte → l'écran reste connecté.
  ✅ Validé hardware (`ble=1` stable, 0 déconnexion, AP levée à ~17 s). GDL90 + écran tiennent ENSEMBLE.
- ⚠️ **Limite résiduelle** : un **reboot de l'écran EN VOL** (AP déjà up) re-déclencherait le crash de découverte (rare).
  Le vrai filet définitif = **migrer le client BLE de l'écran de Bluedroid → NimBLE** (heap ×, comme le boîtier) — TODO si
  le cas se présente.
- ⚠️ **Ne pas monitorer le série USB-CDC de l'écran pendant un connect BLE** : ça aggrave la marge heap et **fausse le
  diagnostic** (le boot-loop apparaissait sous monitoring, disparaissait sur powerbank seul — piège rencontré ce jour).

## 2026-09-19 — v267 : FIX « No flights on SD » = bug NimBLE-Arduino 2.x (lecture BLE > 275 o corrompue)

- **Symptôme** : page Flight Logs → « No flights on SD » alors que le boîtier sert une liste JSON valide (453 o, 7 vols ;
  prouvé côté boîtier par `onRead` CHR_FLIGHTS + copie exacte sur SD, et par un test natif ArduinoJson 7.4.3 sur le Mac = Ok).
- **Cause** (v266 diag : dump hexa de la valeur lue) : à partir de l'**octet 275** la valeur lue est de la mémoire quelconque.
  `NimBLERemoteValueAttribute::onReadCB` (NimBLE-Arduino 2.5.0, idem master 09/2026) fait
  `valBuf->append(attr->om->om_data, OS_MBUF_PKTLEN(attr->om))` = longueur TOTALE copiée depuis le PREMIER mbuf seulement.
  Un bloc mbuf = 292 o → 275 o de données ; toute lecture > 275 o (CHR_FLIGHTS, CHR_WSCAN…) est corrompue → ArduinoJson
  « InvalidInput » → l'ancien code retournait en silence (label « No flights » restant / liste vide).
- **Fix** : `tools/patch_nimble_readcb.py` (PlatformIO `extra_scripts = pre:` sur WS-241, WS-241-dev, T4-S3) patche la lib
  après téléchargement : copie via `os_mbuf_copydata` à travers la chaîne. Idempotent ; log « motif introuvable » si la lib
  change. + `volsBuildList` : « List error - retry » si parse KO (plus de retour silencieux) + log `[VOLS] read N o … parse OK n=7`.
- **Validé** banc 19/09 : écran v267 + boîtier X.1.203 → `[VOLS] parse OK: isArray=1 n=7`, 7 vols affichés.
- **À publier** : tag `atv/ws241` = 267 (la flotte est en 264 → même bug partout). Boîtier : rien à changer (serveur non concerné).

## 2026-09-19 — v268/v269 : fids longs + CALIBRATION IMU « Level IMU » + fix stabilité (biais gyro)

- **v268** : `VolItem.fid` 20→28, trame `uploadlist` 240→360 o (boîtier v204 : `ATC-<BOX>-YYMMDD-HHMM`).
- **v269 — cause de l'ADI décalé au replay** : (1) l'auto-zéro continu exigeait `gmag<3 °/s` sur le module gyro BRUT ; cet écran a un
  **biais gyro ≈ 5,3 °/s au repos** → jamais stable → **jamais d'auto-zéro, jamais d'IMU** (mesuré : `|a|=1.00 gyro=5.3 cal=0` en
  continu) ; les valeurs vues auparavant (pitch 37/roll -5) venaient d'un repère figé à un instant quelconque. (2) même sans biais,
  l'auto-zéro continu re-figeait le repère au dernier instant stable (pente au point d'attente, écran en main) → ADI faux tout le vol.
- **Fix** : stabilité jugée sur l'ACCÉLÉROMÈTRE (vecteur quasi constant `dev<0.03 g`, `|a|≈1 g`), biais gyro appris au repos et
  soustrait (`gmag` corrigé, utilisé pour le peak-hold) ; bouton **Diagnostic → « Level IMU »** (2 taps, DERNIER de l'ordre molette, sous
  « Club mode », page scrollable) : moyenne de 12 échantillons stables (abandon 6 s → « IMU not stable »), repère repos **persisté NVS**
  `atview/imu_cal, imu_d0x..z, imu_f0x..z`, chargé au boot, **prioritaire sur l'auto-zéro** (`g_imu_manual`). Trace Serial
  `[IMU] raw |a| gyro dev still cal level` tant que non calibré ou en cours.
- **Procédure pilote** : avion À PLAT au sol (moteur coupé), écran dans sa position de montage → Diagnostic → Level IMU ×2 → « IMU levelled ».
  À refaire si l'écran est déplacé/remonté. Validé banc 19/09 : `calibration à plat SAUVÉE d0=(0.994,0.033,-0.101)`, nz 1.00, pitch 0, roll 0.
- Les vols DÉJÀ enregistrés gardent leurs angles bruts (replay ADI décalé) ; seuls les vols après calibration sont corrects.
- **À publier ensemble** : ATV 269 (tag ws241) + ATC 205 (tag s3/wrover) — les fids longs exigent l'écran ≥ 268.

## 2026-09-20 — Design system AirKi sur l'écran : v270 (fondations) + v271 (radar)

Source DS : `01 - Documentation/design_handoff_airki/` (règles : AirKi jamais en capitales, pas de dégradé/ombre, ambre = accent jamais
texte, Instrument Sans + Geist Mono, AKV/AKT = désignateurs). Décisions Christophe 20/09 : plein panneau 600×450 ✓ (déjà le cas :
RAD_CX 375 v230) ; radar SafeSky blanc / AT-1 vert / own ambre ; cibles TRIANGLE ou ICÔNE au choix menu ; bouton NOIR/BLANC sur le radar ;
polices converties ✓ ; AKV/AKT ✓.
- **v270** : `tools/fonts/` = Instrument Sans (VF Google Fonts → instances SemiBold 600 / Bold 700 via fonttools) + Geist Mono Medium 500 ;
  `examples/at_core_debug/fonts/airki_sans_{12..40}.c`, `airki_mono_{13..40}.c` (lv_font_conv 1.5.2, bpp 4, Latin étendu + symboles
  FontAwesome de LVGL). Les `lv_font_montserrat_NN` historiques sont REDIRIGÉS par macro (26→28, 30/34→32, 38→40) ; `FM_NN` = mono.
  Jetons : UI_BG #141414, UI_SURF #1C1C1A, focus #2C2C2C, UI_INK blanc, UI_INK2 #9A9A94, chevron #8D9096, C_BRAND = ambre, thème clair
  = papier #F4F2ED / encre. `VIEW_VER_STR` = « AKV ». Flash +150 Ko (les tailles non référencées ne sont pas liées).
  Régénérer une taille : `npx lv_font_conv@1.5.2 --bpp 4 --size N --font tools/fonts/InstrumentSans-SemiBold.ttf -r 0x20-0x7F,0xA0-0x17F,0x2013-0x2026
  --font <lvgl>/scripts/built_in_font/FontAwesome5-Solid+Brands+Regular.woff -r <syms de built_in_font_gen.py> --format lvgl --lv-include lvgl.h -o …`.
- **v271** : Traffic → **TARGETS : ICONS / TRIANGLES** (`g_cfg.trf_tri` relu en NVS, dessin v115 déjà en place ; section Traffic passe à
  8 lignes Y0=16/DY=56) ; **bouton W/B** en haut à gauche du radar (40 px, bordure 1 px) : bascule `g_cfg.dark` + `rebuildAllPages()` en
  différé (`lv_async_call`). Reste (étape 3) : page d'accueil AirKi View (monogramme, points GPS/LTE/TRAFFIC, UTC, immat), couleurs cibles
  radar (blanc/vert/ambre), listes vols / code pilote / réglages restylés, textes « AT-VIEW/AT-CORE » → « AirKi View / AirKi Core ».
- Flashé sur l'écran de Christophe (v271). Non publié (tag ws241 = 269).

**RÈGLE DE CONCEPTION (Christophe, 2026-09-20) — toutes les pages** : utiliser TOUT le panneau 600×450 (pages en `lv_obj_set_size(p,SCR_W,SCR_H)` à (0,0),
plus de canevas 480 centré) et réserver **5 px minimum vierges sur les 4 bords** : aucun texte ni objet collé au bord. Thème noir/blanc = RADAR
uniquement ; toutes les autres pages sont sur encre #141414. Couleurs = jetons DS exacts (#141414, #9A9A94, #8D9096, #2C2C2C, #F5A623, #22C55E).

## 2026-09-20 — v272→v274 : page d'ACCUEIL « AirKi View » (maquette validée)
- Plein panneau 600×450 à (0,0), coins carrés, encre #141414 (l'accueil ne suit PAS le thème N/B, réservé au radar).
- Monogramme deux couleurs `img_airki_mark` (96 px, généré : public/logo/AirKi_mark_duo_white.png → tools/png2lvgl_logos.py).
- « AirKi » Bold 40 + « View » SemiBold 20 sur la même ligne de base (décalage +18 = (49-9)-(29-7), MESURÉ dans les fontes — toujours
  calculer les alignements à partir de `.line_height`/`.base_line`, jamais à l'œil). Baseline « Not alone in the sky » 18 #9A9A94.
- Lignes GPS / LTE / TRAFFIC : pastille 12 px (#22C55E prêt · #F5A623 attente · #8D9096 hors service) + mono 20 + détail mono 18.
- Colonne droite calée à −40 du bord : PILOT (authentifié, sinon PROPRIÉTAIRE reçu du boîtier « own », v274) · BOX (STATUS box) ·
  HEX (g_ac_hex) en mono 28 · FIRMWARE « AKV n · AKT n » mono 18. Bas : UTC (trame FLIGHT « utc », v273) mono 28 + immat mono 28.
- Bascule auto vers le radar : BLE + fix GPS + LTE (csq>5) tenus **5 s d'affilée**.
- Gestes conservés : appui long monogramme = oublier l'appairage ; appui long 8 s ligne firmware (mode club) = PIN admin.
- LEÇON v272 : polices lv_font_conv générées COMPRESSÉES par défaut alors que `LV_USE_FONT_COMPRESSED 0` → texte invisible →
  toujours `--no-compress`. Les 17 fontes sont régénérées ainsi.

### v278 (20/09) — radar épuré (WS241 uniquement)
- Cercles seuls : graduations 30° (`tm`), quadrants `hl`/`vl` et lettres N/S/E/W (`r_card`) masqués sous `BOARD_WS241` (objets conservés → le code commun T4/T-RGB est inchangé).
- Nord = un trait TFG largeur 4 (`r_north`, de R-16 à R+2), repositionné dans la boucle `r_card` de `updateAllPages` (tourne avec `radarEffHdg()`).
- Position propre = chevron ambre PLEIN : objet 44×44 centré sur (RAD_CX,RAD_CY), dessin `lv_draw_polygon` ×2 triangles convexes (LVGL 8 ne remplit pas les concaves) au `LV_EVENT_DRAW_MAIN_END`.
- GND/FLT (`r_ss_gnd`) en bas à DROITE, Geist Mono 22 (`FM_22`), align BOTTOM_RIGHT(-16,-12). Toujours affiché seulement en mode éco sol SafeSky (ss_mode==1).
- Icônes bas-gauche resserrées : SafeSky pill 40×40 (16,394) zoom 400 ; LTE pill (56,394) barres 4 px {8,12,16,21} ; GPS pill (108,394) symbole 24 abaissé de 5 px. Centres x 36/88/140, axe y 414.
- v279 : `r_gear_btn` — `updateAllPages` faisait `clear_flag(HIDDEN)` hors mode club → engrenage noir fantôme au-dessus de SafeSky malgré le masquage v276. Sous `BOARD_WS241` : toujours HIDDEN. Règle : tout objet masqué dans build*Page doit être vérifié dans updateAllPages (même piège que le haut-parleur v277).

### v280 (20/09) — radar = spec « AirKi View Radar » (01 - Documentation/design_handoff_airki, MAJ 20/09)
Source : `AirKi View Radar.dc.html` (maquette 1:1), `AirKi Status Icons.dc.html`, `CLAUDE.md` du bundle (section « Radar screen — layout rules »). Tout sous `BOARD_WS241`.
- Géométrie : `RAD_CY 228`, `RAD_R 174` (intérieur 87). Anneaux 2 px TFG à 34 % (`border_opa 87`), repères de quart 2 px (R-6→R+4) fixes aux 4 points écran, trait NORD 3 px plein (R-8→R+6) qui tourne avec le cap (choix : le trait plein = nord, pas le cap ; en north-up au sol il est en haut).
- Blocs de coin : `g_akY[4]` calculés sur `FM_13/FM_40->line_height` (18 / +3 / +14). GS KT + ALT FT haut-gauche x=20, HDG + RANGE haut-droite x=-20. RANGE = nombre seul (`kScaleNum` via macro `RAD_SCALE_TXT`) + « NM » FM_22 gris aligné sur la ligne de base (base_line). HDG absent = « --- » etch (plus de « NF » ambre), cap `%03d` sans °.
- Icônes d'état dessinées (`mkIcon` + `icGpsDraw/icLteDraw/icSsDraw`, DRAW_MAIN_END) 26 px à x=20/64/108, y=408 ; états `g_ic_gps` (0 etch / 1 ambre / 2 blanc + point vert), `g_ic_lte` (barres 0-4, éteintes etch), `g_ic_ss` (0 etch / 1 contour blanc / 2 cœur vert). Pas de rouge (spec). Pilules SafeSky/LTE/GPS masquées. Créées APRÈS `r_aip_layer` (au-dessus du wash).
- Bas-droite : `r_ak_reg` (immat boîtier, FM_14 gris) + `r_ss_gnd` encadré 1 px TGRID, pad 5/9, radius 3, FM_14, TOUJOURS visible (`#ifdef` dans updateAllPages), `lv_obj_align_to` OUT_LEFT_MID -16.
- Avion propre : chevron ambre échancré (0,-20)(14,18)(0,10)(-14,18). Aérodromes AIP en etch (plus d'ambre hors avion propre ; la MENACE reste ambre = sécurité, choix assumé).
- Cibles : `srcCol` forcé vert #22C55E (suivi et à jour) ; `TrfScr.stale` ; triangle spec 13/11 plein, périmé = contour (intérieur repeint TBG, centroïde (0,3)) ; libellés FM_13 : Δalt ft (alt_m×100) au-dessus blanc, distance NM dessous gris, largeur 64, côté extérieur (±22 px), masqués si un contact précédent < 40 px. Immat/callsign plus affiché sur la carte WS241.
- AIP : wash `#60A5FA` opa 36 (type 13 → 20), filet 2 px opa 140 (type 13 → 90 + tirets 6/5).
- PIÈGE corrigé en cours de route : `if(false){…} else hide` masquait les libellés → les anciens blocs show_cs/show_vdiff sont sous `#ifndef BOARD_WS241`.
- Non fait : police 15 px (immat/GND en 14), cibles tactiles 44 px, STOP inchangé (ambre, en vol seulement).
- v282 : CRASH au retour radar après THEME (2 traces capturées : LoadProhibited dans `_lv_obj_get_ext_draw_size` ← `lv_obj_set_pos` ← `updClubUi()` ; puis `realloc() pointer is outside heap areas`). Cause : `rebuildAllPages()` fait `lv_obj_clean(g_pages[1])` mais `r_spk_btn`/`r_spk_arc[]` (créés à la volée dans updClubUi, pas dans buildRadarPage) gardaient un pointeur mort. Fix : remise à nullptr avant le clean. RÈGLE : tout objet créé hors build*Page sur une page doit être remis à zéro dans rebuildAllPages. Méthode : sonde série `dtr=rts=True` (pas de reset) + `xtensa-esp32s3-elf-addr2line -pfiaC -e /tmp/pio_build_atview/WS-241/firmware.elf <bt>` (build_dir = /tmp/pio_build_atview).
- v283/v284 : libellés trafic Δalt en CENTAINES de ft (« +9 », règle AT-VIEW) Geist Mono 22 au-dessus (+4 px), IMMAT 14 gris dessous (+4 px) — la distance NM de la spec a été refusée par Christophe (« code bidon »). v285 : couleur par origine (TFG / vert AT-1 / bleu SafeSky natif src=2 ATC ≥208) + `RAD_R 180` (échelle : `px_per_nm = RAD_R/scale_nm` → anneau extérieur = RANGE, intérieur = RANGE/2).
- v286 : `TrfScr.src` ; src 3 (AirKi, ATC ≥209) = bleu #1E90FF + halo `lv_draw_arc` r 20 largeur 2 dans aipDrawCb (avant alertRingsDraw). src 2 = bleu sans halo.
- v287 : canal OTA écran suit le boîtier (FLIGHT « dev », ATC ≥210) : `g_box_dev` (NVS atview/otadev) → `atvOtaTag()` = ATV_OTA_BASE + "dev" / base / tag compilé si inconnu. v288 : ssm 2 (parking) = trafic gris, accueil « parking ».
- 2026-09-20 16h35 : **ATV 288 PUBLIÉ flotte (ws241) + dev (ws241dev)**, avec ATC 211. Contient tout le radar AirKi v275-288 + fix crash thème v282.
- v289 : désignateur boîtier = **AKC** (AirKi Core), PAS « AKT » (le bundle design_handoff écrit AKT : coquille, règle Christophe 20/09). AKV = AirKi View. Non publié (flotte 288).
- RÈGLE (Christophe 20/09) : on travaille et on publie **uniquement sur les canaux DEV** (s3dev / ws241dev = CE276D + 885685 Pierre) ; la flotte (s3 / wrover / ws241) ne reçoit qu'une version validée en DEV, sur décision explicite de Christophe. ATV 289 publié ws241dev.
- v290/v291 : SETTINGS page 1 = maquette « 5 · Réglages » (artefact mockups AirKi View, validée « j'aime assez bien ») : `p1Row`/`p1Switch` (WS241) — lignes 42 px radius 6 bord 1 px #2C2C2C, nom sans 20, valeur mono 18 muted, chevron etch, pilule 44×24 (on = blanc/knob encre, off = surface/knob etch), focus = surface + bord ambre. Lignes : Aircraft › (portail), Display ›, Flight ›, Setup › (sections existantes), Level IMU › (2 taps, `_p1_level_cb`), Club mode (pilule), Diagnostic › (AKV · AKC · SD). Pied « AirKi View · AKV n · date ». Groupe molette = `g_p1Rows[]` (ordre visuel). L'ancienne liste mkMenuRow reste compilée sous `if(false)` (T4 non-WS241 la garde). PIÈGE : le bloc `{char ac[40]…}` partagé #else/#endif ferme le `if(false){` — ne pas réorganiser sans vérifier les accolades. « Cloud upload » de la maquette n'existe pas comme fonction → non ajouté.
- v292 : SOUS-PAGES Settings (WS241) — `secRow` (560×48, bord 1 px #2C2C2C radius 6, libellé sans 20), `secTrack`/`secCell` (segments : actif = blanc/encre), pilule pour les lignes OFF/ON (`SegCtl.pill/knob`, segA/segB cachés 1×1, `_segRowToggleCb`), `mkPopRow` = valeur mono 18 + chevron, brightness = slider fin blanc + `s_bright_sl` (le focus molette est la LIGNE), `mkNavRow` = nom + chevron, en-tête section titre sans 28 + retour rond 1 px, `encFocusOutline` = bord ambre (focus) / vert (édition) sans halo, titres toujours blancs. Debug = « AirKi View / AirKi Core » valeurs mono. `mkActRow` (AT-1 setup, overlays) PAS restylé.
- v293 : page 1 = retour v290 (Christophe : « je préfère ta première version ») ; liste plate v291 conservée sous `if(false)`. `DEV_UI()` (= `g_box_dev==1` ou -DATV_OTA_DEV) : Debug (menu), Diagnostic et Test (Setup) visibles seulement sur le canal DEV. Sur la flotte, Level IMU n'est donc accessible que via Diagnostic sur DEV → à reconsidérer si un pilote flotte doit niveler (proposer une ligne « Level IMU » dans Display ?).
- v294 : SOUS-SOUS-PAGES (overlays) restylées sous WS241 — `ovHeader` (titre sans 28 + retour rond 44 px 1 px etch, focus bord ambre), `mkActRow` (560×48 bordée ; ENFANTS [0] pastille fantôme 1×1 pour garder `child(1)` = libellé dans les callbacks d'armement, [2] pilule), `mkInfoRow` (sans bord, valeur mono 14 largeur 400 clip), `mkSwitchRow`/`switchSet` (pilule via p1SwitchSet), `rowValue` mono 18, Updates = « AirKi Core / AirKi View » valeurs gris, `pickShow` (panneau bordé, sélection blanc/encre, options mono 18), WiFi Setup (page réécrite : étapes sans 18, identifiants mono, primaire blanc / secondaire bordé), Flight Logs (cadre bordé, lignes mono 14 bordées, sélection blanc/encre, `volActBtn` bordés focus ambre). `ROW_BG` (UI_BG sur WS241, UI_SURF ailleurs) = fond de repos remis par les confirmations. Non restylé : page Test (dev), clavier Hotspot/AT-1 (textareas), toasts.
- v295 : bouton RETOUR (`backBtnStyle`) 52 px bord blanc, chevron sans 22 recentré (-2 px), focus/appui = disque ambre + chevron encre. **Publié ws241dev = 295** (20/09 soir). Flotte reste ATV 288 / ATC 211. ÉTAT DU CHANTIER DESIGN ÉCRAN (pause « on cloisonne ») : accueil ✓, radar ✓, Settings page 1 ✓ (v290 style), sections Display/Flight/Setup/Debug ✓, overlays Updates/Flight Logs/Diagnostic/WiFi Setup/popups ✓. RESTE : page Test (dev), clavier Hotspot/AT-1, toasts, ligne « AT-1 traffic setup » (mkActRow → déjà restylée via mkActRow WS241 ✓), page code pilote, mode club verrouillé, Level IMU pour la flotte (Diagnostic = DEV seulement).
- v296 : MENUS RESTRUCTURÉS (décision Christophe 20/09) — page 1 : Display · Traffic · Aircraft · Flight Logs · System · Diagnostic (DEV), lignes 58 px (sans 22 + sous-titre 14). `kSecName` WS241 = {Display,Traffic,Aircraft,System,Diagnostic,Flight Logs} ; section 4 = overlay Diagnostic, 5 = overlay Flight Logs (settingsOpenSection). DISPLAY (scrollable) = Brightness, Theme, Radar scale, Targets, Icon size, Alt difference, Callsign, AIP airspace, Grounded traffic. TRAFFIC = Traffic source, Vertical filter, Alert sound, AT-1 setup, GDL90 to EFB, NMEA to EFB. AIRCRAFT = Identity › (portail, valeur immat · type · hex rafraîchie à l'ouverture, `s_ac_ident_v`), Level IMU (2 taps, `_p1_level_cb`, accessible flotte). SYSTEM = WiFi Setup, Updates, Club mode (pilule), Reboot box (2 taps). FLIGHT LOGS = overlay + bouton « Cloud upload: ON/OFF » (6e bouton, à côté de Del all). DIAGNOSTIC (DEV) = WiFi, AirKi View/Core versions, batterie/SD, Reboot, Unpair, Report to fleet, Test. Les anciens blocs Flight/PILOT/Setup restent sous `#ifndef BOARD_WS241` ; s_sec[4] Debug est toujours construit (pointeurs s_sys_*) mais plus listé. Ordre molette page 1 `gordT4={0,1,2,5,3,4}`.
- v297 : molette morte au retour Test → Diagnostic : overlays IMBRIQUÉS partagent `g_ovGroup` (ovBegin de l'enfant vide le groupe du parent, ovClose le laisse vide). Fix : `diagRebuild()` appelé à la fermeture de Test. RÈGLE : tout overlay ouvert depuis un autre overlay doit, à sa fermeture, reconstruire le parent (ou le parent doit être fermé avant).
- v298 : MISE EN PAGE — les 8 overlays (vols, upd, diag, wifisetup, relay, maint/hotspot, test, pick) étaient en 600×480 sur une dalle 450 → tout ce qui s'aligne en BAS débordait de 30 px (boutons WiFi Setup à y 416-460, clavier hotspot, popup centré à 240). `OV_H` (450 sur WS241). WiFi Setup : ovBegin/ovAdd/ovReady (molette Open portal / Close, focus bord ambre), ovClose à la fermeture. Diagnostic : WiFi 84, Versions 120 (« AKV n · AKC n »), Battery/SD 156, actions 200/256/312/368 → 416, plus de scroll. RÈGLE MEP WS241 : conteneur plein écran = 600×450, marge 20 px (≥5 px), rien aligné en bas sans OV_H, lignes 48 px bord 1 px radius 6, boutons radius 4, focus = bord ambre.

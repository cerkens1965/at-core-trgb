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

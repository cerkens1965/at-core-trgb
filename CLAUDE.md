# AT-VIEW — at-core-trgb — CLAUDE.md

## Matériel cible

| Composant | Détail |
|-----------|--------|
| Board | LilyGo T-RGB 2.8" circular |
| SoC | ESP32-S3 (WiFi 2.4GHz intégré) |
| Écran | 480×480 px circulaire, RGB panel |
| Touch | Capacitif intégré |
| SD | Slot natif SD_MMC (SDIO 1-bit) — EN=7, SCK=39, CMD=40, DAT=38 |
| BLE | Client — se connecte à AT-CORE (NimBLE) |

## Fichier principal

`examples/at_core_debug/at_core_debug.ino` — firmware AT-VIEW v0.6

## Build — PlatformIO

```bash
pio run -e T-RGB -t upload
```

Le `platformio.ini` à la racine pointe sur `src_dir = examples/at_core_debug`.

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

## Écran Settings BLE + Config aéronef (à implémenter)

Nouvel écran Settings pour :

### Pairing BLE
- Scan BLE filtré sur préfixe `ATCORE-`
- Liste LVGL des AT-CORE détectés (nom + RSSI)
- Sélection → MAC stockée en NVS (`paired_mac`)
- Au démarrage : connexion directe si MAC connue + device présent

### Configuration aéronef
| Champ | Exemple | Notes |
|-------|---------|-------|
| Immatriculation | `FJFVB` | 2-6 chars, sans préfixe pays |
| Type OACI | `VL3`, `MCR01`, `FK9` | Liste déroulante — codes à fournir |
| Hex transpondeur | `38EDC5` | 6 digits hex |

Stockage NVS (`Preferences` namespace `atview`) :
- `unit_name` — nom BLE complet
- `paired_mac` — MAC AT-CORE associé
- `ac_reg` — immatriculation
- `ac_type` — type OACI
- `ac_hex` — hex transpondeur

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
- `wifi_pass` — mot de passe AP local

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

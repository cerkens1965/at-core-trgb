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
| 0 — Boot/Status | Démarrage | Logos AeroTrace + AT-VIEW, état BLE/GPS/LTE/bat |
| 1 — Radar | Swipe → | Trafic SafeSky, position relative, alertes |
| 2 — Settings | Swipe → | Échelle radar, filtre sol, debug |
| Debug (caché) | Long press version | Logs sysLog BLE |

Navigation : swipe gauche/droite entre pages.

## BLE — Client NimBLE

AT-VIEW est **client BLE** — se connecte à AT-CORE serveur.

Scan filtre actuellement sur nom `"AT-CORE NimBLE"`.
**Migration prévue** : filtrer sur préfixe `ATCORE-` (nommage structuré).

| Caractéristique souscrite | UUID | Contenu |
|--------------------------|------|---------|
| STATUS | `6E400002-...` | mode, GPS, LTE, BLE, batterie |
| FLIGHT | `6E400004-...` | lat, lon, alt, spd, hdg |
| TRAFFIC | `6E400005-...` | tableau trafic (5 max) |
| ALERTS | `6E400006-...` | CO ppm, trafic <500 m |
| DEBUG | `6E400003-...` | logs sysLog |

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

`Preferences` déjà utilisé pour :
- `scale_nm` — échelle radar
- `show_grnd` — filtre sol

## Roadmap

### Court terme
- Nommage BLE structuré `ATVIEW-<OACI><N>-<SEQ>`
- Écran pairing BLE + saisie config aéronef

### Moyen terme
- Migration config aéronef → NVS (immatriculation, type, hex transpondeur)
- Auto-découverte hex via OpenSky Network (WiFi AT-VIEW hotspot smartphone)

### Long terme
- Affichage AIP/CTR sur radar : contours CTR + aérodromes (OpenAIP Belgique)
  - Pas héliports, pas hydrobases
  - Option activable dans Settings
  - Données stockées sur SD (slot natif SD_MMC, 16-32GB)
  - Mise à jour via WiFi → hotspot iPhone (SSID/pass en NVS)
  - `panel.installSD()` déjà disponible dans la board library
  - Vecteur GeoJSON pour CTR/aérodromes — tuiles raster possibles si SD
  - WiFi AT-VIEW indépendant du AT-CORE (pas de consommation carte IoT)

## État du projet (2026-05)

- AT-VIEW v0.6 stable — radar, alertes, settings scale OK
- GitHub Actions CI corrigé (`pio.yml` + `arduino_ci.yml`)
- Repo `at-core-trgb` public → Actions gratuites illimitées
- Prochaine étape : nommage BLE + écran pairing + config aéronef NVS

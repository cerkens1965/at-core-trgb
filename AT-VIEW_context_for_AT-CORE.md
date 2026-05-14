# AT-VIEW — Contexte pour développement AT-CORE

**Document de référence à partager avec le projet AT-CORE (SIM7600)**
Version firmware : AT-VIEW v0.7 — 2026-05-14

---

## Matériel AT-VIEW

- Board : LilyGo T-RGB 2.8" Full Circle
- SoC : ESP32-S3, 16MB Flash, 8MB OPI PSRAM
- Écran : ST7701S 480×480 circulaire
- Touch : CST820 (LVGL event-based)
- SD : SD_MMC 1-bit (EN=GPIO7, SCK=39, CMD=40, DAT=38)
- BLE : Client NimBLE — se connecte à AT-CORE serveur

---

## Architecture pages LVGL

| Page | Navigation | Contenu |
|------|-----------|---------|
| **0 — Status** | Boot / swipe | Logos AeroTrace + AT-VIEW, état BLE/GPS/LTE/SD/bat, nom AT-CORE connecté |
| **1 — Radar** | Swipe → | Trafic heading-up, icônes 17 types, dead reckoning 200ms, overlay AIP (CTR + aérodromes) |
| **2 — Settings** | Swipe → | 2 sous-pages (pills < 1/2 >) |
| **Aircraft (overlay)** | Settings 2/2 → Aircraft | Saisie IMMAT / TYPE OACI / HEX transpondeur |
| **Debug (caché)** | Long press version label dans Settings 2/2 | Logs sysLog BLE |

---

## BLE — Identification et pairing

### Nommage
- AT-VIEW s'annonce : `ATVIEW-<OACI><N°école>-<N°appareil>` — ex: `ATVIEW-EBBY1-01`
- AT-CORE doit s'annoncer : `ATCORE-<OACI><N°école>-<N°appareil>` — ex: `ATCORE-EBBY1-01`
- AT-VIEW filtre le scan sur le préfixe `ATCORE-`

### Connexion
- Si `paired_mac` en NVS → connexion directe à ce MAC uniquement
- Sinon → premier `ATCORE-` trouvé
- Connexion réussie → MAC sauvegardé en NVS (`unit/paired_mac`)

### UUIDs BLE (référence absolue)

**Service AT-CORE :** `4FAFC201-1FB5-459E-8FCC-C5C9C331914B`

| Caractéristique | UUID | Direction | Fréquence |
|---|---|---|---|
| STATUS  | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | AT-CORE → AT-VIEW | 1s |
| DEBUG   | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | AT-CORE → AT-VIEW | 1s |
| FLIGHT  | `6E400004-B5A3-F393-E0A9-E50E24DCCA9E` | AT-CORE → AT-VIEW | 1s |
| TRAFFIC | `6E400005-B5A3-F393-E0A9-E50E24DCCA9E` | AT-CORE → AT-VIEW | 2-5s |
| ALERTS  | `6E400006-B5A3-F393-E0A9-E50E24DCCA9E` | AT-CORE → AT-VIEW | sur changement |

**MTU négocié :** 512 bytes → payload max 509 bytes par notification

---

## Formats JSON BLE — Ce qu'AT-VIEW attend

### STATUS (notify 1s)
```json
{
  "mode": 0,
  "gps_fix": true,
  "gps_sat": 8,
  "csq": 15,
  "sd_ok": true,
  "frames": 120,
  "lat": 50.1234,
  "lon": 4.5678,
  "alt": 500,
  "spd": 85,
  "hdg": 270,
  "flarm": false,
  "adsb": false,
  "chg": false,
  "bat": 85
}
```
- `chg`: **false** = en charge (USB/chargeur), **true** = sur batterie seule
- `bat`: pourcentage 0-100, -1 si inconnu
- `spd`: vitesse sol en km/h
- `hdg`: cap GPS 0-359°
- `alt`: altitude MSL en mètres

### FLIGHT (notify 1s)
```json
{
  "gf": 1.02,
  "co": 12,
  "rpm": 2200,
  "phase": 1
}
```
- `gf`: facteur de charge vertical (G-force axe Z)
- `co`: CO en ppm
- `rpm`: tours/minute moteur
- `phase`: 0=sol, 1=vol

### TRAFFIC (notify 2-5s)
```json
{
  "count": 2,
  "t": [
    {"cs":"OO-ABC","d":2500,"a":650,"b":135,"c":200,"w":150,"s":95,"v":true,"tp":8},
    {"cs":"PH-XYZ","d":4800,"a":320,"b":270,"c":90,"w":-200,"s":0,"v":false,"tp":0}
  ]
}
```

| Champ | Signification | Unité |
|---|---|---|
| `cs` | Callsign / immatriculation | string max 8 chars |
| `d` | Distance à l'avion propre | mètres |
| `a` | Altitude MSL | mètres |
| `b` | Bearing absolu Nord (calculé côté AT-CORE avec haversine) | degrés 0-359 |
| `c` | Cap propre du trafic — **utilisé pour dead reckoning ET rotation icône** | degrés 0-359 |
| `w` | Taux de montée/descente (nouveau champ, ignoré par AT-VIEW jusqu'à implémentation) | feet/min |
| `s` | Vitesse sol | knots |
| `v` | Visible ADS-B (sinon FLARM/SafeSky) | bool |
| `tp` | Type d'aéronef SafeSky (0-17) | int → sélection icône |

**Types `tp` SafeSky (utilisés pour icônes) :**
0=unknown, 1=glider, 2=tow, 3=helicopter, 4=parachute, 5=drop plane, 6=hang glider,
7=para glider, 8=powered aircraft, 9=jet, 10=UFO, 11=balloon, 12=airship, 13=UAV,
14=ground vehicle, 15=static obstacle, 16=para motor, 17=military

**Max 5 entrées** (`MAX_TRF = 5`). Envoyer les 5 plus proches.

### ALERTS (notify sur changement)
```json
{
  "co": false,
  "gf": false,
  "rpm": false,
  "tfc": true,
  "msg": "TRAFIC PROCHE"
}
```
- Envoyer dès qu'une condition change (pas en polling fixe)
- `tfc: false` + `msg: ""` = fin alerte → AT-VIEW retourne page précédente

### DEBUG (notify 1s)
```json
{
  "hb_gps": 2,
  "hb_lte": 3,
  "hb_sd": 1,
  "csq": 15,
  "http_ms": 850,
  "code": 200,
  "lte_ok": true,
  "dis_lte": false,
  "ss_ago": 4,
  "fa_ago": 4,
  "heap": 180000,
  "bat_pct": -1,
  "mode": 0,
  "pending": 0,
  "ftx": 0,
  "frx": 0,
  "adsb_rx": 0,
  "fid": "ATC12345"
}
```

---

## Identité aéronef — Saisie côté AT-VIEW

AT-VIEW permet la saisie locale via page **Aircraft** (overlay depuis Settings 2/2) :

| Champ | Variable | NVS namespace/clé | Format | Exemple |
|---|---|---|---|---|
| Immatriculation | `g_ac_reg[8]` | `aircraft/reg` | Alphanum sans tiret, max 7 chars | `FJFVB` |
| Type OACI | `g_ac_type[8]` | `aircraft/type` | Code OACI 2-4 chars | `VL3`, `MCR1`, `FK9` |
| Hex transpondeur | `g_ac_hex[7]` | `aircraft/hex24` | 6 digits hexadécimaux | `38EDC5` |

**Interface saisie :**
- Onglet IMMAT : clavier alphanumérique LVGL, caractère par caractère
- Onglet TYPE : roller sur 193 codes OACI triés alphabétiquement + clavier A-Z recherche préfixe
- Onglet HEX : clavier hexadécimal 0-9/A-F

**Ces données sont pour l'instant locales à AT-VIEW.** 
→ À terme, AT-CORE doit être la source de vérité et envoyer ces champs via BLE (caractéristique STATUS ou dédiée).

---

## Settings — Paramètres configurables

### Sous-page 1/2 — RADAR + DISPLAY
| Paramètre | NVS clé | Défaut | Plage |
|---|---|---|---|
| Scale (échelle radar) | `cfg/scale` | 4 nm | 1/2/4/8/16/32 nm |
| VFilt (filtre vertical) | `cfg/vfilt` | 2000 ft | ±500 → ±5000 ft |
| Dist (unité distance) | `cfg/dist_nm` | NM | NM / KM |
| Alt (unité altitude) | `cfg/alt_ft` | FT | FT / M |
| Brightness | `cfg/bright` | 16 | 1-16 |
| Theme | `cfg/dark` | Dark | Dark / Light |

### Sous-page 2/2 — TRAFFIC + AIRCRAFT + SYSTEM
| Paramètre | NVS clé | Défaut | Options |
|---|---|---|---|
| Source trafic | `cfg/trf_src` | ALL | SSKY / FLRM / ADSB / ALL |
| Ground (filtre sol) | `cfg/show_grnd` | ON | ON (affiche) / OFF (masque <20kt) |
| Icons (taille) | `cfg/icon_sz` | M | S / M / L |
| AIP overlay | `cfg/aip_en` | ON | ON / OFF |
| Heliports | `cfg/ad_heli` | OFF | ON (affiche types 7+10) / OFF (masque) |
| WiFi AP | `cfg/wifi_en` | OFF | ON / OFF (activer = SSID visible) |
| Aircraft | — | — | → overlay saisie identité |
| SD card | — | — | taille en GB affichée |
| Version | — | — | long press → page Debug |

---

## Overlay AIP — Carte sur radar

Données stockées sur SD (`/sdcard/aip/`), chargées en PSRAM au boot :

| Fichier | Format | Contenu |
|---|---|---|
| `france_ctr.bin` | CTR\0 + polygones | Zones CTR (bleu) + ATZ (gris clair) France |
| `belgium_ctr.bin` | CTR\0 + polygones | Zones CTR/ATZ Belgique |
| `aerodromes.bin` | ADP2 + points | ~3500 aérodromes Europe avec type_id |

**Affichage :**
- CTR : contour gris `0x9ca3af` — clippé au cercle radar RAD_R=175px
- ATZ : contour gris pâle `0xb0bcc8`
- Aérodromes : point ambre 4×4px (heliports/hydrobases masqués si toggle OFF)

**Mise à jour fichiers :** WiFi AP AT-VIEW → Safari → `http://atview.local` ou `http://192.168.4.1`

---

## NVS — Namespaces utilisés par AT-VIEW

| Namespace | Clés | Usage |
|---|---|---|
| `cfg` | scale, bright, trf_src, dist_nm, alt_ft, dark, show_grnd, wifi_en, aip_en, ad_heli, icon_sz | Configuration radar/display |
| `unit` | name, paired_mac, wifi_pass | Identité BLE + pairing + WiFi |
| `aircraft` | reg, type, hex24 | Identité aéronef |
| `auth` | owner | Code propriétaire (auth phase 2) |

---

## Alertes — Logique AT-VIEW

| Condition | Déclencheur | Action AT-VIEW |
|---|---|---|
| CO ≥ 35 ppm | `alert.co = true` | Passage page radar + indicateur rouge CO |
| Trafic < 500m | `alert.tfc = true` | Passage page radar + indicateur orange trafic |
| Fin alerte | `tfc/co = false` + `msg = ""` | Retour page précédente automatique |

---

## Ce qu'AT-CORE doit implémenter (backlog)

1. **UUIDs corrects** : FLIGHT = `6E400004`, TRAFFIC = `6E400005`, ALERTS = `6E400006`
2. **Champ `chg`** dans STATUS : `false` = charge secteur, `true` = batterie seule
3. **Champ `hdg`** dans STATUS : cap GPS 0-359° (AT-VIEW l'utilise pour heading-up radar)
4. **Champ `s`** dans TRAFFIC entries : vitesse sol en knots (défaut 100 si absent → filtre ground ne fonctionne pas)
5. **Champ `b`** dans TRAFFIC entries : bearing absolu calculé côté AT-CORE (haversine depuis position GPS)
6. **Notify FLIGHT** avec gf/co/rpm/phase
7. **Notify TRAFFIC** avec les 5 aéronefs les plus proches
8. **Notify ALERTS** sur changement d'état
9. **Nom BLE** : adopter format `ATCORE-<OACI><N>-<SEQ>` (ex: `ATCORE-EBBY1-01`)

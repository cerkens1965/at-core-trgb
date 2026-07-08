# AeroTrace — Simulateur d'alertes trafic (`altsim/`)

Banc de validation des **alertes trafic** sans vol réel : on fabrique des situations de
conflit à la demande et on vérifie que l'alerte se déclenche **au bon moment, sur le bon
avion, sans faux positif**. Sert aussi à **régler les seuils**.

## Ouvrir
Double-clic sur **`alertsim.html`** (aucune installation, aucun serveur, 100 % offline).
Cible écran simulée = **WS-241 rev2.0** (600 × 450).

## Les 3 vues
- **Live map (plan)** : espaces AIP (CTR/TMA/R-D-P/ATZ) + aérodromes + trafic (icônes avion).
- **Coupe altitude** (vue de côté) : altitude de chaque avion vs distance ; la bande colorée
  = la fenêtre verticale d'alerte → on voit *co-altitude vs étagé*.
- **Écran avion 600 × 450** : radar own-centré track-up + overlay alerte (flash ×1,5 en rouge).

Sliders **vitesse/altitude** de *own* et *intrus*, sliders **seuils** (effet immédiat),
audio, scénarios scriptés + **2 CSV réels croisés**. Panneau **diagnostic** = closing, tCPA,
dCPA, Δalt, règle déclenchée et **marge à chaque seuil**.

---

## 📋 Les règles d'alerte (à ce jour)

**Philosophie** : on alerte **seulement** sur un avion qui est **à ta hauteur, sur ta route,
et qui se rapproche**. Deux niveaux : **🟠 ORANGE** (attention) · **🔴 ROUGE** (urgent, flash + son).

1. **Au sol → rien.** Tant que non en vol (`flt_st==0`), aucune alerte.
2. **Trop haut/bas → ignoré.** Au-delà de ~±400-500 ft d'écart vertical, pas d'alerte
   (il faut être *co-altitude*).
3. **Il faut se RAPPROCHER.** Si l'écart ne diminue pas (avion qui s'éloigne, ou **qui suit
   à la même vitesse**) → **jamais d'alerte**. (Deux avions en file ne s'alarment pas.)
4. **Si co-altitude ET on se rapproche**, deux déclencheurs :
   - 🥚 **La bulle** : l'intrus entre dans une zone en forme d'œuf autour de toi — **grande
     devant** (≈ ce que tu parcours en 30 s), **petite derrière** → 🟠.
   - 🎯 **La trajectoire (CPA)** : vous allez vous croiser **près** (< 0,5 nm) et **bientôt**
     (< 25 s) → 🔴 (plus large : 1 nm / 45 s → 🟠).
5. **Garde-fou : trop près (< 150 m) + même altitude → 🔴** automatiquement.

### Seuils actuels (défauts — réglables dans le simu)

| | 🟠 ORANGE | 🔴 ROUGE |
|---|---|---|
| Écart vertical max | 500 ft | 400 ft |
| Temps avant croisement (tCPA) | 45 s | 25 s |
| Distance de passage (dCPA) | 1 nm | 0,5 nm |
| Bulle avant | vitesse × 30 s (borné 400–2000 m) | — |
| Bulle arrière | 20 % de l'avant (min 150 m) | — |
| Plancher | — | 150 m |

---

## Architecture (source de vérité unique)

| Fichier | Rôle |
|---|---|
| `../examples/at_core_debug/alert_core.h` | **Moteur d'alerte** (C, pur). Le firmware AT-VIEW le délègue. |
| `alert_core.js` | **Miroir JS** du `.h` (moteur du simu). |
| `conformance/` | `gen_golden.cpp` (compile le `.h`) + `check.js` → **5000/5000 cas identiques**. |
| `alertsim.html` | Le simulateur (vues, sliders, diagnostic, audio, CSV). |
| `aip_ebby.js` | AIP OpenAIP région EBBY, embarqué (offline). |

➡️ **Toute modif de la LOGIQUE d'alerte se fait dans `alert_core.h`**, puis :
1. reporter la même modif dans `alert_core.js` ;
2. relancer la conformance :
   ```bash
   cd conformance
   clang++ -std=c++17 -O2 gen_golden.cpp -o gen_golden && ./gen_golden > golden.txt
   node check.js golden.txt      # doit afficher 0 divergence
   ```
➡️ **Régler les seuils** : bouge les sliders dans le simu ; quand tu es content, reporte les
valeurs dans `acDefaultParams()` (dans `alert_core.h` ET `alert_core.js`) → firmware + simu alignés.

## Régénérer l'AIP (`aip_ebby.js`)
```bash
KEY=<clé OpenAIP>
curl -s "https://api.core.openaip.net/api/airspaces?bbox=3.4,49.9,5.8,51.4&limit=1000" -H "x-openaip-api-key: $KEY" -o /tmp/aip_as.json
curl -s "https://api.core.openaip.net/api/airports?bbox=3.4,49.9,5.8,51.4&limit=1000"  -H "x-openaip-api-key: $KEY" -o /tmp/aip_ad.json
# puis compacter en window.AIP = {origin,airspaces,airports} (cf. historique git de aip_ebby.js)
```

## CSV
Format `ATCORE_*.csv` (Garmin G3X). Le simu lit Lat/Lon/AltGPS/GndSpd/TRK + **UTC Time**
pour rejouer à la **cadence réelle**. ⚠️ nos CSV ne contiennent **que notre trajectoire**
(pas le trafic reçu) → l'intrus est soit synthétique, soit un 2ᵉ CSV.

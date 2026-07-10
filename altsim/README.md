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

1. **Moi au sol → rien.** Tant que non en vol (`flt_st==0`), aucune alerte.
1bis. **Intrus au sol → rien.** Un intrus dont la vitesse-sol < 30 kt (parqué/roulage) ne peut
   **jamais** déclencher d'alerte sur moi — symétrique de la règle 1 (SafeSky diffuse aussi le
   trafic au sol). *(Défaut à 100 kt si le beacon n'a pas de vitesse → jamais gaté par erreur.)*
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
| Intrus au sol (gate) | vitesse-sol intrus < 30 kt → ignoré | idem |

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

---

## Vision 2022 (SafeSky UI Road Map) ↔ implémentation 2026
Le modèle descend de la road map SafeSky d'oct. 2022
(`01 - Documentation/RoadMap-V3.0 Oct-2022.key.pdf`). **Fidèle** sur : alertes par **TEMPS**
(tCPA), **convergence** obligatoire, **bulle de protection**, **bandes verticales**, couleur par
conflit, AIP + aérodromes. Les temps **TA = 45 s / RA = 25 s sont IDENTIQUES** à nos ORANGE/ROUGE.

**Évolutions 2026 assumées** :
- **2 niveaux** (ORANGE/ROUGE) au lieu de **4** (No-Threat / PA-info / TA / RA) → l'étage « info » a été retiré.
- **CPA en mouvement relatif** `−(r·v)/|v|²` au lieu du « temps le plus lent des deux avions » (plus rigoureux).
- **Bulle décalée vers l'avant** (œuf) au lieu d'une zone symétrique → un suiveur ne déclenche pas.
- **Fenêtres verticales plus serrées** : 500/400 ft (vs 850/600 ft en 2022).

### Features EXPÉRIMENTALES du simu (hors core/firmware — à évaluer, puis promouvoir si OK)
- 🔵 **Niveau « info » 90 s** : reprend l'étage **PA** de 2022 (convergence, 45 s < tCPA ≤ 90 s, ±1200 ft).
  **Affichage SIMU uniquement** — `acEvalThreats` reste 0/1/2 ; c'est `infoTier()`/`curInfo` côté HTML.
- 🔍 **Auto-zoom (« Dynamic View »)** : l'échelle radar s'ajuste au trafic le plus proche (2/4/6/8/10/16 nm).
- 〜 **Trace (sillage)** : ~90 s de passé sur la live map (utile en rejeu CSV).

➡️ Si l'un de ces trois est validé, il faut le **promouvoir** : le niveau info dans `alert_core.h`
(+ `.js` + conformance + payload BLE + rendu écran firmware) ; l'auto-zoom/trace côté firmware écran.

### Améliorations du MODÈLE (togglables — critique 2026-07-09)
Faiblesses réelles du modèle actuel, à évaluer dans le simu avant promotion au core :
- 🧪 **CPA 3D** (`e3d`) : fenêtre verticale testée sur le **Δalt PROJETÉ au tCPA** (fermeture
  verticale) → un intrus qui **monte/descend** vers ton niveau alerte **tôt** (aujourd'hui :
  seulement en entrant dans la bande, ~12 s avant). **Identique au core en palier** (vspd=0).
  ⚠️ nécessite la vspd de l'intrus (SafeSky la donne ; à propager dans `TrafficEntry` écran pour la promo).
- 🧪 **Hystérésis** (`ehy`) : maintien 3-4 s → fin du **clignotement** au ras des seuils.
- 🧪 **Bruit** (`enoise`) : perturbe ce que voit le moteur (±40 m, ±80 ft, ±1-6°) → montre pourquoi
  l'hystérésis est nécessaire sur données réelles.

**⭐ Rejeu de VRAIS conflits — FAIT (firmware ATC v61)** : le boîtier loggue la table trafic IN
dans `/ATCORE_TRF_<fid>.csv` (own + intrus, en vol, 1 ligne/intrus/2 s). Charge-le dans le simu via
**« Rejeu trafic RÉEL (boîtier) »** → on rejoue une **vraie rencontre** (own + trafic reçu) au lieu
d'inventer. C'était le point le plus important pour une validation crédible. ⚠️ nécessite de **flasher
un boîtier en v61** puis récupérer le fichier après un vol.

Autres pistes (pas codées) : gate dCPA sur la bulle · dégrader sur beacon vieux · de-weight du CPA
quand own tourne · scénario **circuit avec virages**.

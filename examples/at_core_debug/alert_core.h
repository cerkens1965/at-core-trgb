// ============================================================================
//  alert_core.h — MOTEUR D'ALERTE TRAFIC (source de vérité PARTAGÉE)
// ----------------------------------------------------------------------------
//  Modèle « bulle-œuf + convergence » (2026-07-08) : universel (pas de mode
//  circuit), gate sol (flt_st==0), CPA (tcpa/dcpa) + bulle décalée vers l'avant
//  dont le lobe avant varie avec la vitesse. Deux niveaux : ORANGE / RED.
//
//  Ce fichier est PUR (aucune dépendance LVGL/Arduino/globals) → il est inclus
//  À L'IDENTIQUE par :
//    - le firmware AT-VIEW  (examples/at_core_debug/at_core_debug.ino)
//    - le simulateur host   (altsim/ : conformance C++ + miroir JS vérifié)
//  ⇒ zéro divergence entre ce que le firmware calcule et ce que le simu montre.
//
//  ⚠️ Toute modif de la LOGIQUE d'alerte se fait ICI, puis on re-génère le
//     golden de conformance (altsim/) pour re-verrouiller le miroir JS.
// ============================================================================
#pragma once
#include <math.h>
#include <stdint.h>

#ifndef AC_PI
#define AC_PI 3.14159265358979323846f
#endif

enum { AC_NONE = 0, AC_ORANGE = 1, AC_RED = 2 };

// Notre avion. spd_kmh = vitesse sol en km/h (comme g_status.spd), hdg en degrés.
// flt_st : 0=sol · 1=en vol · 2=arrêt imminent (mode GND = 0 → aucune alerte).
struct AC_Own {
    float   spd_kmh;
    float   hdg_deg;
    uint8_t flt_st;
    bool    gps_fix;
    bool    valid;
};

// Un intrus (aligné TrafficEntry de l'écran).
//   alt_rel_100ft : altitude RELATIVE en CENTAINES de pieds (= TrafficEntry.alt_m ; ×100 → ft)
//   bear_deg      : relèvement ABSOLU depuis nous (0-359)
//   dist_m        : distance horizontale (m)
//   hdg_deg       : cap propre de l'intrus
//   spd_kt        : vitesse sol de l'intrus (nœuds)
struct AC_Intruder {
    bool  visible;
    float alt_rel_100ft;
    float bear_deg;
    float dist_m;
    float hdg_deg;
    float spd_kt;
};

// Seuils réglables (défauts = comportement firmware). Exposés pour les sliders du simu.
struct AC_Params {
    float fwd_sec;   // lobe avant bulle = vitesse_sol(m/s) × fwd_sec
    float fwd_min;   // clamp bas lobe avant (m)
    float fwd_max;   // clamp haut (m)
    float aft_min;   // lobe arrière mini (m)
    float floor_m;   // plancher co-altitude (m) → RED quoi qu'il arrive
    float vOrg, vRed;   // gate vertical (ft) orange / rouge
    float tOrg, tRed;   // tCPA (s) orange / rouge
    float dOrg, dRed;   // dCPA distance de passage (m) orange / rouge
    float gnd_kt;       // plancher vitesse-sol intrus : sous ce seuil = AU SOL/roulage → jamais d'alerte
    float tImm;         // (v181) tau (s) sous lequel un RED devient « IMMINENT » (restitution renforcée)
};

// Résultat global (menace la plus grave retenue).
struct AC_Out {
    uint8_t level;      // AC_NONE / AC_ORANGE / AC_RED
    int     clock;      // position horaire relative au nez (1..12)
    int     dist_m;
    int     dalt_ft;
    int     closing_kt;
    int     idx;        // index de l'intrus retenu (-1 si aucun) → callsign côté appelant
    bool    valid;
    bool    imminent;   // (v181) RED + convergence rapide (tau ≤ tImm ou plancher) → restitution renforcée
    int     tcpa_s;     // (v181) tCPA de l'intrus retenu (s ; -1 si divergent) → info restitution
};

// Défauts = valeurs figées historiquement dans alertEngineTick (comportement inchangé).
static inline AC_Params acDefaultParams() {
    AC_Params p;
    p.fwd_sec = 30.0f; p.fwd_min = 400.0f; p.fwd_max = 2000.0f;
    p.aft_min = 150.0f; p.floor_m = 150.0f;
    const float NM = 1852.0f;
    p.vOrg = 500.0f; p.vRed = 400.0f;
    p.tOrg = 45.0f;  p.tRed = 25.0f;
    p.dOrg = 0.6f * NM; p.dRed = 0.5f * NM;   // dCPA orange resserré 1→0,6 nm (densité circuit)
    p.gnd_kt = 30.0f;
    p.tImm = 12.0f;   // (v181) RED avec tau ≤ 12 s = IMMINENT (restitution renforcée écran)
    return p;
}

// Évalue toutes les menaces. Remplit per[i] (niveau par intrus, index aligné sur t[])
// et out (menace la plus grave). Retourne out.level. GATE : sol (flt_st==0) ou pas de
// fix/statut invalide → tout à AC_NONE.
static inline uint8_t acEvalThreats(const AC_Own& o, const AC_Intruder* t, int n,
                                    const AC_Params& P, uint8_t* per, AC_Out& out) {
    out.level = AC_NONE; out.clock = 0; out.dist_m = 0; out.dalt_ft = 0;
    out.closing_kt = 0; out.idx = -1; out.valid = false; out.imminent = false; out.tcpa_s = -1;
    for (int i = 0; i < n; i++) per[i] = AC_NONE;

    if (!o.valid || !o.gps_fix) return AC_NONE;
    if (o.flt_st == 0) return AC_NONE;              // mode GND : aucune alerte au sol

    const float ogs = o.spd_kmh / 3.6f;             // own m/s
    float rFwd = ogs * P.fwd_sec;
    if (rFwd < P.fwd_min) rFwd = P.fwd_min;
    if (rFwd > P.fwd_max) rFwd = P.fwd_max;
    float rAft = rFwd * 0.20f;
    if (rAft < P.aft_min) rAft = P.aft_min;

    const float ot  = o.hdg_deg * AC_PI / 180.0f;
    const float ovx = ogs * sinf(ot), ovy = ogs * cosf(ot);

    for (int i = 0; i < n; i++) {
        const AC_Intruder& e = t[i];
        if (!e.visible) continue;
        if (e.spd_kt < P.gnd_kt) continue;              // intrus au sol/roulage → jamais d'alerte
        const float dalt = fabsf(e.alt_rel_100ft * 100.0f);          // |Δalt| ft
        const float br = e.bear_deg * AC_PI / 180.0f;
        const float Rx = e.dist_m * sinf(br), Ry = e.dist_m * cosf(br);
        const float tt = e.hdg_deg * AC_PI / 180.0f, tgs = e.spd_kt * 0.514444f;
        const float vrx = tgs * sinf(tt) - ovx, vry = tgs * cosf(tt) - ovy;
        const float vr2 = vrx * vrx + vry * vry;
        const float tcpa = (vr2 > 0.05f) ? -(Rx * vrx + Ry * vry) / vr2 : -1.0f;
        float dcpa = e.dist_m;
        if (tcpa > 0) { float cx = Rx + vrx * tcpa, cy = Ry + vry * tcpa; dcpa = sqrtf(cx * cx + cy * cy); }
        const int  closing = (int)(sqrtf(vr2) / 0.514444f);
        // TAU (2026-07-13) : rapprochement RADIAL signé (>0 = on se rapproche) → temps = dist / rapprochement.
        // Remplace le converging = (tcpa>0) purement géométrique : un trafic qui passe ABEAM ou s'éloigne a un
        // rapprochement radial ~0 → tau ∞ → PLUS d'alerte (se relâche au passage = circuit/false-alert apaisé).
        // dcpa reste calculé sur la géométrie CPA ; seul le TEMPS et le « converge » passent en radial.
        const float closeRate  = -(Rx * vrx + Ry * vry) / e.dist_m;
        const float tau        = (closeRate > 0.1f) ? e.dist_m / closeRate : 1e9f;
        const bool  converging = (closeRate > 0.1f);
        const float T          = tau;

        const float rel = e.bear_deg - o.hdg_deg;
        const float ceg = cosf(rel * AC_PI / 180.0f);
        const float rEgg = rAft + (rFwd - rAft) * (1.0f + ceg) * 0.5f;

        uint8_t lvl = AC_NONE;
        if (dalt <= P.vOrg && converging && T <= P.tOrg && dcpa <= P.dOrg) lvl = AC_ORANGE;
        if (dalt <= P.vRed && converging && T <= P.tRed && dcpa <= P.dRed) lvl = AC_RED;
        if (dalt <= P.vOrg && converging && e.dist_m <= rEgg && T <= P.tOrg && lvl < AC_ORANGE) lvl = AC_ORANGE;  // (TAU) la bulle respecte aussi le temps
        // (v181 — P1) PLANCHER co-altitude : n'arme RED que si on CONVERGE réellement. Avant, la seule
        // proximité (<150 m) suffisait → 2 avions en FORMATION/tour de piste (parallèles, séparation STABLE,
        // rapprochement radial ~0) déclenchaient RED en continu (fausse alerte : 215 s sur le vol 21/07).
        // Un vrai danger co-altitude à <150 m converge forcément → couvert ; la formation (closeRate~0) non.
        if (dalt <= P.vRed && converging && e.dist_m <= P.floor_m) lvl = AC_RED;

        per[i] = lvl;
        if (lvl > out.level) {
            int clk = ((int)(e.bear_deg - o.hdg_deg) % 360 + 360) % 360;
            int hr = ((clk + 15) / 30) % 12; if (hr == 0) hr = 12;
            out.level = lvl; out.clock = hr; out.dist_m = (int)e.dist_m;
            out.dalt_ft = (int)(e.alt_rel_100ft * 100.0f); out.closing_kt = closing;
            out.idx = i; out.valid = true;
            out.tcpa_s = (tcpa > 0) ? (int)tcpa : -1;
            // (v181 — P3-lite) IMMINENT : RED qui converge vite (tau ≤ tImm) ou déjà au plancher → l'écran
            // renforce la restitution (flash plus rapide + libellé) sans ajouter un 3e niveau de menace.
            out.imminent = (lvl == AC_RED) && converging && (T <= P.tImm || e.dist_m <= P.floor_m);
        }
    }
    return out.level;
}

// ── Hystérésis temporelle (post-filtre du niveau de sortie) ───────────────────
// Monte IMMÉDIATEMENT vers un niveau supérieur ; ne REDESCEND qu'après un maintien
// (ROUGE 4 s / ORANGE 3 s) → l'alerte ne clignote pas quand la géométrie oscille au ras
// d'un seuil. STATEFUL → l'appelant garde un AC_Hyst persistant (pas dans acEvalThreats,
// qui reste pur/conforme). Miroir JS = acHysteresis() dans alert_core.js.
struct AC_Hyst { uint8_t level; uint32_t until_ms; };
static inline uint8_t acHysteresis(uint8_t raw, uint32_t now_ms, AC_Hyst* h) {
    if (raw > h->level) { h->level = raw; h->until_ms = now_ms + (raw == AC_RED ? 4000u : 3000u); }
    else if (raw < h->level && now_ms >= h->until_ms) { h->level = raw; }
    return h->level;
}

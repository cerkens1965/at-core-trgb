// ============================================================================
//  alert_core.js — MIROIR JS de alert_core.h (source de vérité C++).
//  Mêmes formules, même ordre. Verrouillé par altsim/conformance/ (le golden
//  vient du C++ ; check.js prouve que CE fichier donne les mêmes sorties).
//  ⚠️ Toute modif ici DOIT refléter alert_core.h — puis relancer la conformance.
//  Utilisé tel quel par alertsim.html (browser) ET check.js (node).
// ============================================================================
(function (root) {
  const AC = { NONE: 0, ORANGE: 1, RED: 2 };
  const NM = 1852.0;

  function acDefaultParams() {
    return {
      fwd_sec: 30, fwd_min: 400, fwd_max: 2000, aft_min: 150, floor_m: 150,
      vOrg: 500, vRed: 400, tOrg: 45, tRed: 25, dOrg: 0.6 * NM, dRed: 0.5 * NM,
      gnd_kt: 30, tImm: 12
    };
  }

  // own:{spd_kmh,hdg_deg,flt_st,gps_fix,valid}
  // t: [{visible,alt_rel_100ft,bear_deg,dist_m,hdg_deg,spd_kt}]
  function acEvalThreats(o, t, P) {
    const per = t.map(() => AC.NONE);
    const out = { level: AC.NONE, clock: 0, dist_m: 0, dalt_ft: 0, closing_kt: 0, idx: -1, valid: false, imminent: false, tcpa_s: -1 };
    if (!o.valid || !o.gps_fix) return { per, out };
    if (o.flt_st === 0) return { per, out };
    const D = Math.PI / 180;
    const ogs = o.spd_kmh / 3.6;
    let rFwd = ogs * P.fwd_sec; if (rFwd < P.fwd_min) rFwd = P.fwd_min; if (rFwd > P.fwd_max) rFwd = P.fwd_max;
    let rAft = rFwd * 0.20; if (rAft < P.aft_min) rAft = P.aft_min;
    const ot = o.hdg_deg * D, ovx = ogs * Math.sin(ot), ovy = ogs * Math.cos(ot);
    for (let i = 0; i < t.length; i++) {
      const e = t[i]; if (!e.visible) continue;
      if (e.spd_kt < P.gnd_kt) continue;              // intrus au sol/roulage → jamais d'alerte
      const dalt = Math.abs(e.alt_rel_100ft * 100);
      const br = e.bear_deg * D, Rx = e.dist_m * Math.sin(br), Ry = e.dist_m * Math.cos(br);
      const tt = e.hdg_deg * D, tgs = e.spd_kt * 0.514444;
      const vrx = tgs * Math.sin(tt) - ovx, vry = tgs * Math.cos(tt) - ovy;
      const vr2 = vrx * vrx + vry * vry;
      const tcpa = (vr2 > 0.05) ? -(Rx * vrx + Ry * vry) / vr2 : -1;
      let dcpa = e.dist_m; if (tcpa > 0) { const cx = Rx + vrx * tcpa, cy = Ry + vry * tcpa; dcpa = Math.hypot(cx, cy); }
      const closing = Math.trunc(Math.sqrt(vr2) / 0.514444);
      // TAU (2026-07-13) : rapprochement RADIAL signé → temps = dist/rapprochement ; abeam/éloigne → tau ∞ → pas d'alerte.
      const closeRate = -(Rx * vrx + Ry * vry) / e.dist_m;
      const tau = (closeRate > 0.1) ? e.dist_m / closeRate : 1e9;
      const converging = (closeRate > 0.1);
      const T = tau;
      const rel = e.bear_deg - o.hdg_deg, ceg = Math.cos(rel * D);
      const rEgg = rAft + (rFwd - rAft) * (1 + ceg) * 0.5;
      let lvl = AC.NONE;
      if (dalt <= P.vOrg && converging && T <= P.tOrg && dcpa <= P.dOrg) lvl = AC.ORANGE;
      if (dalt <= P.vRed && converging && T <= P.tRed && dcpa <= P.dRed) lvl = AC.RED;
      if (dalt <= P.vOrg && converging && e.dist_m <= rEgg && T <= P.tOrg && lvl < AC.ORANGE) lvl = AC.ORANGE;
      // (v181 — P1) plancher co-altitude : RED seulement si on CONVERGE (formation/circuit parallèle non-convergent = plus de fausse alerte).
      if (dalt <= P.vRed && converging && e.dist_m <= P.floor_m) lvl = AC.RED;
      per[i] = lvl;
      e._tcpa = tcpa; e._dcpa = dcpa;
      if (lvl > out.level) {
        let clk = (((Math.trunc(e.bear_deg - o.hdg_deg)) % 360) + 360) % 360;
        let hr = Math.trunc((clk + 15) / 30) % 12; if (hr === 0) hr = 12;
        out.level = lvl; out.clock = hr; out.dist_m = Math.trunc(e.dist_m);
        out.dalt_ft = Math.trunc(e.alt_rel_100ft * 100); out.closing_kt = closing; out.idx = i; out.valid = true;
        out.tcpa_s = (tcpa > 0) ? Math.trunc(tcpa) : -1;
        // (v181 — P3-lite) IMMINENT : RED convergeant vite (tau ≤ tImm) ou au plancher → restitution renforcée écran.
        out.imminent = (lvl === AC.RED) && converging && (T <= P.tImm || e.dist_m <= P.floor_m);
      }
    }
    return { per, out };
  }

  // Hystérésis temporelle (post-filtre du niveau) — miroir de acHysteresis() dans alert_core.h.
  // Monte direct, redescend après maintien (ROUGE 4 s / autre 3 s). st = {level, until} persistant.
  function acHysteresis(raw, nowMs, st) {
    if (raw > st.level) { st.level = raw; st.until = nowMs + (raw === AC.RED ? 4000 : 3000); }
    else if (raw < st.level && nowMs >= st.until) { st.level = raw; }
    return st.level;
  }

  const api = { AC, NM, acDefaultParams, acEvalThreats, acHysteresis };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;   // node
  else { root.AlertCore = api; root.AC = AC; root.NM = NM; root.acDefaultParams = acDefaultParams; root.acEvalThreats = acEvalThreats; root.acHysteresis = acHysteresis; }
})(typeof window !== 'undefined' ? window : globalThis);

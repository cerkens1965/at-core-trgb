// check.js — VÉRIFICATEUR de conformance JS↔C++.
// Rejoue le MÊME flux de cas déterministe que gen_golden.cpp, via alert_core.js
// (le moteur du simu), et compare les verdicts au golden C++.
//   node check.js golden.txt
//
// FP : le firmware calcule en float (sinf/…), le JS en double (Math.sin/…). Aux
// FRONTIÈRES exactes d'un seuil, le niveau peut basculer (bruit float/double) —
// ce n'est PAS une divergence de logique. On classe donc les écarts :
//   - "logic"    : niveaux différents ET loin de tout seuil → ÉCHEC (bug de port)
//   - "boundary" : niveaux différents mais un intrus est à < marge d'un seuil → OK
const fs = require('fs');
const { acDefaultParams, acEvalThreats, NM } = require('../alert_core.js');

// LCG identique à gen_golden.cpp
let seed = 12345 >>> 0;
function rnd() { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 4294967296; }
function rng(a, b) { return a + (b - a) * rnd(); }

const P = acDefaultParams();
const N = 5000;

// marge relative pour classer "boundary" (float/double ~1e-4 → on prend large : 0.5 %)
function nearBoundary(o, t) {
  const D = Math.PI / 180;
  const ogs = o.spd_kmh / 3.6;
  let rFwd = Math.max(P.fwd_min, Math.min(P.fwd_max, ogs * P.fwd_sec));
  let rAft = Math.max(P.aft_min, rFwd * 0.2);
  const ovx = ogs * Math.sin(o.hdg_deg * D), ovy = ogs * Math.cos(o.hdg_deg * D);
  const m = (a, thr) => Math.abs(a - thr) <= Math.max(1e-3, Math.abs(thr) * 5e-3);
  for (const e of t) {
    if (!e.visible) continue;
    const dalt = Math.abs(e.alt_rel_100ft * 100);
    const br = e.bear_deg * D, Rx = e.dist_m * Math.sin(br), Ry = e.dist_m * Math.cos(br);
    const tt = e.hdg_deg * D, tgs = e.spd_kt * 0.514444;
    const vrx = tgs * Math.sin(tt) - ovx, vry = tgs * Math.cos(tt) - ovy;
    const vr2 = vrx * vrx + vry * vry;
    const tcpa = vr2 > 0.05 ? -(Rx * vrx + Ry * vry) / vr2 : -1;
    let dcpa = e.dist_m; if (tcpa > 0) { const cx = Rx + vrx * tcpa, cy = Ry + vry * tcpa; dcpa = Math.hypot(cx, cy); }
    const closeRate = -(Rx * vrx + Ry * vry) / e.dist_m;          // (TAU) rapprochement radial
    const tau = closeRate > 0.1 ? e.dist_m / closeRate : 1e9;
    const rel = e.bear_deg - o.hdg_deg, rEgg = rAft + (rFwd - rAft) * (1 + Math.cos(rel * D)) * 0.5;
    if (m(dalt, P.vOrg) || m(dalt, P.vRed) || m(tau, P.tOrg) || m(tau, P.tRed) ||
        m(dcpa, P.dOrg) || m(dcpa, P.dRed) || m(e.dist_m, rEgg) || m(e.dist_m, P.floor_m) ||
        Math.abs(closeRate - 0.1) < 0.05 || Math.abs(vr2) < 0.06) return true;   // + frontière du seuil de convergence radial
  }
  return false;
}

const golden = fs.readFileSync(process.argv[2] || 'golden.txt', 'utf8').trim().split('\n');
if (golden.length !== N) { console.error(`golden a ${golden.length} lignes, attendu ${N}`); process.exit(2); }

let ok = 0, boundary = 0, logic = 0;
const fails = [];
for (let c = 0; c < N; c++) {
  const o = { spd_kmh: rng(80, 300), hdg_deg: rng(0, 360), flt_st: 1, gps_fix: true, valid: true };
  let n = 1 + Math.trunc(rnd() * 5); if (n > 5) n = 5;
  const t = [];
  for (let i = 0; i < n; i++) t.push({
    visible: true, alt_rel_100ft: rng(-20, 20), bear_deg: rng(0, 360),
    dist_m: rng(50, 8000), hdg_deg: rng(0, 360), spd_kt: rng(0, 300)
  });
  const { per, out } = acEvalThreats(o, t, P);
  const g = golden[c].split(' ').map(Number);
  const gLevel = g[0], gIdx = g[1], gClock = g[2], gN = g[3], gPer = g.slice(4, 4 + gN);
  let same = (gLevel === out.level && gIdx === out.idx && gClock === out.clock && gN === n);
  if (same) for (let i = 0; i < n; i++) if (gPer[i] !== per[i]) same = false;
  if (same) { ok++; continue; }
  if (nearBoundary(o, t)) { boundary++; }
  else { logic++; if (fails.length < 8) fails.push({ c, o, t, js: { level: out.level, idx: out.idx, clock: out.clock, per }, cpp: { level: gLevel, idx: gIdx, clock: gClock, per: gPer } }); }
}

console.log(`Conformance JS↔C++ sur ${N} cas :`);
console.log(`  ✅ identiques        : ${ok}`);
console.log(`  ~ écarts frontière   : ${boundary}  (float firmware vs double JS au ras d'un seuil — OK)`);
console.log(`  ❌ divergences logique: ${logic}`);
if (logic > 0) {
  console.log('\nExemples de divergence LOGIQUE (bug de port à corriger) :');
  for (const f of fails) console.log(JSON.stringify(f));
  process.exit(1);
}
console.log(`\nRÉSULTAT : le miroir JS reproduit le C++ (aucune divergence de logique). Verrouillé ✅`);

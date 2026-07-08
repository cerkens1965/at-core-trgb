// gen_golden.cpp — GÉNÉRATEUR DE RÉFÉRENCE (golden) pour la conformance.
// Compile le VRAI moteur firmware (alert_core.h) et sort le verdict pour un flux
// de cas déterministe (PRNG LCG identique à check.js). check.js rejoue le MÊME
// flux avec alert_core.js et compare → prouve que le miroir JS == le C++.
//
//   clang++ -std=c++17 -O2 gen_golden.cpp -o gen_golden && ./gen_golden > golden.txt
//
#include <cstdio>
#include <cstdint>
#include "../../examples/at_core_debug/alert_core.h"

// LCG 32 bits — DOIT être identique à check.js (Math.imul(s,1664525)+1013904223 >>>0).
static uint32_t g_seed = 12345u;
static inline double rnd() {              // 0..1
    g_seed = (uint32_t)(g_seed * 1664525u + 1013904223u);
    return (double)g_seed / 4294967296.0;
}
static inline double rng(double a, double b) { return a + (b - a) * rnd(); }

int main() {
    const int N = 5000;
    AC_Params P = acDefaultParams();
    for (int c = 0; c < N; c++) {
        AC_Own o;
        o.spd_kmh = (float)rng(80, 300);
        o.hdg_deg = (float)rng(0, 360);
        o.flt_st  = 1; o.gps_fix = true; o.valid = true;
        int n = 1 + (int)(rnd() * 5); if (n > 5) n = 5;
        AC_Intruder t[5];
        for (int i = 0; i < n; i++) {
            t[i].visible       = true;
            t[i].alt_rel_100ft = (float)rng(-20, 20);
            t[i].bear_deg      = (float)rng(0, 360);
            t[i].dist_m        = (float)rng(50, 8000);
            t[i].hdg_deg       = (float)rng(0, 360);
            t[i].spd_kt        = (float)rng(0, 300);
        }
        uint8_t per[5]; AC_Out out;
        acEvalThreats(o, t, n, P, per, out);
        printf("%d %d %d %d", out.level, out.idx, out.clock, n);
        for (int i = 0; i < n; i++) printf(" %d", per[i]);
        printf("\n");
    }
    return 0;
}

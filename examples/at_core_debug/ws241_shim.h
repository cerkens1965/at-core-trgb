#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// AT-VIEW — Shim hardware Waveshare ESP32-S3-Touch-AMOLED-2.41 (BOARD_WS241)
//
// 4ᵉ cible. Dalle AMOLED RM690B0 native **450×600 portrait** → on la pilote en
// **PAYSAGE 600×450** (rotation) pour réutiliser TOUT le layout T4 (BOARD_WS241
// #define BOARD_T4S3 dans le .ino → chemins UI T4 ; PANEL_WS241 override panneau).
//   • écran : Arduino_GFX / Arduino_RM690B0 (QSPI), rotation paysage
//   • touch : SensorLib / TouchDrvFT6X36 (FT6336, I²C @0x38)
//   • rail écran ON au POR (bring-up OK sans init PMIC ETA6098 / expandeur TCA9554)
// Expose la même surface `panel.*` que LilyGo (begin/installSD/setBrightness/
// getTouchModelName/width/height) + beginLvglHelper(panel). Pins/validation : bring-up
// (examples/ws241_bringup), couleurs/tactile validés HW 2026-06-27.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrvFT6X36.hpp>

// ── Pins (cf examples/ws241_bringup/pin_config_ws241.h) ──────────────────────────
#define WS241_LCD_CS    9
#define WS241_LCD_SCK   10
#define WS241_LCD_D0    11
#define WS241_LCD_D1    12
#define WS241_LCD_D2    13
#define WS241_LCD_D3    14
#define WS241_LCD_RST   21
#ifdef PANEL_WS241_SH8601
#define WS241_NATIVE_W 600   // SH8601 : rotation HARDWARE (MADCTL 0x30) → adressage paysage DIRECT 600×450
#define WS241_NATIVE_H 450
#define WS241_ROT      0     // pas de rotation software (la dalle tourne elle-même via MADCTL 0x30)
#define WS241_LCD_H    450
#else
#define WS241_NATIVE_W 480   // RM690B0 (ancien) : 480 (450 → bande noire en bas, cf 2026-06-30)
#define WS241_NATIVE_H 600
#define WS241_ROT      3     // paysage 90° CCW software
#define WS241_LCD_H    480
#endif
#define WS241_LCD_W    600   // logique paysage = ce que voit l'UI T4
#define WS241_MADCTL  0x00

#define WS241_I2C_SDA  47
#define WS241_I2C_SCL  48
#define WS241_TP_RST    3
#define WS241_TP_INT   -1
#define WS241_TP_ADDR 0x38   // FT6336

// ── Objets bas-niveau ───────────────────────────────────────────────────────────
static Arduino_DataBus *ws_bus = new Arduino_ESP32QSPI(
    WS241_LCD_CS, WS241_LCD_SCK, WS241_LCD_D0, WS241_LCD_D1, WS241_LCD_D2, WS241_LCD_D3);
// (bus, rst, rotation, w_natif, h_natif, offsets…) → width()/height() = dims post-rotation.
// ⚠️ Waveshare a changé le contrôleur AMOLED sur le stock récent (RM690B0 → SH8601, constaté
// 2026-07 : dalles neuves noires avec le driver RM690B0, la démo d'usine loggue "sh8601").
// Le flag PANEL_WS241_SH8601 sélectionne le SH8601 (même constructeur, même bus QSPI, mêmes pins).
#ifdef PANEL_WS241_SH8601
// rotation 0 + row_offset1=16 → _yStart=16 → PASET démarre à +16 (= offset Y paysage de la démo
// Waveshare : draw_bitmap(x, y+16)). MADCTL 0x30 (dans begin) fait la rotation HW.
static Arduino_SH8601 *ws_gfx = new Arduino_SH8601(
    ws_bus, WS241_LCD_RST, WS241_ROT, WS241_NATIVE_W, WS241_NATIVE_H, 0, 16, 0, 0);
#else
static Arduino_RM690B0 *ws_gfx = new Arduino_RM690B0(
    ws_bus, WS241_LCD_RST, WS241_ROT, WS241_NATIVE_W, WS241_NATIVE_H, 0, 0, 0, 0);
#endif
static TouchDrvFT6X36 ws_touch;

class WS241_Panel {
public:
    bool begin() {
        Wire.begin(WS241_I2C_SDA, WS241_I2C_SCL);
        if (!ws_gfx->begin()) return false;   // init graphique (RM690B0 par défaut, comportement v90 validé)
        // (2026-07-06) L'amorçage vendeur SH8601 (0xFE/0x26/0x24) a été testé ici (v93) : INUTILE pour
        // les dalles v1.0.0 (marchent sans) et INEFFICACE pour v2.0.1 (valeurs différentes, non
        // disponibles) → retiré pour ne pas risquer les vraies dalles RM690B0. Cf mémoire
        // ws241_sh8601_port_reference. Rev2.0 v2.0.1 = bloqué tant qu'on n'a pas l'init v2.0.1.
#ifdef PANEL_WS241_SH8601
        // Init SH8601 v2.0.1 EXTRAITE du dump usine (ws241_sh8601_reference/WS241_v2.0.1_init_extracted.txt),
        // ordre EXACT, envoyée APRÈS begin() (écrase l'init générique Arduino_GFX). CLÉ vs essai précédent :
        // on ENVOIE bien le MADCTL 0x30 (rotation HARDWARE de la dalle) — l'omettre = noir.
        ws_bus->beginWrite();
        ws_bus->writeC8D8(0xFE, 0x20);      // page CMD2 vendeur
        ws_bus->writeC8D8(0x26, 0x0A);
        ws_bus->writeC8D8(0x24, 0x80);
        ws_bus->writeC8D8(0xFE, 0x00);      // retour page user
        ws_bus->writeC8D8(0x3A, 0x55);      // pixel format RGB565
        ws_bus->writeC8D8(0xC2, 0x00);
        ws_bus->endWrite(); delay(10);
        ws_bus->beginWrite();
        ws_bus->writeCommand(0x35);         // TE on (data_bytes=0 dans l'init v2.0.1)
        ws_bus->writeC8D8(0x51, 0x00);      // brightness 0 (remontée ensuite)
        ws_bus->endWrite(); delay(10);
        ws_bus->beginWrite(); ws_bus->writeCommand(0x11); ws_bus->endWrite(); delay(80);  // SLPOUT
        ws_bus->beginWrite(); ws_bus->writeCommand(0x29); ws_bus->endWrite(); delay(10);  // DISPON
        ws_bus->beginWrite();
        ws_bus->writeC8D8(0x36, 0x30);      // MADCTL paysage HARDWARE (MV+MX) — LA clé manquante
        ws_bus->writeC8D8(0x51, 0xFF);      // brightness max
        ws_bus->endWrite();
#endif
        ws_gfx->fillScreen(0x0000);
        ws_gfx->setBrightness(0);                // remonté via panelBright()
        pinMode(WS241_TP_RST, OUTPUT);
        digitalWrite(WS241_TP_RST, LOW);  delay(20);
        digitalWrite(WS241_TP_RST, HIGH); delay(50);
        ws_touch.setPins(WS241_TP_RST, WS241_TP_INT);
        _touch_ok = ws_touch.begin(Wire, WS241_TP_ADDR, WS241_I2C_SDA, WS241_I2C_SCL);
        return true;
    }
    // SD : pins non encore identifiés sur la 2.41 (microSD présente, possiblement via
    // l'expandeur TCA9554) → stub best-effort. AIP/Vols dégradés tant que non câblé. TODO.
    bool installSD() { return false; }
    void setBrightness(uint8_t v) { ws_gfx->setBrightness(v); }
    const char *getTouchModelName() { return _touch_ok ? "FT6336" : "FT6336 (init FAIL)"; }
    int16_t width()  { return WS241_LCD_W; }
    int16_t height() { return WS241_LCD_H; }
    bool _touch_ok = false;
};

// ── Câblage LVGL ────────────────────────────────────────────────────────────────
static lv_disp_draw_buf_t ws_draw_buf;
static lv_disp_drv_t  ws_disp_drv;
static lv_indev_drv_t ws_indev_drv;
static lv_color_t *ws_buf = nullptr;

static void ws_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;
    ws_gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
    lv_disp_flush_ready(drv);
}
// RM690B0 : fenêtre RAM alignée 2 px (début pair / fin impair) — sinon flush partiel
// impair (label live) décalé d'1 px → texte baveux. (Même précaution que le CO5300.)
static void ws_rounder(lv_disp_drv_t *drv, lv_area_t *area) {
    area->x1 &= ~1; area->y1 &= ~1; area->x2 |= 1; area->y2 |= 1;
}
// Tactile : FT6336 renvoie les coords NATIVES (450×600 portrait) ; on les transforme
// pour le paysage 600×450 rotation 3 (90° CCW) : sx = (LCD_W-1) - ty ; sy = tx.
// (Validé HW 2026-06-26 : aligné sous le doigt sur les 4 coins en rotation 3.)
static void ws_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int16_t x[1], y[1];
    uint8_t n = ws_touch.getPoint(x, y, 1);
    if (n > 0) {
        int16_t sx = (WS241_LCD_W - 1) - y[0];
        int16_t sy = x[0];
        if (sx < 0) sx = 0; else if (sx >= WS241_LCD_W) sx = WS241_LCD_W - 1;
        if (sy < 0) sy = 0; else if (sy >= WS241_LCD_H) sy = WS241_LCD_H - 1;
        data->point.x = sx; data->point.y = sy;
        data->state = LV_INDEV_STATE_PR;
    } else data->state = LV_INDEV_STATE_REL;
}

static inline void beginLvglHelper(WS241_Panel &p, bool debug = false) {
    lv_init();
    const int BUF_LINES = 40;
    size_t px = (size_t)p.width() * BUF_LINES;
    ws_buf = (lv_color_t *)heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ws_buf) ws_buf = (lv_color_t *)heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_8BIT);
    assert(ws_buf);
    lv_disp_draw_buf_init(&ws_draw_buf, ws_buf, NULL, px);
    lv_disp_drv_init(&ws_disp_drv);
    ws_disp_drv.hor_res  = p.width();
    ws_disp_drv.ver_res  = p.height();
    ws_disp_drv.flush_cb = ws_disp_flush;
    ws_disp_drv.rounder_cb = ws_rounder;
    ws_disp_drv.draw_buf = &ws_draw_buf;
    lv_disp_drv_register(&ws_disp_drv);
    lv_indev_drv_init(&ws_indev_drv);
    ws_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    ws_indev_drv.read_cb = ws_touch_read;
    lv_indev_drv_register(&ws_indev_drv);
}

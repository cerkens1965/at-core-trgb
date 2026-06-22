#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// AT-VIEW — Shim hardware Waveshare ESP32-S3-Touch-AMOLED-2.16 (BOARD_WS216)
//
// Remplace le couple LilyGo (panel + LV_Helper) absent pour cette carte :
//   • écran  : Arduino_GFX / Arduino_CO5300 (QSPI)
//   • touch  : SensorLib / TouchDrvCSTXXX (CST9220, I²C @0x5A)
//   • SD     : SPI
// Fournit un objet `WS216_Panel` exposant la même surface que les `panel.*`
// utilisés par le .ino (begin / installSD / setBrightness / getTouchModelName /
// width / height) + un `beginLvglHelper(panel)` qui câble LVGL (flush + indev).
//
// Orientation / mapping : validés au bring-up (cf. examples/ws216_bringup, CLAUDE.md).
//   Affichage droit : MADCTL 0x36 = 0xA0, rotation 0, ordre couleur RGB OK.
//   Dalle tactile tournée 90° vs l'affichage → screen_x = (W-1)-touch_y ; screen_y = touch_x.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrvCSTXXX.hpp>

// ── Pins (source = BSP ESP-IDF officiel ; cf. ws216_bringup/pin_config_ws216.h) ──
#define WS216_LCD_CS   12
#define WS216_LCD_SCK  38
#define WS216_LCD_D0    4
#define WS216_LCD_D1    5
#define WS216_LCD_D2    6
#define WS216_LCD_D3    7
#define WS216_LCD_RST  39
#define WS216_LCD_W   480
#define WS216_LCD_H   480
#define WS216_MADCTL 0xA0      // orientation droite (validée bring-up)

#define WS216_I2C_SDA 15
#define WS216_I2C_SCL 14
#define WS216_TP_INT  11
#define WS216_TP_RST  40
#define WS216_TP_ADDR 0x5A     // CST9220

#define WS216_SD_MOSI  1
#define WS216_SD_SCK   2
#define WS216_SD_MISO  3
#define WS216_SD_CS   41

// ── Objets bas-niveau (instanciés une fois : header inclus dans l'unique TU .ino) ──
static Arduino_DataBus *ws_bus = new Arduino_ESP32QSPI(
    WS216_LCD_CS, WS216_LCD_SCK, WS216_LCD_D0, WS216_LCD_D1, WS216_LCD_D2, WS216_LCD_D3);
// GFX 1.6.4 (core 3.x) : le constructeur Arduino_CO5300 (base Arduino_OLED) n'a plus le param `ips`.
static Arduino_CO5300 *ws_gfx = new Arduino_CO5300(
    ws_bus, WS216_LCD_RST, 0 /*rotation*/, WS216_LCD_W, WS216_LCD_H, 0, 0, 0, 0);
static TouchDrvCSTXXX ws_touch;

// ── Panel : surface compatible avec les call-sites panel.* du .ino ──────────────
class WS216_Panel {
public:
    bool begin() {
        Wire.begin(WS216_I2C_SDA, WS216_I2C_SCL);
        if (!ws_gfx->begin()) return false;
        // Noir propre : assuré par GFX 1.6.4 (core 3.x, driver CO5300/Arduino_OLED réécrit) —
        // la 1.5.0/core2 rendait le noir verdâtre. AUCUN registre vendeur supplémentaire requis
        // (cf. ws216_blacktest validé). ⚠ Le rendu propre dépend AUSSI de l'isolation SPI de la
        // SD sur HSPI (cf. installSD) : sinon SPI.begin() réinitialise le bus QSPI de l'écran.
        ws_bus->writeC8D8(0x36, WS216_MADCTL);   // orientation droite
        ws_gfx->fillScreen(0x0000);
        ws_gfx->setBrightness(0);                // remontée plus tard via panelBright()
        // Touch CST9220
        pinMode(WS216_TP_RST, OUTPUT);
        digitalWrite(WS216_TP_RST, LOW);  delay(20);
        digitalWrite(WS216_TP_RST, HIGH); delay(50);
        ws_touch.setPins(WS216_TP_RST, WS216_TP_INT);
        _touch_ok = ws_touch.begin(Wire, WS216_TP_ADDR, WS216_I2C_SDA, WS216_I2C_SCL);
        return true;
    }
    // SD en SPI. Best-effort : un échec n'est pas fatal (UI tourne, AIP/Vols dégradés).
    // ⚠ HÔTE DÉDIÉ HSPI (SPI3_HOST) : l'écran CO5300 occupe SPI2_HOST/FSPI via
    // Arduino_ESP32QSPI. L'objet Arduino global `SPI` est aussi sur FSPI ; un SPI.begin()
    // dessus RÉINITIALISE le bus de l'écran → rendu corrompu/verdâtre. On isole la SD sur
    // un SPIClass HSPI séparé pour ne jamais toucher au bus de l'affichage.
    SPIClass ws_sdspi{HSPI};
    bool installSD() {
        ws_sdspi.begin(WS216_SD_SCK, WS216_SD_MISO, WS216_SD_MOSI, WS216_SD_CS);
        return SD.begin(WS216_SD_CS, ws_sdspi, 20000000);
    }
    void setBrightness(uint8_t v) { ws_gfx->setBrightness(v); }
    const char *getTouchModelName() { return _touch_ok ? "CST9220" : "CST9220 (init FAIL)"; }
    int16_t width()  { return WS216_LCD_W; }
    int16_t height() { return WS216_LCD_H; }
    bool _touch_ok = false;
};

// ── Câblage LVGL (équivalent du LV_Helper vendeur) ──────────────────────────────
static lv_disp_draw_buf_t ws_draw_buf;
static lv_disp_drv_t  ws_disp_drv;
static lv_indev_drv_t ws_indev_drv;
static lv_color_t *ws_buf  = nullptr;
static lv_color_t *ws_buf1 = nullptr;

// Flush : avec LV_COLOR_16_SWAP=0 (lv_conf_ws216), la fonction CORRECTE est
// draw16bitRGBBitmap (sans swap) — cf. flush de l'exemple LVGL Waveshare :
//   #if LV_COLOR_16_SWAP != 0 → draw16bitBeRGBBitmap   #else → draw16bitRGBBitmap
// (draw16bitBeRGBBitmap = pour swap=1 ; l'utiliser ici inversait les couleurs).
static void ws_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;
    ws_gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
    lv_disp_flush_ready(drv);
}

// ⚠ CO5300 : la fenêtre de RAM (CASET/PASET) doit être alignée sur 2 px — début pair,
// fin impair (donc largeur/hauteur PAIRES). Sans ce rounder, une zone de flush partielle
// impaire (typiquement un label mis à jour en direct) est décalée d'1 px par le panneau →
// texte baveux/dédoublé. Les rendus plein écran (0..479, déjà alignés) ne montraient rien.
// Repris du BSP d'usine Waveshare (bsp_lvgl_rounder_cb).
static void ws_rounder(lv_disp_drv_t *drv, lv_area_t *area) {
    area->x1 &= ~1;          // début → pair
    area->y1 &= ~1;
    area->x2 |= 1;           // fin → impair (largeur/hauteur paires)
    area->y2 |= 1;
}

// Indev : applique le mapping 90° calibré (dalle tournée vs affichage).
static void ws_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int16_t x[1], y[1];
    uint8_t n = ws_touch.getPoint(x, y, 1);
    if (n > 0) {
        int16_t sx = (WS216_LCD_W - 1) - y[0];
        int16_t sy = x[0];
        if (sx < 0) sx = 0; else if (sx >= WS216_LCD_W) sx = WS216_LCD_W - 1;
        if (sy < 0) sy = 0; else if (sy >= WS216_LCD_H) sy = WS216_LCD_H - 1;
        data->point.x = sx;
        data->point.y = sy;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

static inline void beginLvglHelper(WS216_Panel &p, bool debug = false) {
    lv_init();
    // Buffer LVGL PARTIEL en RAM interne (modèle de l'exemple LVGL fourni par GFX 1.6.4 :
    // partial buffer, MALLOC_CAP_INTERNAL). writePixels copie de toute façon vers son propre
    // buffer DMA interne → pas besoin de MALLOC_CAP_DMA ici. 40 lignes ≈ 38 KB.
    const int BUF_LINES = 40;
    size_t px = (size_t)p.width() * BUF_LINES;
    ws_buf  = (lv_color_t *)heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ws_buf) ws_buf = (lv_color_t *)heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_8BIT);
    assert(ws_buf);
    lv_disp_draw_buf_init(&ws_draw_buf, ws_buf, NULL, px);

    lv_disp_drv_init(&ws_disp_drv);
    ws_disp_drv.hor_res  = p.width();
    ws_disp_drv.ver_res  = p.height();
    ws_disp_drv.flush_cb = ws_disp_flush;
    ws_disp_drv.rounder_cb = ws_rounder;   // alignement 2 px requis par le CO5300
    ws_disp_drv.draw_buf = &ws_draw_buf;
    lv_disp_drv_register(&ws_disp_drv);

    lv_indev_drv_init(&ws_indev_drv);
    ws_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    ws_indev_drv.read_cb = ws_touch_read;
    lv_indev_drv_register(&ws_indev_drv);
}

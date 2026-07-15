#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// WS-241 chemin esp_lcd SH8601 (PANEL_WS241_SH8601_ESPLCD)
//
// Pilote la dalle SH8601 (stock récent, Rev1 v1.0.0 + Rev2 v2.0.1) via le VRAI driver
// esp_lcd_sh8601 (compilé depuis esp_lcd_sh8601_ws241.c), au lieu d'Arduino_GFX.
// RÉPLIQUE À LA LETTRE la démo Waveshare 09_LVGL_Test.ino (ws241_sh8601_reference/) :
//   - double draw buffer MALLOC_CAP_DMA, taille H_RES*V_RES/10
//   - drv_update_cb = example_lvgl_update_cb (swap_xy/mirror pour l'orientation)
//   - user_data = panel_handle
//   - offset +16 en Y ajouté MANUELLEMENT dans le flush (PAS de set_gap)
//   - PAS de WRCTRLD 0x53 ajouté (l'init usine n'en envoie pas)
// LVGL est câblé au-dessus (flush → esp_lcd_panel_draw_bitmap). Pins de ws241_shim.h.
// Touch : SensorLib TouchDrvFT6X36 (I2C), best-effort.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <TouchDrvFT6X36.hpp>
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh8601.h"

#define WS241_LCD_HOST   SPI2_HOST
#define WS241_LCD_W      600
#define WS241_LCD_H      450   // dalle 2.41 = 600×450 (spec Waveshare ; natif SH8601 450×600 portrait)
#define WS241_LCD_YGAP   16    // offset EXACT +16 : la dalle démarre en colonne 16 (CASET 0x0010→0x01D1, natif portrait 450 large offset +16). En paysage (swap_xy) ça tombe sur l'axe vertical → sans ça, contenu remonté de 16 px + bande noire de 16 px en bas (bug v97 qui l'avait mis à 0).

// Init SH8601 v2.0.1 — COPIE EXACTE de lcd_init_cmds[] de la démo (AMOLED_Rotate=Rotate_90).
static const sh8601_lcd_init_cmd_t ws241_lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x20}, 1, 0},
    {0x26, (uint8_t[]){0x0A}, 1, 0},
    {0x24, (uint8_t[]){0x80}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0xC2, (uint8_t[]){0x00}, 1, 10},
    {0x35, (uint8_t[]){0x00}, 0, 0},
    {0x51, (uint8_t[]){0x00}, 1, 10},
    {0x11, (uint8_t[]){0x00}, 0, 80},
    {0x2A, (uint8_t[]){0x00, 0x10, 0x01, 0xD1}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x02, 0x57}, 4, 0},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x36, (uint8_t[]){0x30}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
};

static esp_lcd_panel_handle_t ws_panel = NULL;
static esp_lcd_panel_io_handle_t ws_io = NULL;
static TouchDrvFT6X36 ws_touch;

static lv_disp_draw_buf_t ws_draw_buf;
static lv_disp_drv_t  ws_disp_drv;
static lv_indev_drv_t ws_indev_drv;
static lv_color_t *ws_buf1 = nullptr;
static lv_color_t *ws_buf2 = nullptr;

// on_color_trans_done → LVGL peut réutiliser le buffer
static bool ws_flush_ready(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *ctx) {
    lv_disp_flush_ready((lv_disp_drv_t *)ctx);
    return false;
}
// RÉPLIQUE example_lvgl_update_cb (orientation) — appelé par LVGL à l'enregistrement du driver.
static void ws_update_cb(lv_disp_drv_t *drv) {
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    switch (drv->rotated) {
        case LV_DISP_ROT_NONE: esp_lcd_panel_swap_xy(panel, false); esp_lcd_panel_mirror(panel, true,  false); break;
        case LV_DISP_ROT_90:   esp_lcd_panel_swap_xy(panel, true);  esp_lcd_panel_mirror(panel, true,  true);  break;
        case LV_DISP_ROT_180:  esp_lcd_panel_swap_xy(panel, false); esp_lcd_panel_mirror(panel, false, true);  break;
        case LV_DISP_ROT_270:  esp_lcd_panel_swap_xy(panel, true);  esp_lcd_panel_mirror(panel, false, false); break;
    }
}
static void ws_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    // Ordre d'octets RGB565 : LVGL (LV_COLOR_16_SWAP=0) sort en little-endian, la dalle SH8601
    // attend big-endian → sans échange, bleu 0x001F devient 0x1F00 = VERT. On échange les octets
    // ici (équivalent de ce que fait Arduino_GFX en interne sur le chemin RM690B0).
    { int32_t n = (area->x2-area->x1+1)*(area->y2-area->y1+1); uint16_t *p = (uint16_t *)color_p;
      for (int32_t i=0;i<n;i++){ uint16_t c=p[i]; p[i]=(uint16_t)((c>>8)|(c<<8)); } }
    // +16 en Y ajouté MANUELLEMENT (comme la démo, AMOLED_Rotate=Rotate_90) — pas de set_gap.
    int x1 = area->x1, x2 = area->x2, y1 = area->y1 + WS241_LCD_YGAP, y2 = area->y2 + WS241_LCD_YGAP;
    esp_lcd_panel_draw_bitmap(panel, x1, y1, x2 + 1, y2 + 1, color_p);
    // flush_ready appelé par le callback on_color_trans_done (ws_flush_ready)
}
static void ws_rounder(lv_disp_drv_t *drv, lv_area_t *area) {
    area->x1 &= ~1; area->y1 &= ~1; area->x2 |= 1; area->y2 |= 1;
}
static void ws_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int16_t x[1], y[1];
    uint8_t n = ws_touch.getPoint(x, y, 1);
    if (n > 0) {
        int16_t sx = (WS241_LCD_W - 1) - y[0];
        int16_t sy = x[0];
        if (sx < 0) sx = 0; else if (sx >= WS241_LCD_W) sx = WS241_LCD_W - 1;
        if (sy < 0) sy = 0; else if (sy >= WS241_LCD_H) sy = WS241_LCD_H - 1;
        data->point.x = sx; data->point.y = sy; data->state = LV_INDEV_STATE_PR;
    } else data->state = LV_INDEV_STATE_REL;
}

class WS241_Panel {
public:
    // TCA9554 @0x20 : 0x01=output reg, 0x03=config (1=in,0=out). Relâche le reset dalle Rev2.0.
    static void tcaW(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(0x20); Wire.write(reg); Wire.write(val); Wire.endTransmission();
    }
    static void tcaReset() {
        tcaW(0x03, 0x00);            // toutes broches en SORTIE
        tcaW(0x01, 0xFF); delay(120);// tout HAUT (alim on / resets relâchés)
        tcaW(0x01, 0x00); delay(20); // PULSE bas (assert reset LCD/touch)
        tcaW(0x01, 0xFF); delay(150);// relâche → dalle prête pour l'init
    }
    bool begin() {
        Wire.begin(WS241_I2C_SDA, WS241_I2C_SCL);
        // ── Reset dalle AMOLED : dépend de la révision → on fait les DEUX (universel) ──
        // v1.0.0 (Rev1) : reset via GPIO21 (fait par esp_lcd, reset_gpio_num plus bas).
        // v2.0.1 (Rev2.0) : reset via l'I/O expander TCA9554 @0x20 (bus 47/48) ci-dessous — au POR
        // l'expandeur est en entrée → LCD bloqué en reset → NOIR sans ça. (Scan I2C 2026-07-07.)
        tcaReset();
        spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
            WS241_LCD_SCK, WS241_LCD_D0, WS241_LCD_D1, WS241_LCD_D2, WS241_LCD_D3,
            WS241_LCD_W * WS241_LCD_H * 2);
        esp_err_t e;
        e = spi_bus_initialize(WS241_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
        Serial.printf("[ESPLCD] spi_bus=0x%x\n", e); if (e != ESP_OK) return false;
        esp_lcd_panel_io_spi_config_t io_config =
            SH8601_PANEL_IO_QSPI_CONFIG(WS241_LCD_CS, ws_flush_ready, &ws_disp_drv);
        e = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)WS241_LCD_HOST, &io_config, &ws_io);
        Serial.printf("[ESPLCD] panel_io=0x%x\n", e); if (e != ESP_OK) return false;
        sh8601_vendor_config_t vendor_config = {
            .init_cmds = ws241_lcd_init_cmds,
            .init_cmds_size = sizeof(ws241_lcd_init_cmds) / sizeof(ws241_lcd_init_cmds[0]),
            .flags = { .use_qspi_interface = 1 },
        };
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = WS241_LCD_RST,   // UNIVERSEL : GPIO21 (reset dalle v1.0.0) + tcaReset() TCA9554 (reset dalle v2.0.1)
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 16,
            .vendor_config = &vendor_config,
        };
        e = esp_lcd_new_panel_sh8601(ws_io, &panel_config, &ws_panel);
        Serial.printf("[ESPLCD] new_panel=0x%x\n", e); if (e != ESP_OK) return false;
        e = esp_lcd_panel_reset(ws_panel);            Serial.printf("[ESPLCD] reset=0x%x\n", e);
        e = esp_lcd_panel_init(ws_panel);             Serial.printf("[ESPLCD] init=0x%x\n", e);
        e = esp_lcd_panel_disp_on_off(ws_panel, true);Serial.printf("[ESPLCD] disp_on=0x%x\n", e);
        // orientation/mirror : faite par ws_update_cb à l'enregistrement LVGL (comme la démo).
        // Touch (SensorLib, best-effort)
        pinMode(WS241_TP_RST, OUTPUT);
        digitalWrite(WS241_TP_RST, LOW);  delay(20);
        digitalWrite(WS241_TP_RST, HIGH); delay(50);
        ws_touch.setPins(WS241_TP_RST, WS241_TP_INT);
        _touch_ok = ws_touch.begin(Wire, WS241_TP_ADDR, WS241_I2C_SDA, WS241_I2C_SCL);
        return true;
    }
    bool installSD() { return false; }
    void setBrightness(uint8_t v) {
        // v est DÉJÀ en 0-255 (panelBright() a fait le mapping 0-16 → ×17). NE PAS re-mapper :
        // le double ×17 faisait retomber tout niveau ≥1 (≥17 après panelBright) sur ≥16 → 255
        // → brightness saturé au max, slider sans effet. Envoi direct de la commande 0x51.
        if (ws_io) esp_lcd_panel_io_tx_param(ws_io, 0x51, &v, 1);
    }
    const char *getTouchModelName() { return _touch_ok ? "FT6336/FT5x06" : "touch FAIL"; }
    int16_t width()  { return WS241_LCD_W; }
    int16_t height() { return WS241_LCD_H; }
    bool _touch_ok = false;
};

static inline void beginLvglHelper(WS241_Panel &p, bool debug = false) {
    lv_init();
    // (v138) Buffer SIMPLE 1/20 au lieu du DOUBLE 1/10 (~108 Ko) → libère ~82 Ko de RAM INTERNE.
    // ROOT FIX : le double buffer 1/10 en RAM DMA affamait le contrôleur Bluetooth (Bluedroid ne
    // peut PAS utiliser la PSRAM) → `BLE_INIT: Malloc failed` → writes écran→boîtier qui échouaient
    // (portail/immat/cloud), crashs de reconnexion, figes. Rendu imperceptiblement plus lent (LVGL
    // flushe en plus de petits morceaux — OK sur cette UI). Buffer simple = LVGL attend le flush DMA.
    const int BUF_H = WS241_LCD_H / 20;                 // ~22 lignes
    size_t px = (size_t)WS241_LCD_W * BUF_H;
    ws_buf1 = (lv_color_t *)heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_DMA);
    ws_buf2 = nullptr;
    assert(ws_buf1);
    lv_disp_draw_buf_init(&ws_draw_buf, ws_buf1, nullptr, px);
    lv_disp_drv_init(&ws_disp_drv);
    ws_disp_drv.hor_res  = WS241_LCD_W;
    ws_disp_drv.ver_res  = WS241_LCD_H;
    ws_disp_drv.flush_cb = ws_disp_flush;
    ws_disp_drv.rounder_cb = ws_rounder;
    ws_disp_drv.drv_update_cb = ws_update_cb;
    ws_disp_drv.draw_buf = &ws_draw_buf;
    ws_disp_drv.user_data = ws_panel;
    lv_disp_drv_register(&ws_disp_drv);
    lv_indev_drv_init(&ws_indev_drv);
    ws_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    ws_indev_drv.read_cb = ws_touch_read;
    lv_indev_drv_register(&ws_indev_drv);
}

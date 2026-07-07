// WS-241 RAW test — driver esp_lcd_sh8601 SEUL, sans LVGL.
// But : trancher si notre toolchain peut allumer la dalle SH8601 v2.0.1.
// Reproduit VERBATIM le setup bus/io/panel de la démo Waveshare 09_LVGL_Test,
// puis remplit l'écran (blanc → rouge → vert → bleu, 2 s chacun) via draw_bitmap.
#include <Arduino.h>
#include <Wire.h>
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh8601.h"

#define LCD_HOST     SPI2_HOST
#define PIN_CS       9
#define PIN_PCLK     10
#define PIN_D0       11
#define PIN_D1       12
#define PIN_D2       13
#define PIN_D3       14
#define PIN_RST      21
#define H_RES        600
#define V_RES        450
#define YGAP         16

static const sh8601_lcd_init_cmd_t init_cmds[] = {
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

static esp_lcd_panel_handle_t panel = NULL;
static esp_lcd_panel_io_handle_t io = NULL;
static uint16_t *fb = NULL;   // une bande (H_RES * BAND lignes)
#define BAND 30

static bool trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx) { return false; }

void fillScreen(uint16_t color) {
    for (int i = 0; i < H_RES * BAND; i++) fb[i] = color;
    for (int y = 0; y < V_RES; y += BAND) {
        int h = (y + BAND <= V_RES) ? BAND : (V_RES - y);
        esp_lcd_panel_draw_bitmap(panel, 0, y + YGAP, H_RES, y + h + YGAP, fb);
        delay(5);
    }
}

void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n[RAW] WS-241 SH8601 raw test");

    // ── SCAN I2C (pins IMU/touch 47/48) : révèle tout PMIC/expandeur d'alim Rev2.0 ──
    Wire.begin(47, 48);
    Serial.println("[RAW] I2C scan 0x01..0x7F:");
    for (uint8_t a = 1; a < 0x78; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) Serial.printf("[RAW]   found 0x%02X\n", a);
    }
    Serial.println("[RAW] I2C scan done");

    // ── TCA9554 @0x20 : relâche le reset LCD (Rev2.0 = reset via expandeur, pas GPIO21) ──
    // Registres TCA9554 : 0x01=output, 0x03=config(1=in,0=out). POR = tout en entrée → LCD en reset.
    auto tca = [](uint8_t reg, uint8_t val){ Wire.beginTransmission(0x20); Wire.write(reg); Wire.write(val); Wire.endTransmission(); };
    tca(0x03, 0x00);            // toutes les broches en SORTIE
    tca(0x01, 0xFF); delay(120);// tout HAUT (alim on / resets relâchés)
    tca(0x01, 0x00); delay(20); // PULSE bas (assert reset LCD/touch)
    tca(0x01, 0xFF); delay(150);// relâche → LCD prêt pour l'init
    Serial.println("[RAW] TCA9554 reset pulse done");

    spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(PIN_PCLK, PIN_D0, PIN_D1, PIN_D2, PIN_D3, H_RES * V_RES * 2);
    Serial.printf("[RAW] spi_bus=0x%x\n", spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(PIN_CS, trans_done, NULL);
    Serial.printf("[RAW] panel_io=0x%x\n", esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io));

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = init_cmds,
        .init_cmds_size = sizeof(init_cmds) / sizeof(init_cmds[0]),
        .flags = { .use_qspi_interface = 1 },
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_RST,   // UNIVERSEL : GPIO21 (reset v1.0.0) + TCA9554 pulse (reset v2.0.1)
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    Serial.printf("[RAW] new_panel=0x%x\n", esp_lcd_new_panel_sh8601(io, &panel_config, &panel));
    Serial.printf("[RAW] reset=0x%x\n", esp_lcd_panel_reset(panel));
    Serial.printf("[RAW] init=0x%x\n", esp_lcd_panel_init(panel));
    // ── DÉTECTION CONTRÔLEUR : lire les registres ID via QSPI (0xDA=ID1 → 0x86 = SH8601) ──
    { uint8_t id[4] = {0,0,0,0};
      esp_err_t r04 = esp_lcd_panel_io_rx_param(io, 0x04, id, 3);
      Serial.printf("[RAW] RDDID(0x04)=0x%x -> %02X %02X %02X\n", r04, id[0], id[1], id[2]);
      uint8_t da=0,db=0,dc=0;
      esp_lcd_panel_io_rx_param(io, 0xDA, &da, 1);
      esp_lcd_panel_io_rx_param(io, 0xDB, &db, 1);
      esp_lcd_panel_io_rx_param(io, 0xDC, &dc, 1);
      Serial.printf("[RAW] ID1(0xDA)=0x%02X ID2(0xDB)=0x%02X ID3(0xDC)=0x%02X  => %s\n",
                    da, db, dc, (da==0x86) ? "SH8601" : "RM690B0/autre"); }
    Serial.printf("[RAW] disp_on=0x%x\n", esp_lcd_panel_disp_on_off(panel, true));
    // mirror comme la démo (ROT_NONE) — swap_xy est un no-op sur ce driver
    esp_lcd_panel_mirror(panel, true, false);

    fb = (uint16_t *)heap_caps_malloc(H_RES * BAND * 2, MALLOC_CAP_DMA);
    Serial.printf("[RAW] fb=%p\n", fb);
    Serial.println("[RAW] setup done");
}

void loop() {
    Serial.println("[RAW] WHITE");  fillScreen(0xFFFF); delay(2000);
    Serial.println("[RAW] RED");    fillScreen(0xF800); delay(2000);
    Serial.println("[RAW] GREEN");  fillScreen(0x07E0); delay(2000);
    Serial.println("[RAW] BLUE");   fillScreen(0x001F); delay(2000);
}

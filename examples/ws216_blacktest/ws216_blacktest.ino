// ─────────────────────────────────────────────────────────────────────────────
// WS-216 — test MINIMAL "noir propre" sur la pile Waveshare officielle :
//   ESP32 Arduino core 3.x (pioarduino) + GFX 1.6.4 (driver CO5300 réécrit Arduino_OLED).
// But : prouver que le noir 0x0000 est PROFOND (pas vert) avec cette pile, avant de
// migrer le firmware AT-VIEW complet. Aucune dépendance LVGL/BLE/SensorLib.
// Pins = BSP officiel (cf. pin_config_ws216.h).
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Arduino_GFX_Library.h>

#define LCD_CS  12
#define LCD_SCK 38
#define LCD_D0   4
#define LCD_D1   5
#define LCD_D2   6
#define LCD_D3   7
#define LCD_RST 39
#define LCD_W  480
#define LCD_H  480

static Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
// GFX 1.6.4 : constructeur sans param `ips`.
static Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RST, 0 /*rot*/, LCD_W, LCD_H, 0, 0, 0, 0);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[ws216_blacktest] GFX 1.6.4 / core 3.x");
  if (!gfx->begin()) Serial.println("gfx->begin FAILED");
  bus->writeC8D8(0x36, 0xA0);   // orientation (comme le shim AT-VIEW)
  gfx->setBrightness(255);      // luminosité MAX = cas le plus défavorable au noir
}

void loop() {
  const uint16_t cc[6] = {0x0000, 0xF800, 0x07E0, 0x001F, 0xFFFF, 0x4208};
  const char *nn[6]    = {"BLACK 0x0000", "RED", "GREEN", "BLUE", "WHITE", "GRAY64"};
  for (int i = 0; i < 6; i++) {
    Serial.printf("[fill] %s\n", nn[i]);
    gfx->fillScreen(cc[i]);
    delay(2500);
  }
}

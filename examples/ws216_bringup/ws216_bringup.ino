/*
 * AT-VIEW — BRING-UP minimal : Waveshare ESP32-S3-Touch-AMOLED-2.16 (3ᵉ cible)
 * ─────────────────────────────────────────────────────────────────────────────
 * Objectif : DÉRISQUER la couche basse AVANT d'y greffer l'UI AT-VIEW.
 * Valide indépendamment, sur le vrai hardware :
 *   1. l'init écran CO5300 (QSPI) via Arduino_GFX,
 *   2. l'ordre des couleurs (RGB vs BGR) — séquence plein écran R / G / B,
 *   3. la géométrie 480×480 et l'orientation — marqueurs de coins + croix,
 *   4. le tactile CST9220 (CST92xx SensorLib) — coords série + pastille à l'écran.
 *
 * Pile : Arduino_GFX (Arduino_CO5300) + SensorLib (TouchDrvCST92xx). Pas de PMU
 * (rail écran ON au POR — cf. HelloWorld vendeur). Build : env PlatformIO WS-216.
 *
 * ⚠ Pins issues du BSP ESP-IDF officiel (le pin_config.h Arduino vendeur est
 *   buggé : 466×466 + RST=2). Voir pin_config_ws216.h. Si écran noir / décalé /
 *   couleurs inversées → premier suspect = MADCTL / RST / offsets, ajustables ici.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrvCSTXXX.hpp>   // umbrella SensorLib (auto-détecte CST92xx/CST9220)
#include "pin_config_ws216.h"

#define WS216_TP_ADDR 0x5A     // CST9220 (CST92XX_SLAVE_ADDRESS)

// ── Bus QSPI + panneau CO5300 ────────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    WS216_LCD_CS, WS216_LCD_SCK, WS216_LCD_D0, WS216_LCD_D1, WS216_LCD_D2, WS216_LCD_D3);

// ips=false, offsets=0 (panneau carré). Si l'image est décalée de quelques px,
// jouer sur col_offset/row_offset (2 derniers groupes d'args).
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, WS216_LCD_RST, 0 /* rotation */, false /* ips */,
    WS216_LCD_W, WS216_LCD_H, 0, 0, 0, 0);

// ── Tactile CST9220 ──────────────────────────────────────────────────────────
TouchDrvCSTXXX touch;
bool touch_ok = false;

// Damier de coins + croix centrale : vérifie l'orientation ET que toute l'étendue
// 480×480 est adressable (si le panneau était piloté en 466, les coins seraient
// rognés). Couleurs choisies distinctes par coin pour lever toute ambiguïté.
static void drawTestPattern() {
  gfx->fillScreen(RGB565_BLACK);

  const int s = 60;
  gfx->fillRect(0,                 0,                 s, s, RGB565_RED);    // TL rouge
  gfx->fillRect(WS216_LCD_W - s,   0,                 s, s, RGB565_GREEN);  // TR vert
  gfx->fillRect(0,                 WS216_LCD_H - s,   s, s, RGB565_BLUE);   // BL bleu
  gfx->fillRect(WS216_LCD_W - s,   WS216_LCD_H - s,   s, s, RGB565_WHITE);  // BR blanc

  // Cadre périmétrique (doit toucher les 4 bords si 480×480 OK)
  gfx->drawRect(0, 0, WS216_LCD_W, WS216_LCD_H, RGB565_WHITE);

  // Croix centrale
  gfx->drawFastHLine(0, WS216_LCD_H / 2, WS216_LCD_W, RGB565_WHITE);
  gfx->drawFastVLine(WS216_LCD_W / 2, 0, WS216_LCD_H, RGB565_WHITE);

  // Légendes de coins
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(8,                 s + 6);             gfx->print("TL");
  gfx->setCursor(WS216_LCD_W - 40,  s + 6);             gfx->print("TR");
  gfx->setCursor(8,                 WS216_LCD_H - s - 22); gfx->print("BL");
  gfx->setCursor(WS216_LCD_W - 40,  WS216_LCD_H - s - 22); gfx->print("BR");

  // Texte central
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_YELLOW);
  gfx->setCursor(WS216_LCD_W / 2 - 92, WS216_LCD_H / 2 - 36);
  gfx->print("WS216 BRING-UP");
  gfx->setCursor(WS216_LCD_W / 2 - 56, WS216_LCD_H / 2 + 14);
  gfx->print("480 x 480");
  gfx->setTextColor(RGB565_CYAN);
  gfx->setCursor(WS216_LCD_W / 2 - 84, WS216_LCD_H / 2 + 44);
  gfx->print("touch = dot");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[WS216] === Bring-up CO5300 + CST9220 ===");

  Wire.begin(WS216_I2C_SDA, WS216_I2C_SCL);

  // 1) Écran
  if (!gfx->begin()) {
    Serial.println("[WS216] gfx->begin() FAILED (vérifier pins QSPI / RST)");
  } else {
    Serial.println("[WS216] gfx->begin() OK");
  }
  bus->writeC8D8(0x36, WS216_MADCTL);   // orientation / ordre couleur
  gfx->setBrightness(180);

  // 2) Test ordre couleur — annoncer ce qui DOIT s'afficher
  Serial.println("[WS216] Plein ecran ROUGE (si BLEU -> MADCTL bit BGR)");
  gfx->fillScreen(RGB565_RED);   delay(900);
  Serial.println("[WS216] Plein ecran VERT");
  gfx->fillScreen(RGB565_GREEN); delay(900);
  Serial.println("[WS216] Plein ecran BLEU");
  gfx->fillScreen(RGB565_BLUE);  delay(900);

  // 3) Géométrie / orientation
  drawTestPattern();
  Serial.println("[WS216] Mire affichee : TL=rouge TR=vert BL=bleu BR=blanc");

  // 4) Tactile
  pinMode(WS216_TP_RST, OUTPUT);
  digitalWrite(WS216_TP_RST, LOW);  delay(20);
  digitalWrite(WS216_TP_RST, HIGH); delay(50);
  touch.setPins(WS216_TP_RST, WS216_TP_INT);
  touch_ok = touch.begin(Wire, WS216_TP_ADDR, WS216_I2C_SDA, WS216_I2C_SCL);
  Serial.printf("[WS216] touch.begin(0x%02X) -> %s\n",
                WS216_TP_ADDR, touch_ok ? "OK" : "FAIL");
}

void loop() {
  if (touch_ok) {
    int16_t x[5], y[5];
    uint8_t n = touch.getPoint(x, y, 5);
    if (n > 0) {
      Serial.printf("[WS216] touch n=%u  p0=(%d,%d)\n", n, x[0], y[0]);
      gfx->fillCircle(x[0], y[0], 6, RGB565_YELLOW);
    }
  }
  delay(15);
}

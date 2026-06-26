/*
 * AT-VIEW — BRING-UP minimal : Waveshare ESP32-S3-Touch-AMOLED-2.41 (4ᵉ cible)
 * ─────────────────────────────────────────────────────────────────────────────
 * Dérisque la couche basse AVANT d'y greffer l'UI AT-VIEW. Valide sur le vrai HW :
 *   1. init écran RM690B0 (QSPI) via Arduino_GFX,
 *   2. ordre couleur (RGB vs BGR) — plein écran R / G / B,
 *   3. géométrie 450×600 + orientation — coins ancrés + croix + cadre,
 *   4. tactile FT6336 (FT6X36 SensorLib) — coords série + pastille écran.
 *
 * Pile : Arduino_GFX (Arduino_RM690B0) + SensorLib (TouchDrvFT6X36). Même pile que
 * le WS-216 → build via l'env PlatformIO **WS-216** en pointant src_dir ici.
 *
 * ⚠ Pins : config ESPHome communautaire + étiquette carte (cf pin_config_ws241.h).
 *   Écran noir / décalé / couleurs inversées → 1er suspect = MADCTL / RST / offsets,
 *   ajustables ici (writeC8D8(0x36,...) + col/row_offset du constructeur).
 */
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrvFT6X36.hpp>
#include "pin_config_ws241.h"

// ── Bus QSPI + panneau RM690B0 ───────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    WS241_LCD_CS, WS241_LCD_SCK, WS241_LCD_D0, WS241_LCD_D1, WS241_LCD_D2, WS241_LCD_D3);

// (bus, rst, rotation, w, h, col_off1, row_off1, col_off2, row_off2). Offsets 0 au
// départ ; si l'image est décalée de quelques px, ajuster col_offset1/row_offset1.
Arduino_RM690B0 *gfx = new Arduino_RM690B0(
    bus, WS241_LCD_RST, 0 /* rotation */, WS241_LCD_W, WS241_LCD_H, 0, 0, 0, 0);

// ── Tactile FT6336 ───────────────────────────────────────────────────────────
TouchDrvFT6X36 touch;
bool touch_ok = false;

static void drawTestPattern() {
  gfx->fillScreen(RGB565_BLACK);
  const int s = 60;
  gfx->fillRect(0,               0,               s, s, RGB565_RED);    // TL rouge
  gfx->fillRect(WS241_LCD_W - s, 0,               s, s, RGB565_GREEN);  // TR vert
  gfx->fillRect(0,               WS241_LCD_H - s, s, s, RGB565_BLUE);   // BL bleu
  gfx->fillRect(WS241_LCD_W - s, WS241_LCD_H - s, s, s, RGB565_WHITE);  // BR blanc
  gfx->drawRect(0, 0, WS241_LCD_W, WS241_LCD_H, RGB565_WHITE);          // cadre (4 bords)
  gfx->drawFastHLine(0, WS241_LCD_H / 2, WS241_LCD_W, RGB565_WHITE);
  gfx->drawFastVLine(WS241_LCD_W / 2, 0, WS241_LCD_H, RGB565_WHITE);
  gfx->setTextSize(2); gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(8, s + 6);                    gfx->print("TL");
  gfx->setCursor(WS241_LCD_W - 40, s + 6);     gfx->print("TR");
  gfx->setCursor(8, WS241_LCD_H - s - 22);     gfx->print("BL");
  gfx->setCursor(WS241_LCD_W - 40, WS241_LCD_H - s - 22); gfx->print("BR");
  gfx->setTextColor(RGB565_YELLOW);
  gfx->setCursor(WS241_LCD_W / 2 - 92, WS241_LCD_H / 2 - 36); gfx->print("WS241 BRING-UP");
  gfx->setCursor(WS241_LCD_W / 2 - 60, WS241_LCD_H / 2 + 14); gfx->print("450 x 600");
  gfx->setTextColor(RGB565_CYAN);
  gfx->setCursor(WS241_LCD_W / 2 - 84, WS241_LCD_H / 2 + 44); gfx->print("touch = dot");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[WS241] === Bring-up RM690B0 + FT6336 ===");
  Wire.begin(WS241_I2C_SDA, WS241_I2C_SCL);

  if (!gfx->begin()) Serial.println("[WS241] gfx->begin() FAILED (pins QSPI / RST ?)");
  else               Serial.println("[WS241] gfx->begin() OK");
  bus->writeC8D8(0x36, WS241_MADCTL);   // orientation / ordre couleur
  gfx->setBrightness(180);

  Serial.println("[WS241] Plein ecran ROUGE (si BLEU -> bit BGR du MADCTL)");
  gfx->fillScreen(RGB565_RED);   delay(900);
  Serial.println("[WS241] Plein ecran VERT");
  gfx->fillScreen(RGB565_GREEN); delay(900);
  Serial.println("[WS241] Plein ecran BLEU");
  gfx->fillScreen(RGB565_BLUE);  delay(900);

  drawTestPattern();
  Serial.println("[WS241] Mire : TL=rouge TR=vert BL=bleu BR=blanc (cadre doit toucher les 4 bords)");

  pinMode(WS241_TP_RST, OUTPUT);
  digitalWrite(WS241_TP_RST, LOW);  delay(20);
  digitalWrite(WS241_TP_RST, HIGH); delay(50);
  touch.setPins(WS241_TP_RST, WS241_TP_INT);
  touch_ok = touch.begin(Wire, WS241_TP_ADDR, WS241_I2C_SDA, WS241_I2C_SCL);
  Serial.printf("[WS241] touch.begin(0x%02X) -> %s\n", WS241_TP_ADDR, touch_ok ? "OK" : "FAIL");
}

void loop() {
  if (touch_ok) {
    int16_t x[2], y[2];
    uint8_t n = touch.getPoint(x, y, 2);
    if (n > 0) {
      int sx = x[0], sy = y[0];   // mapping brut — à calibrer comme WS-216 si tourné
      if (sx < 0) sx = 0; else if (sx >= WS241_LCD_W) sx = WS241_LCD_W - 1;
      if (sy < 0) sy = 0; else if (sy >= WS241_LCD_H) sy = WS241_LCD_H - 1;
      Serial.printf("[WS241] touch n=%u raw=(%d,%d)\n", n, x[0], y[0]);
      gfx->fillCircle(sx, sy, 6, RGB565_YELLOW);
    }
  }
  delay(15);
}

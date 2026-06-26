#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Waveshare ESP32-S3-Touch-AMOLED-2.41 — cartographie GPIO (4ᵉ cible AT-VIEW)
//
// SoC ESP32-S3 N16R8 (8 MB PSRAM OPI / 16 MB flash). AMOLED 2,41" **450×600**
// (natif portrait), driver **RM690B0** (QSPI), tactile **FT6336** (I²C @0x38),
// PMIC **ETA6098**, expandeur **TCA9554** @0x20, IMU QMI8658, RTC PCF85063.
//
// SOURCE DES PINS : config ESPHome communautaire pour CETTE carte
//   (github.com/SentientCustard/esphome-waveshare-amoled2.41) + étiquette carte.
//   → À CONFIRMER AU BRING-UP (MADCTL/offsets/couleurs) sur le vrai hardware.
// ─────────────────────────────────────────────────────────────────────────────

// Écran RM690B0 — QSPI
#define WS241_LCD_CS    9
#define WS241_LCD_SCK   10   // PCLK
#define WS241_LCD_D0    11
#define WS241_LCD_D1    12
#define WS241_LCD_D2    13
#define WS241_LCD_D3    14
#define WS241_LCD_RST   21
#define WS241_LCD_W    450   // natif portrait (à l'intégration : rotation → 600×450 paysage comme T4)
#define WS241_LCD_H    600
#define WS241_MADCTL  0x00   // orientation/ordre couleur — à figer au bring-up

// I²C (tactile + RTC + IMU + expandeur/PMIC, bus partagé)
#define WS241_I2C_SDA  47
#define WS241_I2C_SCL  48

// Tactile FT6336 (FT6X36)
#define WS241_TP_RST    3
#define WS241_TP_INT   -1    // INT non exposé dans la config ESPHome → polling I²C
#define WS241_TP_ADDR 0x38

// Expandeur TCA9554 @0x20 (rails d'alim / LED RGB). RST écran = GPIO21 DIRECT → pas
// requis pour le bring-up display. À init (XL9555/TCA9554) si un rail écran est dessus.
#define WS241_TCA9554_ADDR 0x20

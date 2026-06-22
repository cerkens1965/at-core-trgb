#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Waveshare ESP32-S3-Touch-AMOLED-2.16 — cartographie GPIO (3ᵉ cible AT-VIEW)
//
// SoC ESP32-S3R8 (8 MB PSRAM OPI / 16 MB flash). Écran AMOLED CARRÉ 480×480,
// driver CO5300 (QSPI), touch CST9220 (I²C), PMIC AXP2101, RTC PCF85063,
// IMU QMI8658, codec ES8311 + ES7210 (audio — non utilisé par AT-VIEW).
//
// ⚠ SOURCE DES PINS — divergence vendeur résolue au profit du BSP ESP-IDF :
//   Le `pin_config.h` Arduino livré par Waveshare est un copier-coller BUGGÉ de
//   la variante ronde 1.75C : il annonce 466×466 et RST écran = GPIO2 (qui
//   collisionne avec SD_CLK !). Le BSP ESP-IDF officiel (composant
//   esp32_s3_touch_amoled_2_16) donne les vraies valeurs, retenues ici :
//     • Résolution 480×480  (BSP_LCD_H_RES / V_RES)
//     • Display RST = GPIO39 (BSP_LCD_RST)        ← PAS 2
//     • Touch  RST = GPIO40 (BSP_LCD_TOUCH_RST)   ← PAS 2
//     • Touch  INT = GPIO11 (BSP_LCD_TOUCH_INT)
//   → À CONFIRMER AU BRING-UP sur le vrai hardware (cf. ws216_bringup.ino).
// ─────────────────────────────────────────────────────────────────────────────

// Écran CO5300 — QSPI
#define WS216_LCD_CS    12
#define WS216_LCD_SCK   38   // PCLK
#define WS216_LCD_D0     4   // SIO0
#define WS216_LCD_D1     5   // SIO1
#define WS216_LCD_D2     6   // SIO2
#define WS216_LCD_D3     7   // SIO3
#define WS216_LCD_RST   39   // BSP (≠ 2 du pin_config Arduino vendeur, qui est faux)
#define WS216_LCD_W    480
#define WS216_LCD_H    480

// MADCTL (registre 0x36) — orientation + ordre couleur. Le HelloWorld vendeur
// applique 0xA0 après begin(). Si rouge↔bleu inversés → essayer 0xA8 (bit BGR),
// si image tournée/miroir → 0x00 / 0xC0 / 0x60. À figer au bring-up.
#define WS216_MADCTL  0xA0

// Touch CST9220 — I²C (adresse 0x5A, cf. CST92XX_SLAVE_ADDRESS de SensorLib)
#define WS216_I2C_SDA  15
#define WS216_I2C_SCL  14
#define WS216_TP_INT   11    // BSP_LCD_TOUCH_INT
#define WS216_TP_RST   40    // BSP_LCD_TOUCH_RST (≠ 2 du pin_config vendeur)

// PMIC AXP2101 — I²C @0x34 (même bus que le touch). Le rail écran est ON par
// défaut au POR (le HelloWorld vendeur n'initialise pas le PMU) → pas requis
// pour le bring-up. À init via XPowersLib lors de l'intégration complète.
#define WS216_AXP2101_ADDR 0x34

// SD — interface SPI (non testée au bring-up ; pour intégration ultérieure)
#define WS216_SD_MOSI   1
#define WS216_SD_SCK    2
#define WS216_SD_MISO   3
#define WS216_SD_CS    41

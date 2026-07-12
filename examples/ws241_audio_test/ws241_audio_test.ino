// WS-241 AUDIO debug — son CONTINU fort (1 kHz, ~100%) en boucle, driver I2S legacy.
// But : trancher DAC/câblage/jack, sans LVGL ni timing. Câblage PCM5102A (violette) :
//   VIN→3V3 · GND→GND · BCK→GPIO5 · LCK→GPIO6 · DIN→GPIO7 · SCK→GND (CRITIQUE).
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

#define AUD_BCK   5
#define AUD_WS    6
#define AUD_DOUT  7
#define AUD_SR    16000
#define TONE_HZ   1000

static bool g_ok = false;

void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n[AUDT] WS-241 audio debug — tone continu 1kHz");

    i2s_config_t cfg; memset(&cfg, 0, sizeof(cfg));
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = AUD_SR;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = 0;
    cfg.dma_buf_count        = 6;
    cfg.dma_buf_len          = 256;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;
    esp_err_t rc = i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    Serial.printf("[AUDT] i2s_driver_install=%s\n", esp_err_to_name(rc));
    if (rc != ESP_OK) return;
    i2s_pin_config_t pins; memset(&pins, 0, sizeof(pins));
    pins.mck_io_num = I2S_PIN_NO_CHANGE;
    pins.bck_io_num = AUD_BCK; pins.ws_io_num = AUD_WS;
    pins.data_out_num = AUD_DOUT; pins.data_in_num = I2S_PIN_NO_CHANGE;
    rc = i2s_set_pin(I2S_NUM_0, &pins);
    Serial.printf("[AUDT] i2s_set_pin=%s (BCK%d/WS%d/DIN%d)\n", esp_err_to_name(rc), AUD_BCK, AUD_WS, AUD_DOUT);
    g_ok = (rc == ESP_OK);
    Serial.println(g_ok ? "[AUDT] TONE ON — tu dois entendre un 1kHz continu" : "[AUDT] init KO");
}

void loop() {
    if (!g_ok) { delay(1000); return; }
    const int N = 256;
    static int16_t buf[N * 2];
    static float ph = 0.0f;
    const float dp = 2.0f * (float)M_PI * TONE_HZ / AUD_SR;
    for (int i = 0; i < N; i++) {
        int16_t s = (int16_t)(30000.0f * sinf(ph));   // ~92% pleine échelle
        ph += dp; if (ph > 2.0f * (float)M_PI) ph -= 2.0f * (float)M_PI;
        buf[2 * i] = s; buf[2 * i + 1] = s;
    }
    size_t wr;
    i2s_write(I2S_NUM_0, buf, N * 2 * sizeof(int16_t), &wr, 100 / portTICK_PERIOD_MS);
    static uint32_t last = 0;
    if (millis() - last > 2000) { last = millis(); Serial.printf("[AUDT] streaming, wrote=%u\n", (unsigned)wr); }
}

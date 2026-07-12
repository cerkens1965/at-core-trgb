// WS-241 AUDIO — outil de bring-up : tone 1 kHz continu (driver legacy driver/i2s.h).
// Sert à valider le câblage d'un DAC PCM5102A. Le firmware AT-VIEW utilise le même chemin.
//
// ⚠️ CÂBLAGE COMPLET PCM5102A (carte violette GY-PCM5102) — validé hardware 2026-07-12 :
//   VIN  → 3V3        BCK  → GPIO 5
//   GND  → GND        LRCK → GPIO 6   (= LCK)
//   SCK  → GND        DIN  → GPIO 7
//   XSMT → 3V3   ← DÉ-MUTE (soft-mute) — SANS LUI : SILENCE TOTAL malgré un I2S parfait !
//   FMT  → GND   ← format I2S (sinon left-justified = son cassé)
//   FLT  → GND · DEMP → GND   ← états définis (propreté)
//   Sortie = jack 3,5 mm (casque = quiet car basse Z ; AUX Funke = idéal).
// Les broches XSMT/FMT/FLT/DEMP FLOTTENT par défaut sur ces cartes (aucun pont d'usine) → à câbler.
// Diag horloge : l'I2S de l'ESP32 est bon (BCLK 1,41 MHz / LRCK 44,1 kHz mesurés au PCNT) ;
// un silence vient TOUJOURS du DAC (XSMT/mute) ou du câblage, pas du driver.
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#define P_BCK 5
#define P_WS  6
#define P_DIN 7
#define AUD_SR 16000
#define TONE_HZ 1000
static bool g_ok=false;
void setup(){
  Serial.begin(115200); delay(400);
  Serial.println("\n[AUDT] WS-241 tone 1kHz — valider câblage PCM5102A (XSMT->3V3 obligatoire !)");
  i2s_config_t cfg; memset(&cfg,0,sizeof(cfg));
  cfg.mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_TX);
  cfg.sample_rate=AUD_SR; cfg.bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format=I2S_CHANNEL_FMT_RIGHT_LEFT; cfg.communication_format=I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags=0; cfg.dma_buf_count=8; cfg.dma_buf_len=256;
  cfg.use_apll=false; cfg.tx_desc_auto_clear=true;
  Serial.printf("[AUDT] install=%s\n",esp_err_to_name(i2s_driver_install(I2S_NUM_0,&cfg,0,NULL)));
  i2s_pin_config_t pins; memset(&pins,0,sizeof(pins));
  pins.mck_io_num=I2S_PIN_NO_CHANGE; pins.bck_io_num=P_BCK; pins.ws_io_num=P_WS;
  pins.data_out_num=P_DIN; pins.data_in_num=I2S_PIN_NO_CHANGE;
  Serial.printf("[AUDT] set_pin=%s (BCK5/WS6/DIN7)\n",esp_err_to_name(i2s_set_pin(I2S_NUM_0,&pins)));
  g_ok=true; Serial.println("[AUDT] tone ON");
}
void loop(){
  if(!g_ok){delay(1000);return;}
  static int16_t buf[512]; static float ph=0; const float dp=2.0f*(float)M_PI*TONE_HZ/AUD_SR;
  for(int i=0;i<256;i++){int16_t s=(int16_t)(28000.0f*sinf(ph)); ph+=dp; if(ph>2*M_PI)ph-=2*M_PI; buf[2*i]=s; buf[2*i+1]=s;}
  size_t wr; i2s_write(I2S_NUM_0,buf,1024,&wr,100/portTICK_PERIOD_MS);
  static uint32_t last=0; if(millis()-last>2000){last=millis(); Serial.printf("[AUDT] streaming wrote=%u\n",(unsigned)wr);}
}

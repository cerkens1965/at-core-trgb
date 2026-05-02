/**
 * AT-CORE AeroTrace Core — Étape 3 TEST HARDWARE
 * ESP32-S3 — LILYGO T-RGB 2.8" Full Circle
 * ST7701S 480x480 RGB — BLE Client debug display
 * Christophe — AT-CORE v0.3 — 01/05/2026
 */

#include <Arduino.h>
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
#include <lvgl.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>

// BLE UUIDs
#define BLE_SVC_UUID    "4FAFC201-1FB5-459E-8FCC-C5C9C331914B"
#define BLE_CHR_STATUS  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_DEBUG   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TARGET      "AT-CORE"

// Couleurs
#define C_BG    lv_color_hex(0x000000)
#define C_WHITE lv_color_hex(0xFFFFFF)
#define C_AMBER lv_color_hex(0xF5A623)
#define C_GREEN lv_color_hex(0x22c55e)
#define C_BLUE  lv_color_hex(0x60a5fa)
#define C_RED   lv_color_hex(0xef4444)
#define C_GREY  lv_color_hex(0x6b7280)

struct StatusData { int mode,gps_sat,csq,frames,alt,spd,hdg; float lat,lon; bool gps_fix,sd_ok,valid; };
struct DebugData  { int hb_gps,hb_lte,hb_sd,csq,http_ms,code,heap,ss_ago,fa_ago; char fid[24]; bool valid; };

static LilyGo_RGBPanel panel;
static BLEClient* g_client = nullptr;
static BLERemoteService* g_svc = nullptr;
static BLERemoteCharacteristic* g_chrS = nullptr;
static BLERemoteCharacteristic* g_chrD = nullptr;
static volatile bool g_connected = false, g_doConnect = false;
static BLEAdvertisedDevice* g_target = nullptr;
static StatusData g_status = {};
static DebugData  g_debug  = {};
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_page = 0;
static uint32_t g_lastPage = 0, g_lastRefresh = 0;

void parseStatus(const char* j) {
    StaticJsonDocument<256> d;
    if(deserializeJson(d,j)) return;
    portENTER_CRITICAL(&g_mux);
    g_status = {d["mode"]|0, d["gps_sat"]|0, d["csq"]|-1, d["frames"]|0,
                d["alt"]|0, d["spd"]|0, d["hdg"]|0,
                d["lat"]|0.0f, d["lon"]|0.0f,
                d["gps_fix"]|false, d["sd_ok"]|false, true};
    portEXIT_CRITICAL(&g_mux);
}

void parseDebug(const char* j) {
    StaticJsonDocument<256> d;
    if(deserializeJson(d,j)) return;
    portENTER_CRITICAL(&g_mux);
    g_debug.hb_gps=d["hb_gps"]|0; g_debug.hb_lte=d["hb_lte"]|0; g_debug.hb_sd=d["hb_sd"]|0;
    g_debug.csq=d["csq"]|0; g_debug.http_ms=d["http_ms"]|0; g_debug.code=d["code"]|0;
    g_debug.heap=d["heap"]|0; g_debug.ss_ago=d["ss_ago"]|0; g_debug.fa_ago=d["fa_ago"]|0;
    strlcpy(g_debug.fid, d["fid"]|"---", 24); g_debug.valid=true;
    portEXIT_CRITICAL(&g_mux);
}

static void notifyS(BLERemoteCharacteristic* c,uint8_t* d,size_t l,bool n){char b[l+1];memcpy(b,d,l);b[l]=0;parseStatus(b);}
static void notifyD(BLERemoteCharacteristic* c,uint8_t* d,size_t l,bool n){char b[l+1];memcpy(b,d,l);b[l]=0;parseDebug(b);}

class ATCCB : public BLEClientCallbacks {
    void onConnect(BLEClient*) override { g_connected=true; Serial.println("[BLE] Connecté ✓"); }
    void onDisconnect(BLEClient*) override { g_connected=false; g_status.valid=false; g_debug.valid=false; Serial.println("[BLE] Déco"); }
};
class ATCAdv : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        if(dev.getName()==BLE_TARGET){BLEDevice::getScan()->stop();g_target=new BLEAdvertisedDevice(dev);g_doConnect=true;}
    }
};

bool connectBLE() {
    g_client=BLEDevice::createClient();
    g_client->setClientCallbacks(new ATCCB());
    if(!g_client->connect(g_target)) return false;
    g_client->setMTU(512);
    g_svc=g_client->getService(BLE_SVC_UUID);
    if(!g_svc){g_client->disconnect();return false;}
    g_chrS=g_svc->getCharacteristic(BLE_CHR_STATUS);
    g_chrD=g_svc->getCharacteristic(BLE_CHR_DEBUG);
    if(g_chrS&&g_chrS->canNotify()) g_chrS->registerForNotify(notifyS);
    if(g_chrD&&g_chrD->canNotify()) g_chrD->registerForNotify(notifyD);
    return true;
}
void startScan(){BLEScan*s=BLEDevice::getScan();s->setAdvertisedDeviceCallbacks(new ATCAdv());s->setActiveScan(true);s->start(5,false);}

// LVGL helpers
const char* modeStr(int m){switch(m){case 0:return"PREFLIGHT";case 1:return"FLIGHT";case 2:return"POSTFLIGHT";default:return"???";}}
lv_color_t modeCol(int m){switch(m){case 0:return C_AMBER;case 1:return C_GREEN;case 2:return C_BLUE;default:return C_GREY;}}
lv_color_t hbCol(int s){return s<10?C_GREEN:s<20?C_AMBER:C_RED;}

void addKV(lv_obj_t* p,int y,const char* k,const char* v,lv_color_t vc){
    lv_obj_t* lk=lv_label_create(p);lv_label_set_text(lk,k);
    lv_obj_set_style_text_color(lk,C_GREY,0);lv_obj_set_style_text_font(lk,&lv_font_montserrat_16,0);
    lv_obj_set_pos(lk,72,y);
    lv_obj_t* lv=lv_label_create(p);lv_label_set_text(lv,v);
    lv_obj_set_style_text_color(lv,vc,0);lv_obj_set_style_text_font(lv,&lv_font_montserrat_16,0);
    lv_obj_align(lv,LV_ALIGN_TOP_RIGHT,-72,y);
}

void buildPage1(){
    lv_obj_clean(lv_scr_act());
    lv_obj_t* s=lv_scr_act();
    lv_obj_set_style_bg_color(s,C_BG,0);

    lv_obj_t* t=lv_label_create(s);
    lv_label_set_text(t,g_connected?"AT-CORE":"RECHERCHE AT-CORE...");
    lv_obj_set_style_text_color(t,C_AMBER,0);
    lv_obj_set_style_text_font(t,&lv_font_montserrat_20,0);
    lv_obj_align(t,LV_ALIGN_TOP_MID,0,55);

    if(!g_status.valid){
        lv_obj_t* w=lv_label_create(s);lv_label_set_text(w,"En attente données BLE...");
        lv_obj_set_style_text_color(w,C_GREY,0);lv_obj_align(w,LV_ALIGN_CENTER,0,0);
    } else {
        lv_obj_t* ml=lv_label_create(s);
        lv_label_set_text(ml,modeStr(g_status.mode));
        lv_obj_set_style_text_color(ml,modeCol(g_status.mode),0);
        lv_obj_set_style_text_font(ml,&lv_font_montserrat_22,0);
        lv_obj_align(ml,LV_ALIGN_TOP_MID,0,90);

        char b[32]; int y=125;
        snprintf(b,32,"%s sat:%d",g_status.gps_fix?"FIX":"NO FIX",g_status.gps_sat);
        addKV(s,y,"GPS",b,g_status.gps_fix?C_GREEN:C_RED);y+=30;
        if(g_status.gps_fix){
            snprintf(b,32,"%.4f",g_status.lat);addKV(s,y,"LAT",b,C_WHITE);y+=30;
            snprintf(b,32,"%.4f",g_status.lon);addKV(s,y,"LON",b,C_WHITE);y+=30;
        }
        snprintf(b,32,"%dm",g_status.alt);addKV(s,y,"ALT",b,C_WHITE);y+=30;
        snprintf(b,32,"%dkt",g_status.spd);addKV(s,y,"SPD",b,C_WHITE);y+=30;
        snprintf(b,32,"%d°",g_status.hdg);addKV(s,y,"HDG",b,C_WHITE);y+=30;
        snprintf(b,32,"CSQ %d",g_status.csq);addKV(s,y,"LTE",b,g_status.csq>5?C_GREEN:C_RED);y+=30;
        snprintf(b,32,"%s %dfr",g_status.sd_ok?"OK":"FAIL",g_status.frames);
        addKV(s,y,"SD",b,g_status.sd_ok?C_GREEN:C_RED);
    }
    lv_obj_t* pg=lv_label_create(s);lv_label_set_text(pg,"1/2");
    lv_obj_set_style_text_color(pg,C_GREY,0);lv_obj_align(pg,LV_ALIGN_BOTTOM_MID,0,-20);
}

void buildPage2(){
    lv_obj_clean(lv_scr_act());
    lv_obj_t* s=lv_scr_act();
    lv_obj_set_style_bg_color(s,C_BG,0);

    lv_obj_t* t=lv_label_create(s);lv_label_set_text(t,"DEBUG");
    lv_obj_set_style_text_color(t,C_BLUE,0);lv_obj_set_style_text_font(t,&lv_font_montserrat_20,0);
    lv_obj_align(t,LV_ALIGN_TOP_MID,0,55);

    if(!g_debug.valid){
        lv_obj_t* w=lv_label_create(s);lv_label_set_text(w,"En attente debug BLE...");
        lv_obj_set_style_text_color(w,C_GREY,0);lv_obj_align(w,LV_ALIGN_CENTER,0,0);
    } else {
        char b[32]; int y=90;
        snprintf(b,32,"%ds",g_debug.hb_gps);addKV(s,y,"HB GPS",b,hbCol(g_debug.hb_gps));y+=30;
        snprintf(b,32,"%ds",g_debug.hb_lte);addKV(s,y,"HB LTE",b,hbCol(g_debug.hb_lte));y+=30;
        snprintf(b,32,"%ds",g_debug.hb_sd); addKV(s,y,"HB SD", b,hbCol(g_debug.hb_sd));y+=30;
        snprintf(b,32,"%d",g_debug.csq);addKV(s,y,"CSQ",b,g_debug.csq>=10?C_GREEN:C_AMBER);y+=30;
        snprintf(b,32,"%dms",g_debug.http_ms);addKV(s,y,"HTTP",b,g_debug.http_ms<2000?C_GREEN:C_RED);y+=30;
        snprintf(b,32,"%d",g_debug.code);addKV(s,y,"CODE",b,(g_debug.code==200||g_debug.code==201)?C_GREEN:C_RED);y+=30;
        snprintf(b,32,"%ds",g_debug.ss_ago);addKV(s,y,"SafeSky",b,g_debug.ss_ago<15?C_GREEN:C_RED);y+=30;
        snprintf(b,32,"%dkB",g_debug.heap/1024);addKV(s,y,"HEAP",b,g_debug.heap>100000?C_GREEN:C_RED);y+=30;
        addKV(s,y,"FLT",g_debug.fid,C_WHITE);
    }
    lv_obj_t* pg=lv_label_create(s);lv_label_set_text(pg,"2/2");
    lv_obj_set_style_text_color(pg,C_GREY,0);lv_obj_align(pg,LV_ALIGN_BOTTOM_MID,0,-20);
}

void refreshDisplay(){g_page==0?buildPage1():buildPage2();}

void setup(){
    Serial.begin(115200);delay(500);
    Serial.println("=== AT-CORE T-RGB 2.8 Debug ===");
    if(!panel.begin(LILYGO_T_RGB_2_8_INCHES)){Serial.println("[DISPLAY] FAIL");while(1)delay(1000);}
    beginLvglHelper(panel);
    Serial.println("[DISPLAY] ST7701S OK");

    // Splash
    lv_obj_set_style_bg_color(lv_scr_act(),C_BG,0);
    lv_obj_t* t1=lv_label_create(lv_scr_act());lv_label_set_text(t1,"AEROTRACE");
    lv_obj_set_style_text_color(t1,C_AMBER,0);lv_obj_set_style_text_font(t1,&lv_font_montserrat_32,0);
    lv_obj_align(t1,LV_ALIGN_CENTER,0,-20);
    lv_obj_t* t2=lv_label_create(lv_scr_act());lv_label_set_text(t2,"AT-CORE Debug v0.3");
    lv_obj_set_style_text_color(t2,C_WHITE,0);lv_obj_set_style_text_font(t2,&lv_font_montserrat_16,0);
    lv_obj_align(t2,LV_ALIGN_CENTER,0,20);
    panel.setBrightness(16);
    for(int i=0;i<30;i++){lv_timer_handler();delay(50);}

    BLEDevice::init("ATCORE-TRGB");
    startScan();
    Serial.println("[BLE] Scan démarré...");
    refreshDisplay();
}

void loop(){
    uint32_t now=millis();
    if(g_doConnect&&!g_connected){
        g_doConnect=false;
        if(connectBLE()) Serial.println("[BLE] OK");
        else{delay(2000);startScan();}
        refreshDisplay();
    }
    if(!g_connected&&!g_doConnect){
        static uint32_t ls=0;
        if(now-ls>8000){ls=now;startScan();}
    }
    if(now-g_lastPage>5000){g_lastPage=now;g_page=(g_page+1)%2;refreshDisplay();}
    if(g_connected&&now-g_lastRefresh>1000){g_lastRefresh=now;refreshDisplay();}
    lv_timer_handler();
    delay(5);
}

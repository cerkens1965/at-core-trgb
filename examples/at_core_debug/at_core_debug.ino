/**
 * AT-CORE AeroTrace Core — T-RGB v0.4
 * ESP32-S3 — LILYGO T-RGB 2.8" Full Circle
 * Architecture : 5 containers persistent, show/hide navigation
 * Data         : BLE notifications SIM7600 -> live label update
 * Christophe — AT-CORE v0.4 — 03/05/2026
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>

LilyGo_RGBPanel panel;

#define BLE_SVC_UUID    "4FAFC201-1FB5-459E-8FCC-C5C9C331914B"
#define BLE_CHR_STATUS  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_FLIGHT  "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_TRAFFIC "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_ALERTS  "6E400006-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_DEBUG   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TARGET      "AT-CORE"

#define C_BG     lv_color_hex(0x000000)
#define C_WHITE  lv_color_hex(0xFFFFFF)
#define C_AMBER  lv_color_hex(0xF5A623)
#define C_GREEN  lv_color_hex(0x22c55e)
#define C_BLUE   lv_color_hex(0x60a5fa)
#define C_RED    lv_color_hex(0xef4444)
#define C_ORANGE lv_color_hex(0xf97316)
#define C_GREY   lv_color_hex(0x6b7280)

struct StatusData {
    int mode,gps_sat,csq,frames,alt,spd,hdg;
    float lat,lon;
    bool gps_fix,sd_ok,flarm_ok,adsb_ok,valid;
};
struct FlightData { float gforce_z; int co_ppm,rpm,phase; bool valid; };
#define MAX_TRF 5
struct TrafficEntry { char cs[9]; int dist_m,alt_m; bool visible; };
struct TrafficData  { TrafficEntry t[MAX_TRF]; int count; bool valid; };
struct AlertData    { bool co,gforce,rpm,traffic; char msg[64]; bool valid; };
struct DebugData {
    int hb_gps,hb_lte,hb_sd,csq,http_ms,code;
    bool lte_ok,disable_lte;
    int ss_ago,fa_ago,heap,bat_pct,mode,pending;
    int flarm_tx,flarm_rx,adsb_rx;
    char fid[24]; bool valid;
};

static StatusData  g_status  = {};
static FlightData  g_flight  = {};
static TrafficData g_traffic = {};
static AlertData   g_alert   = {};
static DebugData   g_debug   = {};
static volatile bool g_dataUpdated = false;

static BLEClient*              g_client  = nullptr;
static BLERemoteService*       g_svc     = nullptr;
static BLERemoteCharacteristic *g_chrS=nullptr,*g_chrF=nullptr,
                                *g_chrT=nullptr,*g_chrA=nullptr,*g_chrD=nullptr;
static volatile bool g_connected=false, g_doConnect=false, g_doReconnect=false;
static BLEAdvertisedDevice*    g_target  = nullptr;

#define NUM_PAGES 5
static lv_obj_t* g_pages[NUM_PAGES];
static uint8_t   g_page=0, g_prevPage=0;
static bool      g_alertForced=false;
static volatile bool    g_navPending=false;
static volatile uint8_t g_navPage=0;

static lv_obj_t *r_title,*r_mode,*r_gps,*r_lte,*r_sd,*r_ble,*r_flarm,*r_adsb,*r_coords;
static lv_obj_t *r_phase,*r_alt,*r_spd,*r_hdg,*r_gforce,*r_co,*r_rpm;
struct TrfRow { lv_obj_t *dot,*cs,*dist,*alt; };
static TrfRow   r_trf[MAX_TRF];
static lv_obj_t *r_noTraffic;
static lv_obj_t *r_noAlert,*r_alertCO,*r_alertGF,*r_alertRPM,*r_alertTFC,*r_alertMsg;
static lv_obj_t *r_hbgps,*r_hblte,*r_hbsd,*r_p5csq,*r_http,*r_code;
static lv_obj_t *r_ss,*r_fa,*r_lteok,*r_dis,*r_heap,*r_bat,*r_p5mode,*r_pend;
static lv_obj_t *r_flarmtx,*r_adsbr,*r_flt;

lv_color_t modeCol(int m){switch(m){case 0:return C_AMBER;case 1:return C_GREEN;case 2:return C_BLUE;default:return C_GREY;}}
const char* modeStr(int m){switch(m){case 0:return"PREFLIGHT";case 1:return"FLIGHT";case 2:return"POSTFLIGHT";default:return"SLEEP";}}
const char* phaseStr(int p){switch(p){case 0:return"GROUND";case 1:return"CRUISE";case 2:return"MANEUVER";case 3:return"APPROACH";case 4:return"CRITICAL";default:return"????";}}
lv_color_t phaseCol(int p){switch(p){case 0:return C_WHITE;case 1:return C_GREEN;case 2:return C_ORANGE;case 3:return C_AMBER;case 4:return C_RED;default:return C_GREY;}}
lv_color_t hbCol(int s){return s<10?C_GREEN:s<20?C_AMBER:C_RED;}

lv_obj_t* mkLbl(lv_obj_t* p,const char* t,lv_color_t c,const lv_font_t* f,lv_align_t a,int ox,int oy){
    lv_obj_t* l=lv_label_create(p);lv_label_set_text(l,t);
    lv_obj_set_style_text_color(l,c,0);lv_obj_set_style_text_font(l,f,0);
    lv_obj_align(l,a,ox,oy);return l;}
lv_obj_t* mkLblP(lv_obj_t* p,const char* t,lv_color_t c,const lv_font_t* f,int x,int y){
    lv_obj_t* l=lv_label_create(p);lv_label_set_text(l,t);
    lv_obj_set_style_text_color(l,c,0);lv_obj_set_style_text_font(l,f,0);
    lv_obj_set_pos(l,x,y);return l;}
lv_obj_t* mkStat(lv_obj_t* p,int x,int y,const char* t,bool ok){
    char b[32];snprintf(b,32,"● %s",t);
    lv_obj_t* l=lv_label_create(p);lv_label_set_text(l,b);
    lv_obj_set_style_text_color(l,ok?C_GREEN:C_RED,0);
    lv_obj_set_style_text_font(l,&lv_font_montserrat_16,0);
    lv_obj_set_pos(l,x,y);return l;}
void updStat(lv_obj_t* l,const char* t,bool ok){
    char b[32];snprintf(b,32,"● %s",t);
    lv_label_set_text(l,b);lv_obj_set_style_text_color(l,ok?C_GREEN:C_RED,0);}
void mkKV(lv_obj_t* p,int y,const char* k,lv_obj_t** vr,const char* vi,lv_color_t vc){
    mkLblP(p,k,C_GREY,&lv_font_montserrat_16,80,y);
    lv_obj_t* lv=lv_label_create(p);lv_label_set_text(lv,vi);
    lv_obj_set_style_text_color(lv,vc,0);lv_obj_set_style_text_font(lv,&lv_font_montserrat_16,0);
    lv_obj_align(lv,LV_ALIGN_TOP_RIGHT,-80,y);if(vr)*vr=lv;}
lv_obj_t* mkDbgL(lv_obj_t* p,int y,const char* k,const char* v,lv_color_t c){
    mkLblP(p,k,C_GREY,&lv_font_montserrat_14,80,y);
    return mkLblP(p,v,c,&lv_font_montserrat_14,158,y);}
lv_obj_t* mkDbgR(lv_obj_t* p,int y,const char* k,const char* v,lv_color_t c){
    mkLblP(p,k,C_GREY,&lv_font_montserrat_14,252,y);
    return mkLblP(p,v,c,&lv_font_montserrat_14,330,y);}
lv_obj_t* mkPage(){
    lv_obj_t* p=lv_obj_create(lv_scr_act());lv_obj_set_size(p,480,480);
    lv_obj_set_pos(p,0,0);lv_obj_set_style_bg_color(p,C_BG,0);
    lv_obj_set_style_border_width(p,0,0);lv_obj_set_style_pad_all(p,0,0);
    lv_obj_clear_flag(p,LV_OBJ_FLAG_SCROLLABLE);return p;}

void parseStatus(const char* j){JsonDocument d;if(deserializeJson(d,j))return;
    g_status.mode=d["mode"]|0;g_status.gps_sat=d["gps_sat"]|0;g_status.csq=d["csq"]|-1;
    g_status.frames=d["frames"]|0;g_status.alt=d["alt"]|0;g_status.spd=d["spd"]|0;
    g_status.hdg=d["hdg"]|0;g_status.lat=d["lat"]|0.0f;g_status.lon=d["lon"]|0.0f;
    g_status.gps_fix=d["gps_fix"]|false;g_status.sd_ok=d["sd_ok"]|false;
    g_status.flarm_ok=d["flarm"]|false;g_status.adsb_ok=d["adsb"]|false;
    g_status.valid=true;g_dataUpdated=true;}
void parseFlight(const char* j){JsonDocument d;if(deserializeJson(d,j))return;
    g_flight.gforce_z=d["gf"]|1.0f;g_flight.co_ppm=d["co"]|0;
    g_flight.rpm=d["rpm"]|0;g_flight.phase=d["phase"]|0;
    g_flight.valid=true;g_dataUpdated=true;}
void parseTraffic(const char* j){JsonDocument d;if(deserializeJson(d,j))return;
    g_traffic.count=min((int)(d["count"]|0),MAX_TRF);
    for(int i=0;i<g_traffic.count;i++){
        strlcpy(g_traffic.t[i].cs,d["t"][i]["cs"]|"???",9);
        g_traffic.t[i].dist_m=d["t"][i]["d"]|0;g_traffic.t[i].alt_m=d["t"][i]["a"]|0;
        g_traffic.t[i].visible=d["t"][i]["v"]|true;}
    g_traffic.valid=true;g_dataUpdated=true;}
void parseAlerts(const char* j){JsonDocument d;if(deserializeJson(d,j))return;
    g_alert.co=d["co"]|false;g_alert.gforce=d["gf"]|false;
    g_alert.rpm=d["rpm"]|false;g_alert.traffic=d["tfc"]|false;
    strlcpy(g_alert.msg,d["msg"]|"",64);g_alert.valid=true;g_dataUpdated=true;}
void parseDebug(const char* j){JsonDocument d;if(deserializeJson(d,j))return;
    g_debug.hb_gps=d["hb_gps"]|0;g_debug.hb_lte=d["hb_lte"]|0;g_debug.hb_sd=d["hb_sd"]|0;
    g_debug.csq=d["csq"]|0;g_debug.http_ms=d["http_ms"]|0;g_debug.code=d["code"]|0;
    g_debug.lte_ok=d["lte_ok"]|false;g_debug.disable_lte=d["dis_lte"]|false;
    g_debug.ss_ago=d["ss_ago"]|0;g_debug.fa_ago=d["fa_ago"]|0;
    g_debug.heap=d["heap"]|0;g_debug.bat_pct=d["bat_pct"]|-1;
    g_debug.mode=d["mode"]|0;g_debug.pending=d["pending"]|0;
    g_debug.flarm_tx=d["ftx"]|0;g_debug.flarm_rx=d["frx"]|0;g_debug.adsb_rx=d["adsb_rx"]|0;
    strlcpy(g_debug.fid,d["fid"]|"---",24);g_debug.valid=true;g_dataUpdated=true;}

#define BLE_BUF 512
static void notifyS(BLERemoteCharacteristic*,uint8_t* d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseStatus(b);}
static void notifyF(BLERemoteCharacteristic*,uint8_t* d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseFlight(b);}
static void notifyT(BLERemoteCharacteristic*,uint8_t* d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseTraffic(b);}
static void notifyA(BLERemoteCharacteristic*,uint8_t* d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseAlerts(b);}
static void notifyD(BLERemoteCharacteristic*,uint8_t* d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseDebug(b);}

class ATCCB:public BLEClientCallbacks{
    void onConnect(BLEClient*)override{g_connected=true;g_dataUpdated=true;Serial.println("[BLE] Connected");}
    void onDisconnect(BLEClient*)override{g_connected=false;
        g_status.valid=g_flight.valid=g_traffic.valid=g_alert.valid=g_debug.valid=false;
        g_dataUpdated=true;g_doReconnect=true;Serial.println("[BLE] Disconnected");}};

class ATCAdv:public BLEAdvertisedDeviceCallbacks{
    void onResult(BLEAdvertisedDevice dev)override{
        if(dev.getName()==BLE_TARGET){BLEDevice::getScan()->stop();
            g_target=new BLEAdvertisedDevice(dev);g_doConnect=true;}}};
bool connectBLE(){
    if(!g_client){g_client=BLEDevice::createClient();g_client->setClientCallbacks(new ATCCB());}
    if(!g_client->connect(g_target))return false;
    g_client->setMTU(512);g_svc=g_client->getService(BLE_SVC_UUID);
    if(!g_svc){g_client->disconnect();return false;}
    g_chrS=g_svc->getCharacteristic(BLE_CHR_STATUS);
    g_chrF=g_svc->getCharacteristic(BLE_CHR_FLIGHT);
    g_chrT=g_svc->getCharacteristic(BLE_CHR_TRAFFIC);
    g_chrA=g_svc->getCharacteristic(BLE_CHR_ALERTS);
    g_chrD=g_svc->getCharacteristic(BLE_CHR_DEBUG);
    if(g_chrS&&g_chrS->canNotify())g_chrS->registerForNotify(notifyS);
    if(g_chrF&&g_chrF->canNotify())g_chrF->registerForNotify(notifyF);
    if(g_chrT&&g_chrT->canNotify())g_chrT->registerForNotify(notifyT);
    if(g_chrA&&g_chrA->canNotify())g_chrA->registerForNotify(notifyA);
    if(g_chrD&&g_chrD->canNotify())g_chrD->registerForNotify(notifyD);
    return true;}
void startScan(){BLEScan*s=BLEDevice::getScan();s->setAdvertisedDeviceCallbacks(new ATCAdv());s->setActiveScan(true);s->start(5,false);}

static void cbPrev(lv_event_t* e){
    if(lv_event_get_code(e)==LV_EVENT_CLICKED){
        g_navPage=(g_page==0)?NUM_PAGES-1:g_page-1;g_navPending=true;}}
static void cbNext(lv_event_t* e){
    if(lv_event_get_code(e)==LV_EVENT_CLICKED){
        g_navPage=(g_page+1)%NUM_PAGES;g_navPending=true;}}

void buildPage1(){
    lv_obj_t* p=g_pages[0];
    r_title=mkLbl(p,"SEARCHING AT-CORE...",C_AMBER,&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,55);
    r_mode =mkLbl(p,"---",C_GREY,&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,85);
    r_gps  =mkStat(p, 82,128,"GPS ---",false);r_lte=mkStat(p,252,128,"LTE ---",false);
    r_sd   =mkStat(p, 82,168,"SD ---", false);r_ble=mkStat(p,252,168,"BLE",    false);
    r_flarm=mkStat(p, 82,208,"FLARM",  false);r_adsb=mkStat(p,252,208,"ADS-B", false);
    r_coords=mkLbl(p,"--- / ---",C_WHITE,&lv_font_montserrat_16,LV_ALIGN_TOP_MID,0,255);
    mkLbl(p,"1/5",C_GREY,&lv_font_montserrat_16,LV_ALIGN_BOTTOM_MID,0,-60);}

void buildPage2(){
    lv_obj_t* p=g_pages[1];
    r_phase=mkLbl(p,"GROUND",C_WHITE,&lv_font_montserrat_22,LV_ALIGN_TOP_MID,0,55);
    r_alt  =mkLbl(p,"---",C_WHITE,&lv_font_montserrat_32,LV_ALIGN_TOP_LEFT,88,95);
    mkLbl(p,"m",C_GREY,&lv_font_montserrat_20,LV_ALIGN_TOP_LEFT,88,131);
    r_spd  =mkLbl(p,"---",C_WHITE,&lv_font_montserrat_32,LV_ALIGN_TOP_RIGHT,-108,95);
    mkLbl(p,"kt",C_GREY,&lv_font_montserrat_20,LV_ALIGN_TOP_RIGHT,-108,131);
    r_hdg  =mkLbl(p,"---",C_WHITE,&lv_font_montserrat_22,LV_ALIGN_TOP_MID,0,100);
    mkLbl(p,"HDG",C_GREY,&lv_font_montserrat_16,LV_ALIGN_TOP_MID,0,128);
    mkKV(p,178,"G-FORCE",&r_gforce,"--- G",C_WHITE);
    mkKV(p,208,"CO",     &r_co,   "--- ppm",C_WHITE);
    mkKV(p,238,"RPM",    &r_rpm,  "---",C_WHITE);
    mkLbl(p,"2/5",C_GREY,&lv_font_montserrat_16,LV_ALIGN_BOTTOM_MID,0,-60);}

void buildPage3(){
    lv_obj_t* p=g_pages[2];
    mkLbl(p,"TRAFFIC",C_BLUE,&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,55);
    r_noTraffic=mkLbl(p,"No traffic detected",C_WHITE,&lv_font_montserrat_16,LV_ALIGN_CENTER,0,-10);
    for(int i=0;i<MAX_TRF;i++){int y=100+i*38;
        r_trf[i].dot =mkLblP(p,"●",C_GREEN,&lv_font_montserrat_16,80,y);
        r_trf[i].cs  =mkLblP(p,"------",C_WHITE,&lv_font_montserrat_16,100,y);
        r_trf[i].dist=mkLbl(p,"-.--km",C_GREEN,&lv_font_montserrat_16,LV_ALIGN_TOP_MID,0,y);
        r_trf[i].alt =mkLbl(p,"---m",C_WHITE,&lv_font_montserrat_16,LV_ALIGN_TOP_RIGHT,-80,y);
        lv_obj_add_flag(r_trf[i].dot,LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r_trf[i].cs, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r_trf[i].dist,LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(r_trf[i].alt, LV_OBJ_FLAG_HIDDEN);}
    mkLbl(p,"white=SafeSky  amber=invisible",C_GREY,&lv_font_montserrat_16,LV_ALIGN_BOTTOM_MID,0,-80);
    mkLbl(p,"3/5",C_GREY,&lv_font_montserrat_16,LV_ALIGN_BOTTOM_MID,0,-60);}

void buildPage4(){
    lv_obj_t* p=g_pages[3];
    mkLbl(p,"! ALERTS !",C_RED,&lv_font_montserrat_22,LV_ALIGN_TOP_MID,0,55);
    r_noAlert =mkLbl(p,"No active alert",C_GREEN,&lv_font_montserrat_20,LV_ALIGN_CENTER,0,-20);
    r_alertCO =mkLblP(p,"▲ CARBON MONOXIDE",C_RED,&lv_font_montserrat_20,78,110);
    r_alertGF =mkLblP(p,"▲ ABNORMAL G-FORCE",C_RED,&lv_font_montserrat_20,78,156);
    r_alertRPM=mkLblP(p,"▲ RPM LIMIT REACHED",C_RED,&lv_font_montserrat_20,78,202);
    r_alertTFC=mkLblP(p,"▲ TRAFFIC NEARBY",C_RED,&lv_font_montserrat_20,78,248);
    r_alertMsg=mkLbl(p,"",C_AMBER,&lv_font_montserrat_16,LV_ALIGN_TOP_MID,0,300);
    lv_obj_add_flag(r_alertCO,LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_alertGF,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(r_alertRPM,LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_alertTFC,LV_OBJ_FLAG_HIDDEN);
    mkLbl(p,"4/5",C_GREY,&lv_font_montserrat_16,LV_ALIGN_BOTTOM_MID,0,-60);}

void buildPage5(){
    lv_obj_t* p=g_pages[4];
    mkLbl(p,"DEBUG SIM7600",C_BLUE,&lv_font_montserrat_16,LV_ALIGN_TOP_MID,0,58);
    int y=86,dy=26;
    r_hbgps=mkDbgL(p,y,"HB GPS","---",C_GREY);r_hblte=mkDbgR(p,y,"HB LTE","---",C_GREY);y+=dy;
    r_hbsd =mkDbgL(p,y,"HB SD", "---",C_GREY);r_p5csq=mkDbgR(p,y,"CSQ",   "---",C_GREY);y+=dy;
    r_http =mkDbgL(p,y,"HTTP",  "---",C_GREY);r_code =mkDbgR(p,y,"CODE",  "---",C_GREY);y+=dy;
    r_ss   =mkDbgL(p,y,"SafeSky","---",C_GREY);r_fa  =mkDbgR(p,y,"FastAPI","---",C_GREY);y+=dy;
    r_lteok=mkDbgL(p,y,"LTE",  "---",C_GREY);r_dis  =mkDbgR(p,y,"DIS",   "---",C_GREY);y+=dy;
    r_heap =mkDbgL(p,y,"HEAP", "---",C_GREY);r_bat  =mkDbgR(p,y,"BAT",   "---",C_GREY);y+=dy;
    r_p5mode=mkDbgL(p,y,"MODE","---",C_GREY);r_pend =mkDbgR(p,y,"PEND",  "---",C_GREY);y+=dy;
    r_flarmtx=mkDbgL(p,y,"FLARM","T0 R0",C_GREY);r_adsbr=mkDbgR(p,y,"ADSB","0",C_GREY);y+=dy;
    mkLblP(p,"FLT",C_GREY,&lv_font_montserrat_14,80,y);
    r_flt=mkLblP(p,"---",C_WHITE,&lv_font_montserrat_14,158,y);
    mkLbl(p,"5/5",C_GREY,&lv_font_montserrat_16,LV_ALIGN_BOTTOM_MID,0,-60);}

void updateAllPages(){
    char b[32];
    // P1
    lv_label_set_text(r_title,g_connected?"AT-CORE":"SEARCHING AT-CORE...");
    lv_obj_set_style_text_color(r_title,g_connected?C_AMBER:C_GREY,0);
    if(g_status.valid){
        lv_label_set_text(r_mode,modeStr(g_status.mode));
        lv_obj_set_style_text_color(r_mode,modeCol(g_status.mode),0);
        snprintf(b,32,"GPS %dsat",g_status.gps_sat);updStat(r_gps,b,g_status.gps_fix);
        snprintf(b,32,"LTE %d",g_status.csq);updStat(r_lte,b,g_status.csq>5);
        snprintf(b,32,"SD %dfr",g_status.frames);updStat(r_sd,b,g_status.sd_ok);
        updStat(r_ble,"BLE",g_connected);updStat(r_flarm,"FLARM",g_status.flarm_ok);
        updStat(r_adsb,"ADS-B",g_status.adsb_ok);
        if(g_status.gps_fix){snprintf(b,32,"%.4f / %.4f",g_status.lat,g_status.lon);lv_label_set_text(r_coords,b);}
    }else{updStat(r_ble,"BLE",g_connected);}
    // P2
    if(g_status.valid){
        snprintf(b,32,"%d",g_status.alt);lv_label_set_text(r_alt,b);
        snprintf(b,32,"%d",g_status.spd);lv_label_set_text(r_spd,b);
        snprintf(b,32,"%d",g_status.hdg);lv_label_set_text(r_hdg,b);}
    if(g_flight.valid){
        lv_label_set_text(r_phase,phaseStr(g_flight.phase));
        lv_obj_set_style_text_color(r_phase,phaseCol(g_flight.phase),0);
        snprintf(b,32,"%.2f G",g_flight.gforce_z);lv_label_set_text(r_gforce,b);
        lv_obj_set_style_text_color(r_gforce,(g_flight.gforce_z>3.5f||g_flight.gforce_z<-0.5f)?C_RED:C_WHITE,0);
        snprintf(b,32,"%d ppm",g_flight.co_ppm);lv_label_set_text(r_co,b);
        lv_obj_set_style_text_color(r_co,g_flight.co_ppm>50?C_RED:g_flight.co_ppm>25?C_AMBER:C_WHITE,0);
        snprintf(b,32,"%d",g_flight.rpm);lv_label_set_text(r_rpm,b);
        lv_obj_set_style_text_color(r_rpm,g_flight.rpm>2700?C_RED:C_WHITE,0);}
    // P3
    if(g_traffic.valid){
        lv_label_set_text(r_noTraffic,g_traffic.count==0?"No traffic detected":"");
        for(int i=0;i<MAX_TRF;i++){
            if(i<g_traffic.count){TrafficEntry& e=g_traffic.t[i];
                lv_color_t dc=e.dist_m<1000?C_RED:e.dist_m<3000?C_AMBER:C_GREEN;
                lv_obj_set_style_text_color(r_trf[i].dot,dc,0);
                lv_label_set_text(r_trf[i].cs,e.cs);
                lv_obj_set_style_text_color(r_trf[i].cs,e.visible?C_WHITE:C_AMBER,0);
                snprintf(b,32,"%d.%dkm",e.dist_m/1000,(e.dist_m%1000)/100);
                lv_label_set_text(r_trf[i].dist,b);lv_obj_set_style_text_color(r_trf[i].dist,dc,0);
                snprintf(b,32,"%dm",e.alt_m);lv_label_set_text(r_trf[i].alt,b);
                lv_obj_clear_flag(r_trf[i].dot,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r_trf[i].cs, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r_trf[i].dist,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(r_trf[i].alt, LV_OBJ_FLAG_HIDDEN);
            }else{
                lv_obj_add_flag(r_trf[i].dot,LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_trf[i].cs, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_trf[i].dist,LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_trf[i].alt, LV_OBJ_FLAG_HIDDEN);}}}
    // P4
    if(g_alert.valid){
        bool any=g_alert.co||g_alert.gforce||g_alert.rpm||g_alert.traffic;
        any?lv_obj_add_flag(r_noAlert,LV_OBJ_FLAG_HIDDEN):lv_obj_clear_flag(r_noAlert,LV_OBJ_FLAG_HIDDEN);
        g_alert.co?lv_obj_clear_flag(r_alertCO,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertCO,LV_OBJ_FLAG_HIDDEN);
        g_alert.gforce?lv_obj_clear_flag(r_alertGF,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertGF,LV_OBJ_FLAG_HIDDEN);
        g_alert.rpm?lv_obj_clear_flag(r_alertRPM,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertRPM,LV_OBJ_FLAG_HIDDEN);
        g_alert.traffic?lv_obj_clear_flag(r_alertTFC,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertTFC,LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(r_alertMsg,g_alert.msg);}
    // P5
    if(g_debug.valid){const char* modes[4]={"PRE","FLT","POST","SLP"};
        snprintf(b,32,"%ds",g_debug.hb_gps);lv_label_set_text(r_hbgps,b);lv_obj_set_style_text_color(r_hbgps,hbCol(g_debug.hb_gps),0);
        snprintf(b,32,"%ds",g_debug.hb_lte);lv_label_set_text(r_hblte,b);lv_obj_set_style_text_color(r_hblte,hbCol(g_debug.hb_lte),0);
        snprintf(b,32,"%ds",g_debug.hb_sd); lv_label_set_text(r_hbsd,b); lv_obj_set_style_text_color(r_hbsd,hbCol(g_debug.hb_sd),0);
        snprintf(b,32,"%d",g_debug.csq);    lv_label_set_text(r_p5csq,b);lv_obj_set_style_text_color(r_p5csq,g_debug.csq>=10?C_GREEN:C_AMBER,0);
        snprintf(b,32,"%dms",g_debug.http_ms);lv_label_set_text(r_http,b);lv_obj_set_style_text_color(r_http,g_debug.http_ms<2000?C_GREEN:C_RED,0);
        snprintf(b,32,"%d",g_debug.code);   lv_label_set_text(r_code,b); lv_obj_set_style_text_color(r_code,(g_debug.code==200||g_debug.code==201)?C_GREEN:C_RED,0);
        snprintf(b,32,"%ds",g_debug.ss_ago);lv_label_set_text(r_ss,b);   lv_obj_set_style_text_color(r_ss,g_debug.ss_ago<15?C_GREEN:C_RED,0);
        snprintf(b,32,"%ds",g_debug.fa_ago);lv_label_set_text(r_fa,b);   lv_obj_set_style_text_color(r_fa,g_debug.fa_ago<15?C_GREEN:C_RED,0);
        lv_label_set_text(r_lteok,g_debug.lte_ok?"OK":"FAIL");lv_obj_set_style_text_color(r_lteok,g_debug.lte_ok?C_GREEN:C_RED,0);
        lv_label_set_text(r_dis,g_debug.disable_lte?"ON":"OFF");lv_obj_set_style_text_color(r_dis,g_debug.disable_lte?C_AMBER:C_GREEN,0);
        snprintf(b,32,"%dkB",g_debug.heap/1024);lv_label_set_text(r_heap,b);lv_obj_set_style_text_color(r_heap,g_debug.heap>100000?C_GREEN:g_debug.heap>50000?C_AMBER:C_RED,0);
        if(g_debug.bat_pct<0){lv_label_set_text(r_bat,"---");lv_obj_set_style_text_color(r_bat,C_GREY,0);}
        else{snprintf(b,32,"%d%%",g_debug.bat_pct);lv_label_set_text(r_bat,b);lv_obj_set_style_text_color(r_bat,g_debug.bat_pct>40?C_GREEN:g_debug.bat_pct>20?C_AMBER:C_RED,0);}
        lv_label_set_text(r_p5mode,modes[min(g_debug.mode,3)]);lv_obj_set_style_text_color(r_p5mode,modeCol(g_debug.mode),0);
        snprintf(b,32,"%d",g_debug.pending);lv_label_set_text(r_pend,b);lv_obj_set_style_text_color(r_pend,g_debug.pending==0?C_GREEN:C_AMBER,0);
        snprintf(b,32,"T%d R%d",g_debug.flarm_tx,g_debug.flarm_rx);lv_label_set_text(r_flarmtx,b);
        lv_obj_set_style_text_color(r_flarmtx,(g_debug.flarm_tx>0||g_debug.flarm_rx>0)?C_GREEN:C_GREY,0);
        snprintf(b,32,"%d",g_debug.adsb_rx);lv_label_set_text(r_adsbr,b);
        lv_obj_set_style_text_color(r_adsbr,g_debug.adsb_rx>0?C_GREEN:C_GREY,0);
        lv_label_set_text(r_flt,g_debug.fid);}}

bool hasAlert(){return g_alert.valid&&(g_alert.co||g_alert.gforce||g_alert.rpm||g_alert.traffic);}

void switchPage(uint8_t np){
    lv_obj_add_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);
    g_page=np;lv_obj_clear_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);}

void createNavButtons(){
    lv_obj_t* btnL=lv_btn_create(lv_scr_act());lv_obj_set_size(btnL,80,240);
    lv_obj_align(btnL,LV_ALIGN_LEFT_MID,0,0);
    lv_obj_set_style_bg_opa(btnL,LV_OPA_TRANSP,LV_PART_MAIN);
    lv_obj_set_style_border_opa(btnL,LV_OPA_TRANSP,LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(btnL,LV_OPA_TRANSP,LV_PART_MAIN);
    lv_obj_add_event_cb(btnL,cbPrev,LV_EVENT_ALL,NULL);
    lv_obj_t* la=lv_label_create(btnL);lv_label_set_text(la,"<");
    lv_obj_set_style_text_color(la,lv_color_hex(0xCCCCCC),0);
    lv_obj_set_style_text_font(la,&lv_font_montserrat_32,0);lv_obj_center(la);

    lv_obj_t* btnR=lv_btn_create(lv_scr_act());lv_obj_set_size(btnR,80,240);
    lv_obj_align(btnR,LV_ALIGN_RIGHT_MID,0,0);
    lv_obj_set_style_bg_opa(btnR,LV_OPA_TRANSP,LV_PART_MAIN);
    lv_obj_set_style_border_opa(btnR,LV_OPA_TRANSP,LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(btnR,LV_OPA_TRANSP,LV_PART_MAIN);
    lv_obj_add_event_cb(btnR,cbNext,LV_EVENT_ALL,NULL);
    lv_obj_t* ra=lv_label_create(btnR);lv_label_set_text(ra,">");
    lv_obj_set_style_text_color(ra,lv_color_hex(0xCCCCCC),0);
    lv_obj_set_style_text_font(ra,&lv_font_montserrat_32,0);lv_obj_center(ra);}

void setup(){
    Serial.begin(115200);
    if(!panel.begin()){while(1){Serial.println("Panel FAIL");delay(1000);}}
    Serial.printf("Touch: %s\n",panel.getTouchModelName());
    beginLvglHelper(panel);
    lv_obj_set_style_bg_color(lv_scr_act(),C_BG,0);
    for(int i=0;i<NUM_PAGES;i++){g_pages[i]=mkPage();lv_obj_add_flag(g_pages[i],LV_OBJ_FLAG_HIDDEN);}
    buildPage1();buildPage2();buildPage3();buildPage4();buildPage5();
    lv_obj_clear_flag(g_pages[0],LV_OBJ_FLAG_HIDDEN);
    createNavButtons();
    BLEDevice::init("ATCORE-TRGB");startScan();
    Serial.println("BLE scan...");
    panel.setBrightness(16);
    Serial.println("Ready");}

void loop(){
    uint32_t now=millis();
    if(g_doReconnect){g_doReconnect=false;startScan();}
    if(g_doConnect&&!g_connected){g_doConnect=false;
        if(connectBLE())Serial.println("[BLE] OK");
        else{delay(2000);startScan();}}
    if(!g_connected&&!g_doConnect){
        static uint32_t ls=0;if(now-ls>8000){ls=now;startScan();}}
    if(g_dataUpdated){g_dataUpdated=false;updateAllPages();}
    bool alert=hasAlert();
    if(alert&&g_page!=3&&!g_alertForced){g_prevPage=g_page;g_alertForced=true;switchPage(3);}
    else if(!alert&&g_alertForced){g_alertForced=false;switchPage(g_prevPage);}
    if(g_navPending){g_navPending=false;switchPage(g_navPage);}
    lv_timer_handler();delay(2);}

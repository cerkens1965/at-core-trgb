/**
 * AT-VIEW AeroTrace — v0.6
 * LilyGo T-RGB 2.8" | ESP32-S3 | 480×480 circular
 * 3 pages: Status(boot+live) | Radar | Settings  (+hidden Debug)
 * Swipe L/R — long press version → Debug
 * Christophe — 2026-05-04
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>
#include "img_vl3.h"

LilyGo_RGBPanel panel;

#define BLE_SVC_UUID    "4FAFC201-1FB5-459E-8FCC-C5C9C331914B"
#define BLE_CHR_STATUS  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_FLIGHT  "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_TRAFFIC "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_ALERTS  "6E400006-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_DEBUG   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TARGET      "AT-CORE"

#define C_AMBER  lv_color_hex(0xF5A623)
#define C_GREEN  lv_color_hex(0x22c55e)
#define C_CYAN   lv_color_hex(0x00E5FF)
#define C_BLUE   lv_color_hex(0x60a5fa)
#define C_RED    lv_color_hex(0xef4444)
#define C_ORANGE lv_color_hex(0xf97316)

static bool g_dark_theme = true;
static inline lv_color_t TBG()  {return g_dark_theme?lv_color_hex(0x000000):lv_color_hex(0xFFFFFF);}
static inline lv_color_t TFG()  {return g_dark_theme?lv_color_hex(0xFFFFFF):lv_color_hex(0x0f172a);}
static inline lv_color_t TGREY(){return g_dark_theme?lv_color_hex(0x6b7280):lv_color_hex(0x6b7280);}
static inline lv_color_t TGRID(){return g_dark_theme?lv_color_hex(0x2a2a2a):lv_color_hex(0xd0d0d0);}
static inline lv_color_t TRING(){return g_dark_theme?lv_color_hex(0x555555):lv_color_hex(0x9ca3af);}
static inline lv_color_t THDG() {return g_dark_theme?lv_color_hex(0x0d1b2a):lv_color_hex(0xe2e8f0);}

// ── Data structs ──────────────────────────────────────────────────────────────
struct StatusData {
    int mode,gps_sat,csq,frames,alt,spd,hdg; float lat,lon;
    bool gps_fix,sd_ok,flarm_ok,adsb_ok,valid; };
struct FlightData  { float gforce_z; int co_ppm,rpm,phase; bool valid; };
#define MAX_TRF 5
struct TrafficEntry { char cs[9]; int dist_m,alt_m,bear_deg,hdg_deg,spd_kt; bool visible; };
struct TrafficData  { TrafficEntry t[MAX_TRF]; int count; bool valid; };
struct AlertData    { bool co,gforce,rpm,traffic; char msg[64]; bool valid; };
struct DebugData    {
    int hb_gps,hb_lte,hb_sd,csq,http_ms,code;
    bool lte_ok,disable_lte;
    int ss_ago,fa_ago,heap,bat_pct,mode,pending,flarm_tx,flarm_rx,adsb_rx;
    char fid[24]; bool valid; };

static const uint8_t kScaleOpts[]={4,8,10,20,40};
static const char*   kSrcNames[] ={"SSKY","FLRM","ADSB","ALL"};
struct CfgData { uint8_t scale_nm,brightness,trf_src; bool dist_nm,alt_ft,dark; int16_t vfilt_ft; };
static CfgData     g_cfg={4,16,3,true,true,true,2000};
static Preferences g_prefs;

static StatusData  g_status  = {};
static FlightData  g_flight  = {};
static TrafficData g_traffic = {};
static AlertData   g_alert   = {};
static DebugData   g_debug   = {};
static volatile bool g_dataUpdated = false;

// ── BLE state ─────────────────────────────────────────────────────────────────
static BLEClient*              g_client = nullptr;
static BLERemoteService*       g_svc    = nullptr;
static BLERemoteCharacteristic *g_chrS=nullptr,*g_chrF=nullptr,
                                *g_chrT=nullptr,*g_chrA=nullptr,*g_chrD=nullptr;
static volatile bool g_connected=false, g_doConnect=false, g_doReconnect=false;
static BLEAdvertisedDevice*    g_target = nullptr;

// ── Pages ─────────────────────────────────────────────────────────────────────
#define NUM_PAGES 3
static lv_obj_t* g_pages[NUM_PAGES];
static lv_obj_t* g_dbgPage  = nullptr;
static uint8_t   g_page=0, g_prevPage=0;
static bool      g_inDebug=false, g_alertForced=false;
static volatile bool    g_navPending=false;
static volatile uint8_t g_navPage=0;
static bool             g_rebuildPages=false;
static bool             g_bootDone=false;
static bool             g_autoNavDone=false;

// ── Widget refs — Status page (page 0) ───────────────────────────────────────
static lv_obj_t *r_title;
static lv_obj_t *r_boot_panel,*r_boot_lvgl,*r_boot_ble,*r_boot_core;
static lv_obj_t *r_gps,*r_lte,*r_sd,*r_ble,*r_flarm,*r_adsb,*r_coords;

// ── Widget refs — Radar (page 1) ──────────────────────────────────────────────
#define RAD_CX 240
#define RAD_CY 240
#define RAD_R  175
static lv_obj_t *r_radar_hdg, *r_radar_scale_lbl;
static lv_obj_t *r_card[4];
static lv_obj_t *r_radar_cs[MAX_TRF],*r_radar_alt[MAX_TRF];
static lv_obj_t *r_trf_img[MAX_TRF],*r_trf_vect[MAX_TRF];
static lv_point_t r_vect_pts[MAX_TRF][2];
static lv_obj_t *r_alert_overlay, *r_aov_text;
static lv_obj_t *r_co_val, *r_co_ball, *r_co_text;
static lv_obj_t *r_hdr_bat;
static lv_obj_t *r_hdr_gps, *r_hdr_lte, *r_hdr_wifi, *r_hdr_ble;
static lv_obj_t *r_hdr_lte_b[4];  // 4 drawn signal bars inside LTE pill

// ── Widget refs — Settings (page 2) ───────────────────────────────────────────
static lv_obj_t *s_scale_v,*s_vfilt_v,*s_dist_v,*s_alt_v,*s_bright_v,*s_src_v,*s_theme_v;

// ── Widget refs — Debug (hidden) ──────────────────────────────────────────────
static lv_obj_t *r_hbgps,*r_hblte,*r_hbsd,*r_p5csq,*r_http,*r_code;
static lv_obj_t *r_ss,*r_fa,*r_lteok,*r_dis,*r_heap,*r_bat,*r_p5mode,*r_pend;
static lv_obj_t *r_flarmtx,*r_adsbr,*r_flt;

// ── Helpers ───────────────────────────────────────────────────────────────────
lv_color_t modeCol(int m){switch(m){case 0:return C_AMBER;case 1:return C_GREEN;case 2:return C_BLUE;default:return TGREY();}}
lv_color_t hbCol(int s){return s<10?C_GREEN:s<20?C_AMBER:C_RED;}

lv_obj_t* mkLbl(lv_obj_t*p,const char*t,lv_color_t c,const lv_font_t*f,lv_align_t a,int ox,int oy){
    lv_obj_t*l=lv_label_create(p);lv_label_set_text(l,t);
    lv_obj_set_style_text_color(l,c,0);lv_obj_set_style_text_font(l,f,0);
    lv_obj_align(l,a,ox,oy);return l;}
lv_obj_t* mkLblP(lv_obj_t*p,const char*t,lv_color_t c,const lv_font_t*f,int x,int y){
    lv_obj_t*l=lv_label_create(p);lv_label_set_text(l,t);
    lv_obj_set_style_text_color(l,c,0);lv_obj_set_style_text_font(l,f,0);
    lv_obj_set_pos(l,x,y);return l;}
lv_obj_t* mkStat(lv_obj_t*p,int x,int y,const char*t,bool ok){
    char b[32];snprintf(b,32,"● %s",t);
    lv_obj_t*l=lv_label_create(p);lv_label_set_text(l,b);
    lv_obj_set_style_text_color(l,ok?C_GREEN:C_RED,0);
    lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
    lv_obj_set_pos(l,x,y);return l;}
void updStat(lv_obj_t*l,const char*t,bool ok){
    char b[32];snprintf(b,32,"● %s",t);
    lv_label_set_text(l,b);lv_obj_set_style_text_color(l,ok?C_GREEN:C_RED,0);}
lv_obj_t* mkDbgL(lv_obj_t*p,int y,const char*k,const char*v,lv_color_t c){
    mkLblP(p,k,TGREY(),&lv_font_montserrat_14,80,y);
    return mkLblP(p,v,c,&lv_font_montserrat_14,158,y);}
lv_obj_t* mkDbgR(lv_obj_t*p,int y,const char*k,const char*v,lv_color_t c){
    mkLblP(p,k,TGREY(),&lv_font_montserrat_14,252,y);
    return mkLblP(p,v,c,&lv_font_montserrat_14,330,y);}
lv_obj_t* mkPage(){
    lv_obj_t*p=lv_obj_create(lv_scr_act());lv_obj_set_size(p,480,480);
    lv_obj_set_pos(p,0,0);lv_obj_set_style_bg_color(p,TBG(),0);
    lv_obj_set_style_border_width(p,0,0);lv_obj_set_style_pad_all(p,0,0);
    lv_obj_clear_flag(p,LV_OBJ_FLAG_SCROLLABLE);return p;}
// Tab pill: 68×20 capsule partly outside the bezel — returns the inner label ref
// Positions are calculated so the pill extends ~10px beyond the display circle edge,
// creating a demi-capsule "tab stuck to the bezel" effect.
lv_obj_t* mkTabPill(lv_obj_t*p,const char*t,lv_color_t c,int x,int y){
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,68,20);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_color(b,THDG(),0);lv_obj_set_style_bg_opa(b,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(b,TRING(),0);lv_obj_set_style_border_width(b,1,0);
    lv_obj_set_style_radius(b,10,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,t);
    lv_obj_set_style_text_color(l,c,0);lv_obj_set_style_text_font(l,&lv_font_montserrat_12,0);
    lv_obj_center(l);return l;}
// LTE pill: 4 drawn signal bars (bottom-aligned) + "LTE" label, returns label ref
lv_obj_t* mkLTEPill(lv_obj_t*p,int x,int y){
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,68,20);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_color(b,THDG(),0);lv_obj_set_style_bg_opa(b,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(b,TRING(),0);lv_obj_set_style_border_width(b,1,0);
    lv_obj_set_style_radius(b,10,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    static const int8_t bh[4]={4,7,10,13};
    for(int i=0;i<4;i++){
        r_hdr_lte_b[i]=lv_obj_create(b);lv_obj_set_size(r_hdr_lte_b[i],3,bh[i]);
        lv_obj_set_pos(r_hdr_lte_b[i],10+i*4,16-bh[i]);
        lv_obj_set_style_radius(r_hdr_lte_b[i],1,0);
        lv_obj_set_style_bg_color(r_hdr_lte_b[i],TGREY(),0);lv_obj_set_style_bg_opa(r_hdr_lte_b[i],LV_OPA_COVER,0);
        lv_obj_set_style_border_width(r_hdr_lte_b[i],0,0);lv_obj_set_style_shadow_opa(r_hdr_lte_b[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_pad_all(r_hdr_lte_b[i],0,0);
        lv_obj_clear_flag(r_hdr_lte_b[i],LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);}
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,"LTE");
    lv_obj_set_style_text_color(l,TGREY(),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_12,0);
    lv_obj_set_pos(l,30,3);return l;}
// Flash a tab pill (3× blink) when its status becomes active
static void _flash_cb(void*var,int32_t v){lv_obj_set_style_opa((lv_obj_t*)var,(lv_opa_t)v,0);}
void flashTab(lv_obj_t*lbl){
    lv_obj_t*pill=lv_obj_get_parent(lbl);
    lv_anim_t a;lv_anim_init(&a);lv_anim_set_var(&a,pill);
    lv_anim_set_exec_cb(&a,(lv_anim_exec_xcb_t)_flash_cb);
    lv_anim_set_values(&a,LV_OPA_COVER,LV_OPA_20);
    lv_anim_set_time(&a,160);lv_anim_set_playback_time(&a,160);
    lv_anim_set_repeat_count(&a,3);lv_anim_start(&a);}

// ── Parsers ───────────────────────────────────────────────────────────────────
void parseStatus(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_status.mode=d["mode"]|0;g_status.gps_sat=d["gps_sat"]|0;g_status.csq=d["csq"]|-1;
    g_status.frames=d["frames"]|0;g_status.alt=d["alt"]|0;g_status.spd=d["spd"]|0;
    g_status.hdg=d["hdg"]|0;g_status.lat=d["lat"]|0.0f;g_status.lon=d["lon"]|0.0f;
    g_status.gps_fix=d["gps_fix"]|false;g_status.sd_ok=d["sd_ok"]|false;
    g_status.flarm_ok=d["flarm"]|false;g_status.adsb_ok=d["adsb"]|false;
    g_status.valid=true;g_dataUpdated=true;}
void parseFlight(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_flight.gforce_z=d["gf"]|1.0f;g_flight.co_ppm=d["co"]|0;
    g_flight.rpm=d["rpm"]|0;g_flight.phase=d["phase"]|0;
    g_flight.valid=true;g_dataUpdated=true;}
void parseTraffic(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_traffic.count=min((int)(d["count"]|0),MAX_TRF);
    for(int i=0;i<g_traffic.count;i++){
        strlcpy(g_traffic.t[i].cs,d["t"][i]["cs"]|"???",9);
        g_traffic.t[i].dist_m=d["t"][i]["d"]|0;g_traffic.t[i].alt_m=d["t"][i]["a"]|0;
        g_traffic.t[i].bear_deg=d["t"][i]["b"]|0;g_traffic.t[i].hdg_deg=d["t"][i]["c"]|0;
        g_traffic.t[i].spd_kt=d["t"][i]["s"]|100;
        g_traffic.t[i].visible=d["t"][i]["v"]|true;}
    g_traffic.valid=true;g_dataUpdated=true;}
void parseAlerts(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_alert.co=d["co"]|false;g_alert.gforce=d["gf"]|false;
    g_alert.rpm=d["rpm"]|false;g_alert.traffic=d["tfc"]|false;
    strlcpy(g_alert.msg,d["msg"]|"",64);g_alert.valid=true;g_dataUpdated=true;}
void parseDebug(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_debug.hb_gps=d["hb_gps"]|0;g_debug.hb_lte=d["hb_lte"]|0;g_debug.hb_sd=d["hb_sd"]|0;
    g_debug.csq=d["csq"]|0;g_debug.http_ms=d["http_ms"]|0;g_debug.code=d["code"]|0;
    g_debug.lte_ok=d["lte_ok"]|false;g_debug.disable_lte=d["dis_lte"]|false;
    g_debug.ss_ago=d["ss_ago"]|0;g_debug.fa_ago=d["fa_ago"]|0;
    g_debug.heap=d["heap"]|0;g_debug.bat_pct=d["bat_pct"]|-1;
    g_debug.mode=d["mode"]|0;g_debug.pending=d["pending"]|0;
    g_debug.flarm_tx=d["ftx"]|0;g_debug.flarm_rx=d["frx"]|0;g_debug.adsb_rx=d["adsb_rx"]|0;
    strlcpy(g_debug.fid,d["fid"]|"---",24);g_debug.valid=true;g_dataUpdated=true;}

// ── BLE ───────────────────────────────────────────────────────────────────────
#define BLE_BUF 512
static void notifyS(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseStatus(b);}
static void notifyF(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseFlight(b);}
static void notifyT(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseTraffic(b);}
static void notifyA(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseAlerts(b);}
static void notifyD(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseDebug(b);}

class ATCCB:public BLEClientCallbacks{
    void onConnect(BLEClient*)override{g_connected=true;g_dataUpdated=true;Serial.println("[BLE] Connected");}
    void onDisconnect(BLEClient*)override{g_connected=false;g_autoNavDone=false;
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

// ── Navigation & swipe ────────────────────────────────────────────────────────
void switchPage(uint8_t np){
    if(g_inDebug){lv_obj_add_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);g_inDebug=false;}
    lv_obj_add_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);
    g_page=np;lv_obj_clear_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);}

static lv_coord_t g_swipe_sx=-1, g_swipe_lx=0;
static void swipeCb(lv_event_t*e){
    lv_event_code_t code=lv_event_get_code(e);
    lv_indev_t*indev=lv_indev_get_act();if(!indev)return;
    lv_point_t pt;lv_indev_get_point(indev,&pt);
    if(code==LV_EVENT_PRESSED){g_swipe_sx=pt.x;g_swipe_lx=pt.x;}
    else if(code==LV_EVENT_PRESSING){g_swipe_lx=pt.x;}
    else if(code==LV_EVENT_RELEASED||code==LV_EVENT_PRESS_LOST){
        if(g_swipe_sx>=0){
            int dx=(int)g_swipe_lx-(int)g_swipe_sx;
            if(g_inDebug){
                if(abs(dx)>40){lv_obj_add_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);g_inDebug=false;}
            }else{
                if(dx>60){g_navPage=(g_page==0)?NUM_PAGES-1:g_page-1;g_navPending=true;}
                else if(dx<-60){g_navPage=(g_page+1)%NUM_PAGES;g_navPending=true;}}}
        g_swipe_sx=-1;}}

static void cbDebugLongPress(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_LONG_PRESSED)return;
    lv_obj_add_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);g_inDebug=true;}

// ── NVS ───────────────────────────────────────────────────────────────────────
void cfgLoad(){
    g_prefs.begin("atview",true);
    g_cfg.scale_nm  =g_prefs.getUChar("scale",4);
    g_cfg.brightness=g_prefs.getUChar("bright",16);
    g_cfg.trf_src   =g_prefs.getUChar("trf_src",3);
    g_cfg.dist_nm   =g_prefs.getBool("dist_nm",true);
    g_cfg.alt_ft    =g_prefs.getBool("alt_ft",true);
    g_cfg.dark      =g_prefs.getBool("dark",true);
    g_cfg.vfilt_ft  =g_prefs.getShort("vfilt",2000);
    g_prefs.end();
    g_dark_theme=g_cfg.dark;}
void cfgSave(){
    g_prefs.begin("atview",false);
    g_prefs.putUChar("scale",g_cfg.scale_nm);
    g_prefs.putUChar("bright",g_cfg.brightness);
    g_prefs.putUChar("trf_src",g_cfg.trf_src);
    g_prefs.putBool("dist_nm",g_cfg.dist_nm);
    g_prefs.putBool("alt_ft",g_cfg.alt_ft);
    g_prefs.putBool("dark",g_cfg.dark);
    g_prefs.putShort("vfilt",g_cfg.vfilt_ft);
    g_prefs.end();}

// ── Forward declarations ──────────────────────────────────────────────────────
void buildStatusPage();
void buildRadarPage();
void buildSettingsPage();
void buildDebugPage();
void createSwipeHandlers();
void updSetPage();

// ── Theme rebuild ─────────────────────────────────────────────────────────────
void rebuildAllPages(){
    g_dark_theme=g_cfg.dark;
    lv_obj_set_style_bg_color(lv_scr_act(),TBG(),0);
    for(int i=0;i<NUM_PAGES;i++){
        lv_obj_clean(g_pages[i]);
        lv_obj_set_style_bg_color(g_pages[i],TBG(),0);}
    lv_obj_clean(g_dbgPage);
    lv_obj_set_style_bg_color(g_dbgPage,TBG(),0);
    buildStatusPage();buildRadarPage();buildSettingsPage();buildDebugPage();
    createSwipeHandlers();updSetPage();}

// ── Page 0 — Status (boot section + live section, same page always) ───────────
void buildStatusPage(){
    lv_obj_t*p=g_pages[0];

    // ── Brand header
    r_title=mkLbl(p,"AT-VIEW",C_AMBER,&lv_font_montserrat_22,LV_ALIGN_TOP_MID,0,52);
    mkLbl(p,"AeroTrace",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,80);

    // ── Boot checks (4 lines, compact, frozen after boot)
    const char* bp_str = g_bootDone ? "● PANEL    OK" : "  PANEL    ...";
    const char* bl_str = g_bootDone ? "● LVGL     OK" : "  LVGL     ...";
    const char* bb_str = g_bootDone ? "● BLE      OK" : "  BLE      ...";
    const char* bc_str = g_bootDone ? "● AT-CORE  SCAN" : "  AT-CORE  ...";
    lv_color_t ok_col  = g_bootDone ? C_GREEN : TGREY();
    lv_color_t core_col= g_bootDone ? C_AMBER : TGREY();
    r_boot_panel=mkLblP(p,bp_str,ok_col,  &lv_font_montserrat_14,120,108);
    r_boot_lvgl =mkLblP(p,bl_str,ok_col,  &lv_font_montserrat_14,120,128);
    r_boot_ble  =mkLblP(p,bb_str,ok_col,  &lv_font_montserrat_14,120,148);
    r_boot_core =mkLblP(p,bc_str,core_col,&lv_font_montserrat_14,120,168);

    // ── Separator line
    static lv_point_t sep[2]={{100,192},{380,192}};
    lv_obj_t*sl=lv_line_create(p);lv_line_set_points(sl,sep,2);
    lv_obj_set_style_line_color(sl,TGRID(),0);lv_obj_set_style_line_width(sl,1,0);

    // ── Live status indicators (2 columns)
    r_gps  =mkStat(p, 82,202,"GPS ---",false);
    r_lte  =mkStat(p,252,202,"LTE ---",false);
    r_sd   =mkStat(p, 82,228,"SD ---", false);
    r_ble  =mkStat(p,252,228,"BLE",    false);
    r_flarm=mkStat(p, 82,254,"FLARM",  false);
    r_adsb =mkStat(p,252,254,"ADS-B",  false);
    r_coords=mkLbl(p,"--- / ---",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,284);
    mkLbl(p,"v0.6  —  2026-05-04",TGREY(),&lv_font_montserrat_12,LV_ALIGN_BOTTOM_MID,0,-60);}

// Boot animation — runs on page 0 before it goes live
// Does NOT clean the page — just animates the boot check labels
void runBootOnPage(){
    lv_obj_t*p=g_pages[0];

    lv_timer_handler();delay(500);

    // PANEL
    lv_label_set_text(r_boot_panel,"  PANEL    ...");
    lv_obj_set_style_text_color(r_boot_panel,TGREY(),0);lv_timer_handler();delay(350);
    lv_label_set_text(r_boot_panel,"● PANEL    OK");
    lv_obj_set_style_text_color(r_boot_panel,C_GREEN,0);lv_timer_handler();

    // LVGL
    lv_label_set_text(r_boot_lvgl,"  LVGL     ...");
    lv_obj_set_style_text_color(r_boot_lvgl,TGREY(),0);lv_timer_handler();delay(300);
    lv_label_set_text(r_boot_lvgl,"● LVGL     OK");
    lv_obj_set_style_text_color(r_boot_lvgl,C_GREEN,0);lv_timer_handler();

    // BLE
    lv_label_set_text(r_boot_ble,"  BLE      ...");
    lv_obj_set_style_text_color(r_boot_ble,TGREY(),0);lv_timer_handler();delay(300);
    BLEDevice::init("ATCORE-TRGB");
    lv_label_set_text(r_boot_ble,"● BLE      OK");
    lv_obj_set_style_text_color(r_boot_ble,C_GREEN,0);lv_timer_handler();

    // AT-CORE scan
    lv_label_set_text(r_boot_core,"  AT-CORE  ...");
    lv_obj_set_style_text_color(r_boot_core,TGREY(),0);lv_timer_handler();delay(400);
    startScan();
    lv_label_set_text(r_boot_core,"● AT-CORE  SCAN");
    lv_obj_set_style_text_color(r_boot_core,C_AMBER,0);lv_timer_handler();
    delay(600);
    g_bootDone=true;

    // Live status indicators are already on the page below — BLE/GPS/etc.
    // will update as soon as data arrives
}

// ── Page 1 — Radar ────────────────────────────────────────────────────────────
void buildRadarPage(){
    lv_obj_t*p=g_pages[1];

    // Heading pill (top centre)
    lv_obj_t*hb=lv_obj_create(p);lv_obj_set_size(hb,72,28);
    lv_obj_align(hb,LV_ALIGN_TOP_MID,0,28);
    lv_obj_set_style_bg_color(hb,THDG(),0);lv_obj_set_style_bg_opa(hb,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(hb,TFG(),0);lv_obj_set_style_border_width(hb,1,0);
    lv_obj_set_style_radius(hb,14,0);lv_obj_set_style_shadow_opa(hb,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(hb,0,0);lv_obj_clear_flag(hb,LV_OBJ_FLAG_SCROLLABLE);
    r_radar_hdg=lv_label_create(hb);lv_label_set_text(r_radar_hdg,"---°");
    lv_obj_set_style_text_color(r_radar_hdg,TFG(),0);
    lv_obj_set_style_text_font(r_radar_hdg,&lv_font_montserrat_16,0);lv_obj_center(r_radar_hdg);

    // Tab pills hugging the bezel arc (each pill extends ~10px outside the display circle)
    // Positions: right_edge = circle_boundary(y) + 10, left_edge = right_edge - 68
    // Left arc:  battery  @ y_c=33: left boundary=119 → pos x=109
    // Right arc: GPS      @ y_c=33: right boundary=361 → pos x=303
    //            LTE      @ y_c=57: right boundary=395 → pos x=337
    //            WiFi     @ y_c=81: right boundary=420 → pos x=362
    //            BLE      @ y_c=105: right boundary=438 → pos x=380
    r_hdr_bat  = mkTabPill(p, LV_SYMBOL_CHARGE,          TGREY(), 109, 23);
    r_hdr_gps  = mkTabPill(p, LV_SYMBOL_GPS " GPS",       TGREY(), 303, 23);
    r_hdr_lte  = mkLTEPill(p, 337, 47);
    r_hdr_wifi = mkTabPill(p, LV_SYMBOL_WIFI " WiFi",     TGREY(), 362, 71);
    r_hdr_ble  = mkTabPill(p, LV_SYMBOL_BLUETOOTH " BLE", TGREY(), 380, 95);

    // Outer ring
    lv_obj_t*ro=lv_obj_create(p);lv_obj_set_size(ro,RAD_R*2,RAD_R*2);
    lv_obj_set_pos(ro,RAD_CX-RAD_R,RAD_CY-RAD_R);lv_obj_set_style_radius(ro,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_opa(ro,LV_OPA_TRANSP,0);lv_obj_set_style_border_color(ro,TRING(),0);
    lv_obj_set_style_border_width(ro,1,0);lv_obj_set_style_shadow_opa(ro,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(ro,0,0);lv_obj_clear_flag(ro,LV_OBJ_FLAG_SCROLLABLE);

    // Inner ring (half scale)
    lv_obj_t*ri=lv_obj_create(p);lv_obj_set_size(ri,RAD_R,RAD_R);
    lv_obj_set_pos(ri,RAD_CX-RAD_R/2,RAD_CY-RAD_R/2);lv_obj_set_style_radius(ri,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_opa(ri,LV_OPA_TRANSP,0);lv_obj_set_style_border_color(ri,TGRID(),0);
    lv_obj_set_style_border_width(ri,1,0);lv_obj_set_style_shadow_opa(ri,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(ri,0,0);lv_obj_clear_flag(ri,LV_OBJ_FLAG_SCROLLABLE);

    // Tick marks — cardinal (every 90°) longer and brighter
    static lv_point_t tick_pts[12][2];
    for(int t=0;t<12;t++){
        float a=(float)t*30.0f*(float)M_PI/180.0f;
        int inner=(t%3==0)?RAD_R-14:RAD_R-8;
        tick_pts[t][0].x=(lv_coord_t)(RAD_CX+sinf(a)*(float)inner);
        tick_pts[t][0].y=(lv_coord_t)(RAD_CY-cosf(a)*(float)inner);
        tick_pts[t][1].x=(lv_coord_t)(RAD_CX+sinf(a)*(float)RAD_R);
        tick_pts[t][1].y=(lv_coord_t)(RAD_CY-cosf(a)*(float)RAD_R);
        lv_obj_t*tm=lv_line_create(p);lv_line_set_points(tm,tick_pts[t],2);
        lv_obj_set_style_line_color(tm,(t%3==0)?TFG():TRING(),0);
        lv_obj_set_style_line_width(tm,(t%3==0)?2:1,0);}

    // Cross lines (faint grid)
    static lv_point_t hpts[2]={{RAD_CX-RAD_R,RAD_CY},{RAD_CX+RAD_R,RAD_CY}};
    static lv_point_t vpts[2]={{RAD_CX,RAD_CY-RAD_R},{RAD_CX,RAD_CY+RAD_R}};
    lv_obj_t*hl=lv_line_create(p);lv_line_set_points(hl,hpts,2);
    lv_obj_set_style_line_color(hl,TGRID(),0);lv_obj_set_style_line_width(hl,1,0);
    lv_obj_t*vl=lv_line_create(p);lv_line_set_points(vl,vpts,2);
    lv_obj_set_style_line_color(vl,TGRID(),0);lv_obj_set_style_line_width(vl,1,0);

    // Own aircraft triangle — small and thin, no heading line
    static lv_point_t tri[4]={{RAD_CX,RAD_CY-14},{RAD_CX-8,RAD_CY+8},{RAD_CX+8,RAD_CY+8},{RAD_CX,RAD_CY-14}};
    lv_obj_t*ot=lv_line_create(p);lv_line_set_points(ot,tri,4);
    lv_obj_set_style_line_color(ot,C_GREEN,0);lv_obj_set_style_line_width(ot,2,0);

    // Cardinal labels N/E/S/W — INSIDE the ring at RAD_R-24, avoiding the pill zone
    const char*cnames[]={"N","E","S","W"};
    for(int ci=0;ci<4;ci++){
        r_card[ci]=lv_label_create(p);
        lv_label_set_text(r_card[ci],cnames[ci]);
        lv_obj_set_style_text_font(r_card[ci],&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(r_card[ci],ci==0?TFG():TRING(),0);
        lv_obj_set_pos(r_card[ci],RAD_CX-5,RAD_CY-(RAD_R-24)-8);}

    // Scale label — clearly below the ring (ring bottom ~y=415, label top ~y=446)
    char scl[12];snprintf(scl,12,"%dnm",g_cfg.scale_nm);
    r_radar_scale_lbl=mkLbl(p,scl,TGREY(),&lv_font_montserrat_14,LV_ALIGN_BOTTOM_MID,0,-20);

    // CO arc gauge — 3 fixed color bands (30° total) in bottom-right quadrant
    // LVGL arc convention: 0°=right(3h), increases CW → compass120°=LVGL30°, compass150°=LVGL60°
    // Each band 10°: green(30-40°) caution(40-50°) danger(50-60°)
    {
        struct Band{int s,e;lv_color_t c;};
        Band bs[]={{30,40,C_GREEN},{40,50,C_ORANGE},{50,60,C_RED}};
        for(auto&b:bs){
            lv_obj_t*ba=lv_arc_create(p);
            lv_obj_set_size(ba,440,440);lv_obj_set_pos(ba,20,20);
            lv_arc_set_bg_start_angle(ba,b.s);lv_arc_set_bg_end_angle(ba,b.e);
            lv_arc_set_range(ba,0,1);lv_arc_set_value(ba,0);
            lv_obj_set_style_arc_color(ba,b.c,LV_PART_MAIN);
            lv_obj_set_style_arc_width(ba,16,LV_PART_MAIN);
            lv_obj_set_style_arc_width(ba,0,LV_PART_INDICATOR);
            lv_obj_set_style_bg_opa(ba,LV_OPA_TRANSP,0);
            lv_obj_set_style_shadow_opa(ba,LV_OPA_TRANSP,0);
            lv_obj_set_style_pad_all(ba,0,0);
            lv_obj_clear_flag(ba,LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_style_opa(ba,LV_OPA_TRANSP,LV_PART_KNOB);}
    }
    // CO ball cursor — 12×12 circle sliding along the arc (LVGL 30°–60°) by CO ppm
    // Arc midline radius = 440/2 - 16/2 = 212px from center (240,240)
    r_co_ball=lv_obj_create(p);lv_obj_set_size(r_co_ball,12,12);
    lv_obj_set_style_radius(r_co_ball,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_color(r_co_ball,TBG(),0);lv_obj_set_style_bg_opa(r_co_ball,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(r_co_ball,TFG(),0);lv_obj_set_style_border_width(r_co_ball,2,0);
    lv_obj_set_style_shadow_opa(r_co_ball,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(r_co_ball,0,0);
    lv_obj_clear_flag(r_co_ball,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(r_co_ball,(int)(240+212*0.866f)-6,(int)(240+212*0.5f)-6);
    // CO text + ppm — OUTSIDE radar ring (r=175), at arc midpoint (LVGL 45°, r≈190)
    // x=240+190*cos45°=374, y=374 → label anchored just outside the ring
    r_co_text=lv_label_create(p);lv_label_set_text(r_co_text,"CO");
    lv_obj_set_style_text_color(r_co_text,TGREY(),0);
    lv_obj_set_style_text_font(r_co_text,&lv_font_montserrat_12,0);lv_obj_set_pos(r_co_text,366,364);
    r_co_val=lv_label_create(p);lv_label_set_text(r_co_val,"---");
    lv_obj_set_style_text_color(r_co_val,TGREY(),0);
    lv_obj_set_style_text_font(r_co_val,&lv_font_montserrat_12,0);lv_obj_set_pos(r_co_val,360,380);

    // Traffic VL3 icons (bitmap rotated) + speed vector + callsign + alt
    for(int i=0;i<MAX_TRF;i++){
        r_vect_pts[i][0]={RAD_CX,RAD_CY};r_vect_pts[i][1]={RAD_CX,RAD_CY};
        r_trf_img[i]=lv_img_create(p);
        lv_img_set_src(r_trf_img[i],&img_vl3);
        lv_img_set_pivot(r_trf_img[i],20,20);
        lv_obj_set_style_img_recolor(r_trf_img[i],C_CYAN,0);
        lv_obj_set_style_img_recolor_opa(r_trf_img[i],LV_OPA_COVER,0);
        lv_obj_set_style_shadow_opa(r_trf_img[i],LV_OPA_TRANSP,0);
        lv_obj_add_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);
        r_trf_vect[i]=lv_line_create(p);lv_line_set_points(r_trf_vect[i],r_vect_pts[i],2);
        lv_obj_set_style_line_color(r_trf_vect[i],C_AMBER,0);lv_obj_set_style_line_width(r_trf_vect[i],2,0);
        lv_obj_add_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
        r_radar_cs[i]=lv_label_create(p);lv_label_set_text(r_radar_cs[i],"");
        lv_obj_set_style_text_font(r_radar_cs[i],&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(r_radar_cs[i],TFG(),0);
        lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
        r_radar_alt[i]=lv_label_create(p);lv_label_set_text(r_radar_alt[i],"");
        lv_obj_set_style_text_font(r_radar_alt[i],&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(r_radar_alt[i],C_CYAN,0);
        lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);}

    // Alert overlay
    r_alert_overlay=lv_obj_create(p);lv_obj_set_size(r_alert_overlay,300,40);
    lv_obj_set_pos(r_alert_overlay,90,358);
    lv_obj_set_style_bg_color(r_alert_overlay,lv_color_hex(0x3d0000),0);
    lv_obj_set_style_bg_opa(r_alert_overlay,LV_OPA_90,0);
    lv_obj_set_style_border_color(r_alert_overlay,C_RED,0);lv_obj_set_style_border_width(r_alert_overlay,1,0);
    lv_obj_set_style_radius(r_alert_overlay,8,0);lv_obj_set_style_shadow_opa(r_alert_overlay,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(r_alert_overlay,0,0);lv_obj_clear_flag(r_alert_overlay,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);
    r_aov_text=lv_label_create(r_alert_overlay);lv_label_set_text(r_aov_text,"");
    lv_obj_set_style_text_color(r_aov_text,C_RED,0);
    lv_obj_set_style_text_font(r_aov_text,&lv_font_montserrat_16,0);lv_obj_center(r_aov_text);}

// ── Page 2 — Settings ─────────────────────────────────────────────────────────
void updSetPage(){
    char b[16];
    snprintf(b,16,"%dnm",g_cfg.scale_nm); lv_label_set_text(s_scale_v,b);
    snprintf(b,16,"%dft",g_cfg.vfilt_ft); lv_label_set_text(s_vfilt_v,b);
    lv_label_set_text(s_dist_v, g_cfg.dist_nm?"NM":"km");
    lv_label_set_text(s_alt_v,  g_cfg.alt_ft?"ft":"m");
    snprintf(b,16,"%d",g_cfg.brightness); lv_label_set_text(s_bright_v,b);
    lv_label_set_text(s_src_v,  kSrcNames[g_cfg.trf_src&3]);
    lv_label_set_text(s_theme_v,g_cfg.dark?"DARK":"LIGHT");
    snprintf(b,12,"%dnm",g_cfg.scale_nm); lv_label_set_text(r_radar_scale_lbl,b);
    panel.setBrightness(g_cfg.brightness);}

static void cbSetBtn(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int id=(int)(intptr_t)lv_event_get_user_data(e);
    int si=0;for(int i=0;i<5;i++)if(kScaleOpts[i]==g_cfg.scale_nm)si=i;
    switch(id){
        case 0:si=max(si-1,0);g_cfg.scale_nm=kScaleOpts[si];break;
        case 1:si=min(si+1,4);g_cfg.scale_nm=kScaleOpts[si];break;
        case 2:g_cfg.vfilt_ft=max((int)g_cfg.vfilt_ft-500,500);break;
        case 3:g_cfg.vfilt_ft=min((int)g_cfg.vfilt_ft+500,5000);break;
        case 4:case 5:g_cfg.dist_nm=!g_cfg.dist_nm;break;
        case 6:case 7:g_cfg.alt_ft=!g_cfg.alt_ft;break;
        case 8:g_cfg.brightness=max((int)g_cfg.brightness-16,4);break;
        case 9:g_cfg.brightness=min((int)g_cfg.brightness+16,255);break;
        case 10:g_cfg.trf_src=(g_cfg.trf_src+3)%4;break;
        case 11:g_cfg.trf_src=(g_cfg.trf_src+1)%4;break;
        case 12:case 13:g_cfg.dark=!g_cfg.dark;g_rebuildPages=true;break;}
    cfgSave();
    if(!g_rebuildPages)updSetPage();}

static lv_obj_t* mkSetRow(lv_obj_t*p,const char*k,int y,const char*v,int idn,int idup){
    mkLblP(p,k,TGREY(),&lv_font_montserrat_16,88,y);
    lv_obj_t*vl=mkLblP(p,v,C_AMBER,&lv_font_montserrat_16,215,y);
    lv_color_t btnbg=g_dark_theme?lv_color_hex(0x1f2937):lv_color_hex(0xdde1e7);
    lv_obj_t*bd=lv_btn_create(p);lv_obj_set_size(bd,38,30);lv_obj_set_pos(bd,285,y-4);
    lv_obj_set_style_bg_color(bd,btnbg,0);lv_obj_set_style_border_width(bd,0,0);
    lv_obj_set_style_radius(bd,6,0);lv_obj_set_style_shadow_opa(bd,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bd,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idn);
    lv_obj_t*ld=lv_label_create(bd);lv_label_set_text(ld,"<");
    lv_obj_set_style_text_color(ld,TFG(),0);lv_obj_center(ld);
    lv_obj_t*bu=lv_btn_create(p);lv_obj_set_size(bu,38,30);lv_obj_set_pos(bu,329,y-4);
    lv_obj_set_style_bg_color(bu,btnbg,0);lv_obj_set_style_border_width(bu,0,0);
    lv_obj_set_style_radius(bu,6,0);lv_obj_set_style_shadow_opa(bu,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bu,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idup);
    lv_obj_t*lu=lv_label_create(bu);lv_label_set_text(lu,">");
    lv_obj_set_style_text_color(lu,TFG(),0);lv_obj_center(lu);
    return vl;}

void buildSettingsPage(){
    lv_obj_t*p=g_pages[2];char b[16];
    mkLbl(p,"SETTINGS",C_AMBER,&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,55);
    mkLbl(p,"CONFIG",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,88);
    snprintf(b,16,"%dnm",g_cfg.scale_nm); s_scale_v=mkSetRow(p,"Scale", 108,b,0,1);
    snprintf(b,16,"%dft",g_cfg.vfilt_ft); s_vfilt_v=mkSetRow(p,"V-Filt",148,b,2,3);
    s_dist_v =mkSetRow(p,"Dist",  188,g_cfg.dist_nm?"NM":"km",4,5);
    s_alt_v  =mkSetRow(p,"Alt",   228,g_cfg.alt_ft?"ft":"m",  6,7);
    mkLbl(p,"DISPLAY",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,262);
    snprintf(b,16,"%d",g_cfg.brightness); s_bright_v=mkSetRow(p,"Bright",282,b,8,9);
    s_theme_v=mkSetRow(p,"Theme",322,g_cfg.dark?"DARK":"LIGHT",12,13);
    mkLbl(p,"TRAFFIC",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,358);
    s_src_v  =mkSetRow(p,"Source",378,kSrcNames[g_cfg.trf_src&3],10,11);
    lv_obj_t*ver=mkLblP(p,"v0.6  ●  AT-VIEW",TGREY(),&lv_font_montserrat_14,160,418);
    lv_obj_add_flag(ver,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_opa(ver,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(ver,cbDebugLongPress,LV_EVENT_LONG_PRESSED,NULL);}

// ── Debug page (hidden) ───────────────────────────────────────────────────────
void buildDebugPage(){
    lv_obj_t*p=g_dbgPage;
    mkLbl(p,"DEBUG SIM7600",C_BLUE,&lv_font_montserrat_16,LV_ALIGN_TOP_MID,0,58);
    int y=86,dy=26;
    r_hbgps =mkDbgL(p,y,"HB GPS","---",TGREY());r_hblte=mkDbgR(p,y,"HB LTE","---",TGREY());y+=dy;
    r_hbsd  =mkDbgL(p,y,"HB SD", "---",TGREY());r_p5csq=mkDbgR(p,y,"CSQ",   "---",TGREY());y+=dy;
    r_http  =mkDbgL(p,y,"HTTP",  "---",TGREY());r_code =mkDbgR(p,y,"CODE",  "---",TGREY());y+=dy;
    r_ss    =mkDbgL(p,y,"SafeSky","---",TGREY());r_fa   =mkDbgR(p,y,"FastAPI","---",TGREY());y+=dy;
    r_lteok =mkDbgL(p,y,"LTE",   "---",TGREY());r_dis  =mkDbgR(p,y,"DIS",   "---",TGREY());y+=dy;
    r_heap  =mkDbgL(p,y,"HEAP",  "---",TGREY());r_bat  =mkDbgR(p,y,"BAT",   "---",TGREY());y+=dy;
    r_p5mode=mkDbgL(p,y,"MODE",  "---",TGREY());r_pend =mkDbgR(p,y,"PEND",  "---",TGREY());y+=dy;
    r_flarmtx=mkDbgL(p,y,"FLARM","T0 R0",TGREY());r_adsbr=mkDbgR(p,y,"ADSB","0",TGREY());y+=dy;
    mkLblP(p,"FLT",TGREY(),&lv_font_montserrat_14,80,y);
    r_flt=mkLblP(p,"---",TFG(),&lv_font_montserrat_14,158,y);
    mkLbl(p,"swipe to exit",TGREY(),&lv_font_montserrat_14,LV_ALIGN_BOTTOM_MID,0,-55);}

// ── Swipe handlers ────────────────────────────────────────────────────────────
void createSwipeHandlers(){
    for(int i=0;i<NUM_PAGES;i++)
        lv_obj_add_event_cb(g_pages[i],swipeCb,LV_EVENT_ALL,NULL);
    lv_obj_add_event_cb(g_dbgPage,swipeCb,LV_EVENT_ALL,NULL);}

// ── Update all live data ──────────────────────────────────────────────────────
void updateAllPages(){
    char b[32];
    // Status page
    lv_label_set_text(r_title,g_connected?"AT-CORE":"AT-VIEW");
    lv_obj_set_style_text_color(r_title,g_connected?C_GREEN:C_AMBER,0);
    if(g_status.valid){
        snprintf(b,32,"GPS %dsat",g_status.gps_sat);updStat(r_gps,b,g_status.gps_fix);
        snprintf(b,32,"LTE %d",g_status.csq);updStat(r_lte,b,g_status.csq>5);
        snprintf(b,32,"SD %dfr",g_status.frames);updStat(r_sd,b,g_status.sd_ok);
        updStat(r_ble,"BLE",g_connected);updStat(r_flarm,"FLARM",g_status.flarm_ok);
        updStat(r_adsb,"ADS-B",g_status.adsb_ok);
        if(g_status.gps_fix){snprintf(b,32,"%.4f / %.4f",g_status.lat,g_status.lon);lv_label_set_text(r_coords,b);}
    }else{updStat(r_ble,"BLE",g_connected);}
    // Header — connectivity tabs + battery
    // Inactive: dim gray text + dark border. Active: bright color + bright border + flash on activation.
    {static bool prev_gps=false,prev_lte=false,prev_ble=false;
     bool gps_ok=g_status.valid&&g_status.gps_fix;
     bool lte_ok=g_status.valid&&g_status.csq>5;
     // Flash blink when status transitions to active
     if(gps_ok&&!prev_gps)flashTab(r_hdr_gps);
     if(lte_ok&&!prev_lte)flashTab(r_hdr_lte);
     if(g_connected&&!prev_ble)flashTab(r_hdr_ble);
     prev_gps=gps_ok;prev_lte=lte_ok;prev_ble=g_connected;
     // GPS tab
     lv_obj_set_style_text_color(r_hdr_gps,gps_ok?C_GREEN:TGREY(),0);
     lv_obj_set_style_border_color(lv_obj_get_parent(r_hdr_gps),gps_ok?C_GREEN:TRING(),0);
     // LTE tab — signal bars colored by strength
     {int csq=g_status.valid?g_status.csq:0;
      int bars=csq>20?4:csq>14?3:csq>8?2:csq>3?1:0;
      for(int bb=0;bb<4;bb++)
          lv_obj_set_style_bg_color(r_hdr_lte_b[bb],bb<bars?C_GREEN:TGREY(),0);
      lv_obj_set_style_text_color(r_hdr_lte,bars>0?C_GREEN:TGREY(),0);
      lv_obj_set_style_border_color(lv_obj_get_parent(r_hdr_lte),lte_ok?C_GREEN:TRING(),0);}
     // WiFi tab (future — always dim)
     // BLE tab
     lv_obj_set_style_text_color(r_hdr_ble,g_connected?C_BLUE:TGREY(),0);
     lv_obj_set_style_border_color(lv_obj_get_parent(r_hdr_ble),g_connected?C_BLUE:TRING(),0);
     // Battery
     if(!g_debug.valid||g_debug.bat_pct<0){
         lv_label_set_text(r_hdr_bat,LV_SYMBOL_CHARGE);
         lv_obj_set_style_text_color(r_hdr_bat,g_connected?C_GREEN:TGREY(),0);
     }else{
         char bb[20];
         const char*bi=g_debug.bat_pct>=75?LV_SYMBOL_BATTERY_FULL:
                        g_debug.bat_pct>=50?LV_SYMBOL_BATTERY_3:
                        g_debug.bat_pct>=25?LV_SYMBOL_BATTERY_2:
                        g_debug.bat_pct>=10?LV_SYMBOL_BATTERY_1:LV_SYMBOL_BATTERY_EMPTY;
         snprintf(bb,20,"%s %d%%",bi,g_debug.bat_pct);
         lv_label_set_text(r_hdr_bat,bb);
         lv_obj_set_style_text_color(r_hdr_bat,
             g_debug.bat_pct>=50?C_GREEN:g_debug.bat_pct>=20?C_AMBER:C_RED,0);}}
    // Auto-navigate to radar once BLE+GPS ready (one-shot per connection)
    if(!g_autoNavDone&&g_connected&&g_status.valid&&g_status.gps_fix&&g_page==0){
        g_autoNavDone=true;g_navPending=true;g_navPage=1;}
    // Radar — heading + cardinal labels
    if(g_status.valid){
        snprintf(b,32,"%d°",g_status.hdg);lv_label_set_text(r_radar_hdg,b);
        const int cbear[]={0,90,180,270};
        for(int ci=0;ci<4;ci++){
            int rel=((cbear[ci]-g_status.hdg)%360+360)%360;
            float ra=(float)rel*(float)M_PI/180.0f;
            int r_inner=RAD_R-24;
            int cx=(int)(RAD_CX+sinf(ra)*(float)r_inner)-5;
            int cy=(int)(RAD_CY-cosf(ra)*(float)r_inner)-8;
            lv_obj_set_pos(r_card[ci],cx,cy);}}
    // Radar — traffic VL3 icons (bitmap, heading-up) + speed vector (1 min)
    if(g_traffic.valid){
        for(int i=0;i<MAX_TRF;i++){
            if(i<g_traffic.count){
                TrafficEntry&e=g_traffic.t[i];
                int rb=((e.bear_deg-g_status.hdg)%360+360)%360;
                float brd=(float)rb*(float)M_PI/180.0f;
                float dpx=fminf((float)e.dist_m*(float)RAD_R/((float)g_cfg.scale_nm*1852.0f),(float)(RAD_R-20));
                int sx=(int)(RAD_CX+sinf(brd)*dpx);
                int sy=(int)(RAD_CY-cosf(brd)*dpx);
                int rel_hdg=((e.hdg_deg-g_status.hdg)%360+360)%360;
                float hr=(float)rel_hdg*(float)M_PI/180.0f;
                float cs=cosf(hr),sn=sinf(hr);
                lv_color_t col=e.dist_m<1000?C_RED:e.dist_m<3000?C_AMBER:C_CYAN;
                // Image pivot au centre (20,20), position corrigée pour centrer sur le blip
                lv_obj_set_pos(r_trf_img[i],sx-20,sy-20);
                lv_img_set_angle(r_trf_img[i],(int16_t)(rel_hdg*10));
                lv_obj_set_style_img_recolor(r_trf_img[i],col,0);
                lv_obj_clear_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);
                // Vecteur vitesse depuis le nez: 1 min de vol à spd_kt
                // Nez = offset (0,-15) depuis centre → après rotation: (+15*sn, -15*cs)
                float px_per_nm=(float)RAD_R/(float)g_cfg.scale_nm;
                float vect_px=fmaxf(6.f,fminf((float)e.spd_kt/60.0f*px_per_nm,35.f));
                r_vect_pts[i][0]={(lv_coord_t)(sx+(int)(15.f*sn)),(lv_coord_t)(sy-(int)(15.f*cs))};
                r_vect_pts[i][1]={(lv_coord_t)(sx+(int)((15.f+vect_px)*sn)),(lv_coord_t)(sy-(int)((15.f+vect_px)*cs))};
                lv_line_set_points(r_trf_vect[i],r_vect_pts[i],2);
                lv_obj_clear_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_pos(r_radar_cs[i],sx+12,sy-8);lv_label_set_text(r_radar_cs[i],e.cs);
                lv_obj_set_style_text_color(r_radar_cs[i],e.visible?TFG():C_AMBER,0);
                lv_obj_clear_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
                int rel_hft=(int)((e.alt_m-g_status.alt)*3.281f/100.0f);
                snprintf(b,32,"%+d",rel_hft);
                lv_obj_set_pos(r_radar_alt[i],sx+12,sy+6);lv_label_set_text(r_radar_alt[i],b);
                lv_obj_set_style_text_color(r_radar_alt[i],col,0);
                lv_obj_clear_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
            }else{
                lv_obj_add_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);}}}
    // CO gauge — ball position + ppm label
    if(g_flight.valid){
        int co=g_flight.co_ppm;
        float co_a=(30.0f+fminf((float)co,150.0f)/150.0f*30.0f)*(float)M_PI/180.0f;
        lv_obj_set_pos(r_co_ball,(int)(240.0f+212.0f*cosf(co_a))-6,(int)(240.0f+212.0f*sinf(co_a))-6);
        lv_color_t co_col=co<35?C_GREEN:co<70?C_ORANGE:C_RED;
        char cb[12];snprintf(cb,12,"%dppm",co);
        lv_label_set_text(r_co_val,cb);lv_obj_set_style_text_color(r_co_val,co_col,0);
        lv_obj_set_style_text_color(r_co_text,co_col,0);}
    // Alert overlay
    if(g_alert.valid){
        bool any=g_alert.co||g_alert.gforce||g_alert.rpm||g_alert.traffic;
        if(any){char ab[48]="";
            if(g_alert.co)strcat(ab,"▲CO  ");if(g_alert.gforce)strcat(ab,"▲G  ");
            if(g_alert.rpm)strcat(ab,"▲RPM  ");if(g_alert.traffic)strcat(ab,"▲TFC");
            lv_label_set_text(r_aov_text,ab);lv_obj_clear_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);
        }else{lv_obj_add_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);}}
    // Debug
    if(g_debug.valid&&g_dbgPage){
        const char*modes[4]={"PRE","FLT","POST","SLP"};
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
        if(g_debug.bat_pct<0){lv_label_set_text(r_bat,"---");lv_obj_set_style_text_color(r_bat,TGREY(),0);}
        else{snprintf(b,32,"%d%%",g_debug.bat_pct);lv_label_set_text(r_bat,b);lv_obj_set_style_text_color(r_bat,g_debug.bat_pct>40?C_GREEN:g_debug.bat_pct>20?C_AMBER:C_RED,0);}
        lv_label_set_text(r_p5mode,modes[min(g_debug.mode,3)]);lv_obj_set_style_text_color(r_p5mode,modeCol(g_debug.mode),0);
        snprintf(b,32,"%d",g_debug.pending);lv_label_set_text(r_pend,b);lv_obj_set_style_text_color(r_pend,g_debug.pending==0?C_GREEN:C_AMBER,0);
        snprintf(b,32,"T%d R%d",g_debug.flarm_tx,g_debug.flarm_rx);lv_label_set_text(r_flarmtx,b);
        lv_obj_set_style_text_color(r_flarmtx,(g_debug.flarm_tx>0||g_debug.flarm_rx>0)?C_GREEN:TGREY(),0);
        snprintf(b,32,"%d",g_debug.adsb_rx);lv_label_set_text(r_adsbr,b);
        lv_obj_set_style_text_color(r_adsbr,g_debug.adsb_rx>0?C_GREEN:TGREY(),0);
        lv_label_set_text(r_flt,g_debug.fid);}}

bool hasAlert(){return g_alert.valid&&(g_alert.co||g_alert.gforce||g_alert.rpm||g_alert.traffic);}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup(){
    Serial.begin(115200);
    if(!panel.begin()){while(1){Serial.println("Panel FAIL");delay(1000);}}
    Serial.printf("Touch: %s\n",panel.getTouchModelName());
    beginLvglHelper(panel);
    cfgLoad();
    g_dark_theme=g_cfg.dark;
    lv_obj_set_style_bg_color(lv_scr_act(),TBG(),0);
    panel.setBrightness(g_cfg.brightness);

    // Create all page containers
    for(int i=0;i<NUM_PAGES;i++){g_pages[i]=mkPage();lv_obj_add_flag(g_pages[i],LV_OBJ_FLAG_HIDDEN);}
    g_dbgPage=mkPage();lv_obj_add_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);

    // Build status page first (creates boot label refs), then show and animate
    buildStatusPage();
    lv_obj_clear_flag(g_pages[0],LV_OBJ_FLAG_HIDDEN);
    runBootOnPage(); // animates boot labels in place, sets g_bootDone=true

    // Build remaining pages
    buildRadarPage();buildSettingsPage();buildDebugPage();
    createSwipeHandlers();updSetPage();
    Serial.println("Ready");}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop(){
    uint32_t now=millis();
    if(g_doReconnect){g_doReconnect=false;startScan();}
    if(g_doConnect&&!g_connected){g_doConnect=false;
        if(connectBLE())Serial.println("[BLE] OK");
        else{delay(2000);startScan();}}
    if(!g_connected&&!g_doConnect){
        static uint32_t ls=0;if(now-ls>8000){ls=now;startScan();}}
    if(g_rebuildPages){g_rebuildPages=false;rebuildAllPages();}
    if(g_dataUpdated){g_dataUpdated=false;updateAllPages();}
    bool alert=hasAlert();
    if(alert&&!g_alertForced&&!g_inDebug){
        g_prevPage=g_page;g_alertForced=true;
        if(g_page!=1)switchPage(1);}
    else if(!alert&&g_alertForced){g_alertForced=false;switchPage(g_prevPage);}
    if(g_navPending){g_navPending=false;switchPage(g_navPage);}
    lv_timer_handler();delay(2);}

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
#include <Preferences.h>
#include <math.h>

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
#define C_CYAN   lv_color_hex(0x00E5FF)
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
struct TrafficEntry { char cs[9]; int dist_m,alt_m,bear_deg,climb_fpm; bool visible; };
struct TrafficData  { TrafficEntry t[MAX_TRF]; int count; bool valid; };
struct AlertData    { bool co,gforce,rpm,traffic; char msg[64]; bool valid; };
struct DebugData {
    int hb_gps,hb_lte,hb_sd,csq,http_ms,code;
    bool lte_ok,disable_lte;
    int ss_ago,fa_ago,heap,bat_pct,mode,pending;
    int flarm_tx,flarm_rx,adsb_rx;
    char fid[24]; bool valid;
};
static const uint8_t kScaleOpts[]={4,8,10,20,40};
static const char*   kSrcNames[] ={"SSKY","FLRM","ADSB","ALL"};
struct CfgData{ uint8_t scale_nm,brightness,trf_src; bool dist_nm,alt_ft; int16_t vfilt_ft; };
static CfgData g_cfg={4,16,3,true,true,2000};
static Preferences g_prefs;

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

#define NUM_PAGES 6
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
static lv_obj_t *r_alert_overlay,*r_aov_text;
static lv_obj_t *r_hbgps,*r_hblte,*r_hbsd,*r_p5csq,*r_http,*r_code;
static lv_obj_t *r_ss,*r_fa,*r_lteok,*r_dis,*r_heap,*r_bat,*r_p5mode,*r_pend;
static lv_obj_t *r_flarmtx,*r_adsbr,*r_flt;

// Radar (page 3)
#define RAD_CX       240
#define RAD_CY       240
#define RAD_R        175
static lv_obj_t *r_radar_hdg, *r_radar_north, *r_radar_scale_lbl;
static lv_obj_t *r_radar_blip[MAX_TRF], *r_radar_cs[MAX_TRF], *r_radar_alt[MAX_TRF], *r_radar_arr[MAX_TRF];

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
        g_traffic.t[i].bear_deg=d["t"][i]["b"]|0;g_traffic.t[i].climb_fpm=d["t"][i]["c"]|0;
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

    // Heading pill (top center, SafeSky style)
    lv_obj_t* hdg_box=lv_obj_create(p);
    lv_obj_set_size(hdg_box,72,28);
    lv_obj_align(hdg_box,LV_ALIGN_TOP_MID,0,28);
    lv_obj_set_style_bg_color(hdg_box,lv_color_hex(0x0d1b2a),0);
    lv_obj_set_style_bg_opa(hdg_box,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(hdg_box,C_WHITE,0);
    lv_obj_set_style_border_width(hdg_box,1,0);
    lv_obj_set_style_radius(hdg_box,14,0);
    lv_obj_set_style_shadow_opa(hdg_box,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(hdg_box,0,0);
    lv_obj_clear_flag(hdg_box,LV_OBJ_FLAG_SCROLLABLE);
    r_radar_hdg=lv_label_create(hdg_box);
    lv_label_set_text(r_radar_hdg,"---°");
    lv_obj_set_style_text_color(r_radar_hdg,C_WHITE,0);
    lv_obj_set_style_text_font(r_radar_hdg,&lv_font_montserrat_16,0);
    lv_obj_center(r_radar_hdg);

    // Outer range ring
    lv_obj_t* ro=lv_obj_create(p);
    lv_obj_set_size(ro,RAD_R*2,RAD_R*2);
    lv_obj_set_pos(ro,RAD_CX-RAD_R,RAD_CY-RAD_R);
    lv_obj_set_style_radius(ro,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_opa(ro,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_color(ro,C_GREY,0);
    lv_obj_set_style_border_width(ro,1,0);
    lv_obj_set_style_shadow_opa(ro,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(ro,0,0);
    lv_obj_clear_flag(ro,LV_OBJ_FLAG_SCROLLABLE);

    // Inner ring (half scale)
    lv_obj_t* ri=lv_obj_create(p);
    lv_obj_set_size(ri,RAD_R,RAD_R);
    lv_obj_set_pos(ri,RAD_CX-RAD_R/2,RAD_CY-RAD_R/2);
    lv_obj_set_style_radius(ri,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_opa(ri,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_color(ri,lv_color_hex(0x303030),0);
    lv_obj_set_style_border_width(ri,1,0);
    lv_obj_set_style_shadow_opa(ri,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(ri,0,0);
    lv_obj_clear_flag(ri,LV_OBJ_FLAG_SCROLLABLE);

    // Tick marks (12, every 30°) — static so LVGL keeps the pointer alive
    static lv_point_t tick_pts[12][2];
    for(int t=0;t<12;t++){
        float a=(float)t*30.0f*(float)M_PI/180.0f;
        tick_pts[t][0].x=(lv_coord_t)(RAD_CX+sinf(a)*(RAD_R-8));
        tick_pts[t][0].y=(lv_coord_t)(RAD_CY-cosf(a)*(RAD_R-8));
        tick_pts[t][1].x=(lv_coord_t)(RAD_CX+sinf(a)*RAD_R);
        tick_pts[t][1].y=(lv_coord_t)(RAD_CY-cosf(a)*RAD_R);
        lv_obj_t* tm=lv_line_create(p);
        lv_line_set_points(tm,tick_pts[t],2);
        lv_obj_set_style_line_color(tm,C_GREY,0);
        lv_obj_set_style_line_width(tm,2,0);}

    // Cross lines (H + V)
    static lv_point_t hpts[2]={{RAD_CX-RAD_R,RAD_CY},{RAD_CX+RAD_R,RAD_CY}};
    static lv_point_t vpts[2]={{RAD_CX,RAD_CY-RAD_R},{RAD_CX,RAD_CY+RAD_R}};
    lv_obj_t* hl=lv_line_create(p);lv_line_set_points(hl,hpts,2);
    lv_obj_set_style_line_color(hl,lv_color_hex(0x303030),0);lv_obj_set_style_line_width(hl,1,0);
    lv_obj_t* vl=lv_line_create(p);lv_line_set_points(vl,vpts,2);
    lv_obj_set_style_line_color(vl,lv_color_hex(0x303030),0);lv_obj_set_style_line_width(vl,1,0);

    // Heading track line (green, center → top)
    static lv_point_t htrk[2]={{RAD_CX,RAD_CY},{RAD_CX,RAD_CY-RAD_R}};
    lv_obj_t* tl=lv_line_create(p);lv_line_set_points(tl,htrk,2);
    lv_obj_set_style_line_color(tl,C_GREEN,0);lv_obj_set_style_line_width(tl,2,0);

    // Own aircraft triangle (3 lv_line points, closed)
    static lv_point_t tri[4]={{RAD_CX,RAD_CY-20},{RAD_CX-12,RAD_CY+12},{RAD_CX+12,RAD_CY+12},{RAD_CX,RAD_CY-20}};
    lv_obj_t* ot=lv_line_create(p);lv_line_set_points(ot,tri,4);
    lv_obj_set_style_line_color(ot,C_GREEN,0);lv_obj_set_style_line_width(ot,3,0);

    // N indicator (repositioned dynamically on each heading update)
    r_radar_north=lv_label_create(p);
    lv_label_set_text(r_radar_north,"N");
    lv_obj_set_style_text_color(r_radar_north,C_WHITE,0);
    lv_obj_set_style_text_font(r_radar_north,&lv_font_montserrat_14,0);
    lv_obj_set_pos(r_radar_north,RAD_CX-5,RAD_CY-RAD_R-20);

    // Scale label + page indicator (stored for live update from settings)
    char scl[16];snprintf(scl,16,"%dnm  3/5",g_cfg.scale_nm);
    r_radar_scale_lbl=mkLbl(p,scl,C_GREY,&lv_font_montserrat_14,LV_ALIGN_BOTTOM_MID,0,-55);

    // Traffic blips (diamond ◆) + callsign + alt + vertical trend arrow
    for(int i=0;i<MAX_TRF;i++){
        r_radar_blip[i]=lv_label_create(p);
        lv_label_set_text(r_radar_blip[i],"◆");
        lv_obj_set_style_text_font(r_radar_blip[i],&lv_font_montserrat_16,0);
        lv_obj_set_style_text_color(r_radar_blip[i],C_CYAN,0);
        lv_obj_add_flag(r_radar_blip[i],LV_OBJ_FLAG_HIDDEN);

        r_radar_cs[i]=lv_label_create(p);
        lv_label_set_text(r_radar_cs[i],"");
        lv_obj_set_style_text_font(r_radar_cs[i],&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(r_radar_cs[i],C_WHITE,0);
        lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);

        r_radar_alt[i]=lv_label_create(p);
        lv_label_set_text(r_radar_alt[i],"");
        lv_obj_set_style_text_font(r_radar_alt[i],&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(r_radar_alt[i],C_CYAN,0);
        lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);

        r_radar_arr[i]=lv_label_create(p);
        lv_label_set_text(r_radar_arr[i],"");
        lv_obj_set_style_text_font(r_radar_arr[i],&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(r_radar_arr[i],C_WHITE,0);
        lv_obj_add_flag(r_radar_arr[i],LV_OBJ_FLAG_HIDDEN);
    }

    // Alert overlay band — semi-transparent, above scale label
    r_alert_overlay=lv_obj_create(p);
    lv_obj_set_size(r_alert_overlay,300,40);
    lv_obj_set_pos(r_alert_overlay,90,358);
    lv_obj_set_style_bg_color(r_alert_overlay,lv_color_hex(0x3d0000),0);
    lv_obj_set_style_bg_opa(r_alert_overlay,LV_OPA_90,0);
    lv_obj_set_style_border_color(r_alert_overlay,C_RED,0);
    lv_obj_set_style_border_width(r_alert_overlay,1,0);
    lv_obj_set_style_radius(r_alert_overlay,8,0);
    lv_obj_set_style_shadow_opa(r_alert_overlay,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(r_alert_overlay,0,0);
    lv_obj_clear_flag(r_alert_overlay,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);
    r_aov_text=lv_label_create(r_alert_overlay);
    lv_label_set_text(r_aov_text,"");
    lv_obj_set_style_text_color(r_aov_text,C_RED,0);
    lv_obj_set_style_text_font(r_aov_text,&lv_font_montserrat_16,0);
    lv_obj_center(r_aov_text);
}

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
    // P3 — Radar heading-up
    if(g_status.valid){
        snprintf(b,32,"%d°",g_status.hdg);
        lv_label_set_text(r_radar_hdg,b);
        float nbr=(float)((360-g_status.hdg)%360)*(float)M_PI/180.0f;
        int nx=(int)(RAD_CX+sinf(nbr)*(float)(RAD_R+20))-5;
        int ny=(int)(RAD_CY-cosf(nbr)*(float)(RAD_R+20))-8;
        lv_obj_set_pos(r_radar_north,nx,ny);}
    if(g_traffic.valid){
        for(int i=0;i<MAX_TRF;i++){
            if(i<g_traffic.count){
                TrafficEntry& e=g_traffic.t[i];
                int rb=((e.bear_deg-g_status.hdg)%360+360)%360;
                float brd=(float)rb*(float)M_PI/180.0f;
                float dpx=fminf((float)e.dist_m*(float)RAD_R/((float)g_cfg.scale_nm*1852.0f),(float)(RAD_R-6));
                int bx=(int)(RAD_CX+sinf(brd)*dpx)-7;
                int by=(int)(RAD_CY-cosf(brd)*dpx)-9;
                lv_color_t col=e.dist_m<1000?C_RED:e.dist_m<3000?C_AMBER:C_CYAN;
                // Diamond blip
                lv_obj_set_pos(r_radar_blip[i],bx,by);
                lv_obj_set_style_text_color(r_radar_blip[i],col,0);
                lv_obj_clear_flag(r_radar_blip[i],LV_OBJ_FLAG_HIDDEN);
                // Callsign (white=visible, amber=FLARM non-coop)
                lv_obj_set_pos(r_radar_cs[i],bx+16,by);
                lv_label_set_text(r_radar_cs[i],e.cs);
                lv_obj_set_style_text_color(r_radar_cs[i],e.visible?C_WHITE:C_AMBER,0);
                lv_obj_clear_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
                // Relative altitude (hundreds of feet, SafeSky convention)
                int rel_hft=(int)((e.alt_m-g_status.alt)*3.281f/100.0f);
                snprintf(b,32,"%+d",rel_hft);
                lv_obj_set_pos(r_radar_alt[i],bx+16,by+14);
                lv_label_set_text(r_radar_alt[i],b);
                lv_obj_set_style_text_color(r_radar_alt[i],col,0);
                lv_obj_clear_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
                // Vertical trend arrow (↑ climbing, ↓ descending)
                if(e.climb_fpm>100){
                    lv_label_set_text(r_radar_arr[i],"↑");
                    lv_obj_set_style_text_color(r_radar_arr[i],C_GREEN,0);
                    lv_obj_set_pos(r_radar_arr[i],bx-14,by);
                    lv_obj_clear_flag(r_radar_arr[i],LV_OBJ_FLAG_HIDDEN);
                }else if(e.climb_fpm<-100){
                    lv_label_set_text(r_radar_arr[i],"↓");
                    lv_obj_set_style_text_color(r_radar_arr[i],C_RED,0);
                    lv_obj_set_pos(r_radar_arr[i],bx-14,by);
                    lv_obj_clear_flag(r_radar_arr[i],LV_OBJ_FLAG_HIDDEN);
                }else{lv_obj_add_flag(r_radar_arr[i],LV_OBJ_FLAG_HIDDEN);}
            }else{
                lv_obj_add_flag(r_radar_blip[i],LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_radar_arr[i],LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    // P4 (page dédiée alertes — accès manuel uniquement)
    if(g_alert.valid){
        bool any=g_alert.co||g_alert.gforce||g_alert.rpm||g_alert.traffic;
        any?lv_obj_add_flag(r_noAlert,LV_OBJ_FLAG_HIDDEN):lv_obj_clear_flag(r_noAlert,LV_OBJ_FLAG_HIDDEN);
        g_alert.co?lv_obj_clear_flag(r_alertCO,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertCO,LV_OBJ_FLAG_HIDDEN);
        g_alert.gforce?lv_obj_clear_flag(r_alertGF,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertGF,LV_OBJ_FLAG_HIDDEN);
        g_alert.rpm?lv_obj_clear_flag(r_alertRPM,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertRPM,LV_OBJ_FLAG_HIDDEN);
        g_alert.traffic?lv_obj_clear_flag(r_alertTFC,LV_OBJ_FLAG_HIDDEN):lv_obj_add_flag(r_alertTFC,LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(r_alertMsg,g_alert.msg);
        // Overlay sur radar
        if(any){
            char ab[48]="";
            if(g_alert.co)    strcat(ab,"▲CO  ");
            if(g_alert.gforce)strcat(ab,"▲G  ");
            if(g_alert.rpm)   strcat(ab,"▲RPM  ");
            if(g_alert.traffic)strcat(ab,"▲TFC");
            lv_label_set_text(r_aov_text,ab);
            lv_obj_clear_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);
        }else{lv_obj_add_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);}
    }
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

// ─── Settings NVS ─────────────────────────────────────────────────────────────
void cfgLoad(){
    g_prefs.begin("atview",true);
    g_cfg.scale_nm   =g_prefs.getUChar("scale",4);
    g_cfg.brightness =g_prefs.getUChar("bright",16);
    g_cfg.trf_src    =g_prefs.getUChar("trf_src",3);
    g_cfg.dist_nm    =g_prefs.getBool("dist_nm",true);
    g_cfg.alt_ft     =g_prefs.getBool("alt_ft",true);
    g_cfg.vfilt_ft   =g_prefs.getShort("vfilt",2000);
    g_prefs.end();
}
void cfgSave(){
    g_prefs.begin("atview",false);
    g_prefs.putUChar("scale",g_cfg.scale_nm);
    g_prefs.putUChar("bright",g_cfg.brightness);
    g_prefs.putUChar("trf_src",g_cfg.trf_src);
    g_prefs.putBool("dist_nm",g_cfg.dist_nm);
    g_prefs.putBool("alt_ft",g_cfg.alt_ft);
    g_prefs.putShort("vfilt",g_cfg.vfilt_ft);
    g_prefs.end();
}

// ─── Settings Page UI ─────────────────────────────────────────────────────────
static lv_obj_t *s_scale_v,*s_vfilt_v,*s_dist_v,*s_alt_v,*s_bright_v,*s_src_v;

void updSetPage(){
    char b[16];
    snprintf(b,16,"%dnm",g_cfg.scale_nm);  lv_label_set_text(s_scale_v,b);
    snprintf(b,16,"%dft",g_cfg.vfilt_ft);  lv_label_set_text(s_vfilt_v,b);
    lv_label_set_text(s_dist_v,g_cfg.dist_nm?"NM":"km");
    lv_label_set_text(s_alt_v, g_cfg.alt_ft?"ft":"m");
    snprintf(b,16,"%d",g_cfg.brightness);  lv_label_set_text(s_bright_v,b);
    lv_label_set_text(s_src_v, kSrcNames[g_cfg.trf_src&3]);
    snprintf(b,16,"%dnm  3/5",g_cfg.scale_nm); lv_label_set_text(r_radar_scale_lbl,b);
    panel.setBrightness(g_cfg.brightness);
}

static void cbSetBtn(lv_event_t* e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int id=(int)(intptr_t)lv_event_get_user_data(e);
    int si=0; for(int i=0;i<5;i++) if(kScaleOpts[i]==g_cfg.scale_nm) si=i;
    switch(id){
        case 0:  si=max(si-1,0);              g_cfg.scale_nm=kScaleOpts[si]; break;
        case 1:  si=min(si+1,4);              g_cfg.scale_nm=kScaleOpts[si]; break;
        case 2:  g_cfg.vfilt_ft=max((int)g_cfg.vfilt_ft-500,500);  break;
        case 3:  g_cfg.vfilt_ft=min((int)g_cfg.vfilt_ft+500,5000); break;
        case 4:  case 5: g_cfg.dist_nm=!g_cfg.dist_nm;             break;
        case 6:  case 7: g_cfg.alt_ft=!g_cfg.alt_ft;               break;
        case 8:  g_cfg.brightness=max((int)g_cfg.brightness-16,4);  break;
        case 9:  g_cfg.brightness=min((int)g_cfg.brightness+16,255);break;
        case 10: g_cfg.trf_src=(g_cfg.trf_src+3)%4;                break;
        case 11: g_cfg.trf_src=(g_cfg.trf_src+1)%4;                break;
    }
    cfgSave(); updSetPage();
}

static lv_obj_t* mkSetRow(lv_obj_t*p,const char*k,int y,const char*v,int idn,int idup){
    mkLblP(p,k,C_GREY,&lv_font_montserrat_16,88,y);
    lv_obj_t* vl=mkLblP(p,v,C_AMBER,&lv_font_montserrat_16,215,y);
    lv_obj_t* bd=lv_btn_create(p);lv_obj_set_size(bd,38,30);lv_obj_set_pos(bd,285,y-4);
    lv_obj_set_style_bg_color(bd,lv_color_hex(0x1f2937),0);lv_obj_set_style_border_width(bd,0,0);
    lv_obj_set_style_radius(bd,6,0);lv_obj_set_style_shadow_opa(bd,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bd,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idn);
    lv_obj_t* ld=lv_label_create(bd);lv_label_set_text(ld,"<");
    lv_obj_set_style_text_color(ld,C_WHITE,0);lv_obj_center(ld);
    lv_obj_t* bu=lv_btn_create(p);lv_obj_set_size(bu,38,30);lv_obj_set_pos(bu,329,y-4);
    lv_obj_set_style_bg_color(bu,lv_color_hex(0x1f2937),0);lv_obj_set_style_border_width(bu,0,0);
    lv_obj_set_style_radius(bu,6,0);lv_obj_set_style_shadow_opa(bu,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bu,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idup);
    lv_obj_t* lu=lv_label_create(bu);lv_label_set_text(lu,">");
    lv_obj_set_style_text_color(lu,C_WHITE,0);lv_obj_center(lu);
    return vl;
}

void buildPage6(){
    lv_obj_t*p=g_pages[5]; char b[16];
    mkLbl(p,"SETTINGS",C_AMBER,&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,55);
    mkLbl(p,"CONFIG",C_GREY,&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,88);
    snprintf(b,16,"%dnm",g_cfg.scale_nm);         s_scale_v=mkSetRow(p,"Scale", 108,b,0,1);
    snprintf(b,16,"%dft",g_cfg.vfilt_ft);          s_vfilt_v=mkSetRow(p,"V-Filt",148,b,2,3);
    s_dist_v =mkSetRow(p,"Dist",  188,g_cfg.dist_nm?"NM":"km",4,5);
    s_alt_v  =mkSetRow(p,"Alt",   228,g_cfg.alt_ft?"ft":"m",  6,7);
    mkLbl(p,"DISPLAY",C_GREY,&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,258);
    snprintf(b,16,"%d",g_cfg.brightness);           s_bright_v=mkSetRow(p,"Bright",278,b,8,9);
    mkLbl(p,"TRAFFIC",C_GREY,&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,308);
    s_src_v  =mkSetRow(p,"Source",328,kSrcNames[g_cfg.trf_src&3],10,11);
    mkLbl(p,"6/6",C_GREY,&lv_font_montserrat_16,LV_ALIGN_BOTTOM_MID,0,-60);
}

void runBootSplash(){
    lv_obj_t* s=lv_scr_act();
    lv_obj_t* lc;

    // Title block
    lv_obj_t* tl=lv_label_create(s);
    lv_label_set_text(tl,"AT-VIEW");
    lv_obj_set_style_text_color(tl,C_AMBER,0);
    lv_obj_set_style_text_font(tl,&lv_font_montserrat_32,0);
    lv_obj_align(tl,LV_ALIGN_TOP_MID,0,115);
    lv_obj_t* sub=lv_label_create(s);
    lv_label_set_text(sub,"AeroTrace");
    lv_obj_set_style_text_color(sub,C_GREY,0);
    lv_obj_set_style_text_font(sub,&lv_font_montserrat_16,0);
    lv_obj_align(sub,LV_ALIGN_TOP_MID,0,162);
    lv_timer_handler();delay(600);

    // PANEL
    lc=lv_label_create(s);lv_label_set_text(lc,"  PANEL    ...");
    lv_obj_set_style_text_color(lc,C_GREY,0);lv_obj_set_style_text_font(lc,&lv_font_montserrat_16,0);
    lv_obj_set_pos(lc,130,222);lv_timer_handler();delay(350);
    lv_label_set_text(lc,"● PANEL    OK");lv_obj_set_style_text_color(lc,C_GREEN,0);lv_timer_handler();

    // LVGL
    lc=lv_label_create(s);lv_label_set_text(lc,"  LVGL     ...");
    lv_obj_set_style_text_color(lc,C_GREY,0);lv_obj_set_style_text_font(lc,&lv_font_montserrat_16,0);
    lv_obj_set_pos(lc,130,252);lv_timer_handler();delay(300);
    lv_label_set_text(lc,"● LVGL     OK");lv_obj_set_style_text_color(lc,C_GREEN,0);lv_timer_handler();

    // BLE init
    lc=lv_label_create(s);lv_label_set_text(lc,"  BLE      ...");
    lv_obj_set_style_text_color(lc,C_GREY,0);lv_obj_set_style_text_font(lc,&lv_font_montserrat_16,0);
    lv_obj_set_pos(lc,130,282);lv_timer_handler();delay(300);
    BLEDevice::init("ATCORE-TRGB");
    lv_label_set_text(lc,"● BLE      OK");lv_obj_set_style_text_color(lc,C_GREEN,0);lv_timer_handler();

    // AT-CORE scan
    lc=lv_label_create(s);lv_label_set_text(lc,"  AT-CORE  ...");
    lv_obj_set_style_text_color(lc,C_GREY,0);lv_obj_set_style_text_font(lc,&lv_font_montserrat_16,0);
    lv_obj_set_pos(lc,130,312);lv_timer_handler();delay(400);
    startScan();
    lv_label_set_text(lc,"● AT-CORE  SCAN");lv_obj_set_style_text_color(lc,C_AMBER,0);lv_timer_handler();
    delay(900);

    lv_obj_clean(s);
}

void setup(){
    Serial.begin(115200);
    if(!panel.begin()){while(1){Serial.println("Panel FAIL");delay(1000);}}
    Serial.printf("Touch: %s\n",panel.getTouchModelName());
    beginLvglHelper(panel);
    lv_obj_set_style_bg_color(lv_scr_act(),C_BG,0);
    cfgLoad();
    panel.setBrightness(g_cfg.brightness);
    runBootSplash();
    for(int i=0;i<NUM_PAGES;i++){g_pages[i]=mkPage();lv_obj_add_flag(g_pages[i],LV_OBJ_FLAG_HIDDEN);}
    buildPage1();buildPage2();buildPage3();buildPage4();buildPage5();buildPage6();
    lv_obj_clear_flag(g_pages[0],LV_OBJ_FLAG_HIDDEN);
    createNavButtons();
    updSetPage();
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
    if(alert&&!g_alertForced){g_prevPage=g_page;g_alertForced=true;if(g_page!=2)switchPage(2);}
    else if(!alert&&g_alertForced){g_alertForced=false;switchPage(g_prevPage);}
    if(g_navPending){g_navPending=false;switchPage(g_navPage);}
    lv_timer_handler();delay(2);}

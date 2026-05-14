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
#include "img_aircraft_icons.h"
#include "img_safesky.h"
#include "img_flarm.h"
#include "img_logos.h"

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
static inline lv_color_t PILL_IC_OFF(){return g_dark_theme?lv_color_hex(0x2d3f52):lv_color_hex(0xb0bcc8);}
static inline lv_color_t PILL_IC_ON() {return g_dark_theme?lv_color_hex(0xffffff):lv_color_hex(0x0d1117);}

// ── Data structs ──────────────────────────────────────────────────────────────
struct StatusData {
    int mode,gps_sat,csq,frames,alt,spd,hdg,bat; float lat,lon;
    bool gps_fix,sd_ok,flarm_ok,adsb_ok,charging,valid; };
struct FlightData  { float gforce_z; int co_ppm,rpm,phase; bool valid; };
#define MAX_TRF 5
struct TrafficEntry { char cs[9]; int dist_m,alt_m,bear_deg,hdg_deg,spd_kt,type; bool visible; };
struct TrafficData  { TrafficEntry t[MAX_TRF]; int count; bool valid; uint32_t recv_ms; };
struct AlertData    { bool co,gforce,rpm,traffic; char msg[64]; bool valid; };
struct DebugData    {
    int hb_gps,hb_lte,hb_sd,csq,http_ms,code;
    bool lte_ok,disable_lte;
    int ss_ago,fa_ago,heap,bat_pct,mode,pending,flarm_tx,flarm_rx,adsb_rx;
    char fid[24]; bool valid; };

static const uint8_t kScaleOpts[]={1,2,4,8,10,20,40};
static const char*   kSrcNames[] ={"SSKY","FLRM","ADSB","ALL"};
static const char*   kIconSzNames[]={"S","M","L"};
static const uint16_t kIconZoom[]={171,213,256};  // zoom for 32/40/48 px from 48px base
static const int8_t  kIconHalf[]={16,20,24};
struct CfgData { uint8_t scale_nm,brightness,trf_src; bool dist_nm,alt_ft,dark,show_grnd; int16_t vfilt_ft; uint8_t icon_sz; };
static CfgData     g_cfg={4,16,3,true,true,true,true,2000,2};
static Preferences g_prefs;

static StatusData  g_status  = {};
static FlightData  g_flight  = {};
static TrafficData g_traffic = {};
static AlertData   g_alert   = {};
static DebugData   g_debug   = {};
static volatile bool g_dataUpdated = false;

// ── Pilot DB / Auth ───────────────────────────────────────────────────────────
struct PilotEntry { char code[5]; char name[32]; char status[12]; char primary_icao[8]; };
#define MAX_PILOTS 24
static PilotEntry  g_pilots[MAX_PILOTS] = {};
static int         g_pilot_cnt = 0;
static char        g_aircraft_icao[8] = "";
struct AuthSession { char name[32]; char status[12]; bool is_owner; bool valid; };
static AuthSession g_session = {};

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
static lv_obj_t *r_coords,*r_bat_p1;
static lv_obj_t *r_sbox[6],*r_sico[6],*r_stxt[6]; // [0]GPS [1]LTE [2]SD [3]BLE [4]FLARM [5]ADS-B

// ── Widget refs — Radar (page 1) ──────────────────────────────────────────────
#define RAD_CX 240
#define RAD_CY 240
#define RAD_R  175
static lv_obj_t *r_radar_hdg, *r_radar_scale_lbl;
static lv_obj_t *r_card[4];
static lv_obj_t *r_radar_cs[MAX_TRF],*r_radar_alt[MAX_TRF];
static lv_obj_t *r_trf_img[MAX_TRF],*r_trf_vect[MAX_TRF];
static lv_point_t r_vect_pts[MAX_TRF][2];
static int r_trf_last_type[MAX_TRF];
static lv_obj_t *r_alert_overlay, *r_aov_text;
static lv_obj_t *r_co_val, *r_co_ball, *r_co_text;
static lv_obj_t *r_hdr_bat;
static lv_obj_t *r_hdr_sky, *r_hdr_flrm, *r_hdr_adsb;  // left arc: SafeSky / FLARM / ADS-B
static lv_obj_t *r_hdr_gps, *r_hdr_lte, *r_hdr_wifi, *r_hdr_ble;
static lv_obj_t *r_hdr_lte_b[4];  // 4 drawn signal bars inside LTE pill

// ── Widget refs — Auth overlay ────────────────────────────────────────────────
static lv_obj_t* g_auth_ov       = nullptr;
static lv_obj_t* g_auth_dots[4]  = {};
static lv_obj_t* g_auth_prompt   = nullptr;
static lv_obj_t* g_auth_name     = nullptr;
static lv_obj_t* g_auth_msg      = nullptr;
static char      g_auth_buf[5]   = {};
static int       g_auth_len      = 0;
static bool      g_auth_p2       = false;   // phase 2: instructor code for student
static char      g_auth_scode[5] = {};      // student code saved during phase 2

// ── Aircraft identity ─────────────────────────────────────────────────────────
#define N_AC_TYPES 193
static const char* kACCodes[N_AC_TYPES] = {
    "GLID","BALL","ULAC","UHEL","GYRO","SHIP","ZZZZ",
    "VL3","STNG","MCR1","FK9","FK12","FK14","EV97","EV10",
    "VIPI","PIVI","PIVE","ALPH","BSTL","CH60","CH70","SVAN","CARE","C42",
    "RANS","FLTD","PION","SVGE","SHRK","A210","TWST","CTLS","CTSW","JABI","P92",
    "C150","C162","C172","C177","C182","C185","C206","C207","C210",
    "PA18","P28A","P28B","P28R","P28T","P32R","P32T","P46T","M101",
    "M20P","M20T","BE23","BE24","BE33","BE35","BE36","BE55","BE58","BE76",
    "BE9L","BE20","B350",
    "DR40","DR22","HR10","R200","R300",
    "TB9","TB10","TB20","TB21","TBM7","TBM8","TBM9",
    "D11","D18","D140","SR20","SR22","EVSS",
    "DA20","DA40","DV20","DA42","DA62",
    "P2002","P2006","P2010","P2012",
    "G109","G115","G120","T67","SV4","Z42","Z526",
    "C310","C340","C402","C404","C414","C421","PA44","PA34","P68",
    "PC12","PC6T","C208","PAY2","PAY3","C90",
    "C25A","C25B","C25C","C510","C525","C550","C560","C680","C750",
    "PHMR","E50P","E55P","PC24","SF50",
    "LJ35","LJ45","LJ60","F2TH","F900","F7X","F8X",
    "CL60","CL30","CL35","G150","G280",
    "AT43","AT45","AT72","AT75","AT76",
    "DH8A","DH8B","DH8C","DH8D",
    "E135","E145","E170","E175","E190","E195",
    "A318","A319","A320","A321",
    "B732","B733","B734","B735","B736","B737","B738","B739","B37M","B38M","B39M",
    "B752","B753",
    "R22","R44","R66","B06","B407","B429","S76","S92",
    "EC35","EC45","EC55","EC25","AS50","AS55","ALO2","ALO3","A109","A119","AW13"};
static const char* kACLabels[N_AC_TYPES] = {
    "Glider / Planeur","Balloon / Ballon","ULM avion","ULM helicoptere",
    "Gyrocopter","Airship / Dirigeable","Autre type (ZZZZ)",
    "VL-3 / VL3 Evolution","TL-3000 Sting S4","MCR-01 UL/Club/VLA",
    "FK-9 Mk IV/V","FK-12 Comet","FK-14 Polaris","EV-97 Eurostar","EV-10 Raven",
    "Virus SW","Virus 912/914","Virus (electric)","Alpha Trainer","Bristell B23",
    "CH-601 Zodiac","CH-700/750 Cruzer","Savannah / Savannah VG","Calidus/MTO Sport","C42",
    "S-6 Coyote / S-7 Courier","Quik GTR / Quik R","Pioneer 200/300",
    "Savage Cub/Classic","Shark","AT01 (A210)","Twister","CT LS/CT SW","CT SW",
    "J120/J160/J170","P92 Echo/JS/RG",
    "150/152","162 Skycatcher","172 Skyhawk","177 Cardinal","182 Skylane",
    "185 Skywagon","206 Stationair","207 Skywagon","210 Centurion",
    "PA-18 Super Cub","PA-28 Cherokee/Archer","PA-28R Arrow",
    "PA-28R-201 Arrow III","PA-28R-201T Turbo Arrow",
    "PA-32R Lance/Saratoga","PA-32RT-300 Turbo Lance","PA-46-500TP Meridian",
    "PA-46-350P Malibu Mirage","M20 F/G/J/K","M20TN Acclaim/Ovation",
    "Musketeer/Sport/Sundowner","Musketeer Super/Sierra",
    "Bonanza V35/F33/G33","Bonanza 35","Bonanza A36/G36",
    "Baron 55","Baron 58/58P/58TC","Duchess",
    "King Air 90","King Air 200/B200","King Air 350",
    "DR 400","DR 220/221","HR 100","HR 200 / R 2000","R 3000",
    "TB 9 Tampico","TB 10 Tobago","TB 20 Trinidad","TB 21 Trinidad TC",
    "TBM 700","TBM 850/900","TBM 960",
    "Jodel D.11/112/113","Jodel D.18","Jodel D.140 Mousquetaire",
    "SR20","SR22/SR22T","Vision SF50 (jet)",
    "DA 20 Katana","DA 40 Star","DV 20 Katana (Rotax)","DA 42 Twin Star","DA 62",
    "P2002 Sierra","P2006T","P2010","P2012 Traveller",
    "G 109/109B Ranger","G 115/115T Acro","G 120 TP","T67 Firefly","SV-4 biplanes",
    "Z 42/43","Z 526 Trener",
    "Cessna 310","Cessna 340","Cessna 402","Cessna 404 Titan",
    "Cessna 414 Chancellor","Cessna 421 Golden Eagle","PA-44 Seminole","PA-34 Seneca",
    "P.68 Victor/Observer","PC-12/PC-12NG","PC-6 Turbo Porter","208 Caravan",
    "PA-42 Cheyenne II","PA-42 Cheyenne III","King Air C90",
    "Citation CJ2","Citation CJ3","Citation CJ4","Citation Mustang",
    "CitationJet CJ1","Citation II/Bravo","Citation V/Ultra","Citation Sovereign",
    "Citation X","HA-420 HondaJet","Phenom 100","Phenom 300","PC-24","Vision SF50",
    "Learjet 35/36","Learjet 45","Learjet 60",
    "Falcon 2000","Falcon 900/900EX","Falcon 7X","Falcon 8X",
    "Challenger 600","Challenger 300","Challenger 350",
    "Gulfstream G150/Galaxy","Gulfstream G280",
    "ATR 42-300/320","ATR 42-500","ATR 72","ATR 72-500","ATR 72-600",
    "Dash 8 Q100","Dash 8 Q200","Dash 8 Q300","Dash 8 Q400",
    "ERJ 135","ERJ 145","Embraer 170","Embraer 175","Embraer 190","Embraer 195",
    "A318","A319/A319neo","A320/A320neo","A321/A321neo",
    "737-200","737-300","737-400","737-500","737-600",
    "737-700/700W","737-800/800W","737-900/900ER","737 MAX 7","737 MAX 8","737 MAX 9",
    "757-200","757-300",
    "Robinson R22","Robinson R44","Robinson R66",
    "Bell 206 JetRanger","Bell 407","Bell 429",
    "Sikorsky S-76","Sikorsky S-92",
    "H135 (EC135)","H145 (EC145)","H155 (EC155)","H225 Super Puma",
    "H125 AS350 Ecureuil","H130 (EC130)",
    "Alouette II","Alouette III","AW109","AW119 Koala","AW139"};
char g_ac_reg[8]  = "";   // immatriculation ex "FJFVB"
char g_ac_type[8] = "";   // code OACI ex "VL3"
char g_ac_hex[7]  = "";   // hex transpondeur ex "38EDC5"
static lv_obj_t* g_ac_ov       = nullptr;
static lv_obj_t* g_ac_disp     = nullptr;
static lv_obj_t* g_ac_hdr_reg  = nullptr;
static lv_obj_t* g_ac_hdr_typ  = nullptr;
static lv_obj_t* g_ac_hdr_hex  = nullptr;
static lv_obj_t* g_ac_ctn[3]   = {};
static lv_obj_t* g_ac_tabs[3]  = {};
static lv_obj_t* g_ac_roller   = nullptr;
static lv_obj_t* g_ac_type_desc= nullptr;
static uint8_t   g_ac_tab      = 0;
static char      g_ac_tmp[8]   = "";

// ── Widget refs — Settings (page 2) ───────────────────────────────────────────
static lv_obj_t *s_scale_v,*s_vfilt_v,*s_dist_v,*s_alt_v,*s_bright_v,*s_src_v,*s_theme_v,*s_grnd_v,*s_icon_sz_v;
static lv_obj_t *s_ac_v;

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
// Tab pill: 52×32 invisible hit-zone, icon floats freely. Returns inner label ref.
lv_obj_t* mkTabPill(lv_obj_t*p,const char*t,int x,int y){
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,52,32);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,t);
    lv_obj_set_style_text_color(l,PILL_IC_OFF(),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_16,0);
    lv_obj_center(l);return l;}
// LTE pill: 4 drawn signal bars (bottom-aligned), returns dummy label ref for parent lookups
lv_obj_t* mkLTEPill(lv_obj_t*p,int x,int y){
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,52,32);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    // 4 signal bars centered in 52×32 — bar widths 3px, gap 2px, bottom at y=27
    static const int8_t bh[4]={5,8,12,16};
    for(int i=0;i<4;i++){
        r_hdr_lte_b[i]=lv_obj_create(b);lv_obj_set_size(r_hdr_lte_b[i],3,bh[i]);
        lv_obj_set_pos(r_hdr_lte_b[i],17+i*5,27-bh[i]);
        lv_obj_set_style_radius(r_hdr_lte_b[i],1,0);
        lv_obj_set_style_bg_color(r_hdr_lte_b[i],PILL_IC_OFF(),0);lv_obj_set_style_bg_opa(r_hdr_lte_b[i],LV_OPA_COVER,0);
        lv_obj_set_style_border_width(r_hdr_lte_b[i],0,0);lv_obj_set_style_shadow_opa(r_hdr_lte_b[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_pad_all(r_hdr_lte_b[i],0,0);
        lv_obj_clear_flag(r_hdr_lte_b[i],LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);}
    // Invisible dummy label — anchor for lv_obj_get_parent() in flashTab/updateAllPages
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,"");
    lv_obj_set_style_opa(l,LV_OPA_TRANSP,0);lv_obj_center(l);return l;}
// Image pill — transparent hit-zone, image floats freely, recolorable
lv_obj_t* mkImgPill(lv_obj_t*p,const lv_img_dsc_t*src,int x,int y){
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,52,32);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*img=lv_img_create(b);lv_img_set_src(img,src);
    lv_obj_set_style_img_recolor(img,PILL_IC_OFF(),0);
    lv_obj_set_style_img_recolor_opa(img,LV_OPA_COVER,0);
    lv_obj_center(img);return img;}
// Flash a tab pill (3× blink) when its status becomes active
static void _flash_cb(void*var,int32_t v){lv_obj_set_style_opa((lv_obj_t*)var,(lv_opa_t)v,0);}
void flashTab(lv_obj_t*lbl){
    lv_obj_t*pill=lv_obj_get_parent(lbl);
    lv_anim_t a;lv_anim_init(&a);lv_anim_set_var(&a,pill);
    lv_anim_set_exec_cb(&a,(lv_anim_exec_xcb_t)_flash_cb);
    lv_anim_set_values(&a,LV_OPA_COVER,LV_OPA_20);
    lv_anim_set_time(&a,160);lv_anim_set_playback_time(&a,160);
    lv_anim_set_repeat_count(&a,3);lv_anim_start(&a);}

void mkSBox(lv_obj_t*p,int idx,int x,int y,const char*txt,bool ok){
    lv_obj_t*box=lv_obj_create(p);lv_obj_set_size(box,22,22);lv_obj_set_pos(box,x,y);
    lv_obj_set_style_radius(box,4,0);
    lv_obj_set_style_bg_color(box,ok?C_GREEN:lv_color_hex(0x1e2b38),0);
    lv_obj_set_style_border_color(box,ok?C_GREEN:TGREY(),0);lv_obj_set_style_border_width(box,1,0);
    lv_obj_set_style_shadow_opa(box,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(box,0,0);
    lv_obj_clear_flag(box,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*ico=lv_label_create(box);lv_label_set_text(ico,ok?LV_SYMBOL_OK:"");
    lv_obj_set_style_text_color(ico,TBG(),0);lv_obj_set_style_text_font(ico,&lv_font_montserrat_14,0);
    lv_obj_center(ico);r_sbox[idx]=box;r_sico[idx]=ico;
    r_stxt[idx]=mkLblP(p,txt,TGREY(),&lv_font_montserrat_14,x+28,y+4);}
void updSBox(int idx,const char*txt,bool ok){
    lv_obj_set_style_bg_color(r_sbox[idx],ok?C_GREEN:lv_color_hex(0x1e2b38),0);
    lv_obj_set_style_border_color(r_sbox[idx],ok?C_GREEN:TGREY(),0);
    lv_label_set_text(r_sico[idx],ok?LV_SYMBOL_OK:"");
    lv_obj_set_style_text_color(r_stxt[idx],ok?TFG():TGREY(),0);
    lv_label_set_text(r_stxt[idx],txt);}

// ── Parsers ───────────────────────────────────────────────────────────────────
void parseStatus(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_status.mode=d["mode"]|0;g_status.gps_sat=d["gps_sat"]|0;g_status.csq=d["csq"]|-1;
    g_status.frames=d["frames"]|0;g_status.alt=d["alt"]|0;g_status.spd=d["spd"]|0;
    g_status.hdg=d["hdg"]|0;g_status.bat=d["bat"]|-1;g_status.lat=d["lat"]|0.0f;g_status.lon=d["lon"]|0.0f;
    g_status.gps_fix=d["gps_fix"]|false;g_status.sd_ok=d["sd_ok"]|false;
    g_status.flarm_ok=d["flarm"]|false;g_status.adsb_ok=d["adsb"]|false;
    g_status.charging=d["chg"]|false;
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
        g_traffic.t[i].visible=d["t"][i]["v"]|true;
        g_traffic.t[i].type=d["t"][i]["tp"]|0;}
    g_traffic.valid=true;g_traffic.recv_ms=millis();g_dataUpdated=true;}
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
    g_cfg.show_grnd =g_prefs.getBool("show_grnd",true);
    g_cfg.vfilt_ft  =g_prefs.getShort("vfilt",2000);
    g_cfg.icon_sz   =g_prefs.getUChar("icon_sz",2);
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
    g_prefs.putBool("show_grnd",g_cfg.show_grnd);
    g_prefs.putShort("vfilt",g_cfg.vfilt_ft);
    g_prefs.putUChar("icon_sz",g_cfg.icon_sz);
    g_prefs.end();}

// ── Forward declarations ──────────────────────────────────────────────────────
void buildStatusPage();
void buildRadarPage();
void buildSettingsPage();
void buildDebugPage();
void createSwipeHandlers();
void updSetPage();
void mkAircraftOverlay();
void acLoad();
void acSave();

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

    // ── Logos (white pixels → recolor via theme)
    lv_obj_t*lAt=lv_img_create(p);
    lv_img_set_src(lAt,&img_logo_aerotrace);
    lv_obj_set_pos(lAt,120,78);
    lv_obj_set_style_img_recolor(lAt,TFG(),0);
    lv_obj_set_style_img_recolor_opa(lAt,LV_OPA_COVER,0);

    lv_obj_t*lVw=lv_img_create(p);
    lv_img_set_src(lVw,&img_logo_atview);
    lv_obj_set_pos(lVw,130,143);
    lv_obj_set_style_img_recolor(lVw,C_AMBER,0);
    lv_obj_set_style_img_recolor_opa(lVw,LV_OPA_COVER,0);

    // ── Connection status (small, below logos)
    r_title=mkLbl(p,"● SCAN",C_AMBER,&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,196);

    // ── Separator
    static lv_point_t sep1[2]={{110,210},{370,210}};
    lv_obj_t*sl1=lv_line_create(p);lv_line_set_points(sl1,sep1,2);
    lv_obj_set_style_line_color(sl1,TGRID(),0);lv_obj_set_style_line_width(sl1,1,0);

    // ── Boot checks
    const char* bp_str = g_bootDone ? "● PANEL    OK" : "  PANEL    ...";
    const char* bl_str = g_bootDone ? "● LVGL     OK" : "  LVGL     ...";
    const char* bb_str = g_bootDone ? "● BLE      OK" : "  BLE      ...";
    const char* bc_str = g_bootDone ? "● AT-CORE  SCAN" : "  AT-CORE  ...";
    lv_color_t ok_col  = g_bootDone ? C_GREEN : TGREY();
    lv_color_t core_col= g_bootDone ? C_AMBER : TGREY();
    r_boot_panel=mkLblP(p,bp_str,ok_col,  &lv_font_montserrat_14,120,224);
    r_boot_lvgl =mkLblP(p,bl_str,ok_col,  &lv_font_montserrat_14,120,240);
    r_boot_ble  =mkLblP(p,bb_str,ok_col,  &lv_font_montserrat_14,120,256);
    r_boot_core =mkLblP(p,bc_str,core_col,&lv_font_montserrat_14,120,272);

    // ── Separator 2
    static lv_point_t sep2[2]={{110,286},{370,286}};
    lv_obj_t*sl2=lv_line_create(p);lv_line_set_points(sl2,sep2,2);
    lv_obj_set_style_line_color(sl2,TGRID(),0);lv_obj_set_style_line_width(sl2,1,0);

    // ── Live status — 2 colonnes
    mkSBox(p,0, 68,300,"GPS ---",false); mkSBox(p,1,252,300,"LTE ---",false);
    mkSBox(p,2, 68,327,"SD ---", false); mkSBox(p,3,252,327,"BLE",    false);
    mkSBox(p,4, 68,354,"FLARM",  false); mkSBox(p,5,252,354,"ADS-B",  false);
    r_coords=mkLbl(p,"--- / ---",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,380);
    r_bat_p1=mkLbl(p,"BAT  ---%",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,400);
    mkLbl(p,"v0.6  —  2026-05-13",TGREY(),&lv_font_montserrat_12,LV_ALIGN_BOTTOM_MID,0,-52);}

// ── Pilot DB / Auth functions ─────────────────────────────────────────────────
void pilotDBLoad(){
    Preferences p;p.begin("auth",true);
    p.getString("icao","").toCharArray(g_aircraft_icao,sizeof(g_aircraft_icao));
    String db=p.getString("pilots","");p.end();
    if(!db.length())return;
    JsonDocument doc;if(deserializeJson(doc,db))return;
    g_pilot_cnt=0;
    for(JsonObject e:doc.as<JsonArray>()){
        if(g_pilot_cnt>=MAX_PILOTS)break;
        PilotEntry&t=g_pilots[g_pilot_cnt++];
        strlcpy(t.code,         e["c"]|"",sizeof(t.code));
        strlcpy(t.name,         e["n"]|"",sizeof(t.name));
        strlcpy(t.status,       e["s"]|"",sizeof(t.status));
        strlcpy(t.primary_icao, e["i"]|"",sizeof(t.primary_icao));}}

PilotEntry* pilotFind(const char*code){
    for(int i=0;i<g_pilot_cnt;i++)if(strcmp(g_pilots[i].code,code)==0)return&g_pilots[i];
    return nullptr;}

bool checkOwnerNVS(){
    if(!g_aircraft_icao[0])return false;
    Preferences p;p.begin("auth",true);
    String oc=p.getString("owner","");p.end();
    if(oc.length()!=4)return false;
    char cb[5];oc.toCharArray(cb,5);
    PilotEntry*pe=pilotFind(cb);
    if(!pe||strcmp(pe->primary_icao,g_aircraft_icao)!=0)return false;
    strlcpy(g_session.name,pe->name,32);strlcpy(g_session.status,pe->status,12);
    g_session.is_owner=true;g_session.valid=true;return true;}

void authUpdateDots(){
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_color(g_auth_dots[i],TFG(),0);
        lv_obj_set_style_border_color(g_auth_dots[i],TFG(),0);
        lv_obj_set_style_bg_opa(g_auth_dots[i],i<g_auth_len?LV_OPA_COVER:LV_OPA_TRANSP,0);}}

static void _auth_err_cb(lv_timer_t*t){
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_opa(g_auth_dots[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_bg_color(g_auth_dots[i],TFG(),0);
        lv_obj_set_style_border_color(g_auth_dots[i],TFG(),0);}
    lv_label_set_text(g_auth_msg,"");lv_timer_del(t);}

void authError(const char*msg){
    lv_label_set_text(g_auth_msg,msg);
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_color(g_auth_dots[i],C_RED,0);
        lv_obj_set_style_bg_opa(g_auth_dots[i],LV_OPA_COVER,0);
        lv_obj_set_style_border_color(g_auth_dots[i],C_RED,0);}
    g_auth_len=0;memset(g_auth_buf,0,5);
    lv_timer_create(_auth_err_cb,1500,nullptr);}

void authSuccess(PilotEntry*pe){
    strlcpy(g_session.name,pe->name,32);strlcpy(g_session.status,pe->status,12);
    bool owner=g_aircraft_icao[0]&&pe->primary_icao[0]&&strcmp(pe->primary_icao,g_aircraft_icao)==0;
    g_session.is_owner=owner;g_session.valid=true;
    if(owner){Preferences p;p.begin("auth",false);p.putString("owner",pe->code);p.end();}
    else{Preferences p;p.begin("auth",false);p.putString("owner","");p.end();}
    if(g_auth_ov){lv_obj_del(g_auth_ov);g_auth_ov=nullptr;}}

void authValidate(){
    if(!g_auth_p2){
        PilotEntry*pe=pilotFind(g_auth_buf);
        if(!pe){authError("Code inconnu - Retry");return;}
        if(strcmp(pe->status,"student")==0){
            strlcpy(g_auth_scode,g_auth_buf,5);
            g_auth_p2=true;g_auth_len=0;memset(g_auth_buf,0,5);
            authUpdateDots();
            lv_label_set_text(g_auth_prompt,"Code Instructeur");
            lv_label_set_text(g_auth_name,pe->name);
            lv_label_set_text(g_auth_msg,"");
        }else{authSuccess(pe);}
    }else{
        PilotEntry*inst=pilotFind(g_auth_buf);
        if(!inst||strcmp(inst->status,"instructor")!=0){authError("Code instructeur invalide");return;}
        PilotEntry*stud=pilotFind(g_auth_scode);
        if(stud)authSuccess(stud);else authError("Erreur session");}}

static void _auth_btn_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    intptr_t d=(intptr_t)lv_event_get_user_data(e);
    if(d==10){if(g_auth_len>0){g_auth_len--;g_auth_buf[g_auth_len]=0;authUpdateDots();}
    }else if(d==11){if(g_auth_len==4)authValidate();
    }else{if(g_auth_len<4){g_auth_buf[g_auth_len++]='0'+d;g_auth_buf[g_auth_len]=0;
        authUpdateDots();if(g_auth_len==4)authValidate();}}}

void mkAuthOverlay(){
    g_auth_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_auth_ov,480,480);lv_obj_set_pos(g_auth_ov,0,0);
    lv_obj_set_style_bg_color(g_auth_ov,TBG(),0);lv_obj_set_style_bg_opa(g_auth_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_auth_ov,0,0);lv_obj_set_style_radius(g_auth_ov,0,0);
    lv_obj_set_style_shadow_opa(g_auth_ov,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(g_auth_ov,0,0);
    lv_obj_clear_flag(g_auth_ov,LV_OBJ_FLAG_SCROLLABLE);
    // ICAO + title
    lv_obj_t*il=lv_label_create(g_auth_ov);
    lv_label_set_text(il,g_aircraft_icao[0]?g_aircraft_icao:"AT-VIEW");
    lv_obj_set_style_text_color(il,TGREY(),0);
    lv_obj_set_style_text_font(il,&lv_font_montserrat_14,0);
    lv_obj_align(il,LV_ALIGN_TOP_MID,0,46);
    g_auth_prompt=lv_label_create(g_auth_ov);
    lv_label_set_text(g_auth_prompt,"Identification");
    lv_obj_set_style_text_color(g_auth_prompt,TFG(),0);
    lv_obj_set_style_text_font(g_auth_prompt,&lv_font_montserrat_16,0);
    lv_obj_align(g_auth_prompt,LV_ALIGN_TOP_MID,0,76);
    // 4 PIN dots — container centered
    lv_obj_t*dc=lv_obj_create(g_auth_ov);
    lv_obj_set_size(dc,130,28);
    lv_obj_set_style_bg_opa(dc,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(dc,0,0);
    lv_obj_set_style_shadow_opa(dc,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(dc,0,0);
    lv_obj_clear_flag(dc,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(dc,LV_ALIGN_TOP_MID,0,114);
    for(int i=0;i<4;i++){
        g_auth_dots[i]=lv_obj_create(dc);
        lv_obj_set_size(g_auth_dots[i],22,22);lv_obj_set_pos(g_auth_dots[i],i*36,3);
        lv_obj_set_style_radius(g_auth_dots[i],LV_RADIUS_CIRCLE,0);
        lv_obj_set_style_bg_color(g_auth_dots[i],TFG(),0);
        lv_obj_set_style_bg_opa(g_auth_dots[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_border_color(g_auth_dots[i],TFG(),0);
        lv_obj_set_style_border_width(g_auth_dots[i],2,0);
        lv_obj_set_style_shadow_opa(g_auth_dots[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_pad_all(g_auth_dots[i],0,0);
        lv_obj_clear_flag(g_auth_dots[i],LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);}
    // Name hint + error
    g_auth_name=lv_label_create(g_auth_ov);lv_label_set_text(g_auth_name,"");
    lv_obj_set_style_text_color(g_auth_name,C_GREEN,0);
    lv_obj_set_style_text_font(g_auth_name,&lv_font_montserrat_14,0);
    lv_obj_align(g_auth_name,LV_ALIGN_TOP_MID,0,156);
    g_auth_msg=lv_label_create(g_auth_ov);lv_label_set_text(g_auth_msg,"");
    lv_obj_set_style_text_color(g_auth_msg,C_RED,0);
    lv_obj_set_style_text_font(g_auth_msg,&lv_font_montserrat_12,0);
    lv_obj_align(g_auth_msg,LV_ALIGN_TOP_MID,0,184);
    // Keypad 3×4 — BW=68 BH=54 BG=8 → width=220 KX=130 bottom=450 (within circle ✓)
    static const char*kL[12]={"1","2","3","4","5","6","7","8","9","<","0","OK"};
    static const intptr_t kV[12]={1,2,3,4,5,6,7,8,9,10,0,11};
    for(int i=0;i<12;i++){
        int r=i/3,c=i%3;
        lv_obj_t*btn=lv_btn_create(g_auth_ov);
        lv_obj_set_size(btn,68,54);
        lv_obj_set_pos(btn,130+c*76,210+r*62);
        lv_obj_set_style_bg_color(btn,lv_color_hex(0x1e2b38),0);
        lv_obj_set_style_bg_color(btn,lv_color_hex(0x2d4358),LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(btn,LV_OPA_COVER,0);
        lv_obj_set_style_radius(btn,12,0);
        lv_obj_set_style_shadow_opa(btn,LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(btn,0,0);
        lv_obj_add_event_cb(btn,_auth_btn_cb,LV_EVENT_CLICKED,(void*)kV[i]);
        lv_obj_t*lb=lv_label_create(btn);lv_label_set_text(lb,kL[i]);
        lv_obj_set_style_text_color(lb,TFG(),0);
        lv_obj_set_style_text_font(lb,&lv_font_montserrat_16,0);
        lv_obj_center(lb);}}

// ── Aircraft identity overlay ─────────────────────────────────────────────────

void acLoad(){
    Preferences p;p.begin("aircraft",true);
    p.getString("reg","").toCharArray(g_ac_reg,sizeof(g_ac_reg));
    p.getString("type","").toCharArray(g_ac_type,sizeof(g_ac_type));
    p.getString("hex24","").toCharArray(g_ac_hex,sizeof(g_ac_hex));
    p.end();}

void acSave(){
    Preferences p;p.begin("aircraft",false);
    p.putString("reg",g_ac_reg);
    p.putString("type",g_ac_type);
    p.putString("hex24",g_ac_hex);
    p.end();}

void acUpdateHeader(){
    if(!g_ac_hdr_reg)return;
    lv_label_set_text(g_ac_hdr_reg,g_ac_reg[0]?g_ac_reg:"---");
    lv_label_set_text(g_ac_hdr_typ,g_ac_type[0]?g_ac_type:"---");
    lv_label_set_text(g_ac_hdr_hex,g_ac_hex[0]?g_ac_hex:"------");}

static void _ac_disp_refresh(){
    if(!g_ac_disp)return;
    if(g_ac_tab==2){
        char d[8];int l=strlen(g_ac_tmp);
        for(int i=0;i<6;i++)d[i]=(i<l)?g_ac_tmp[i]:'_';d[6]=0;
        lv_label_set_text(g_ac_disp,d);
    }else{lv_label_set_text(g_ac_disp,g_ac_tmp[0]?g_ac_tmp:"_");}}

static void _ac_key_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    intptr_t d=(intptr_t)lv_event_get_user_data(e);
    if(d==200){ // OK — save current tab, stay open
        if(g_ac_tab==0){strlcpy(g_ac_reg,g_ac_tmp,sizeof(g_ac_reg));}
        else if(g_ac_tab==2){
            if(strlen(g_ac_tmp)!=6)return; // enforce 6 hex digits
            strlcpy(g_ac_hex,g_ac_tmp,sizeof(g_ac_hex));}
        acSave();acUpdateHeader();
        if(s_ac_v){char t[20];snprintf(t,20,"%s  %s",g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---");lv_label_set_text(s_ac_v,t);}
        return;}
    if(d==202){lv_obj_del(g_ac_ov);g_ac_ov=nullptr;return;} // × close
    if(d==201){int l=strlen(g_ac_tmp);if(l>0)g_ac_tmp[l-1]=0;}
    else{int l=strlen(g_ac_tmp);int mx=(g_ac_tab==2)?6:7;
        if(l<mx){g_ac_tmp[l]=(char)d;g_ac_tmp[l+1]=0;}}
    _ac_disp_refresh();}

static void _ac_roller_ok_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(!g_ac_roller)return;
    uint16_t sel=lv_roller_get_selected(g_ac_roller);
    if(sel<N_AC_TYPES)strlcpy(g_ac_type,kACCodes[sel],sizeof(g_ac_type));
    acSave();acUpdateHeader();
    if(s_ac_v){char t[20];snprintf(t,20,"%s  %s",g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---");lv_label_set_text(s_ac_v,t);}}

static void _ac_roller_change_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_VALUE_CHANGED)return;
    if(!g_ac_roller||!g_ac_type_desc)return;
    uint16_t sel=lv_roller_get_selected(g_ac_roller);
    if(sel<N_AC_TYPES)lv_label_set_text(g_ac_type_desc,kACLabels[sel]);}

void acSwitchTab(uint8_t tab){
    g_ac_tab=tab;
    lv_color_t abg=g_dark_theme?lv_color_hex(0x1f4068):lv_color_hex(0x94b4d4);
    lv_color_t ibg=g_dark_theme?lv_color_hex(0x1a2332):lv_color_hex(0xdde3ea);
    for(int i=0;i<3;i++)lv_obj_set_style_bg_color(g_ac_tabs[i],i==tab?abg:ibg,0);
    for(int i=0;i<3;i++){
        if(i==tab)lv_obj_clear_flag(g_ac_ctn[i],LV_OBJ_FLAG_HIDDEN);
        else       lv_obj_add_flag(g_ac_ctn[i],LV_OBJ_FLAG_HIDDEN);}
    if(tab==0)strlcpy(g_ac_tmp,g_ac_reg,sizeof(g_ac_tmp));
    else if(tab==2)strlcpy(g_ac_tmp,g_ac_hex,sizeof(g_ac_tmp));
    if(tab!=1){lv_obj_clear_flag(g_ac_disp,LV_OBJ_FLAG_HIDDEN);_ac_disp_refresh();}
    else       lv_obj_add_flag(g_ac_disp,LV_OBJ_FLAG_HIDDEN);}

static void _ac_tab_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    acSwitchTab((uint8_t)(intptr_t)lv_event_get_user_data(e));}

lv_obj_t* mkAcKey(lv_obj_t*p,const char*t,int x,int y,int w,int h,intptr_t d){
    lv_color_t bg=g_dark_theme?lv_color_hex(0x1e2b38):lv_color_hex(0xd0dce8);
    lv_obj_t*b=lv_btn_create(p);lv_obj_set_size(b,w,h);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_color(b,bg,0);
    lv_obj_set_style_bg_color(b,lv_color_hex(0x2d4358),LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(b,LV_OPA_COVER,0);lv_obj_set_style_radius(b,8,0);
    lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(b,0,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_add_event_cb(b,_ac_key_cb,LV_EVENT_CLICKED,(void*)d);
    lv_obj_t*lb=lv_label_create(b);lv_label_set_text(lb,t);
    lv_obj_set_style_text_color(lb,TFG(),0);
    lv_obj_set_style_text_font(lb,&lv_font_montserrat_14,0);
    lv_obj_center(lb);return b;}

void mkAircraftOverlay(){
    // Fullscreen overlay
    g_ac_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_ac_ov,480,480);lv_obj_set_pos(g_ac_ov,0,0);
    lv_obj_set_style_bg_color(g_ac_ov,TBG(),0);lv_obj_set_style_bg_opa(g_ac_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_ac_ov,0,0);lv_obj_set_style_radius(g_ac_ov,0,0);
    lv_obj_set_style_shadow_opa(g_ac_ov,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(g_ac_ov,0,0);
    lv_obj_clear_flag(g_ac_ov,LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t*tl=lv_label_create(g_ac_ov);lv_label_set_text(tl,"APPAREIL");
    lv_obj_set_style_text_color(tl,C_AMBER,0);lv_obj_set_style_text_font(tl,&lv_font_montserrat_20,0);
    lv_obj_align(tl,LV_ALIGN_TOP_MID,0,48);

    // Close ×
    lv_obj_t*xb=lv_btn_create(g_ac_ov);lv_obj_set_size(xb,36,36);lv_obj_set_pos(xb,390,44);
    lv_obj_set_style_bg_color(xb,lv_color_hex(0x3d0000),0);lv_obj_set_style_radius(xb,18,0);
    lv_obj_set_style_border_width(xb,0,0);lv_obj_set_style_shadow_opa(xb,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(xb,_ac_key_cb,LV_EVENT_CLICKED,(void*)202);
    lv_obj_t*xl=lv_label_create(xb);lv_label_set_text(xl,LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_color(xl,C_RED,0);lv_obj_center(xl);

    // Summary: IMMAT / TYPE / HEX current values
    mkLblP(g_ac_ov,"IMMAT",TGREY(),&lv_font_montserrat_12,58,78);
    g_ac_hdr_reg=mkLblP(g_ac_ov,g_ac_reg[0]?g_ac_reg:"---",TFG(),&lv_font_montserrat_12,58,92);
    mkLblP(g_ac_ov,"TYPE",TGREY(),&lv_font_montserrat_12,198,78);
    g_ac_hdr_typ=mkLblP(g_ac_ov,g_ac_type[0]?g_ac_type:"---",C_AMBER,&lv_font_montserrat_12,198,92);
    mkLblP(g_ac_ov,"HEX",TGREY(),&lv_font_montserrat_12,322,78);
    g_ac_hdr_hex=mkLblP(g_ac_ov,g_ac_hex[0]?g_ac_hex:"------",C_CYAN,&lv_font_montserrat_12,322,92);

    // Separator
    static lv_point_t ac_sep[2]={{100,112},{380,112}};
    lv_obj_t*sep=lv_line_create(g_ac_ov);lv_line_set_points(sep,ac_sep,2);
    lv_obj_set_style_line_color(sep,TGRID(),0);lv_obj_set_style_line_width(sep,1,0);

    // Tab buttons
    static const char*tNames[3]={"IMMAT","TYPE","HEX"};
    lv_color_t abg=g_dark_theme?lv_color_hex(0x1f4068):lv_color_hex(0x94b4d4);
    lv_color_t ibg=g_dark_theme?lv_color_hex(0x1a2332):lv_color_hex(0xdde3ea);
    int tbw=100,tbh=30,tbg=8,tbx=(480-(3*tbw+2*tbg))/2;
    for(int i=0;i<3;i++){
        lv_obj_t*tb=lv_btn_create(g_ac_ov);lv_obj_set_size(tb,tbw,tbh);
        lv_obj_set_pos(tb,tbx+i*(tbw+tbg),118);
        lv_obj_set_style_bg_color(tb,i==0?abg:ibg,0);lv_obj_set_style_bg_opa(tb,LV_OPA_COVER,0);
        lv_obj_set_style_radius(tb,8,0);lv_obj_set_style_border_width(tb,0,0);
        lv_obj_set_style_shadow_opa(tb,LV_OPA_TRANSP,0);
        lv_obj_add_event_cb(tb,_ac_tab_cb,LV_EVENT_CLICKED,(void*)(intptr_t)i);
        lv_obj_t*tl2=lv_label_create(tb);lv_label_set_text(tl2,tNames[i]);
        lv_obj_set_style_text_color(tl2,TFG(),0);lv_obj_set_style_text_font(tl2,&lv_font_montserrat_14,0);
        lv_obj_center(tl2);g_ac_tabs[i]=tb;}

    // Editing display (IMMAT / HEX value being typed)
    g_ac_disp=lv_label_create(g_ac_ov);
    lv_label_set_text(g_ac_disp,g_ac_reg[0]?g_ac_reg:"_");
    lv_obj_set_style_text_color(g_ac_disp,C_GREEN,0);
    lv_obj_set_style_text_font(g_ac_disp,&lv_font_montserrat_20,0);
    lv_obj_align(g_ac_disp,LV_ALIGN_TOP_MID,0,158);

    // 3 content containers at y=196, h=240
    for(int i=0;i<3;i++){
        g_ac_ctn[i]=lv_obj_create(g_ac_ov);lv_obj_set_size(g_ac_ctn[i],480,240);
        lv_obj_set_pos(g_ac_ctn[i],0,196);
        lv_obj_set_style_bg_opa(g_ac_ctn[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(g_ac_ctn[i],0,0);lv_obj_set_style_shadow_opa(g_ac_ctn[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_pad_all(g_ac_ctn[i],0,0);lv_obj_clear_flag(g_ac_ctn[i],LV_OBJ_FLAG_SCROLLABLE);
        if(i>0)lv_obj_add_flag(g_ac_ctn[i],LV_OBJ_FLAG_HIDDEN);}

    // ── Container 0: IMMAT keyboard ──────────────────────────────────────────
    {lv_obj_t*p=g_ac_ctn[0];
    int bw=38,bh=28,gp=3;
    int x0=(480-(10*(bw+gp)-gp))/2; // center 10 keys
    static const char row1[]="ABCDEFGHIJ";
    static const char row2[]="KLMNOPQRST";
    for(int i=0;i<10;i++){
        char c1[2]={row1[i],0},c2[2]={row2[i],0};
        mkAcKey(p,c1,x0+i*(bw+gp), 8,bw,bh,(intptr_t)row1[i]);
        mkAcKey(p,c2,x0+i*(bw+gp),40,bw,bh,(intptr_t)row2[i]);}
    // UVWXYZ- (7 keys, centered)
    static const char row3[]="UVWXYZ-";
    int r3x=(480-(7*(bw+gp)-gp))/2;
    for(int i=0;i<7;i++){char c[2]={row3[i],0};mkAcKey(p,c,r3x+i*(bw+gp),72,bw,bh,(intptr_t)row3[i]);}
    // 0-9 (10 keys)
    static const char row4[]="0123456789";
    for(int i=0;i<10;i++){char c[2]={row4[i],0};mkAcKey(p,c,x0+i*(bw+gp),104,bw,bh,(intptr_t)row4[i]);}
    // ⌫ and OK
    mkAcKey(p,LV_SYMBOL_BACKSPACE,104,142,100,32,201);
    mkAcKey(p,"OK",         284,142,100,32,200);}

    // ── Container 1: TYPE roller (kACCodes, 193 entries) ─────────────────────
    {lv_obj_t*p=g_ac_ctn[1];
    // Build roller options string (codes only, ~950 bytes)
    static char ac_opts[1024]="";
    ac_opts[0]=0;
    for(int i=0;i<N_AC_TYPES;i++){strcat(ac_opts,kACCodes[i]);if(i<N_AC_TYPES-1)strcat(ac_opts,"\n");}
    g_ac_roller=lv_roller_create(p);
    lv_roller_set_options(g_ac_roller,ac_opts,LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_ac_roller,4);
    lv_obj_set_size(g_ac_roller,120,120);
    lv_obj_align(g_ac_roller,LV_ALIGN_TOP_MID,0,8);
    lv_obj_set_style_bg_color(g_ac_roller,g_dark_theme?lv_color_hex(0x0d1117):lv_color_hex(0xf0f2f5),0);
    lv_obj_set_style_text_color(g_ac_roller,TFG(),0);
    lv_obj_set_style_text_font(g_ac_roller,&lv_font_montserrat_16,0);
    lv_obj_set_style_border_color(g_ac_roller,TGREY(),0);lv_obj_set_style_border_width(g_ac_roller,1,0);
    lv_obj_set_style_shadow_opa(g_ac_roller,LV_OPA_TRANSP,0);
    lv_obj_set_style_bg_color(g_ac_roller,C_AMBER,LV_PART_SELECTED);
    lv_obj_set_style_text_color(g_ac_roller,TBG(),LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(g_ac_roller,LV_OPA_COVER,LV_PART_SELECTED);
    lv_obj_add_event_cb(g_ac_roller,_ac_roller_change_cb,LV_EVENT_VALUE_CHANGED,NULL);
    // Pre-select current type
    int presel=0;
    for(int i=0;i<N_AC_TYPES;i++){if(strcmp(kACCodes[i],g_ac_type)==0){presel=i;break;}}
    lv_roller_set_selected(g_ac_roller,presel,LV_ANIM_OFF);
    // Description label (updates on scroll)
    g_ac_type_desc=lv_label_create(p);
    lv_label_set_text(g_ac_type_desc,kACLabels[presel]);
    lv_obj_set_style_text_color(g_ac_type_desc,TGREY(),0);
    lv_obj_set_style_text_font(g_ac_type_desc,&lv_font_montserrat_12,0);
    lv_obj_set_width(g_ac_type_desc,280);
    lv_obj_set_style_text_align(g_ac_type_desc,LV_TEXT_ALIGN_CENTER,0);
    lv_label_set_long_mode(g_ac_type_desc,LV_LABEL_LONG_WRAP);
    lv_obj_align(g_ac_type_desc,LV_ALIGN_TOP_MID,0,136);
    // OK button
    lv_obj_t*ok=lv_btn_create(p);lv_obj_set_size(ok,120,34);
    lv_obj_align(ok,LV_ALIGN_TOP_MID,0,168);
    lv_obj_set_style_bg_color(ok,lv_color_hex(0x1f4068),0);lv_obj_set_style_radius(ok,10,0);
    lv_obj_set_style_border_width(ok,0,0);lv_obj_set_style_shadow_opa(ok,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(ok,_ac_roller_ok_cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t*okl=lv_label_create(ok);lv_label_set_text(okl,"OK");
    lv_obj_set_style_text_color(okl,TFG(),0);lv_obj_set_style_text_font(okl,&lv_font_montserrat_16,0);
    lv_obj_center(okl);}

    // ── Container 2: HEX keyboard (6 digits 0-9/A-F) ─────────────────────────
    {lv_obj_t*p=g_ac_ctn[2];
    int bw=64,bh=34,gp=8;
    int x0=(480-(4*(bw+gp)-gp))/2;
    static const char* hr[4]={"0123","4567","89AB","CDEF"};
    for(int r=0;r<4;r++)for(int c=0;c<4;c++){
        char ch[2]={hr[r][c],0};
        mkAcKey(p,ch,x0+c*(bw+gp),8+r*(bh+gp),bw,bh,(intptr_t)hr[r][c]);}
    mkAcKey(p,LV_SYMBOL_BACKSPACE,(480-256)/2,8+4*(bh+gp),110,34,201);
    mkAcKey(p,"OK",(480-256)/2+126,8+4*(bh+gp),130,34,200);}

    // init state
    strlcpy(g_ac_tmp,g_ac_reg,sizeof(g_ac_tmp));
    g_ac_tab=0;}

static void _open_aircraft_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(!g_ac_ov)mkAircraftOverlay();}

// Boot animation — runs on page 0 before it goes live
void runBootOnPage(){
    // Logos visibles — pause splash
    lv_timer_handler();delay(900);

    // PANEL
    lv_label_set_text(r_boot_panel,"  PANEL    ...");
    lv_obj_set_style_text_color(r_boot_panel,TGREY(),0);lv_timer_handler();delay(300);
    lv_label_set_text(r_boot_panel,"● PANEL    OK");
    lv_obj_set_style_text_color(r_boot_panel,C_GREEN,0);lv_timer_handler();

    // LVGL
    lv_label_set_text(r_boot_lvgl,"  LVGL     ...");
    lv_obj_set_style_text_color(r_boot_lvgl,TGREY(),0);lv_timer_handler();delay(250);
    lv_label_set_text(r_boot_lvgl,"● LVGL     OK");
    lv_obj_set_style_text_color(r_boot_lvgl,C_GREEN,0);lv_timer_handler();

    // BLE
    lv_label_set_text(r_boot_ble,"  BLE      ...");
    lv_obj_set_style_text_color(r_boot_ble,TGREY(),0);lv_timer_handler();delay(250);
    BLEDevice::init("ATCORE-TRGB");
    lv_label_set_text(r_boot_ble,"● BLE      OK");
    lv_obj_set_style_text_color(r_boot_ble,C_GREEN,0);lv_timer_handler();

    // AT-CORE scan
    lv_label_set_text(r_boot_core,"  AT-CORE  ...");
    lv_obj_set_style_text_color(r_boot_core,TGREY(),0);lv_timer_handler();delay(350);
    startScan();
    lv_label_set_text(r_boot_core,"● AT-CORE  SCAN");
    lv_obj_set_style_text_color(r_boot_core,C_AMBER,0);lv_timer_handler();
    delay(500);
    g_bootDone=true;
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

    // Tab pills 52×32 — outer edge is AT the display circle boundary (8-12px behind bezel).
    // The circular LCD naturally clips the outer rounded corner → flat outer edge = "D" shape.
    // Only the inner rounded end (radius=16 half-circle) is fully visible.
    // Left:  Battery  y_c=96  x=40   SafeSky y_c=134 x=17   FLARM y_c=172 x=2   ADS-B y_c=210 x=-6
    // Right: GPS      y_c=96  x=388  LTE     y_c=134 x=411  WiFi  y_c=172 x=426  BLE   y_c=210 x=434
    r_hdr_bat  = mkTabPill(p, LV_SYMBOL_CHARGE,       40, 80);
    r_hdr_sky  = mkImgPill(p, &img_safesky,           17, 118);
    r_hdr_flrm = mkImgPill(p, &img_flarm,              2, 156);
    r_hdr_adsb = mkTabPill(p, "ADS-B",                -6, 194);
    // ADS-B: font_10 + shift label toward visible inner side so text clears the physical bezel
    lv_obj_set_style_text_font(r_hdr_adsb,&lv_font_montserrat_10,0);
    lv_obj_align(r_hdr_adsb, LV_ALIGN_CENTER, 12, 0);
    r_hdr_gps  = mkTabPill(p, LV_SYMBOL_GPS,         388, 80);
    r_hdr_lte  = mkLTEPill(p, 411, 118);
    r_hdr_wifi = mkTabPill(p, LV_SYMBOL_WIFI,        426, 156);
    r_hdr_ble  = mkTabPill(p, LV_SYMBOL_BLUETOOTH,   434, 194);

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
    lv_obj_set_style_text_color(r_co_text,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(r_co_text,&lv_font_montserrat_12,0);lv_obj_set_pos(r_co_text,366,364);
    r_co_val=lv_label_create(p);lv_label_set_text(r_co_val,"");
    lv_obj_set_style_text_font(r_co_val,&lv_font_montserrat_12,0);lv_obj_set_pos(r_co_val,360,380);

    // Traffic VL3 icons (bitmap rotated) + speed vector + callsign + alt
    for(int i=0;i<MAX_TRF;i++){
        r_vect_pts[i][0]={RAD_CX,RAD_CY};r_vect_pts[i][1]={RAD_CX,RAD_CY};
        r_trf_img[i]=lv_img_create(p);
        lv_img_set_src(r_trf_img[i],&img_dot);
        lv_img_set_zoom(r_trf_img[i],kIconZoom[g_cfg.icon_sz]);
        lv_img_set_pivot(r_trf_img[i],24,24);
        r_trf_last_type[i]=-1;
        lv_obj_set_style_img_recolor(r_trf_img[i],TFG(),0);
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
    lv_label_set_text(s_grnd_v, g_cfg.show_grnd?"ON":"OFF");
    lv_label_set_text(s_theme_v,g_cfg.dark?"DARK":"LIGHT");
    lv_label_set_text(s_icon_sz_v,kIconSzNames[g_cfg.icon_sz&2]);
    snprintf(b,12,"%dnm",g_cfg.scale_nm); lv_label_set_text(r_radar_scale_lbl,b);
    panel.setBrightness(g_cfg.brightness);}

static void cbSetBtn(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int id=(int)(intptr_t)lv_event_get_user_data(e);
    int si=0;for(int i=0;i<7;i++)if(kScaleOpts[i]==g_cfg.scale_nm)si=i;
    switch(id){
        case 0:si=max(si-1,0);g_cfg.scale_nm=kScaleOpts[si];break;
        case 1:si=min(si+1,6);g_cfg.scale_nm=kScaleOpts[si];break;
        case 2:g_cfg.vfilt_ft=max((int)g_cfg.vfilt_ft-500,500);break;
        case 3:g_cfg.vfilt_ft=min((int)g_cfg.vfilt_ft+500,5000);break;
        case 4:case 5:g_cfg.dist_nm=!g_cfg.dist_nm;break;
        case 6:case 7:g_cfg.alt_ft=!g_cfg.alt_ft;break;
        case 8:g_cfg.brightness=max((int)g_cfg.brightness-16,4);break;
        case 9:g_cfg.brightness=min((int)g_cfg.brightness+16,255);break;
        case 10:g_cfg.trf_src=(g_cfg.trf_src+3)%4;break;
        case 11:g_cfg.trf_src=(g_cfg.trf_src+1)%4;break;
        case 14:case 15:g_cfg.show_grnd=!g_cfg.show_grnd;break;
        case 12:case 13:g_cfg.dark=!g_cfg.dark;g_rebuildPages=true;break;
        case 16:g_cfg.icon_sz=max((int)g_cfg.icon_sz-1,0);for(int i=0;i<MAX_TRF;i++)lv_img_set_zoom(r_trf_img[i],kIconZoom[g_cfg.icon_sz]);break;
        case 17:g_cfg.icon_sz=min((int)g_cfg.icon_sz+1,2);for(int i=0;i<MAX_TRF;i++)lv_img_set_zoom(r_trf_img[i],kIconZoom[g_cfg.icon_sz]);break;}
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

static lv_obj_t* mkSetRowBtn(lv_obj_t*p,const char*k,int y,const char*v,lv_event_cb_t cb){
    mkLblP(p,k,TGREY(),&lv_font_montserrat_16,88,y);
    lv_obj_t*vl=mkLblP(p,v,TFG(),&lv_font_montserrat_14,215,y+1);
    lv_color_t bg=g_dark_theme?lv_color_hex(0x1f4068):lv_color_hex(0xdde1e7);
    lv_obj_t*b=lv_btn_create(p);lv_obj_set_size(b,80,30);lv_obj_set_pos(b,285,y-4);
    lv_obj_set_style_bg_color(b,bg,0);lv_obj_set_style_border_width(b,0,0);
    lv_obj_set_style_radius(b,6,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,"EDIT");
    lv_obj_set_style_text_color(l,TFG(),0);lv_obj_center(l);
    return vl;}

void buildSettingsPage(){
    lv_obj_t*p=g_pages[2];char b[16];
    mkLbl(p,"SETTINGS",C_AMBER,&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,55);
    mkLbl(p,"CONFIG", TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0, 82);
    snprintf(b,16,"%dnm",g_cfg.scale_nm); s_scale_v=mkSetRow(p,"Scale", 100,b,0,1);
    snprintf(b,16,"%dft",g_cfg.vfilt_ft); s_vfilt_v=mkSetRow(p,"V-Filt",132,b,2,3);
    s_dist_v =mkSetRow(p,"Dist",  164,g_cfg.dist_nm?"NM":"km",4,5);
    s_alt_v  =mkSetRow(p,"Alt",   196,g_cfg.alt_ft?"ft":"m",  6,7);
    mkLbl(p,"DISPLAY",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,226);
    snprintf(b,16,"%d",g_cfg.brightness); s_bright_v=mkSetRow(p,"Bright",244,b,8,9);
    s_theme_v=mkSetRow(p,"Theme",276,g_cfg.dark?"DARK":"LIGHT",12,13);
    mkLbl(p,"TRAFFIC",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,306);
    s_src_v   =mkSetRow(p,"Source",324,kSrcNames[g_cfg.trf_src&3],10,11);
    s_grnd_v  =mkSetRow(p,"Ground",356,g_cfg.show_grnd?"ON":"OFF",14,15);
    s_icon_sz_v=mkSetRow(p,"Icons", 388,kIconSzNames[g_cfg.icon_sz&2],16,17);
    mkLbl(p,"AIRCRAFT",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,410);
    {char t[20];snprintf(t,20,"%s  %s",g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---");
    s_ac_v=mkSetRowBtn(p,"Aircraft",426,t,_open_aircraft_cb);}
    lv_obj_t*ver=mkLblP(p,"v0.6  ●  AT-VIEW",TGREY(),&lv_font_montserrat_12,168,452);
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

// ── Dead reckoning — radar blips (called every loop for smooth movement) ──────
// Both g_status.spd and spd_kt in knots → multiply by 0.5144 to get m/s.
// Capped at 10s to avoid runaway extrapolation on BLE dropout.
void updateRadarDR(){
    if(!g_traffic.valid||!g_status.valid)return;
    float dt=fminf((float)(millis()-g_traffic.recv_ms)/1000.0f,10.0f);
    float our_spd_ms=(float)g_status.spd*0.5144f;
    float our_rad=(float)g_status.hdg*(float)M_PI/180.0f;
    float our_dx=our_spd_ms*dt*sinf(our_rad);
    float our_dy=our_spd_ms*dt*cosf(our_rad);
    char b[32];
    for(int i=0;i<MAX_TRF;i++){
        if(i<g_traffic.count){
            TrafficEntry&e=g_traffic.t[i];
            // Filtre ground : masque les aéronefs à vitesse < 20 kt (taxi/stationnement).
            // Seuil 20 kt valable si AT-CORE fournit le champ "s" (spd_kt) dans le JSON TRAFFIC.
            // Si "s" absent, défaut = 100 kt → l'avion reste visible même filtré. Normal.
            float scale_m=(float)g_cfg.scale_nm*1852.0f;
            if((!g_cfg.show_grnd&&e.spd_kt<20)||(e.dist_m>scale_m)){
                lv_obj_add_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
            } else {
            // Convert last-known polar position to cartesian (absolute north/east)
            float b_rad=(float)e.bear_deg*(float)M_PI/180.0f;
            float ex=(float)e.dist_m*sinf(b_rad);
            float ny=(float)e.dist_m*cosf(b_rad);
            // Extrapolate: traffic moves on its heading, we move on ours
            float trf_spd_ms=(float)e.spd_kt*0.5144f;
            float trf_rad=(float)e.hdg_deg*(float)M_PI/180.0f;
            ex+=trf_spd_ms*dt*sinf(trf_rad)-our_dx;
            ny+=trf_spd_ms*dt*cosf(trf_rad)-our_dy;
            // Back to polar
            float dr_dist=sqrtf(ex*ex+ny*ny);
            float dr_bear=atan2f(ex,ny)*180.0f/(float)M_PI;
            if(dr_bear<0.0f)dr_bear+=360.0f;
            // Heading-up projection on screen
            int rb=((int)dr_bear-g_status.hdg+360)%360;
            float brd=(float)rb*(float)M_PI/180.0f;
            float dpx=fminf(dr_dist*(float)RAD_R/scale_m,(float)(RAD_R-8));
            int sx=(int)(RAD_CX+sinf(brd)*dpx);
            int sy=(int)(RAD_CY-cosf(brd)*dpx);
            int rel_hdg=((e.hdg_deg-g_status.hdg)%360+360)%360;
            float hr=(float)rel_hdg*(float)M_PI/180.0f;
            float cs=cosf(hr),sn=sinf(hr);
            lv_color_t col=dr_dist<1000?C_RED:dr_dist<3000?C_AMBER:TFG();
            if(e.type!=r_trf_last_type[i]){
                lv_img_set_src(r_trf_img[i],getAircraftIcon(e.type));
                r_trf_last_type[i]=e.type;}
            int ih=kIconHalf[g_cfg.icon_sz];
            lv_obj_set_pos(r_trf_img[i],sx-ih,sy-ih);
            lv_img_set_angle(r_trf_img[i],(int16_t)(rel_hdg*10));
            lv_obj_set_style_img_recolor(r_trf_img[i],col,0);
            lv_obj_clear_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);
            float px_per_nm=(float)RAD_R/(float)g_cfg.scale_nm;
            float vect_px=fmaxf(6.f,fminf((float)e.spd_kt/60.0f*px_per_nm,35.f));
            float fih=(float)ih;
            r_vect_pts[i][0]={(lv_coord_t)(sx+(int)(fih*sn)),(lv_coord_t)(sy-(int)(fih*cs))};
            r_vect_pts[i][1]={(lv_coord_t)(sx+(int)((fih+vect_px)*sn)),(lv_coord_t)(sy-(int)((fih+vect_px)*cs))};
            lv_line_set_points(r_trf_vect[i],r_vect_pts[i],2);
            lv_obj_clear_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_pos(r_radar_cs[i],sx+12,sy-8);lv_label_set_text(r_radar_cs[i],e.cs);
            lv_obj_set_style_text_color(r_radar_cs[i],e.visible?TFG():C_AMBER,0);
            lv_obj_clear_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
            snprintf(b,32,"%+d",e.alt_m); // already delta in hundreds of feet from AT-CORE
            lv_obj_set_pos(r_radar_alt[i],sx+12,sy+6);lv_label_set_text(r_radar_alt[i],b);
            lv_obj_set_style_text_color(r_radar_alt[i],col,0);
            lv_obj_clear_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
            } // end else (not grounded)
        }else{
            lv_obj_add_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);}}}

// ── Update all live data ──────────────────────────────────────────────────────
void updateAllPages(){
    char b[32];
    // Status page
    lv_label_set_text(r_title,g_connected?"● AT-CORE":"● SCAN");
    lv_obj_set_style_text_color(r_title,g_connected?C_GREEN:C_AMBER,0);
    if(g_status.valid){
        snprintf(b,32,"GPS %dsat",g_status.gps_sat);updSBox(0,b,g_status.gps_fix);
        snprintf(b,32,"LTE %d",g_status.csq);updSBox(1,b,g_status.csq>5);
        snprintf(b,32,"SD %dfr",g_status.frames);updSBox(2,b,g_status.sd_ok);
        updSBox(3,"BLE",g_connected);updSBox(4,"FLARM",g_status.flarm_ok);updSBox(5,"ADS-B",g_status.adsb_ok);
        if(g_status.gps_fix){snprintf(b,32,"%.4f / %.4f",g_status.lat,g_status.lon);lv_label_set_text(r_coords,b);}
        if(g_status.bat<0){lv_label_set_text(r_bat_p1,"BAT  ---%");lv_obj_set_style_text_color(r_bat_p1,TGREY(),0);}
        else if(g_status.charging){snprintf(b,32,"BAT  %d%%  " LV_SYMBOL_CHARGE,g_status.bat);lv_label_set_text(r_bat_p1,b);lv_obj_set_style_text_color(r_bat_p1,C_GREEN,0);}
        else{snprintf(b,32,"BAT  %d%%",g_status.bat);lv_label_set_text(r_bat_p1,b);
        lv_obj_set_style_text_color(r_bat_p1,g_status.bat>=50?C_GREEN:g_status.bat>=20?C_AMBER:C_RED,0);}
    }else{updSBox(3,"BLE",g_connected);}
    // Header — connectivity tabs + battery — B&W scheme: active=bright bg+black icon, inactive=dark bg+gray icon
    {static bool prev_gps=false,prev_lte=false,prev_ble=false;
     bool gps_ok=g_status.valid&&g_status.gps_fix;
     bool lte_ok=g_status.valid&&g_status.csq>5;
     if(gps_ok&&!prev_gps)flashTab(r_hdr_gps);
     if(lte_ok&&!prev_lte)flashTab(r_hdr_lte);
     if(g_connected&&!prev_ble)flashTab(r_hdr_ble);
     prev_gps=gps_ok;prev_lte=lte_ok;prev_ble=g_connected;
     #define SET_PILL_TXT(lbl,act) \
         lv_obj_set_style_text_color(lbl,(act)?PILL_IC_ON():PILL_IC_OFF(),0)
     #define SET_PILL_IMG(img,act) \
         lv_obj_set_style_img_recolor(img,(act)?PILL_IC_ON():PILL_IC_OFF(),0)
     // GPS
     SET_PILL_TXT(r_hdr_gps, gps_ok);
     // LTE — bars reflect signal level
     {int csq=g_status.valid?g_status.csq:0;
      int bars=csq>20?4:csq>14?3:csq>8?2:csq>3?1:0;
      for(int bb=0;bb<4;bb++)
          lv_obj_set_style_bg_color(r_hdr_lte_b[bb],bb<bars?PILL_IC_ON():PILL_IC_OFF(),0);}
     // WiFi — always inactive (T-RGB has no WiFi ground link)
     SET_PILL_TXT(r_hdr_wifi, false);
     // BLE
     SET_PILL_TXT(r_hdr_ble, g_connected);
     // SafeSky — active when AT-CORE streams live traffic
     {bool sky_ok=g_connected&&g_traffic.valid&&g_traffic.count>0;
      SET_PILL_IMG(r_hdr_sky, sky_ok);}
     // FLARM
     {bool flrm_ok=g_connected&&g_status.valid&&g_status.flarm_ok;
      SET_PILL_IMG(r_hdr_flrm, flrm_ok);}
     // ADS-B
     {bool adsb_ok=g_connected&&g_status.valid&&g_status.adsb_ok;
      SET_PILL_TXT(r_hdr_adsb, adsb_ok);}
     // Battery — g_status.bat (STATUS char, ~1s) prioritaire sur g_debug.bat_pct (DEBUG char).
     // Charging détecté par AT-CORE via tendance tension LiPo (champ "chg" JSON STATUS).
     // ⚠ TEST EN COURS : brancher AT-CORE sur secteur et vérifier "BAT xx% ⚡" page Status.
     {int bat=(g_status.valid&&g_status.bat>=0)?g_status.bat:
              (g_debug.valid&&g_debug.bat_pct>=0)?g_debug.bat_pct:-1;
       if(bat<0){
          // Pas de données — symbole charge grisé
          lv_label_set_text(r_hdr_bat,LV_SYMBOL_CHARGE);SET_PILL_TXT(r_hdr_bat,false);
      }else if(!g_status.charging){
          // En charge (USB) — éclair seul, actif/bright
          lv_label_set_text(r_hdr_bat,LV_SYMBOL_CHARGE);SET_PILL_TXT(r_hdr_bat,true);
      }else{
          // Sur batterie — jauge seule, sans %, rouge si <20%
          const char*bi=bat>=75?LV_SYMBOL_BATTERY_FULL:bat>=50?LV_SYMBOL_BATTERY_3:
                         bat>=25?LV_SYMBOL_BATTERY_2:bat>=10?LV_SYMBOL_BATTERY_1:LV_SYMBOL_BATTERY_EMPTY;
          lv_label_set_text(r_hdr_bat,bi);SET_PILL_TXT(r_hdr_bat,bat>=20);}}
     #undef SET_PILL_TXT
     #undef SET_PILL_IMG
     }
    // Auto-navigate to radar once BLE+GPS ready (one-shot per connection)
    if(!g_autoNavDone&&g_connected&&g_status.valid&&g_status.gps_fix&&g_page==0){
        g_autoNavDone=true;g_navPending=true;g_navPage=1;}
    // Radar — cap GPS heading-up + rotation cardinaux N/E/S/W
    // ⚠ TEST EN COURS : cap GPS valide uniquement en mouvement (>15 km/h).
    // À l'arrêt AT-CORE envoie hdg=0 (track over ground non calculable).
    // Si le cap reste bloqué à 0° en mouvement → vérifier que AT-CORE
    // envoie bien g_status.hdg != 0 via le champ "hdg" du JSON STATUS.
    // Évolution possible : ajouter un magnétomètre côté AT-CORE pour
    // un cap indépendant de la vitesse (cap magnétique vs cap sol).
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
    // Radar blips — handled by updateRadarDR() called every loop (dead reckoning)
    // CO gauge — ball position + ppm label
    if(g_flight.valid){
        int co=g_flight.co_ppm;
        float co_a=(30.0f+fminf((float)co,150.0f)/150.0f*30.0f)*(float)M_PI/180.0f;
        lv_obj_set_pos(r_co_ball,(int)(240.0f+212.0f*cosf(co_a))-6,(int)(240.0f+212.0f*sinf(co_a))-6);
        lv_obj_set_style_text_color(r_co_text,lv_color_hex(0x000000),0);
        lv_label_set_text(r_co_val,"");}
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
    cfgLoad();acLoad();
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
    // Auth — show overlay unless owner auto-login from NVS
    pilotDBLoad();
    if(g_pilot_cnt>0 && !checkOwnerNVS()) mkAuthOverlay();
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
    static uint32_t drLast=0;
    if(g_page==1&&now-drLast>=200){drLast=now;updateRadarDR();}
    lv_timer_handler();delay(5);}

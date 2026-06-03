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
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>   // OTA firmware AT-VIEW (WP7) — réception .bin via l'AP du WebServer
#include <ESPmDNS.h>
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
#define BLE_CHR_AUTH    "6E400007-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_PILOTS  "6E400008-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_CONFIG  "6E400009-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_CONTROL "6E40000A-B5A3-F393-E0A9-E50E24DCCA9E"  // write : binding {"cmd":"bind"|"unpair"}
#define BLE_CHR_FLIGHTS "6E40000B-B5A3-F393-E0A9-E50E24DCCA9E"  // read  : liste vols SD (WP8)
// ── Bypass auth pilote (temporaire) ───────────────────────────────────────────
// 1 = on saute la page #02 "Select your name" / #03 welcome : dès que la machine
// est appairée (BLE) + GPS fix, on file direct au radar. L'appairage machine
// (MAC AT-CORE en NVS) reste inchangé. Repasser à 0 pour réactiver l'auth pilote.
#define BYPASS_PILOT_AUTH 1
// BLE identity — loaded from NVS namespace "unit" at boot
static char g_unit_name[24]   = "ATVIEW-EBBY1-01";
static char g_paired_mac[18]  = "";              // empty = connect to first ATCORE- found
static char g_peer_name[24]   = "";              // name of the connected AT-CORE (cleared on disconnect)

#define C_AMBER  lv_color_hex(0xF5A623)
#define C_GREEN  lv_color_hex(0x22c55e)
#define C_CYAN   lv_color_hex(0x00E5FF)
#define C_BLUE   lv_color_hex(0x60a5fa)
#define C_RED    lv_color_hex(0xef4444)
#define C_ORANGE lv_color_hex(0xf97316)
#define C_BRAND  lv_color_hex(0x7393B4)  // bleu logo AeroTrace (provisoire)

static bool g_dark_theme = true;
static inline lv_color_t TBG()  {return g_dark_theme?lv_color_hex(0x000000):lv_color_hex(0xFFFFFF);}
static inline lv_color_t TFG()  {return g_dark_theme?lv_color_hex(0xFFFFFF):lv_color_hex(0x0f172a);}
static inline lv_color_t TGREY(){return g_dark_theme?lv_color_hex(0x6b7280):lv_color_hex(0x6b7280);}
static inline lv_color_t TGRID(){return g_dark_theme?lv_color_hex(0x2a2a2a):lv_color_hex(0xd0d0d0);}
static inline lv_color_t TRING(){return g_dark_theme?lv_color_hex(0x888888):lv_color_hex(0x9ca3af);}
static inline lv_color_t THDG() {return g_dark_theme?lv_color_hex(0x0d1b2a):lv_color_hex(0xe2e8f0);}
static inline lv_color_t PILL_IC_OFF(){return g_dark_theme?lv_color_hex(0x2d3f52):lv_color_hex(0xb0bcc8);}
static inline lv_color_t PILL_IC_ON() {return g_dark_theme?lv_color_hex(0xffffff):lv_color_hex(0x0d1117);}

// ── Data structs ──────────────────────────────────────────────────────────────
struct StatusData {
    int mode,gps_sat,csq,frames,alt,spd,hdg,bat; float lat,lon;
    bool gps_fix,sd_ok,flarm_ok,adsb_ok,charging,valid;
    uint8_t flt_phase;   // 0=fly 1=ended 2=closed 3=uploading 4=done 5=fail (tâche D)
    uint8_t upload_pct;  // 0..100 (tâche D)
    uint8_t flt_rdy;     // WP8 : 1=liste vols prête à lire (CHR_FLIGHTS)
    };
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
struct CfgData { uint8_t scale_nm,brightness,trf_src; bool dist_nm,alt_ft,dark,show_grnd,wifi_en,aip_en,ad_heli,spd_kt; int16_t vfilt_ft; uint8_t icon_sz; };
static CfgData     g_cfg={4,16,3,true,true,true,true,false,true,false,2000,2};
static Preferences g_prefs;

static StatusData  g_status  = {};
static FlightData  g_flight  = {};
static TrafficData g_traffic = {};
static AlertData   g_alert   = {};
static DebugData   g_debug   = {};
static volatile bool g_dataUpdated = false;

// ── Pilot DB / Auth ───────────────────────────────────────────────────────────
struct PilotEntry { char code[5]; char name[32]; char status[12]; char primary_icao[8]; char trigram[4]; bool is_instructor; };
#define MAX_PILOTS 24
static PilotEntry  g_pilots[MAX_PILOTS] = {};
static int         g_pilot_cnt = 0;
static char        g_pilots_date[16] = {};   // date du dernier chargement reussi (YYYY-MM-DD)
static char        g_aircraft_icao[8] = "";
struct AuthSession { char name[32]; char status[12]; char trigram[4]; bool is_owner; bool valid; };
static AuthSession g_session = {};
static char        s_session_pc[5] = {0};  // code pilote courant — pour re-lookup si pilots arrivent après auth

// ── BLE state ─────────────────────────────────────────────────────────────────
static BLEClient*              g_client = nullptr;
static BLERemoteService*       g_svc    = nullptr;
static BLERemoteCharacteristic *g_chrS=nullptr,*g_chrF=nullptr,
                                *g_chrT=nullptr,*g_chrA=nullptr,*g_chrD=nullptr,
                                *g_chrW=nullptr,   // AUTH write (6E400007)
                                *g_chrP=nullptr,   // PILOTS notify (6E400008)
                                *g_chrCfg=nullptr, // CONFIG write (6E400009) — identité aéronef
                                *g_chrCtl=nullptr, // CONTROL write (6E40000A) — binding bind/unpair
                                *g_chrFl=nullptr;  // FLIGHTS read (6E40000B) — liste vols SD (WP8)
// Pilot list BLE reassembly buffer
static char    g_prx_buf[4096] = {};
static int     g_prx_len       = 0;
static volatile bool g_connected=false, g_doConnect=false, g_doReconnect=false;
uint32_t g_connect_ms = 0;   // millis() au moment de la connexion BLE (0 si pas connecte)
static BLEAdvertisedDevice*    g_target = nullptr;

// ── Pairing AT-CORE (Phase 3) ─────────────────────────────────────────────────
// Quand aucun AT-CORE n'est lié (g_paired_mac vide), on N'AUTO-CONNECTE PAS au
// premier boîtier venu : on collecte ceux en mode pairing (manuf-data FF FF 01)
// et l'utilisateur choisit + confirme (LED fixe) → write {"cmd":"bind"} sur
// CHR_CONTROL. Empêche de se lier au boîtier d'un avion voisin sur un parking.
#define PAIR_MAX 6
struct PairCand{ char mac[18]; char name[24]; int rssi; uint32_t seen; };
static PairCand g_pcand[PAIR_MAX];
static int      g_pcand_n  = 0;
static SemaphoreHandle_t g_pcand_mx = nullptr;     // protège g_pcand (rempli dans le cb scan)
static volatile bool g_binding      = false;       // connexion dans le cadre d'une cérémonie → pas d'auto-save MAC
static volatile bool g_bind_confirm = false;       // connecté au candidat → attend confirmation utilisateur
static uint32_t g_bind_t0       = 0;               // millis() du tap candidat → timeout si pas de connexion
static char     g_bind_mac[18]  = {};              // MAC du candidat qu'on tente de lier
static char     g_bind_name[24] = {};
static lv_obj_t* g_pair_ov    = nullptr;           // overlay pairing (liste + confirmation)
static lv_obj_t* g_pair_list  = nullptr;           // conteneur liste candidats
static lv_obj_t* g_pair_confirm = nullptr;         // conteneur confirmation LED
static lv_obj_t* g_pair_cf_txt  = nullptr;         // label "La LED du boîtier ... fixe ?"
static lv_obj_t* g_pair_rows[PAIR_MAX] = {};       // boutons candidats pré-créés
static lv_obj_t* g_pair_row_lbl[PAIR_MAX] = {};

static bool advPairable(BLEAdvertisedDevice& d);
void pcandUpsert(BLEAdvertisedDevice& d);
void connectTarget(BLEAdvertisedDevice& d);
void sendCtl(const char* cmd);
static void macToBoxId(const char* mac, char* out, size_t sz);  // défini plus bas
void pairOverlayShow(); void pairOverlayHide();
void pairListRefresh(); void pairShowConfirm();

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
static bool             g_authShown=false;   // reset on disconnect → popup reapparaît

// ── Widget refs — Status page (page 0) ───────────────────────────────────────
static lv_obj_t *r_sess_trig,*r_sess_name;
// Nouveau layout page 0 — 6 check rows (cercle + label) + label AT-CORE + batterie + version
#define N_CHK 6
enum { CHK_CORE=0, CHK_BT=1, CHK_GPS=2, CHK_LTE=3, CHK_ADSB=4, CHK_OGN=5 };
static lv_obj_t *r_chk_dot[N_CHK]={};   // cercles
static lv_obj_t *r_chk_ico[N_CHK]={};   // ✓ blanc (visible si actif)
static lv_obj_t *r_chk_lbl[N_CHK]={};   // texte
static bool      g_chk_latched[N_CHK]={};  // une fois actif, V reste (reset au disconnect)
static lv_obj_t *r_p0_bat=nullptr;      // "Battery AT-CORE : XX%"

// ── Widget refs — Radar (page 1) ──────────────────────────────────────────────
#define RAD_CX 240
#define RAD_CY 240
#define RAD_R  175
static lv_obj_t *r_radar_hdg, *r_radar_scale_lbl, *r_radar_gs;
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
static lv_obj_t* g_auth_diag     = nullptr;  // ligne "DB: N pilots" en bas de page #02 (refresh live)
static char      g_auth_buf[5]   = {};
static int       g_auth_len      = 0;
static bool      g_auth_p2       = false;   // phase 2: instructor entry
static char      g_auth_scode[5] = {};      // pilot code saved during phase 2
static lv_obj_t* g_keypad_btns[11] = {};    // 10 digits + ENTER — toggle visibility en step PICK

// ── Picker pilote (step 0 — sélection nom avant PIN) ─────────────────────────
// step 0 = PICK pilot, 1 = PIN pilot, 2 = PICK instructor, 3 = PIN instructor
static uint8_t   g_auth_step          = 0;
static PilotEntry* g_picked_pilot     = nullptr;
static PilotEntry* g_picked_instructor= nullptr;
static lv_obj_t* g_picker_filter_lbl  = nullptr;
static lv_obj_t* g_picker_list        = nullptr;
static lv_obj_t* g_picker_rows[MAX_PILOTS]   = {};
static lv_obj_t* g_picker_row_trig[MAX_PILOTS] = {};
static lv_obj_t* g_picker_row_name[MAX_PILOTS] = {};
static lv_obj_t* g_picker_keys[27]    = {};   // A-Z + backspace
static char      g_picker_filter[8]   = {};
static int       g_picker_filter_len  = 0;
static int       g_picker_last_cnt    = 0;   // détecter arrivée nouveaux pilotes

// ── Aircraft identity ─────────────────────────────────────────────────────────
#define N_AC_TYPES 193
// Sorted alphabetically by ICAO code
static const char* kACCodes[N_AC_TYPES] = {
    "A109","A119","A210","A318","A319","A320","A321","ALO2",
    "ALO3","ALPH","AS50","AS55","AT43","AT45","AT72","AT75",
    "AT76","AW13","B06","B350","B37M","B38M","B39M","B407",
    "B429","B732","B733","B734","B735","B736","B737","B738",
    "B739","B752","B753","BALL","BE20","BE23","BE24","BE33",
    "BE35","BE36","BE55","BE58","BE76","BE9L","BSTL","C150",
    "C162","C172","C177","C182","C185","C206","C207","C208",
    "C210","C25A","C25B","C25C","C310","C340","C402","C404",
    "C414","C42","C421","C510","C525","C550","C560","C680",
    "C750","C90","CARE","CH60","CH70","CL30","CL35","CL60",
    "CTLS","CTSW","D11","D140","D18","DA20","DA40","DA42",
    "DA62","DH8A","DH8B","DH8C","DH8D","DR22","DR40","DV20",
    "E135","E145","E170","E175","E190","E195","E50P","E55P",
    "EC25","EC35","EC45","EC55","EV10","EV97","EVSS","F2TH",
    "F7X","F8X","F900","FK12","FK14","FK9","FLTD","G109",
    "G115","G120","G150","G280","GLID","GYRO","HR10","JABI",
    "LJ35","LJ45","LJ60","M101","M20P","M20T","MCR1","P2002",
    "P2006","P2010","P2012","P28A","P28B","P28R","P28T","P32R",
    "P32T","P46T","P68","P92","PA18","PA34","PA44","PAY2",
    "PAY3","PC12","PC24","PC6T","PHMR","PION","PIVE","PIVI",
    "R200","R22","R300","R44","R66","RANS","S76","S92",
    "SF50","SHIP","SHRK","SR20","SR22","STNG","SV4","SVAN",
    "SVGE","T67","TB10","TB20","TB21","TB9","TBM7","TBM8",
    "TBM9","TWST","UHEL","ULAC","VIPI","VL3","Z42","Z526",
    "ZZZZ"};
static const char* kACLabels[N_AC_TYPES] = {
    "AW109","AW119 Koala","AT01 (A210)",
    "A318","A319/A319neo","A320/A320neo","A321/A321neo",
    "Alouette II","Alouette III","Alpha Trainer",
    "H125 AS350 Ecureuil","H130 (EC130)",
    "ATR 42-300/320","ATR 42-500","ATR 72","ATR 72-500","ATR 72-600","AW139",
    "Bell 206 JetRanger","King Air 350",
    "737 MAX 7","737 MAX 8","737 MAX 9","Bell 407","Bell 429",
    "737-200","737-300","737-400","737-500","737-600",
    "737-700/700W","737-800/800W","737-900/900ER","757-200","757-300",
    "Balloon / Ballon","King Air 200/B200",
    "Musketeer/Sport/Sundowner","Musketeer Super/Sierra",
    "Bonanza V35/F33/G33","Bonanza 35","Bonanza A36/G36",
    "Baron 55","Baron 58/58P/58TC","Duchess","King Air 90","Bristell B23",
    "150/152","162 Skycatcher","172 Skyhawk","177 Cardinal","182 Skylane",
    "185 Skywagon","206 Stationair","207 Skywagon","208 Caravan","210 Centurion",
    "Citation CJ2","Citation CJ3","Citation CJ4",
    "Cessna 310","Cessna 340","Cessna 402","Cessna 404 Titan",
    "Cessna 414 Chancellor","C42","Cessna 421 Golden Eagle",
    "Citation Mustang","CitationJet CJ1","Citation II/Bravo",
    "Citation V/Ultra","Citation Sovereign","Citation X","King Air C90",
    "Calidus/MTO Sport","CH-601 Zodiac","CH-700/750 Cruzer",
    "Challenger 300","Challenger 350","Challenger 600",
    "CT LS/CT SW","CT SW",
    "Jodel D.11/112/113","Jodel D.140 Mousquetaire","Jodel D.18",
    "DA 20 Katana","DA 40 Star","DA 42 Twin Star","DA 62",
    "Dash 8 Q100","Dash 8 Q200","Dash 8 Q300","Dash 8 Q400",
    "DR 220/221","DR 400","DV 20 Katana (Rotax)",
    "ERJ 135","ERJ 145","Embraer 170","Embraer 175","Embraer 190","Embraer 195",
    "Phenom 100","Phenom 300",
    "H225 Super Puma","H135 (EC135)","H145 (EC145)","H155 (EC155)",
    "EV-10 Raven","EV-97 Eurostar","Vision SF50 (jet)",
    "Falcon 2000","Falcon 7X","Falcon 8X","Falcon 900/900EX",
    "FK-12 Comet","FK-14 Polaris","FK-9 Mk IV/V","Quik GTR / Quik R",
    "G 109/109B Ranger","G 115/115T Acro","G 120 TP",
    "Gulfstream G150/Galaxy","Gulfstream G280",
    "Glider / Planeur","Gyrocopter","HR 100","J120/J160/J170",
    "Learjet 35/36","Learjet 45","Learjet 60",
    "PA-46-350P Malibu Mirage","M20 F/G/J/K","M20TN Acclaim/Ovation","MCR-01 UL/Club/VLA",
    "P2002 Sierra","P2006T","P2010","P2012 Traveller",
    "PA-28 Cherokee/Archer","PA-28R Arrow","PA-28R-201 Arrow III","PA-28R-201T Turbo Arrow",
    "PA-32R Lance/Saratoga","PA-32RT-300 Turbo Lance","PA-46-500TP Meridian",
    "P.68 Victor/Observer","P92 Echo/JS/RG","PA-18 Super Cub","PA-34 Seneca","PA-44 Seminole",
    "PA-42 Cheyenne II","PA-42 Cheyenne III","PC-12/PC-12NG","PC-24","PC-6 Turbo Porter",
    "HA-420 HondaJet","Pioneer 200/300","Virus (electric)","Virus 912/914",
    "HR 200 / R 2000","Robinson R22","R 3000","Robinson R44","Robinson R66",
    "S-6 Coyote / S-7 Courier","Sikorsky S-76","Sikorsky S-92",
    "Vision SF50","Airship / Dirigeable","Shark","SR20","SR22/SR22T",
    "TL-3000 Sting S4","SV-4 biplanes","Savannah / Savannah VG","Savage Cub/Classic",
    "T67 Firefly","TB 10 Tobago","TB 20 Trinidad","TB 21 Trinidad TC","TB 9 Tampico",
    "TBM 700","TBM 850/900","TBM 960","Twister",
    "ULM helicoptere","ULM avion","Virus SW","VL-3 / VL3 Evolution",
    "Z 42/43","Z 526 Trener","Autre type (ZZZZ)"};
char g_ac_reg[8]  = "";   // immatriculation ex "FJFVB"
char g_ac_type[8] = "";   // code OACI ex "VL3"
char g_ac_hex[7]  = "";   // hex transpondeur ex "38EDC5"
static lv_obj_t* g_ac_ov       = nullptr;
static lv_obj_t* g_ac_disp     = nullptr;
// Maintenance (Modèle 1 : hotspot téléphone à pousser vers AT-CORE + transfert vol)
char g_hs_ssid[33] = "";   // SSID hotspot (à pousser via BLE {"cmd":"wifi"})
char g_hs_pass[64] = "";   // mot de passe hotspot (NVS unit/hs_*, ≠ wifi_pass = AP propre AT-VIEW)
static lv_obj_t* g_maint_ov      = nullptr;
static lv_obj_t* g_maint_ssid_ta = nullptr;
static lv_obj_t* g_maint_pass_ta = nullptr;
static lv_obj_t* g_maint_kb      = nullptr;
static lv_obj_t* g_ac_hdr_reg  = nullptr;
static lv_obj_t* g_ac_hdr_typ  = nullptr;
static lv_obj_t* g_ac_hdr_hex  = nullptr;
static lv_obj_t* g_p0_acid     = nullptr;   // page 0 : ligne identité sous logo AEROTRACE
static lv_obj_t* g_ac_ctn[3]   = {};
static lv_obj_t* g_ac_tabs[3]  = {};
static lv_obj_t* g_ac_roller   = nullptr;
static lv_obj_t* g_ac_type_desc= nullptr;
static uint8_t   g_ac_tab      = 0;
static char      g_ac_tmp[8]    = "";
static char      g_ac_search[5] = "";
static lv_obj_t* g_ac_search_disp = nullptr;

// ── Widget refs — Settings (page 2) ───────────────────────────────────────────
static lv_obj_t *s_scale_v,*s_vfilt_v,*s_dist_v,*s_alt_v,*s_spd_v,*s_bright_v,*s_src_v,*s_theme_v,*s_grnd_v,*s_icon_sz_v;
static lv_obj_t *s_ac_v,*s_wifi_v,*s_sd_v;
static lv_obj_t *s_pg[2]  = {};
static uint8_t   s_pg_idx = 0;
static lv_obj_t *r_p2_bat = nullptr;   // "Battery AT-CORE : XX%" footer settings page

// ── SD card (AT-VIEW local) ───────────────────────────────────────────────────
static bool     g_sd_ok = false;

// ── AIP overlay data (PSRAM) ─────────────────────────────────────────────────
struct AipCtrEntry { uint16_t pt_start,n_pts; uint8_t type_id; };
#define AIP_MAX_CTR 1024
#define AIP_MAX_PTS 24000
#define AIP_MAX_AD  5500
static AipCtrEntry g_aip_ctr[AIP_MAX_CTR];
static int32_t*    g_aip_lat    = nullptr;   // ps_malloc, lat×1e6
static int32_t*    g_aip_lon    = nullptr;   // ps_malloc, lon×1e6
static uint16_t    g_aip_ctr_cnt= 0;
static uint16_t    g_aip_pts_cnt= 0;
struct AipAd { char icao[5]; int32_t lat_e6,lon_e6; uint8_t type_id; };
static AipAd*      g_aip_ads    = nullptr;   // ps_malloc
static uint16_t    g_aip_ad_cnt = 0;
static bool        g_aip_loaded = false;
static lv_obj_t*   r_aip_layer  = nullptr;
static lv_obj_t*   s_aip_v      = nullptr;
static lv_obj_t*   s_heli_v     = nullptr;
static uint32_t g_sd_gb = 0;

// ── WiFi AP + Web server ──────────────────────────────────────────────────────
static WebServer* g_webserver  = nullptr;
static bool       g_wifi_active = false;
static char       g_wifi_pass[16] = "atview1234"; // NVS unit/wifi_pass

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

// Check row "● label" — cercle brand-blue (toujours visible) + ✓ blanc si actif.
// Layout : cercle Ø22 px à x, texte à x+30, y+4.
void mkCheckRow(lv_obj_t*p,int idx,int x,int y,const char*txt){
    lv_obj_t*dot=lv_obj_create(p);
    lv_obj_set_size(dot,22,22);lv_obj_set_pos(dot,x,y);
    lv_obj_set_style_radius(dot,11,0);
    lv_obj_set_style_bg_color(dot,C_BRAND,0);lv_obj_set_style_bg_opa(dot,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(dot,0,0);
    lv_obj_set_style_shadow_opa(dot,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(dot,0,0);
    lv_obj_clear_flag(dot,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*ico=lv_label_create(dot);lv_label_set_text(ico,"");
    lv_obj_set_style_text_color(ico,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(ico,&lv_font_montserrat_14,0);
    lv_obj_center(ico);
    r_chk_dot[idx]=dot;r_chk_ico[idx]=ico;
    r_chk_lbl[idx]=mkLblP(p,txt,TGREY(),&lv_font_montserrat_14,x+30,y+4);
}
void updCheckRow(int idx,const char*txt,bool ok){
    if(!r_chk_dot[idx])return;
    // Latch : une fois actif, le V reste affiche jusqu'au reset BLE
    if(ok) g_chk_latched[idx]=true;
    bool show = g_chk_latched[idx];
    lv_label_set_text(r_chk_ico[idx],show?LV_SYMBOL_OK:"");
    // Fond page 0 blanc → texte noir si actif, gris sinon
    lv_obj_set_style_text_color(r_chk_lbl[idx],show?lv_color_hex(0x0f172a):TGREY(),0);
    if(txt)lv_label_set_text(r_chk_lbl[idx],txt);
}

// ── Parsers ───────────────────────────────────────────────────────────────────
void parseStatus(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_status.mode=d["mode"]|0;g_status.gps_sat=d["gps_sat"]|0;g_status.csq=d["csq"]|-1;
    g_status.frames=d["frames"]|0;g_status.alt=d["alt"]|0;g_status.spd=d["spd"]|0;
    g_status.hdg=d["hdg"]|0;g_status.bat=d["bat"]|-1;g_status.lat=d["lat"]|0.0f;g_status.lon=d["lon"]|0.0f;
    g_status.gps_fix=d["gps_fix"]|false;g_status.sd_ok=d["sd_ok"]|false;
    g_status.flarm_ok=d["flarm"]|false;g_status.adsb_ok=d["adsb"]|false;
    g_status.charging=d["chg"]|false;
    g_status.flt_phase=d["flt_ph"]|0;g_status.upload_pct=d["up_pct"]|0;g_status.flt_rdy=d["flt_rdy"]|0;
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
#define BLE_BUF 600   // > payload ATT max (~509 @ MTU 512) → pas de drop silencieux d'un notify proche limite
static void notifyS(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseStatus(b);}
static void notifyF(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseFlight(b);}
static void notifyT(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseTraffic(b);}
static void notifyA(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseAlerts(b);}
static void notifyD(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){if(l>=BLE_BUF)return;static char b[BLE_BUF];memcpy(b,d,l);b[l]=0;parseDebug(b);}

// PILOTS (6E400008) — framing: d[0]=0x01 start+data, 0x02 continue+data, 0x03 end+data+parse
// single-chunk: AT-CORE envoie directement 0x03+data (pas de START vide)
// multi-chunk:  0x01+data | 0x02+data... | 0x03+data
static void notifyP(BLERemoteCharacteristic*,uint8_t*d,size_t l,bool){
    Serial.printf("[Auth] notifyP: l=%u byte0=0x%02X\n",(unsigned)l,l>0?d[0]:0xFF);
    if(l<2){Serial.println("[Auth] notifyP: l<2, drop");return;}
    uint8_t cmd=d[0];size_t dlen=l-1;const char*chunk=(const char*)(d+1);
    if(cmd==0x01){g_prx_len=0;memset(g_prx_buf,0,sizeof(g_prx_buf));}
    if(g_prx_len+dlen<sizeof(g_prx_buf)-1){
        memcpy(g_prx_buf+g_prx_len,chunk,dlen);g_prx_len+=dlen;g_prx_buf[g_prx_len]=0;}
    else Serial.println("[Auth] notifyP: buffer overflow, chunk dropped");
    if(cmd==0x03){
        Serial.printf("[Auth] notifyP: end-of-stream, total=%d bytes\n",g_prx_len);
        int n=_parsePilotJSON(g_prx_buf);g_prx_len=0;
        if(n>0)Serial.printf("[Auth] pilots BLE ok (%d)\n",n);
        else   Serial.println("[Auth] pilots BLE rejected - DB preserved");}}

// Boîtier en mode pairing ? AT-CORE pousse une manuf-data FF FF <pairable>.
static bool advPairable(BLEAdvertisedDevice& d){
    if(!d.haveManufacturerData())return false;
    std::string m=d.getManufacturerData();
    if(m.size()<3)return false;
    return (uint8_t)m[0]==0xFF && (uint8_t)m[1]==0xFF && (uint8_t)m[2]==0x01;}

// Insère/actualise un candidat pairing dans g_pcand (appelé depuis le cb scan).
void pcandUpsert(BLEAdvertisedDevice& d){
    if(!g_pcand_mx)return;
    std::string mac=d.getAddress().toString();
    if(xSemaphoreTake(g_pcand_mx,pdMS_TO_TICKS(20))!=pdTRUE)return;
    int idx=-1;
    for(int i=0;i<g_pcand_n;i++) if(mac==std::string(g_pcand[i].mac)){idx=i;break;}
    if(idx<0 && g_pcand_n<PAIR_MAX) idx=g_pcand_n++;
    if(idx>=0){
        strlcpy(g_pcand[idx].mac,mac.c_str(),sizeof(g_pcand[idx].mac));
        strlcpy(g_pcand[idx].name,d.getName().c_str(),sizeof(g_pcand[idx].name));
        g_pcand[idx].rssi=d.haveRSSI()?d.getRSSI():0;
        g_pcand[idx].seen=millis();}
    xSemaphoreGive(g_pcand_mx);}

// Arme la connexion vers un device (chemin commun : reconnexion liée + pairing).
void connectTarget(BLEAdvertisedDevice& dev){
    BLEDevice::getScan()->stop();
    if(g_target){delete g_target;g_target=nullptr;}
    g_target=new BLEAdvertisedDevice(dev);g_doConnect=true;}

class ATCCB:public BLEClientCallbacks{
    void onConnect(BLEClient*)override{g_connected=true;g_dataUpdated=true;
        g_connect_ms=millis();
        Serial.println("[BLE] Connected");}
    void onDisconnect(BLEClient*)override{g_connected=false;g_autoNavDone=false;g_authShown=false;
        g_connect_ms=0;
        g_status.valid=g_flight.valid=g_traffic.valid=g_alert.valid=g_debug.valid=false;
        g_peer_name[0]=0;g_prx_len=0;  // reset pilots buffer — évite données résiduelles
        // Reset latches page 0 → progression repart de zero a la reconnexion
        for(int i=0;i<N_CHK;i++)g_chk_latched[i]=false;
        g_dataUpdated=true;g_doReconnect=true;Serial.println("[BLE] Disconnected");}};
class ATCAdv:public BLEAdvertisedDeviceCallbacks{
    void onResult(BLEAdvertisedDevice dev)override{
        String nm=dev.getName().c_str();
        if(!nm.startsWith("ATCORE-"))return;
        std::string mac=dev.getAddress().toString();
        if(g_paired_mac[0]!=0){
            // Déjà lié → on ne se connecte qu'à NOTRE boîtier (anti-cross-talk).
            if(mac!=std::string(g_paired_mac))return;
            connectTarget(dev);return;}
        // Non lié : cérémonie de pairing.
        if(g_binding){
            // On attend le candidat choisi par l'utilisateur dans la liste.
            if(mac==std::string(g_bind_mac))connectTarget(dev);
            return;}
        // Sinon : on collecte les boîtiers en mode pairing (sans se connecter).
        if(advPairable(dev))pcandUpsert(dev);}};
void acPushBLE();  // défini plus bas — appelé ici pour auto-push à la connexion
bool connectBLE(){
    if(!g_client){g_client=BLEDevice::createClient();g_client->setClientCallbacks(new ATCCB());}
    if(!g_client->connect(g_target))return false;
    // Persist paired MAC seulement hors cérémonie de pairing : pendant un bind,
    // on attend la confirmation utilisateur (LED fixe) avant de figer le MAC.
    if(!g_binding){
        unitSaveMac(g_target->getAddress().toString().c_str());
        // g_peer_name = Box ID (DD-EE-FF) du boîtier connecté, pas le nom BLE
        // générique (tous "ATCORE-EBBY1-01") → identifiant unique à l'écran.
        macToBoxId(g_target->getAddress().toString().c_str(), g_peer_name, sizeof(g_peer_name));}
    g_client->setMTU(512);g_svc=g_client->getService(BLE_SVC_UUID);
    if(!g_svc){g_client->disconnect();return false;}
    g_chrS=g_svc->getCharacteristic(BLE_CHR_STATUS);
    g_chrF=g_svc->getCharacteristic(BLE_CHR_FLIGHT);
    g_chrT=g_svc->getCharacteristic(BLE_CHR_TRAFFIC);
    g_chrA=g_svc->getCharacteristic(BLE_CHR_ALERTS);
    g_chrD=g_svc->getCharacteristic(BLE_CHR_DEBUG);
    g_chrW=g_svc->getCharacteristic(BLE_CHR_AUTH);
    g_chrP=g_svc->getCharacteristic(BLE_CHR_PILOTS);
    g_chrCfg=g_svc->getCharacteristic(BLE_CHR_CONFIG);
    g_chrCtl=g_svc->getCharacteristic(BLE_CHR_CONTROL);
    g_chrFl =g_svc->getCharacteristic(BLE_CHR_FLIGHTS);
    if(g_chrS&&g_chrS->canNotify())g_chrS->registerForNotify(notifyS);
    if(g_chrF&&g_chrF->canNotify())g_chrF->registerForNotify(notifyF);
    if(g_chrT&&g_chrT->canNotify())g_chrT->registerForNotify(notifyT);
    if(g_chrA&&g_chrA->canNotify())g_chrA->registerForNotify(notifyA);
    // CHR_DEBUG notify retiré : Bluedroid client limite CONFIG_BT_GATTC_NOTIF_REG_MAX=5,
    // CHR_PILOTS (6ème) échouait silencieusement → DB Firebase non chargée côté UI.
    // Les logs sysLog circulaires de DEBUG ne sont pas critiques pour AT-VIEW.
    // if(g_chrD&&g_chrD->canNotify())g_chrD->registerForNotify(notifyD);
    if(g_chrP&&g_chrP->canNotify())g_chrP->registerForNotify(notifyP);
    // Cérémonie de pairing : connecté au candidat → l'AT-CORE passe LED fixe.
    // On demande la confirmation visuelle avant d'écrire {"cmd":"bind"}.
    if(g_binding){g_bind_confirm=true;return true;}
    // Auto-push identité aéronef (reg/type/hex24) dès la connexion : un AT-CORE
    // au NVS vide (carte neuve / réinitialisée) repartait sinon sur son fallback
    // compilé (OO-E07) tant qu'on n'éditait pas l'écran Aircraft. acPushBLE no-op
    // si l'identité locale n'est pas renseignée ou si CHR_CONFIG non inscriptible.
    acPushBLE();
    return true;}
void startScan(){BLEScan*s=BLEDevice::getScan();s->setAdvertisedDeviceCallbacks(new ATCAdv());s->setActiveScan(true);s->start(5,false);}

// ── Navigation & swipe ────────────────────────────────────────────────────────
void switchPage(uint8_t np){
    if(g_inDebug){lv_obj_add_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);g_inDebug=false;}
    lv_obj_add_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);
    g_page=np;lv_obj_clear_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);}

static lv_coord_t g_swipe_sx=-1, g_swipe_lx=0, g_swipe_sy=0, g_swipe_ly=0;
// Vrai pendant le drag du slider brightness — bloque le swipe horizontal
// qui sinon déclencherait une navigation de page parasite.
static bool g_bright_drag=false;
static void swipeCb(lv_event_t*e){
    // Bloque la navigation tant que l'auth code ou le pairing n'est pas résolu
    if(g_auth_ov||g_pair_ov)return;
    if(g_bright_drag)return;
    lv_event_code_t code=lv_event_get_code(e);
    lv_indev_t*indev=lv_indev_get_act();if(!indev)return;
    lv_point_t pt;lv_indev_get_point(indev,&pt);
    if(code==LV_EVENT_PRESSED){g_swipe_sx=pt.x;g_swipe_lx=pt.x;g_swipe_sy=pt.y;g_swipe_ly=pt.y;}
    else if(code==LV_EVENT_PRESSING){g_swipe_lx=pt.x;g_swipe_ly=pt.y;}
    else if(code==LV_EVENT_RELEASED||code==LV_EVENT_PRESS_LOST){
        if(g_swipe_sx>=0){
            int dx=(int)g_swipe_lx-(int)g_swipe_sx;
            int dy=(int)g_swipe_ly-(int)g_swipe_sy;
            if(g_inDebug){
                if(abs(dx)>40){lv_obj_add_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);g_inDebug=false;}
            }else if(abs(dy)>abs(dx) && abs(dy)>60 && g_page==2){
                // Swipe vertical sur Settings → bascule sous-page (haut↔bas)
                uint8_t np = (dy<0) ? 1 : 0;
                if(np!=s_pg_idx && s_pg[s_pg_idx] && s_pg[np]){
                    lv_obj_add_flag(s_pg[s_pg_idx],LV_OBJ_FLAG_HIDDEN);
                    s_pg_idx=np;
                    lv_obj_clear_flag(s_pg[s_pg_idx],LV_OBJ_FLAG_HIDDEN);}
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
    // brightness = niveau hardware 0-16 (clé bright_lv, défaut 16=max)
    {uint8_t bl=g_prefs.getUChar("bright_lv",16); if(bl>16)bl=16; g_cfg.brightness=bl;}
    g_cfg.trf_src   =g_prefs.getUChar("trf_src",3);
    g_cfg.dist_nm   =g_prefs.getBool("dist_nm",true);
    g_cfg.alt_ft    =g_prefs.getBool("alt_ft",true);
    g_cfg.dark      =g_prefs.getBool("dark",true);
    g_cfg.show_grnd =g_prefs.getBool("show_grnd",true);
    g_cfg.wifi_en   =g_prefs.getBool("wifi_en",false);
    g_cfg.aip_en    =g_prefs.getBool("aip_en",true);
    g_cfg.ad_heli   =g_prefs.getBool("ad_heli",false);
    g_cfg.vfilt_ft  =g_prefs.getShort("vfilt",2000);
    g_cfg.icon_sz   =g_prefs.getUChar("icon_sz",2);
    g_cfg.spd_kt    =g_prefs.getBool("spd_kt",true);
    g_prefs.end();
    g_dark_theme=g_cfg.dark;}
void cfgSave(){
    g_prefs.begin("atview",false);
    g_prefs.putUChar("scale",g_cfg.scale_nm);
    g_prefs.putUChar("bright_lv",g_cfg.brightness);
    g_prefs.putUChar("trf_src",g_cfg.trf_src);
    g_prefs.putBool("dist_nm",g_cfg.dist_nm);
    g_prefs.putBool("alt_ft",g_cfg.alt_ft);
    g_prefs.putBool("dark",g_cfg.dark);
    g_prefs.putBool("show_grnd",g_cfg.show_grnd);
    g_prefs.putBool("wifi_en",g_cfg.wifi_en);
    g_prefs.putBool("aip_en",g_cfg.aip_en);
    g_prefs.putBool("ad_heli",g_cfg.ad_heli);
    g_prefs.putShort("vfilt",g_cfg.vfilt_ft);
    g_prefs.putUChar("icon_sz",g_cfg.icon_sz);
    g_prefs.putBool("spd_kt",g_cfg.spd_kt);
    g_prefs.end();}

void unitLoad(){
    Preferences p;p.begin("unit",true);
    p.getString("name","ATVIEW-EBBY1-01").toCharArray(g_unit_name,sizeof(g_unit_name));
    p.getString("paired_mac","").toCharArray(g_paired_mac,sizeof(g_paired_mac));
    p.getString("wifi_pass","atview1234").toCharArray(g_wifi_pass,sizeof(g_wifi_pass));
    p.getString("hs_ssid","").toCharArray(g_hs_ssid,sizeof(g_hs_ssid));
    p.getString("hs_pass","").toCharArray(g_hs_pass,sizeof(g_hs_pass));
    p.end();}
void unitSaveMac(const char*mac){
    strlcpy(g_paired_mac,mac,sizeof(g_paired_mac));
    Preferences p;p.begin("unit",false);p.putString("paired_mac",mac);p.end();}
// Persiste localement les creds hotspot téléphone (clés dédiées hs_ssid/hs_pass,
// distinctes de wifi_pass qui est le mot de passe de l'AP propre d'AT-VIEW).
void unitSaveHotspot(const char*ssid,const char*pass){
    strlcpy(g_hs_ssid,ssid,sizeof(g_hs_ssid));
    strlcpy(g_hs_pass,pass?pass:"",sizeof(g_hs_pass));
    Preferences p;p.begin("unit",false);
    p.putString("hs_ssid",g_hs_ssid);p.putString("hs_pass",g_hs_pass);p.end();}

// Efface la MAC AT-CORE pairée du NVS et de la RAM. À utiliser quand on
// change de carte AT-CORE (la MAC du nouveau hardware diffère). Sans ça
// le filtre ligne 517 rejette tous les devices sauf l'ancienne MAC.
// Déclenché par long-press sur le logo AT-VIEW dans la page Settings.
void unitForgetMac(){
    g_paired_mac[0]=0;
    Preferences p;p.begin("unit",false);p.remove("paired_mac");p.end();
    Serial.println("[BLE] Pair AT-CORE oublié — reboot");}

// Callback long-press logo AT-VIEW = oublie la pair BLE + reboot.
// On demande d'abord à l'AT-CORE de ré-armer son mode pairing ({"cmd":"unpair"})
// pour qu'il accepte un nouveau binding au prochain boot.
static void _cbForgetPair(lv_event_t*e){
    sendCtl("unpair");
    delay(150);
    unitForgetMac();
    delay(500);
    ESP.restart();}

// ── Pairing AT-CORE — overlay + binding (Phase 3) ─────────────────────────────
// Écrit une commande binding sur CHR_CONTROL (6E40000A).
void sendCtl(const char* cmd){
    if(!g_connected||!g_chrCtl||!g_chrCtl->canWrite())return;
    char p[32];snprintf(p,sizeof(p),"{\"cmd\":\"%s\"}",cmd);
    g_chrCtl->writeValue((uint8_t*)p,strlen(p),false);
    Serial.printf("[BLE] CTRL %s\n",p);}

// Échappe " et \ pour insertion sûre dans une chaîne JSON (SSID/pass WPA2 peuvent
// en contenir → sinon ArduinoJson côté AT-CORE rejette la trame).
static void jsonEsc(const char* in,char* out,size_t osz){
    size_t o=0;
    for(size_t i=0;in&&in[i]&&o+2<osz;i++){
        if(in[i]=='"'||in[i]=='\\')out[o++]='\\';
        out[o++]=in[i];
    }
    out[o]=0;}
// Pousse les credentials hotspot vers AT-CORE via CHR_CONTROL {"cmd":"wifi",...}.
// sendCtl ne suffit pas (buffer 32 + champs s/p). Limite AT-CORE = 200 B.
void sendWifiCreds(const char* ssid,const char* pass){
    if(!g_connected||!g_chrCtl||!g_chrCtl->canWrite())return;
    char es[68],ep[130];
    jsonEsc(ssid,es,sizeof(es));jsonEsc(pass,ep,sizeof(ep));
    char p[220];
    int n=snprintf(p,sizeof(p),"{\"cmd\":\"wifi\",\"s\":\"%s\",\"p\":\"%s\"}",es,ep);
    if(n<=0||n>200)return;   // respecte la limite write AT-CORE
    g_chrCtl->writeValue((uint8_t*)p,strlen(p),false);
    Serial.printf("[BLE] CTRL wifi s=%s\n",ssid);}

// Box ID = 3 derniers octets du MAC → "DD-EE-FF" (majuscules). Identifiant unique
// gravé en usine, imprimé sur le sticker sous le boîtier → authentification physique.
// mac attendu au format "aa:bb:cc:dd:ee:ff".
static void macToBoxId(const char* mac, char* out, size_t sz){
    int n=strlen(mac);
    if(n>=8){
        const char* p=mac+n-8;   // "dd:ee:ff"
        snprintf(out,sz,"%c%c-%c%c-%c%c",
            toupper(p[0]),toupper(p[1]),toupper(p[3]),toupper(p[4]),toupper(p[6]),toupper(p[7]));
    }else strlcpy(out,mac,sz);}

// Tap sur un candidat : on mémorise son MAC/nom, on passe en mode binding et on
// relance le scan → ATCAdv se connecte à CE boîtier (sans figer le MAC encore).
static void cbPairPick(lv_event_t*e){
    int i=(int)(intptr_t)lv_event_get_user_data(e);
    if(g_pcand_mx && xSemaphoreTake(g_pcand_mx,pdMS_TO_TICKS(20))==pdTRUE){
        if(i>=0&&i<g_pcand_n){
            strlcpy(g_bind_mac, g_pcand[i].mac, sizeof(g_bind_mac));
            strlcpy(g_bind_name,g_pcand[i].name,sizeof(g_bind_name));}
        xSemaphoreGive(g_pcand_mx);}
    if(!g_bind_mac[0])return;
    g_binding=true;g_bind_confirm=false;g_bind_t0=millis();
    Serial.printf("[PAIR] candidat %s (%s)\n",g_bind_name,g_bind_mac);
    startScan();}

// Confirmation utilisateur (LED fixe vérifiée) → fige le binding des deux côtés.
static void cbPairConfirm(lv_event_t*e){
    sendCtl("bind");
    unitSaveMac(g_bind_mac);
    macToBoxId(g_bind_mac,g_peer_name,sizeof(g_peer_name));   // Box ID, pas le nom générique
    g_binding=false;g_bind_confirm=false;
    Serial.printf("[PAIR] lié → %s\n",g_paired_mac);
    pairOverlayHide();
    acPushBLE();}

// Annulation : on se déconnecte du candidat et on revient à la liste.
static void cbPairCancel(lv_event_t*e){
    if(g_connected&&g_client)g_client->disconnect();
    g_binding=false;g_bind_confirm=false;g_bind_mac[0]=0;g_bind_name[0]=0;
    if(g_pair_confirm)lv_obj_add_flag(g_pair_confirm,LV_OBJ_FLAG_HIDDEN);
    if(g_pair_list)lv_obj_clear_flag(g_pair_list,LV_OBJ_FLAG_HIDDEN);}

void pairOverlayHide(){
    if(!g_pair_ov)return;
    lv_obj_del(g_pair_ov);
    g_pair_ov=g_pair_list=g_pair_confirm=g_pair_cf_txt=nullptr;
    for(int i=0;i<PAIR_MAX;i++){g_pair_rows[i]=g_pair_row_lbl[i]=nullptr;}}

void pairOverlayShow(){
    if(g_pair_ov)return;
    g_pair_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_pair_ov,480,480);lv_obj_set_pos(g_pair_ov,0,0);
    lv_obj_set_style_bg_color(g_pair_ov,lv_color_hex(0x000000),0);
    lv_obj_set_style_bg_opa(g_pair_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_pair_ov,0,0);lv_obj_set_style_radius(g_pair_ov,0,0);
    lv_obj_set_style_pad_all(g_pair_ov,0,0);
    lv_obj_clear_flag(g_pair_ov,LV_OBJ_FLAG_SCROLLABLE);

    mkLbl(g_pair_ov,"APPAIRAGE",C_BRAND,&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,70);
    mkLbl(g_pair_ov,"Choisis ton boitier",TGREY(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,102);

    // Conteneur liste candidats (rangées pré-créées, peuplées par pairListRefresh)
    g_pair_list=lv_obj_create(g_pair_ov);
    lv_obj_set_size(g_pair_list,360,220);lv_obj_align(g_pair_list,LV_ALIGN_TOP_MID,0,130);
    lv_obj_set_style_bg_color(g_pair_list,lv_color_hex(0x0d1117),0);
    lv_obj_set_style_bg_opa(g_pair_list,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(g_pair_list,C_BRAND,0);
    lv_obj_set_style_border_width(g_pair_list,1,0);
    lv_obj_set_style_radius(g_pair_list,8,0);lv_obj_set_style_pad_all(g_pair_list,6,0);
    lv_obj_set_scroll_dir(g_pair_list,LV_DIR_VER);
    lv_obj_set_scrollbar_mode(g_pair_list,LV_SCROLLBAR_MODE_AUTO);
    for(int i=0;i<PAIR_MAX;i++){
        lv_obj_t* row=lv_btn_create(g_pair_list);
        lv_obj_set_size(row,332,32);lv_obj_set_pos(row,0,i*36);
        lv_obj_set_style_bg_color(row,C_BRAND,0);
        lv_obj_set_style_bg_color(row,lv_color_hex(0x5a7a99),LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(row,LV_OPA_COVER,0);
        lv_obj_set_style_radius(row,6,0);lv_obj_set_style_shadow_opa(row,LV_OPA_TRANSP,0);
        lv_obj_add_event_cb(row,cbPairPick,LV_EVENT_CLICKED,(void*)(intptr_t)i);
        lv_obj_add_flag(row,LV_OBJ_FLAG_HIDDEN);
        lv_obj_t* l=lv_label_create(row);lv_label_set_text(l,"");
        lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);
        g_pair_rows[i]=row;g_pair_row_lbl[i]=l;}

    mkLbl(g_pair_ov,"Box ID = sticker sous le boitier",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,366);

    // Conteneur confirmation (caché tant qu'on n'est pas connecté au candidat)
    g_pair_confirm=lv_obj_create(g_pair_ov);
    lv_obj_set_size(g_pair_confirm,400,250);lv_obj_align(g_pair_confirm,LV_ALIGN_TOP_MID,0,120);
    lv_obj_set_style_bg_opa(g_pair_confirm,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(g_pair_confirm,0,0);lv_obj_set_style_pad_all(g_pair_confirm,0,0);
    lv_obj_clear_flag(g_pair_confirm,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_pair_confirm,LV_OBJ_FLAG_HIDDEN);
    g_pair_cf_txt=lv_label_create(g_pair_confirm);
    lv_label_set_text(g_pair_cf_txt,"");
    lv_obj_set_width(g_pair_cf_txt,360);
    lv_label_set_long_mode(g_pair_cf_txt,LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_pair_cf_txt,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_style_text_color(g_pair_cf_txt,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(g_pair_cf_txt,&lv_font_montserrat_16,0);
    lv_obj_align(g_pair_cf_txt,LV_ALIGN_TOP_MID,0,10);
    {lv_obj_t* b=lv_btn_create(g_pair_confirm);
     lv_obj_set_size(b,170,52);lv_obj_align(b,LV_ALIGN_BOTTOM_LEFT,10,-10);
     lv_obj_set_style_bg_color(b,C_GREEN,0);lv_obj_set_style_radius(b,8,0);
     lv_obj_add_event_cb(b,cbPairConfirm,LV_EVENT_CLICKED,NULL);
     lv_obj_t* l=lv_label_create(b);lv_label_set_text(l,"CONFIRMER");lv_obj_center(l);
     lv_obj_set_style_text_color(l,lv_color_hex(0x000000),0);}
    {lv_obj_t* b=lv_btn_create(g_pair_confirm);
     lv_obj_set_size(b,170,52);lv_obj_align(b,LV_ALIGN_BOTTOM_RIGHT,-10,-10);
     lv_obj_set_style_bg_color(b,C_RED,0);lv_obj_set_style_radius(b,8,0);
     lv_obj_add_event_cb(b,cbPairCancel,LV_EVENT_CLICKED,NULL);
     lv_obj_t* l=lv_label_create(b);lv_label_set_text(l,"ANNULER");lv_obj_center(l);
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);}}

// Bascule l'overlay en mode confirmation et renseigne le nom du boîtier.
void pairShowConfirm(){
    if(!g_pair_ov||!g_pair_confirm)return;
    if(!lv_obj_has_flag(g_pair_confirm,LV_OBJ_FLAG_HIDDEN))return;  // déjà affiché
    if(g_pair_cf_txt){
        char id[12];macToBoxId(g_bind_mac,id,sizeof(id));
        char b[96];snprintf(b,sizeof(b),
            "Boitier  %s\n\nVerifie le sticker\nsous le boitier,\npuis confirme.",
            id);
        lv_label_set_text(g_pair_cf_txt,b);}
    lv_obj_add_flag(g_pair_list,LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_pair_confirm,LV_OBJ_FLAG_HIDDEN);}

// Peuple la liste depuis g_pcand (entrées vues < 15 s). Throttlé par l'appelant.
// Mapping 1:1 slot==index candidat → le user_data des rows (figé à la création)
// reste valide, pas de ré-attache de callback.
void pairListRefresh(){
    if(!g_pair_ov||!g_pair_list)return;
    if(g_pair_confirm&&!lv_obj_has_flag(g_pair_confirm,LV_OBJ_FLAG_HIDDEN))return;
    uint32_t now=millis();
    if(g_pcand_mx && xSemaphoreTake(g_pcand_mx,pdMS_TO_TICKS(20))==pdTRUE){
        for(int i=0;i<PAIR_MAX;i++){
            if(!g_pair_rows[i])continue;
            bool fresh=(i<g_pcand_n)&&((now-g_pcand[i].seen)<15000);
            if(fresh){
                char id[12];macToBoxId(g_pcand[i].mac,id,sizeof(id));
                char b[40];snprintf(b,sizeof(b),"%s    %ddBm",id,g_pcand[i].rssi);
                lv_label_set_text(g_pair_row_lbl[i],b);
                lv_obj_clear_flag(g_pair_rows[i],LV_OBJ_FLAG_HIDDEN);
            }else lv_obj_add_flag(g_pair_rows[i],LV_OBJ_FLAG_HIDDEN);}
        xSemaphoreGive(g_pcand_mx);}}

// ── Forward declarations ──────────────────────────────────────────────────────
void buildStatusPage();
void buildRadarPage();
void buildSettingsPage();
void buildDebugPage();
void createSwipeHandlers();
void updSetPage();
static void cbSetBtn(lv_event_t*e);
void mkAircraftOverlay();
void acLoad();
void acSave();
void acSwitchTab(uint8_t tab);
void acUpdateHeader();
void p0UpdateAcId();

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

// ── Page 0 — Écran d'accueil et de chargement ─────────────────────────────────
// Layout maquette : AT-VIEW (petit) + AEROTRACE (grand) + loop + ligne pointillée
// + 6 check rows live (AT-CORE / Bluetooth / GPS / LTE / ADS-B / OGN-FLARM)
// + batterie AT-CORE + version en bas.
// Fond force blanc pour preserver la lisibilite du logo bicolore (A bleu + noir).
// Met à jour la ligne d'identité appareil (page 0, sous le logo AEROTRACE) :
// "IMMAT / TYPE / HEX" centré, ou rouge "APPAREIL NON CONFIGURE" tant que le pilote
// n'a pas encodé l'appareil (sans identité, AT-CORE n'émet RIEN vers SafeSky).
void p0UpdateAcId(){
    if(!g_p0_acid)return;
    if(g_ac_reg[0]&&g_ac_type[0]&&g_ac_hex[0]){
        char t[40];
        snprintf(t,sizeof(t),"%s  /  %s  /  %s",
            g_ac_reg, g_ac_type, g_ac_hex);
        lv_label_set_text(g_p0_acid,t);
        lv_obj_set_style_text_color(g_p0_acid,lv_color_hex(0x0f172a),0);  // fixe foncé : la page statut force un fond blanc (TFG=blanc en dark → invisible)
    }else{
        lv_label_set_text(g_p0_acid,LV_SYMBOL_WARNING " APPAREIL NON CONFIGURE");
        lv_obj_set_style_text_color(g_p0_acid,lv_color_hex(0xD32F2F),0);
    }
}

void buildStatusPage(){
    lv_obj_t*p=g_pages[0];
    lv_obj_set_style_bg_color(p,lv_color_hex(0xffffff),0);
    lv_obj_set_style_bg_opa(p,LV_OPA_COVER,0);

    // ── Logos bicolores (A bleu + reste noir) — zoom LVGL pour respecter proportions maquette
    lv_obj_t*lVw=lv_img_create(p);
    lv_img_set_src(lVw,&img_logo_atview);          // 110×22 source
    lv_img_set_zoom(lVw,320);                      // ×1.25 → ~137×27
    lv_obj_align(lVw,LV_ALIGN_TOP_MID,0,70);
    // Long-press 1.5s sur ce logo = oublie pair BLE + reboot. Même geste
    // qu'en page Settings (logo footer). Critique ici : c'est le SEUL moyen
    // de sortir d'un blocage "Scanning BLE…" quand la MAC stockée pointe
    // vers une carte AT-CORE absente (swap hardware) — Settings inaccessible
    // sans connexion BLE. Action explicite utilisateur, pas d'auto-pair
    // dangereux qui risquerait de se connecter au mauvais AT-CORE voisin.
    lv_obj_add_flag(lVw,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lVw,_cbForgetPair,LV_EVENT_LONG_PRESSED,NULL);

    lv_obj_t*lAt=lv_img_create(p);
    lv_img_set_src(lAt,&img_logo_aerotrace);       // 240×50 source
    lv_img_set_zoom(lAt,384);                      // ×1.5 → ~360×75
    lv_obj_align(lAt,LV_ALIGN_TOP_MID,0,108);

    // ── Identité appareil transmise à SafeSky (sous le logo, centrée).
    g_p0_acid=mkLbl(p,"",TFG(),&lv_font_montserrat_14,LV_ALIGN_TOP_MID,0,190);
    p0UpdateAcId();

    // ── 6 check rows (cercle bleu + label) — décalées à gauche, plus d'air entre lignes
    const int X = 105;  // colonne cercle (decalee a gauche)
    const int Y0= 218;  // 1ere ligne (sous AEROTRACE, gap ~30px)
    const int DY= 30;   // espacement vertical (6 rows : dernier dot top y=368, bottom y=390, battery debute y=418)
    mkCheckRow(p,CHK_CORE,X,Y0+0*DY,"AT-CORE");
    mkCheckRow(p,CHK_BT,  X,Y0+1*DY,"Bluetooth");
    mkCheckRow(p,CHK_GPS, X,Y0+2*DY,"GPS");
    mkCheckRow(p,CHK_LTE, X,Y0+3*DY,"LTE");
    mkCheckRow(p,CHK_ADSB,X,Y0+4*DY,"ADS-B / ADS-L");
    mkCheckRow(p,CHK_OGN, X,Y0+5*DY,"OGN / FLARM (868Mhz)");

    // ── Batterie AT-CORE + version
    r_p0_bat=mkLbl(p,"AT-CORE : ---%",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,418);
    mkLbl(p,"v0.7  --  2026-05-14",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,438);
}

// ── Pilot DB / Auth functions ─────────────────────────────────────────────────
// Accepte array nu OU wrapper {"_date":"YYYY-MM-DD","pilots":[...]}.
// Retourne nb pilotes charges ou -1 en cas d'erreur (DB precedente preservee).
static int _parsePilotJSON(const char* json){
    JsonDocument doc;
    if(deserializeJson(doc,json)){
        Serial.println("[Auth] pilot JSON parse error - keeping old DB");return -1;}
    JsonArray arr;
    const char* jdate = nullptr;
    if(doc.is<JsonObject>()){
        JsonObject obj = doc.as<JsonObject>();
        if(!obj["pilots"].is<JsonArray>()){
            Serial.println("[Auth] pilot JSON missing 'pilots' array - keeping old DB");return -1;}
        arr = obj["pilots"].as<JsonArray>();
        jdate = obj["_date"] | (const char*)nullptr;
    }else if(doc.is<JsonArray>()){
        arr = doc.as<JsonArray>();
    }else{
        Serial.println("[Auth] pilot JSON unexpected format - keeping old DB");return -1;}
    if(arr.size()==0){
        Serial.println("[Auth] pilot JSON empty - keeping old DB");return -1;}
    // Parse OK + non vide → on remplace
    g_pilot_cnt=0;
    for(JsonObject e:arr){
        if(g_pilot_cnt>=MAX_PILOTS)break;
        PilotEntry&t=g_pilots[g_pilot_cnt++];
        strlcpy(t.code,         e["c"]|"",sizeof(t.code));
        strlcpy(t.name,         e["n"]|"",sizeof(t.name));
        strlcpy(t.status,       e["r"]|"pilot",sizeof(t.status));
        strlcpy(t.primary_icao, "",sizeof(t.primary_icao));
        strlcpy(t.trigram,      e["t"]|"",sizeof(t.trigram));
        t.is_instructor =       e["i"]|false;}
    if(jdate)strlcpy(g_pilots_date,jdate,sizeof(g_pilots_date));
    Serial.printf("[Auth] %d pilots loaded (date=%s)\n",g_pilot_cnt,g_pilots_date[0]?g_pilots_date:"?");
    // Race condition: session ouverte avant réception pilots, ou SD sans trigram → re-lookup
    if(g_session.valid && (!g_session.name[0]||!g_session.trigram[0]) && s_session_pc[0]){
        PilotEntry*pe=pilotFind(s_session_pc);
        if(pe){
            strlcpy(g_session.name,pe->name,sizeof(g_session.name));
            strlcpy(g_session.status,pe->status,sizeof(g_session.status));
            strlcpy(g_session.trigram,pe->trigram,sizeof(g_session.trigram));
            g_session.is_owner=(strcmp(pe->status,"owner")==0);
            Serial.printf("[Auth] session trigram/name refreshed from BLE: %s %s\n",pe->trigram,pe->name);
        }
    }
    g_dataUpdated=true;  // force UI refresh (labels trigramme/nom)
    return g_pilot_cnt;
}

void pilotDBLoad(){
    if(!g_sd_ok)return;
    File f=SD_MMC.open("/pilots.json");
    if(!f){Serial.println("[Auth] No /pilots.json on SD");return;}
    String db=f.readString();f.close();
    _parsePilotJSON(db.c_str());
    Serial.println("[Auth] source: SD");}

PilotEntry* pilotFind(const char*code){
    for(int i=0;i<g_pilot_cnt;i++)if(strcmp(g_pilots[i].code,code)==0)return&g_pilots[i];
    return nullptr;}

bool checkOwnerNVS(){
    Preferences p;p.begin("auth",true);
    String oc=p.getString("owner","");p.end();
    if(oc.length()!=4)return false;
    char cb[5];oc.toCharArray(cb,5);
    PilotEntry*pe=pilotFind(cb);
    if(!pe||strcmp(pe->status,"owner")!=0)return false;
    strlcpy(g_session.name,pe->name,32);strlcpy(g_session.status,"owner",12);
    strlcpy(g_session.trigram,pe->trigram,sizeof(g_session.trigram));
    g_session.is_owner=true;g_session.valid=true;return true;}

// ── Page #02 — Auth code pilote (style page, plein ecran, fond blanc) ────────

static char s_instr_name[32] = {};  // Capture nom instructeur pour page #03 student

// ── Picker helpers ───────────────────────────────────────────────────────────
// Forward decls — fonctions auth définies plus bas mais référencées par les callbacks.
void authUpdateDots();

static inline char _upcase(char c){ return (c>='a'&&c<='z')?(c-'a'+'A'):c; }

// Prefix-match insensible casse sur trigramme OU début de name.
// Filtre vide → aucun pilote (variante B : on force la saisie d'au moins une lettre).
// En step 2 (picker instructeur), on ne montre que les pilotes is_instructor.
static bool pickerMatchPilot(const PilotEntry* pe,const char* flt,int flt_len){
    if(flt_len<=0) return false;
    if(g_auth_step==2 && !pe->is_instructor) return false;
    // trigramme
    bool tm = true;
    for(int i=0;i<flt_len;i++){
        char a=_upcase(pe->trigram[i]), b=_upcase(flt[i]);
        if(a==0 || a!=b){ tm=false; break; }
    }
    if(tm) return true;
    // name (premier caractère du champ)
    bool nm = true;
    for(int i=0;i<flt_len;i++){
        char a=_upcase(pe->name[i]), b=_upcase(flt[i]);
        if(a==0 || a!=b){ nm=false; break; }
    }
    return nm;
}

static void pickerUpdateFilterLabel(){
    if(!g_picker_filter_lbl) return;
    char b[20];
    if(g_picker_filter_len>0) snprintf(b,sizeof(b),"Filter: %s_",g_picker_filter);
    else                      snprintf(b,sizeof(b),"Type letters to filter");
    lv_label_set_text(g_picker_filter_lbl,b);
}

// Remplit/cache les rows selon g_pilot_cnt et le filtre courant.
// Réordonne verticalement les rows visibles pour éviter trous dans la liste.
static void pickerRefreshList(){
    if(!g_picker_list) return;
    const int ROW_H = 32;
    int y_off = 0;
    for(int i=0;i<MAX_PILOTS;i++){
        if(!g_picker_rows[i]) continue;
        if(i>=g_pilot_cnt){
            lv_obj_add_flag(g_picker_rows[i],LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        // mettre à jour labels (au cas où la DB a évolué depuis l'ouverture)
        if(g_picker_row_trig[i]) lv_label_set_text(g_picker_row_trig[i],g_pilots[i].trigram);
        if(g_picker_row_name[i]) lv_label_set_text(g_picker_row_name[i],g_pilots[i].name);
        bool m = pickerMatchPilot(&g_pilots[i],g_picker_filter,g_picker_filter_len);
        if(m){
            lv_obj_clear_flag(g_picker_rows[i],LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_pos(g_picker_rows[i],0,y_off);
            y_off += ROW_H;
        }else{
            lv_obj_add_flag(g_picker_rows[i],LV_OBJ_FLAG_HIDDEN);
        }
    }
    g_picker_last_cnt = g_pilot_cnt;
}

static inline void _setHidden(lv_obj_t* o,bool h){
    if(!o) return;
    if(h) lv_obj_add_flag(o,LV_OBJ_FLAG_HIDDEN);
    else  lv_obj_clear_flag(o,LV_OBJ_FLAG_HIDDEN);
}

// Bascule visibilité picker ↔ PIN. Met aussi à jour le texte du prompt.
// step 0 = pick pilot, 1 = PIN pilot, 2 = pick instructor, 3 = PIN instructor.
static void authStepSet(uint8_t step){
    g_auth_step = step;
    bool showPicker = (step==0 || step==2);
    bool showPIN    = (step==1 || step==3);
    // Sync legacy flag g_auth_p2 (utilisé par d'autres fonctions auth)
    g_auth_p2 = (step>=2);
    // Widgets picker
    _setHidden(g_picker_filter_lbl, !showPicker);
    _setHidden(g_picker_list,       !showPicker);
    for(int i=0;i<27;i++) _setHidden(g_picker_keys[i],!showPicker);
    // Widgets PIN (dots + keypad + DB diag)
    for(int i=0;i<4;i++)  _setHidden(g_auth_dots[i],   !showPIN);
    for(int i=0;i<11;i++) _setHidden(g_keypad_btns[i], !showPIN);
    _setHidden(g_auth_diag, !showPIN);
    // À l'entrée d'un picker, vider le filtre + refresh liste
    if(showPicker){
        memset(g_picker_filter,0,sizeof(g_picker_filter));
        g_picker_filter_len = 0;
        pickerUpdateFilterLabel();
        pickerRefreshList();
    }
    // Prompt text
    if(g_auth_prompt){
        char b[48];
        if(step==0)
            lv_label_set_text(g_auth_prompt,"Select your name");
        else if(step==1){
            snprintf(b,sizeof(b),"Code for %s",
                g_picked_pilot && g_picked_pilot->trigram[0]
                    ? g_picked_pilot->trigram : "?");
            lv_label_set_text(g_auth_prompt,b);
        }else if(step==2)
            lv_label_set_text(g_auth_prompt,"Select your instructor");
        else{
            snprintf(b,sizeof(b),"Code for instructor %s",
                g_picked_instructor && g_picked_instructor->trigram[0]
                    ? g_picked_instructor->trigram : "?");
            lv_label_set_text(g_auth_prompt,b);
        }
        lv_obj_set_style_text_color(g_auth_prompt,lv_color_hex(0x0f172a),0);
    }
}

static void _picker_key_cb(lv_event_t* e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED) return;
    intptr_t k=(intptr_t)lv_event_get_user_data(e);
    if(k==27){
        if(g_picker_filter_len>0){
            g_picker_filter_len--;
            g_picker_filter[g_picker_filter_len]=0;
        }
    }else if(k>=0 && k<=25){
        if(g_picker_filter_len < (int)sizeof(g_picker_filter)-1){
            g_picker_filter[g_picker_filter_len++] = 'A'+(char)k;
            g_picker_filter[g_picker_filter_len] = 0;
        }
    }
    pickerUpdateFilterLabel();
    pickerRefreshList();
}

static void _picker_pilot_cb(lv_event_t* e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED) return;
    intptr_t idx=(intptr_t)lv_event_get_user_data(e);
    if(idx<0 || idx>=g_pilot_cnt) return;
    g_auth_len=0; memset(g_auth_buf,0,5);
    if(g_auth_step==0){
        // Picker pilote → step 1 (PIN pilote)
        g_picked_pilot = &g_pilots[idx];
        authUpdateDots();
        authStepSet(1);
        Serial.printf("[Auth] picker pilot → %s (%s)\n",
            g_picked_pilot->name, g_picked_pilot->trigram);
    }else if(g_auth_step==2){
        // Picker instructeur → step 3 (PIN instructeur)
        g_picked_instructor = &g_pilots[idx];
        authUpdateDots();
        authStepSet(3);
        Serial.printf("[Auth] picker instr → %s (%s)\n",
            g_picked_instructor->name, g_picked_instructor->trigram);
    }
}

static const char* _authPromptText(){
    static char buf[48];
    if(g_auth_step==0) return "Select your name";
    if(g_auth_step==2) return "Select your instructor";
    if(g_auth_step==1 && g_picked_pilot && g_picked_pilot->trigram[0]){
        snprintf(buf,sizeof(buf),"Code for %s",g_picked_pilot->trigram);
        return buf;
    }
    if(g_auth_step==3 && g_picked_instructor && g_picked_instructor->trigram[0]){
        snprintf(buf,sizeof(buf),"Code for instructor %s",g_picked_instructor->trigram);
        return buf;
    }
    return g_auth_p2 ? "Encode your Instructor Code" : "Encode your Pilot Code";
}

void authUpdateDots(){
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_color(g_auth_dots[i],C_BRAND,0);
        lv_obj_set_style_border_color(g_auth_dots[i],C_BRAND,0);
        lv_obj_set_style_bg_opa(g_auth_dots[i],i<g_auth_len?LV_OPA_COVER:LV_OPA_TRANSP,0);}}

static void _auth_err_cb(lv_timer_t*t){
    // Reset visuel apres erreur : ronds bleus vides + prompt par phase
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_opa(g_auth_dots[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_bg_color(g_auth_dots[i],C_BRAND,0);
        lv_obj_set_style_border_color(g_auth_dots[i],C_BRAND,0);}
    if(g_auth_prompt){
        lv_label_set_text(g_auth_prompt,_authPromptText());
        lv_obj_set_style_text_color(g_auth_prompt,lv_color_hex(0x0f172a),0);}
    lv_timer_del(t);}

void authError(const char*msg){
    // Maquette : ronds rouges + prompt rouge "Wrong code - not recognised\nPlease try again"
    if(g_auth_prompt){
        lv_label_set_text(g_auth_prompt,msg?msg:"Wrong code - not recognised\nPlease try again");
        lv_obj_set_style_text_color(g_auth_prompt,C_RED,0);}
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_color(g_auth_dots[i],C_RED,0);
        lv_obj_set_style_bg_opa(g_auth_dots[i],LV_OPA_COVER,0);
        lv_obj_set_style_border_color(g_auth_dots[i],C_RED,0);}
    g_auth_len=0;memset(g_auth_buf,0,5);
    lv_timer_create(_auth_err_cb,1800,nullptr);}

// "Welcome back <TRG> !" en rouge + ronds verts pleins
static void _authOkVisual(const char*trg){
    if(g_auth_prompt){
        char b[40];snprintf(b,sizeof(b),"Welcome back %s !",trg&&trg[0]?trg:"???");
        lv_label_set_text(g_auth_prompt,b);
        lv_obj_set_style_text_color(g_auth_prompt,C_RED,0);}
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_color(g_auth_dots[i],C_GREEN,0);
        lv_obj_set_style_bg_opa(g_auth_dots[i],LV_OPA_COVER,0);
        lv_obj_set_style_border_color(g_auth_dots[i],C_GREEN,0);}}

// ── Page #03 — Have a nice flight (3 variantes) ──────────────────────────────
static lv_obj_t* g_welcome_ov = nullptr;
static void _welcomeClose(lv_timer_t*t){lv_timer_del(t);if(g_welcome_ov){lv_obj_del(g_welcome_ov);g_welcome_ov=nullptr;}}

static void showWelcome(const char* pilotName, const char* instrName){
    if(g_welcome_ov){lv_obj_del(g_welcome_ov);g_welcome_ov=nullptr;}
    bool dual = (instrName && instrName[0]);
    bool isOwner   = g_session.valid && strcmp(g_session.status,"owner")==0;
    bool isStudent = g_session.valid && strcmp(g_session.status,"student")==0;
    const char* statusStr = isStudent ? "STUDENT - Renter"
                          : isOwner   ? "PILOT - Owner"
                                      : "PILOT - Renter";

    g_welcome_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_welcome_ov,480,480); lv_obj_set_pos(g_welcome_ov,0,0);
    lv_obj_set_style_bg_color(g_welcome_ov,lv_color_hex(0xffffff),0);
    lv_obj_set_style_bg_opa(g_welcome_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_welcome_ov,0,0);
    lv_obj_set_style_radius(g_welcome_ov,0,0);
    lv_obj_clear_flag(g_welcome_ov,LV_OBJ_FLAG_SCROLLABLE);

    // Logo A en haut — lv_obj_align garantit le centrage LVGL
    lv_obj_t*lA=lv_img_create(g_welcome_ov);
    lv_img_set_src(lA,&img_logo_a);
    lv_obj_align(lA,LV_ALIGN_TOP_MID,0,18);

    // "Have a nice flight !"
    lv_obj_t*tf=lv_label_create(g_welcome_ov);
    lv_label_set_text(tf,"Have a nice flight !");
    lv_obj_set_style_text_color(tf,lv_color_hex(0x0f172a),0);
    lv_obj_set_style_text_font(tf,&lv_font_montserrat_16,0);
    lv_obj_align(tf,LV_ALIGN_TOP_MID,0,140);

    // Bandeau bleu plein avec nom du pilote (blanc gros)
    lv_obj_t*band=lv_obj_create(g_welcome_ov);
    lv_obj_set_size(band,360,52);
    lv_obj_set_pos(band,60,180);
    lv_obj_set_style_bg_color(band,C_BRAND,0);lv_obj_set_style_bg_opa(band,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(band,0,0);
    lv_obj_set_style_radius(band,2,0);
    lv_obj_set_style_shadow_opa(band,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(band,0,0);
    lv_obj_clear_flag(band,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*tn=lv_label_create(band);
    lv_label_set_text(tn,pilotName&&pilotName[0]?pilotName:"Pilote");
    lv_obj_set_style_text_color(tn,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(tn,&lv_font_montserrat_20,0);
    lv_obj_center(tn);

    if(dual){
        char b[80];snprintf(b,sizeof(b),"%s is your instructor today",instrName);
        lv_obj_t*ti=lv_label_create(g_welcome_ov);
        lv_label_set_text(ti,b);
        lv_obj_set_style_text_color(ti,C_RED,0);
        lv_obj_set_style_text_font(ti,&lv_font_montserrat_14,0);
        lv_obj_set_width(ti,420);
        lv_obj_set_style_text_align(ti,LV_TEXT_ALIGN_CENTER,0);
        lv_obj_align(ti,LV_ALIGN_TOP_MID,0,245);
    }

    char sb[40];snprintf(sb,sizeof(sb),"Status: %s",statusStr);
    lv_obj_t*ts=lv_label_create(g_welcome_ov);
    lv_label_set_text(ts,sb);
    lv_obj_set_style_text_color(ts,lv_color_hex(0x0f172a),0);
    lv_obj_set_style_text_font(ts,&lv_font_montserrat_16,0);
    lv_obj_align(ts,LV_ALIGN_TOP_MID,0,dual?290:330);

    // Footer batterie + version
    char bbuf[40];
    if(g_status.valid && g_status.bat>=0)
        snprintf(bbuf,sizeof(bbuf),"%s : %d%%",g_peer_name[0]?g_peer_name:"AT-CORE",g_status.bat);
    else
        snprintf(bbuf,sizeof(bbuf),"%s : ---%%",g_peer_name[0]?g_peer_name:"AT-CORE");
    mkLbl(g_welcome_ov,bbuf,TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,418);
    mkLbl(g_welcome_ov,"v0.7  --  2026-05-14",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,438);

    lv_timer_create(_welcomeClose,3000,nullptr);}

static void _authCloseOv(lv_timer_t*t){
    lv_timer_del(t);
    if(g_auth_ov){lv_obj_del(g_auth_ov);g_auth_ov=nullptr;
        g_auth_prompt=g_auth_name=g_auth_msg=g_auth_diag=nullptr;
        for(int i=0;i<4;i++)g_auth_dots[i]=nullptr;
        // Reset picker refs (les widgets sont détruits avec g_auth_ov)
        g_picker_filter_lbl=g_picker_list=nullptr;
        for(int i=0;i<MAX_PILOTS;i++){
            g_picker_rows[i]=g_picker_row_trig[i]=g_picker_row_name[i]=nullptr;}
        for(int i=0;i<27;i++) g_picker_keys[i]=nullptr;
        for(int i=0;i<11;i++) g_keypad_btns[i]=nullptr;
        g_picked_pilot=nullptr;
        g_picked_instructor=nullptr;}
    if(g_session.valid){
        const char* nm=g_session.name[0]?g_session.name:"Pilote";
        showWelcome(nm, s_instr_name[0]?s_instr_name:nullptr);}}

static void _authSendBLE(const char*pc,const char*ic){
    strlcpy(s_session_pc,pc,sizeof(s_session_pc));
    // Préfère g_picked_pilot (sélection picker non ambiguë) au fallback pilotFind
    // — sinon collision PIN ferait choisir le mauvais pilote pour la session locale.
    PilotEntry*pe= g_picked_pilot ? g_picked_pilot : pilotFind(pc);
    PilotEntry*ie= g_picked_instructor ? g_picked_instructor
                  : ((ic&&ic[0])?pilotFind(ic):nullptr);
    if(ie) strlcpy(s_instr_name,ie->name,sizeof(s_instr_name));
    else   s_instr_name[0]=0;
    const char* role = pe?pe->status:"pilot";
    const char* trig = pe?pe->trigram:"";
    char payload[96];
    if(ic&&ic[0])snprintf(payload,sizeof(payload),"{\"pc\":\"%s\",\"ic\":\"%s\",\"role\":\"%s\",\"t\":\"%s\"}",pc,ic,role,trig);
    else         snprintf(payload,sizeof(payload),"{\"pc\":\"%s\",\"role\":\"%s\",\"t\":\"%s\"}",pc,role,trig);
    // g_connected d'abord : un timer auth différé peut tirer après une déco →
    // g_chrW pointe alors un characteristic libéré (canWrite() crasherait).
    if(g_connected&&g_chrW&&g_chrW->canWrite())g_chrW->writeValue((uint8_t*)payload,strlen(payload),false);
    bool isOwner = (pe && strcmp(pe->status,"owner")==0);
    if(isOwner){Preferences p;p.begin("auth",false);p.putString("owner",pc);p.end();}
    g_session.valid    = true;
    g_session.is_owner = isOwner;
    strlcpy(g_session.name,    pe?pe->name:"",    sizeof(g_session.name));
    strlcpy(g_session.status,  pe?pe->status:"pilot", sizeof(g_session.status));
    strlcpy(g_session.trigram, pe?pe->trigram:"", sizeof(g_session.trigram));
    lv_timer_create(_authCloseOv,250,nullptr);}

// Timers de chainage apres "Welcome back" (1.5s d'affichage des ronds verts)
static void _authNextAfterPilotOk(lv_timer_t*t){
    lv_timer_del(t);_authSendBLE(g_auth_buf,nullptr);}
static void _authNextAfterStudentOk(lv_timer_t*t){
    lv_timer_del(t);
    g_auth_len=0;memset(g_auth_buf,0,5);
    for(int i=0;i<4;i++){
        lv_obj_set_style_bg_opa(g_auth_dots[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_bg_color(g_auth_dots[i],C_BRAND,0);
        lv_obj_set_style_border_color(g_auth_dots[i],C_BRAND,0);}
    // Pas de PIN keypad direct — on passe par le picker instructeur (step 2)
    g_picked_instructor = nullptr;
    authStepSet(2);
}
static void _authNextAfterInstructorOk(lv_timer_t*t){
    lv_timer_del(t);_authSendBLE(g_auth_scode,g_auth_buf);}

void authValidate(){
    if(g_auth_len<4)return;
    if(!g_auth_p2){
        // Phase 1 : le pilote a déjà été sélectionné via le picker.
        // On compare directement le PIN tapé contre le code du pilote choisi
        // — plus aucune ambiguïté de collision possible (cf. plan RGPD V1.x).
        PilotEntry* pe = g_picked_pilot;
        Serial.printf("[Auth] phase1 typed=%s picked=%s code=%s → %s\n",
            g_auth_buf,
            pe?pe->trigram:"(none)",
            pe?pe->code:"-",
            (pe && strcmp(g_auth_buf,pe->code)==0)?"OK":"REJECT");
        if(!pe || strcmp(g_auth_buf,pe->code)!=0){
            authError(nullptr);return;}
        bool isStudent = strcmp(pe->status,"student")==0;
        _authOkVisual(pe->trigram);
        if(isStudent){
            strlcpy(g_auth_scode,pe->code,5);
            lv_timer_create(_authNextAfterStudentOk,1500,nullptr);
        }else{
            lv_timer_create(_authNextAfterPilotOk,1500,nullptr);}
    }else{
        // Phase 2 (instructeur) — l'instructeur a été sélectionné via le picker.
        // Match direct sur g_picked_instructor → plus de collision PIN possible.
        PilotEntry* ie = g_picked_instructor;
        Serial.printf("[Auth] phase2 typed=%s picked=%s code=%s → %s\n",
            g_auth_buf,
            ie?ie->trigram:"(none)",
            ie?ie->code:"-",
            (ie && ie->is_instructor && strcmp(g_auth_buf,ie->code)==0)?"OK":"REJECT");
        if(!ie || !ie->is_instructor || strcmp(g_auth_buf,ie->code)!=0){
            authError(nullptr);return;}
        _authOkVisual(ie->trigram);
        lv_timer_create(_authNextAfterInstructorOk,1500,nullptr);}}

static void _auth_btn_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    intptr_t d=(intptr_t)lv_event_get_user_data(e);
    if(d==11){if(g_auth_len==4)authValidate();
    }else{if(g_auth_len<4){g_auth_buf[g_auth_len++]='0'+d;g_auth_buf[g_auth_len]=0;
        authUpdateDots();if(g_auth_len==4)authValidate();}}}

// Tap sur rond rempli = backspace. Si buf vide → retour au picker correspondant.
static void _auth_dot_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(g_auth_len>0){
        g_auth_len--;g_auth_buf[g_auth_len]=0;authUpdateDots();
    }else if(g_auth_step==1){
        // Retour au picker pilote — utile si mauvais pilote sélectionné
        g_picked_pilot=nullptr;
        authStepSet(0);
    }else if(g_auth_step==3){
        // Retour au picker instructeur
        g_picked_instructor=nullptr;
        authStepSet(2);
    }
}

// Page #02 — style page plein ecran (fond blanc, logo A, keypad brand, footer)
void mkAuthOverlay(){
    g_auth_len=0;memset(g_auth_buf,0,5);g_auth_p2=false;memset(g_auth_scode,0,5);
    s_instr_name[0]=0;
    // Reset picker state — étape 0 par défaut, filtre vide, aucun pilote sélectionné
    g_auth_step=0;
    g_picked_pilot=nullptr;
    g_picked_instructor=nullptr;
    memset(g_picker_filter,0,sizeof(g_picker_filter));
    g_picker_filter_len=0;
    g_picker_last_cnt=0;
    // Full screen background blanc
    g_auth_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_auth_ov,480,480);lv_obj_set_pos(g_auth_ov,0,0);
    lv_obj_set_style_bg_color(g_auth_ov,lv_color_hex(0xffffff),0);
    lv_obj_set_style_bg_opa(g_auth_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_auth_ov,0,0);lv_obj_set_style_radius(g_auth_ov,0,0);
    lv_obj_set_style_shadow_opa(g_auth_ov,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(g_auth_ov,0,0);
    lv_obj_clear_flag(g_auth_ov,LV_OBJ_FLAG_SCROLLABLE);

    // Logo A en haut centre — lv_obj_align garantit le centrage LVGL
    lv_obj_t*lA=lv_img_create(g_auth_ov);
    lv_img_set_src(lA,&img_logo_a);
    lv_obj_align(lA,LV_ALIGN_TOP_MID,0,12);

    // Prompt
    g_auth_prompt=lv_label_create(g_auth_ov);
    lv_label_set_text(g_auth_prompt,_authPromptText());
    lv_obj_set_style_text_color(g_auth_prompt,lv_color_hex(0x0f172a),0);
    lv_obj_set_style_text_font(g_auth_prompt,&lv_font_montserrat_16,0);
    lv_obj_set_width(g_auth_prompt,420);
    lv_obj_set_style_text_align(g_auth_prompt,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_align(g_auth_prompt,LV_ALIGN_TOP_MID,0,80);

    // 4 ronds PIN — cliquables = backspace
    int total = 4*22 + 3*18;
    int x0 = (480 - total)/2;
    for(int i=0;i<4;i++){
        lv_obj_t*dot=lv_obj_create(g_auth_ov);
        lv_obj_set_size(dot,22,22);
        lv_obj_set_pos(dot,x0+i*(22+18),130);
        lv_obj_set_style_radius(dot,LV_RADIUS_CIRCLE,0);
        lv_obj_set_style_bg_color(dot,C_BRAND,0);
        lv_obj_set_style_bg_opa(dot,LV_OPA_TRANSP,0);
        lv_obj_set_style_border_color(dot,C_BRAND,0);
        lv_obj_set_style_border_width(dot,3,0);
        lv_obj_set_style_shadow_opa(dot,LV_OPA_TRANSP,0);
        lv_obj_set_style_pad_all(dot,0,0);
        lv_obj_clear_flag(dot,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(dot,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(dot,_auth_dot_cb,LV_EVENT_CLICKED,nullptr);
        g_auth_dots[i]=dot;
    }
    g_auth_name=nullptr;
    g_auth_msg =nullptr;

    // Diagnostic DB pilotes (entre dots et keypad) — refresh live via updateAllPages
    char dbg[48];
    if(g_pilot_cnt>0)
        snprintf(dbg,sizeof(dbg),"DB: %d pilots (%s)",g_pilot_cnt,g_pilots_date[0]?g_pilots_date:"?");
    else
        snprintf(dbg,sizeof(dbg),"DB Firebase non chargee");
    g_auth_diag=mkLbl(g_auth_ov,dbg,g_pilot_cnt>0?TGREY():C_RED,&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,162);

    // Keypad 3x4 : 7-8-9 / 4-5-6 / 1-2-3 / 0 ENTER(2cols)
    static const struct { int8_t r,c,span; const char* lbl; int8_t val; } kK[] = {
        {0,0,1,"7",7},{0,1,1,"8",8},{0,2,1,"9",9},
        {1,0,1,"4",4},{1,1,1,"5",5},{1,2,1,"6",6},
        {2,0,1,"1",1},{2,1,1,"2",2},{2,2,1,"3",3},
        {3,0,1,"0",0},{3,1,2,"ENTER",11},
    };
    const int BW=70, BH=48, BG=6;
    const int KX=129, KY=190;
    for(unsigned i=0;i<sizeof(kK)/sizeof(kK[0]);i++){
        int x = KX + kK[i].c*(BW+BG);
        int y = KY + kK[i].r*(BH+BG);
        int w = kK[i].span==1 ? BW : (BW*2 + BG);
        lv_obj_t*btn=lv_btn_create(g_auth_ov);
        lv_obj_set_size(btn,w,BH);lv_obj_set_pos(btn,x,y);
        lv_obj_set_style_bg_color(btn,C_BRAND,0);
        lv_obj_set_style_bg_color(btn,lv_color_hex(0x5a7a99),LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(btn,LV_OPA_COVER,0);
        lv_obj_set_style_radius(btn,10,0);
        lv_obj_set_style_shadow_opa(btn,LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(btn,0,0);
        lv_obj_add_event_cb(btn,_auth_btn_cb,LV_EVENT_CLICKED,(void*)(intptr_t)kK[i].val);
        lv_obj_t*lb=lv_label_create(btn);lv_label_set_text(lb,kK[i].lbl);
        lv_obj_set_style_text_color(lb,lv_color_hex(0xffffff),0);
        lv_obj_set_style_text_font(lb,kK[i].span==1?&lv_font_montserrat_20:&lv_font_montserrat_16,0);
        lv_obj_center(lb);
        if(i<11) g_keypad_btns[i]=btn;}

    // ── PICKER (step 0) ──────────────────────────────────────────────────────
    // Label filtre — y=95, juste sous le prompt
    g_picker_filter_lbl = lv_label_create(g_auth_ov);
    lv_label_set_text(g_picker_filter_lbl,"Type letters to filter");
    lv_obj_set_style_text_color(g_picker_filter_lbl,TGREY(),0);
    lv_obj_set_style_text_font(g_picker_filter_lbl,&lv_font_montserrat_14,0);
    lv_obj_set_width(g_picker_filter_lbl,420);
    lv_obj_set_style_text_align(g_picker_filter_lbl,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_align(g_picker_filter_lbl,LV_ALIGN_TOP_MID,0,115);

    // Liste pilotes — container scrollable y=140..260 (h=120, ~3-4 rows visibles)
    const int PK_LIST_X=50, PK_LIST_Y=140, PK_LIST_W=380, PK_LIST_H=120, PK_ROW_H=32;
    g_picker_list = lv_obj_create(g_auth_ov);
    lv_obj_set_size(g_picker_list,PK_LIST_W,PK_LIST_H);
    lv_obj_set_pos(g_picker_list,PK_LIST_X,PK_LIST_Y);
    lv_obj_set_style_bg_color(g_picker_list,lv_color_hex(0xf3f4f6),0);
    lv_obj_set_style_bg_opa(g_picker_list,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(g_picker_list,C_BRAND,0);
    lv_obj_set_style_border_width(g_picker_list,1,0);
    lv_obj_set_style_radius(g_picker_list,6,0);
    lv_obj_set_style_pad_all(g_picker_list,4,0);
    lv_obj_set_scroll_dir(g_picker_list,LV_DIR_VER);
    lv_obj_set_scrollbar_mode(g_picker_list,LV_SCROLLBAR_MODE_AUTO);

    // Pré-création de MAX_PILOTS rows. pickerRefreshList() les peuplera/cachera.
    for(int i=0;i<MAX_PILOTS;i++){
        lv_obj_t* row = lv_btn_create(g_picker_list);
        lv_obj_set_size(row,PK_LIST_W-16,PK_ROW_H-2);
        lv_obj_set_pos(row,0,i*PK_ROW_H);
        lv_obj_set_style_bg_color(row,lv_color_hex(0xffffff),0);
        lv_obj_set_style_bg_color(row,lv_color_hex(0xe5e7eb),LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(row,LV_OPA_COVER,0);
        lv_obj_set_style_border_width(row,0,0);
        lv_obj_set_style_radius(row,4,0);
        lv_obj_set_style_shadow_opa(row,LV_OPA_TRANSP,0);
        lv_obj_set_style_pad_all(row,2,0);
        lv_obj_add_event_cb(row,_picker_pilot_cb,LV_EVENT_CLICKED,(void*)(intptr_t)i);
        lv_obj_add_flag(row,LV_OBJ_FLAG_HIDDEN);  // visible si pickerRefreshList valide
        // Trigramme à gauche (bold via Montserrat 16 brand color)
        lv_obj_t* tl = lv_label_create(row);
        lv_label_set_text(tl,"");
        lv_obj_set_style_text_color(tl,C_BRAND,0);
        lv_obj_set_style_text_font(tl,&lv_font_montserrat_16,0);
        lv_obj_set_pos(tl,4,4);
        // Nom à droite
        lv_obj_t* nl = lv_label_create(row);
        lv_label_set_text(nl,"");
        lv_obj_set_style_text_color(nl,lv_color_hex(0x0f172a),0);
        lv_obj_set_style_text_font(nl,&lv_font_montserrat_14,0);
        lv_obj_set_pos(nl,60,6);
        g_picker_rows[i]     = row;
        g_picker_row_trig[i] = tl;
        g_picker_row_name[i] = nl;
    }

    // Clavier alpha — 3 rangées 9/9/(8+⌫), conçu pour rester inscrit dans
    // l'écran rond 480×480 (largeur dispo ≈ 378 px à y=388, on cible 352 max).
    const int KEY_W=36, KEY_H=38, KEY_G=2;
    const int BS_W=48;
    const int ROW_Y0=270;
    static const char kAlpha1[] = "ABCDEFGHI";
    static const char kAlpha2[] = "JKLMNOPQR";
    static const char kAlpha3[] = "STUVWXYZ";
    auto mkAlphaKey = [&](const char* lbl,int x,int y,int w,intptr_t val){
        lv_obj_t* btn = lv_btn_create(g_auth_ov);
        lv_obj_set_size(btn,w,KEY_H);
        lv_obj_set_pos(btn,x,y);
        lv_obj_set_style_bg_color(btn,C_BRAND,0);
        lv_obj_set_style_bg_color(btn,lv_color_hex(0x5a7a99),LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(btn,LV_OPA_COVER,0);
        lv_obj_set_style_radius(btn,6,0);
        lv_obj_set_style_shadow_opa(btn,LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(btn,0,0);
        lv_obj_set_style_pad_all(btn,0,0);
        lv_obj_add_event_cb(btn,_picker_key_cb,LV_EVENT_CLICKED,(void*)val);
        lv_obj_t* l = lv_label_create(btn);
        lv_label_set_text(l,lbl);
        lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_16,0);
        lv_obj_center(l);
        return btn;
    };
    // Placer une rangée centrée horizontalement (largeur calculée auto)
    auto placeRow = [&](const char* letters, int n, int y){
        int total = n*KEY_W + (n-1)*KEY_G;
        int x = (480 - total) / 2;
        for(int i=0;i<n;i++){
            char s[2]={letters[i],0};
            g_picker_keys[(int)(letters[i]-'A')] = mkAlphaKey(s,
                x+i*(KEY_W+KEY_G), y, KEY_W, letters[i]-'A');
        }
    };
    placeRow(kAlpha1, 9, ROW_Y0);
    placeRow(kAlpha2, 9, ROW_Y0 + (KEY_H+KEY_G));
    // Rangée 3 : 8 lettres + backspace (large) — centré
    {
        int n=8;
        int total = n*KEY_W + BS_W + n*KEY_G;
        int x = (480 - total) / 2;
        int y = ROW_Y0 + 2*(KEY_H+KEY_G);
        for(int i=0;i<n;i++){
            char s[2]={kAlpha3[i],0};
            g_picker_keys[(int)(kAlpha3[i]-'A')] = mkAlphaKey(s,
                x+i*(KEY_W+KEY_G), y, KEY_W, kAlpha3[i]-'A');
        }
        g_picker_keys[26] = mkAlphaKey(LV_SYMBOL_BACKSPACE,
            x+n*(KEY_W+KEY_G), y, BS_W, 27);
    }

    // Footer : batterie + version (cohérent avec page #01)
    char bbuf[40];
    if(g_status.valid && g_status.bat>=0)
        snprintf(bbuf,sizeof(bbuf),"%s : %d%%",g_peer_name[0]?g_peer_name:"AT-CORE",g_status.bat);
    else
        snprintf(bbuf,sizeof(bbuf),"%s : ---%%",g_peer_name[0]?g_peer_name:"AT-CORE");
    mkLbl(g_auth_ov,bbuf,TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,418);
    mkLbl(g_auth_ov,"v0.7  --  2026-05-14",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,438);

    // Étape initiale = PICK : montre filtre + liste + clavier alpha, cache PIN
    pickerRefreshList();
    pickerUpdateFilterLabel();
    authStepSet(0);
}

// ── Upload progress overlay (tâche F) ────────────────────────────────────────
// Affiche un modal full-screen quand AT-CORE entre en phase post-vol
// (flt_phase >= FLT_ENDED). Auto-dismiss 5s après FLT_UPLOADED.
static lv_obj_t* g_up_ov       = nullptr;
static lv_obj_t* g_up_title    = nullptr;
static lv_obj_t* g_up_status   = nullptr;
static lv_obj_t* g_up_bar      = nullptr;
static lv_obj_t* g_up_pct_lbl  = nullptr;
static uint32_t  g_up_done_ms  = 0;
// Init à true → un ph=4 résiduel reçu au boot (état post-upload AT-CORE figé) est ignoré.
// Reset à false dès que ph repasse à 0 (nouveau vol), pour que le prochain cycle 1→2→3→4 affiche bien l'overlay.
static bool      g_up_acked    = true;

void mkUploadOverlay(){
    if(g_up_ov) return;
    g_up_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_up_ov,400,260);lv_obj_center(g_up_ov);
    lv_obj_set_style_bg_color(g_up_ov,lv_color_hex(0x0d1117),0);
    lv_obj_set_style_bg_opa(g_up_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(g_up_ov,C_AMBER,0);
    lv_obj_set_style_border_width(g_up_ov,2,0);
    lv_obj_set_style_radius(g_up_ov,12,0);
    lv_obj_clear_flag(g_up_ov,LV_OBJ_FLAG_SCROLLABLE);

    g_up_title=mkLblP(g_up_ov,"TRANSFERT VOL",C_AMBER,&lv_font_montserrat_20,118,18);
    g_up_status=mkLblP(g_up_ov,"Vol termine — close CSV...",lv_color_hex(0xffffff),&lv_font_montserrat_16,40,72);
    lv_obj_set_width(g_up_status,320);
    lv_obj_set_style_text_align(g_up_status,LV_TEXT_ALIGN_CENTER,0);

    g_up_bar=lv_bar_create(g_up_ov);
    lv_obj_set_size(g_up_bar,320,18);lv_obj_set_pos(g_up_bar,40,130);
    lv_bar_set_range(g_up_bar,0,100);lv_bar_set_value(g_up_bar,0,LV_ANIM_OFF);
    lv_obj_set_style_bg_color(g_up_bar,lv_color_hex(0x1f2937),0);
    lv_obj_set_style_bg_color(g_up_bar,C_AMBER,LV_PART_INDICATOR);

    g_up_pct_lbl=mkLblP(g_up_ov,"0%%",lv_color_hex(0xffffff),&lv_font_montserrat_16,178,160);
}

void hideUploadOverlay(){
    if(g_up_ov){lv_obj_del(g_up_ov);g_up_ov=nullptr;
        g_up_title=g_up_status=g_up_bar=g_up_pct_lbl=nullptr;}
    g_up_done_ms=0;
}

// Met à jour l'overlay selon flt_phase + upload_pct reçus par BLE STATUS.
// Doit être appelée régulièrement (depuis updateAllPages).
void updUploadOverlay(){
    uint8_t ph = g_status.flt_phase;
    uint8_t pct= g_status.upload_pct;

    // Phase 0 (FLYING) ou status invalide → hide overlay + reset ack
    if(!g_status.valid || ph == 0){
        if(g_up_ov) hideUploadOverlay();
        g_up_acked = false;
        return;
    }

    // Reset ack dès que la phase quitte 4 (nouveau cycle d'upload possible)
    if(ph != 4) g_up_acked = false;

    // Phase 4 déjà acquittée → ne pas réafficher
    if(ph == 4 && g_up_acked) return;

    // Auto-dismiss en phase 4 (UPLOADED) après 5s, sans attendre retour ph=0
    if(ph == 4 && g_up_done_ms && millis()-g_up_done_ms>5000){
        hideUploadOverlay();
        g_up_acked = true;
        return;
    }

    // Phase >= 1 → afficher si pas encore là
    if(!g_up_ov) mkUploadOverlay();

    // Texte selon phase
    const char* msg = "...";
    switch(ph){
        case 1: msg="Flight ended — closing CSV"; break;
        case 2: msg="CSV closed — waiting upload"; break;
        case 3: msg="Uploading to Firebase..."; break;
        case 4: msg="Transfer OK";
                if(g_up_done_ms==0) g_up_done_ms=millis(); break;
        case 5: msg="Failed — retrying...";
                g_up_done_ms=0; break;
    }
    if(g_up_status) lv_label_set_text(g_up_status,msg);
    if(g_up_bar)    lv_bar_set_value(g_up_bar,pct,LV_ANIM_OFF);
    if(g_up_pct_lbl){
        char p[8]; snprintf(p,sizeof(p),"%d%%",pct);
        lv_label_set_text(g_up_pct_lbl,p);
    }

    // Couleur progress + bordure : amber par défaut, vert sur succès, rouge sur fail
    if(g_up_bar){
        lv_color_t c = ph==4?C_GREEN : ph==5?C_RED : C_AMBER;
        lv_obj_set_style_bg_color(g_up_bar,c,LV_PART_INDICATOR);
        lv_obj_set_style_border_color(g_up_ov,c,0);
    }
}

// ── Aircraft identity overlay ─────────────────────────────────────────────────

void acLoad(){
    Preferences p;p.begin("aircraft",true);
    p.getString("reg","").toCharArray(g_ac_reg,sizeof(g_ac_reg));
    p.getString("type","").toCharArray(g_ac_type,sizeof(g_ac_type));
    p.getString("hex24","").toCharArray(g_ac_hex,sizeof(g_ac_hex));
    p.end();}

// Push identité aéronef vers AT-CORE via BLE CHR_CONFIG (6E400009).
// Payload : {"r":"FJFVB","t":"VL3","h":"38ED5C"}
void acPushBLE(){
    if(!g_chrCfg||!g_chrCfg->canWrite())return;
    if(!g_ac_reg[0]||!g_ac_hex[0])return;  // reg+hex24 obligatoires
    char payload[96];
    snprintf(payload,sizeof(payload),
        "{\"r\":\"%s\",\"t\":\"%s\",\"h\":\"%s\"}",
        g_ac_reg,g_ac_type,g_ac_hex);
    g_chrCfg->writeValue((uint8_t*)payload,strlen(payload),false);
    Serial.printf("[BLE] CONFIG push %s\n",payload);}

void acSave(){
    Preferences p;p.begin("aircraft",false);
    p.putString("reg",g_ac_reg);
    p.putString("type",g_ac_type);
    p.putString("hex24",g_ac_hex);
    p.end();
    acPushBLE();}

void acUpdateHeader(){
    p0UpdateAcId();                 // page 0 : refresh ligne identité (toujours)
    if(!g_ac_hdr_reg)return;        // overlay aircraft pas ouvert → stop ici
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
    if(d==202){ // FERMER — INTERDIT tant que immat+type+hex non saisis (encodage forcé).
                // Sans identité complète, la box ne transmet rien à SafeSky : on
                // bloque le pilote sur cette page et on le pousse au champ manquant.
        if(!g_ac_reg[0]){acSwitchTab(0);return;}
        if(!g_ac_type[0]){acSwitchTab(1);return;}
        if(!g_ac_hex[0]){acSwitchTab(2);return;}
        lv_obj_del(g_ac_ov);g_ac_ov=nullptr;return;}
    if(d==203){ // RESET — efface immat/type/hex (globals + NVS) pour ré-encodage
        g_ac_reg[0]=0;g_ac_type[0]=0;g_ac_hex[0]=0;g_ac_tmp[0]=0;
        Preferences pr;pr.begin("aircraft",false);pr.clear();pr.end();
        acUpdateHeader();      // refresh header overlay + ligne page 0 → "NON CONFIGURE"
        acSwitchTab(0);        // revient sur l'onglet IMMAT, champ vidé
        if(s_ac_v)lv_label_set_text(s_ac_v,"---  ---");
        return;}
    if(d==201){int l=strlen(g_ac_tmp);if(l>0)g_ac_tmp[l-1]=0;}
    else{int l=strlen(g_ac_tmp);int mx=(g_ac_tab==2)?6:7;
        if(l<mx){g_ac_tmp[l]=(char)d;g_ac_tmp[l+1]=0;}}
    _ac_disp_refresh();}

static void _ac_roller_change_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_VALUE_CHANGED)return;
    if(!g_ac_roller||!g_ac_type_desc)return;
    uint16_t sel=lv_roller_get_selected(g_ac_roller);
    if(sel<N_AC_TYPES)lv_label_set_text(g_ac_type_desc,kACLabels[sel]);
    g_ac_search[0]=0;
    if(g_ac_search_disp)lv_label_set_text(g_ac_search_disp,"_");}

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

static void _ac_search_key_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    intptr_t d=(intptr_t)lv_event_get_user_data(e);
    if(d==200){
        if(!g_ac_roller)return;
        uint16_t sel=lv_roller_get_selected(g_ac_roller);
        if(sel<N_AC_TYPES)strlcpy(g_ac_type,kACCodes[sel],sizeof(g_ac_type));
        acSave();acUpdateHeader();
        if(s_ac_v){char t[20];snprintf(t,20,"%s  %s",g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---");lv_label_set_text(s_ac_v,t);}
        g_ac_search[0]=0;
        if(g_ac_search_disp)lv_label_set_text(g_ac_search_disp,"_");
        return;}
    if(d==201){int l=strlen(g_ac_search);if(l>0)g_ac_search[l-1]=0;}
    else{int l=strlen(g_ac_search);if(l<4){g_ac_search[l]=(char)d;g_ac_search[l+1]=0;}}
    if(g_ac_search_disp){
        char s[8];snprintf(s,8,"%s_",g_ac_search[0]?g_ac_search:"");
        lv_label_set_text(g_ac_search_disp,s);}
    if(!g_ac_roller)return;
    int slen=strlen(g_ac_search);if(slen==0)return;
    for(int i=0;i<N_AC_TYPES;i++){
        if(strncmp(kACCodes[i],g_ac_search,slen)==0){
            lv_roller_set_selected(g_ac_roller,i,LV_ANIM_ON);
            if(g_ac_type_desc)lv_label_set_text(g_ac_type_desc,kACLabels[i]);
            break;}}}

lv_obj_t* mkSrchKey(lv_obj_t*p,const char*t,int x,int y,int w,int h,intptr_t d){
    lv_color_t bg=(d==200)?lv_color_hex(0x1f4068):
        (g_dark_theme?lv_color_hex(0x1e2b38):lv_color_hex(0xd0dce8));
    lv_obj_t*b=lv_btn_create(p);lv_obj_set_size(b,w,h);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_color(b,bg,0);
    lv_obj_set_style_bg_color(b,lv_color_hex(0x2d4358),LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(b,LV_OPA_COVER,0);lv_obj_set_style_radius(b,8,0);
    lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(b,0,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_add_event_cb(b,_ac_search_key_cb,LV_EVENT_CLICKED,(void*)d);
    lv_obj_t*lb=lv_label_create(b);lv_label_set_text(lb,t);
    lv_obj_set_style_text_color(lb,TFG(),0);
    lv_obj_set_style_text_font(lb,&lv_font_montserrat_12,0);
    lv_obj_center(lb);return b;}

void mkAircraftOverlay(){
    if(g_ac_ov)return;   // déjà ouvert → évite la fuite + l'écrasement des refs g_ac_*
    // Fullscreen overlay
    g_ac_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_ac_ov,480,480);lv_obj_set_pos(g_ac_ov,0,0);
    lv_obj_set_style_bg_color(g_ac_ov,TBG(),0);lv_obj_set_style_bg_opa(g_ac_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_ac_ov,0,0);lv_obj_set_style_radius(g_ac_ov,0,0);
    lv_obj_set_style_shadow_opa(g_ac_ov,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(g_ac_ov,0,0);
    lv_obj_clear_flag(g_ac_ov,LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t*tl=lv_label_create(g_ac_ov);lv_label_set_text(tl,"AIRCRAFT");
    lv_obj_set_style_text_color(tl,C_AMBER,0);lv_obj_set_style_text_font(tl,&lv_font_montserrat_20,0);
    lv_obj_align(tl,LV_ALIGN_TOP_MID,0,48);

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

    // ── Container 1: TYPE roller + A-Z search keyboard ───────────────────────
    {lv_obj_t*p=g_ac_ctn[1];
    // Roller (codes only)
    static char ac_opts[1024]="";
    ac_opts[0]=0;
    for(int i=0;i<N_AC_TYPES;i++){strcat(ac_opts,kACCodes[i]);if(i<N_AC_TYPES-1)strcat(ac_opts,"\n");}
    g_ac_roller=lv_roller_create(p);
    lv_roller_set_options(g_ac_roller,ac_opts,LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_ac_roller,3);
    lv_obj_set_width(g_ac_roller,120);
    lv_obj_align(g_ac_roller,LV_ALIGN_TOP_MID,0,6);
    lv_obj_set_style_bg_color(g_ac_roller,g_dark_theme?lv_color_hex(0x0d1117):lv_color_hex(0xf0f2f5),0);
    lv_obj_set_style_text_color(g_ac_roller,TFG(),0);
    lv_obj_set_style_text_font(g_ac_roller,&lv_font_montserrat_14,0);
    lv_obj_set_style_border_color(g_ac_roller,TGREY(),0);lv_obj_set_style_border_width(g_ac_roller,1,0);
    lv_obj_set_style_shadow_opa(g_ac_roller,LV_OPA_TRANSP,0);
    lv_obj_set_style_bg_color(g_ac_roller,C_AMBER,LV_PART_SELECTED);
    lv_obj_set_style_text_color(g_ac_roller,TBG(),LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(g_ac_roller,LV_OPA_COVER,LV_PART_SELECTED);
    lv_obj_add_event_cb(g_ac_roller,_ac_roller_change_cb,LV_EVENT_VALUE_CHANGED,NULL);
    int presel=0;
    for(int i=0;i<N_AC_TYPES;i++){if(strcmp(kACCodes[i],g_ac_type)==0){presel=i;break;}}
    lv_roller_set_selected(g_ac_roller,presel,LV_ANIM_OFF);
    // Description label
    g_ac_type_desc=lv_label_create(p);
    lv_label_set_text(g_ac_type_desc,kACLabels[presel]);
    lv_obj_set_style_text_color(g_ac_type_desc,TGREY(),0);
    lv_obj_set_style_text_font(g_ac_type_desc,&lv_font_montserrat_12,0);
    lv_obj_set_width(g_ac_type_desc,280);
    lv_obj_set_style_text_align(g_ac_type_desc,LV_TEXT_ALIGN_CENTER,0);
    lv_label_set_long_mode(g_ac_type_desc,LV_LABEL_LONG_WRAP);
    lv_obj_align(g_ac_type_desc,LV_ALIGN_TOP_MID,0,88);
    // Search display (typed prefix + cursor)
    g_ac_search[0]=0;
    g_ac_search_disp=lv_label_create(p);
    lv_label_set_text(g_ac_search_disp,"_");
    lv_obj_set_style_text_color(g_ac_search_disp,C_GREEN,0);
    lv_obj_set_style_text_font(g_ac_search_disp,&lv_font_montserrat_14,0);
    lv_obj_align(g_ac_search_disp,LV_ALIGN_TOP_MID,0,108);
    // A-Z jump keyboard: 4 rows, bw=32 bh=22 gp=3 → 7 keys=242px, x_start=119
    int bw=32,bh=22,gp=3;
    int kx=(480-(7*(bw+gp)-gp))/2;
    static const char* kr[3]={"ABCDEFG","HIJKLMN","OPQRSTU"};
    int ky[4]={130,155,180,205};
    for(int r=0;r<3;r++)for(int c=0;c<7;c++){
        char ch[2]={kr[r][c],0};
        mkSrchKey(p,ch,kx+c*(bw+gp),ky[r],bw,bh,(intptr_t)kr[r][c]);}
    // Row 4: V W X Y Z ⌫ OK
    static const char row4[]="VWXYZ";
    for(int c=0;c<5;c++){char ch[2]={row4[c],0};mkSrchKey(p,ch,kx+c*(bw+gp),ky[3],bw,bh,(intptr_t)row4[c]);}
    mkSrchKey(p,LV_SYMBOL_BACKSPACE,kx+5*(bw+gp),ky[3],bw,bh,201);
    mkSrchKey(p,"OK",kx+6*(bw+gp),ky[3],bw,bh,200);}

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

    // RESET + FERMER — 2 boutons centrés bas, dans le cercle (y=440)
    {lv_obj_t*rb=lv_btn_create(g_ac_ov);
     lv_obj_set_size(rb,120,36);lv_obj_set_pos(rb,116,440);
     lv_obj_set_style_bg_color(rb,lv_color_hex(0x5a1e1e),0);
     lv_obj_set_style_bg_color(rb,lv_color_hex(0x8a2d2d),LV_STATE_PRESSED);
     lv_obj_set_style_bg_opa(rb,LV_OPA_COVER,0);
     lv_obj_set_style_border_color(rb,lv_color_hex(0xD32F2F),0);
     lv_obj_set_style_border_width(rb,1,0);
     lv_obj_set_style_radius(rb,18,0);
     lv_obj_set_style_shadow_opa(rb,LV_OPA_TRANSP,0);
     lv_obj_t*rl=lv_label_create(rb);lv_label_set_text(rl,"RESET");
     lv_obj_set_style_text_color(rl,lv_color_hex(0xff9a9a),0);
     lv_obj_set_style_text_font(rl,&lv_font_montserrat_12,0);lv_obj_center(rl);
     lv_obj_add_event_cb(rb,_ac_key_cb,LV_EVENT_CLICKED,(void*)203);}
    {lv_obj_t*fb=lv_btn_create(g_ac_ov);
     lv_obj_set_size(fb,120,36);lv_obj_set_pos(fb,244,440);
     lv_obj_set_style_bg_color(fb,lv_color_hex(0x1e2b38),0);
     lv_obj_set_style_bg_color(fb,lv_color_hex(0x2d4358),LV_STATE_PRESSED);
     lv_obj_set_style_bg_opa(fb,LV_OPA_COVER,0);
     lv_obj_set_style_border_color(fb,lv_color_hex(0x4a6078),0);
     lv_obj_set_style_border_width(fb,1,0);
     lv_obj_set_style_radius(fb,18,0);
     lv_obj_set_style_shadow_opa(fb,LV_OPA_TRANSP,0);
     lv_obj_t*fl=lv_label_create(fb);lv_label_set_text(fl,"FERMER");
     lv_obj_set_style_text_color(fl,lv_color_hex(0x8899aa),0);
     lv_obj_set_style_text_font(fl,&lv_font_montserrat_12,0);lv_obj_center(fl);
     lv_obj_add_event_cb(fb,_ac_key_cb,LV_EVENT_CLICKED,(void*)202);}

    // init state
    strlcpy(g_ac_tmp,g_ac_reg,sizeof(g_ac_tmp));
    g_ac_tab=0;}

static void _open_aircraft_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(!g_ac_ov)mkAircraftOverlay();}

// Boot sequence — init BLE et lance scan ; les checks live sont anim s par updateAllPages.
void runBootOnPage(){
    lv_timer_handler();delay(900);          // laisse les logos visibles
    if(!g_pcand_mx)g_pcand_mx=xSemaphoreCreateMutex();
    BLEDevice::init(g_unit_name);
    lv_timer_handler();delay(200);
    startScan();
    lv_timer_handler();delay(300);
    g_bootDone=true;
}

// ── AIP loading ───────────────────────────────────────────────────────────────
static void _aipLoadCtrFile(File& f){
    char magic[4];f.read((uint8_t*)magic,4);
    if(memcmp(magic,"CTR\x00",4)!=0)return;
    uint16_t cnt;f.read((uint8_t*)&cnt,2);
    for(int i=0;i<cnt&&g_aip_ctr_cnt<AIP_MAX_CTR;i++){
        uint8_t nlen;f.read(&nlen,1);f.seek(f.position()+nlen);
        uint8_t tid;f.read(&tid,1);
        uint16_t npts;f.read((uint8_t*)&npts,2);
        if(g_aip_pts_cnt+npts>AIP_MAX_PTS){f.seek(f.position()+npts*8);continue;}
        g_aip_ctr[g_aip_ctr_cnt]={g_aip_pts_cnt,npts,tid};
        g_aip_ctr_cnt++;
        for(int j=0;j<npts;j++){
            int32_t la,lo;f.read((uint8_t*)&la,4);f.read((uint8_t*)&lo,4);
            g_aip_lat[g_aip_pts_cnt]=la;g_aip_lon[g_aip_pts_cnt]=lo;
            g_aip_pts_cnt++;}}}

static void _aipLoadAdFile(File& f){
    char magic[4];f.read((uint8_t*)magic,4);
    if(memcmp(magic,"ADP2",4)!=0)return;  // format v2: includes type_id
    uint16_t cnt;f.read((uint8_t*)&cnt,2);
    for(int i=0;i<cnt&&g_aip_ad_cnt<AIP_MAX_AD;i++){
        char ic[4];f.read((uint8_t*)ic,4);
        int32_t la,lo;f.read((uint8_t*)&la,4);f.read((uint8_t*)&lo,4);
        uint8_t tid;f.read(&tid,1);
        memcpy(g_aip_ads[g_aip_ad_cnt].icao,ic,4);g_aip_ads[g_aip_ad_cnt].icao[4]=0;
        g_aip_ads[g_aip_ad_cnt].lat_e6=la;g_aip_ads[g_aip_ad_cnt].lon_e6=lo;
        g_aip_ads[g_aip_ad_cnt].type_id=tid;
        g_aip_ad_cnt++;}}

void aipLoad(){
    if(!g_sd_ok)return;
    g_aip_lat=(int32_t*)ps_malloc(AIP_MAX_PTS*sizeof(int32_t));
    g_aip_lon=(int32_t*)ps_malloc(AIP_MAX_PTS*sizeof(int32_t));
    g_aip_ads=(AipAd*)ps_malloc(AIP_MAX_AD*sizeof(AipAd));
    if(!g_aip_lat||!g_aip_lon||!g_aip_ads){Serial.println("[AIP] PSRAM alloc failed");return;}
    File dir=SD_MMC.open("/aip");if(!dir)return;
    File f;
    while((f=dir.openNextFile())){
        String nm=String(f.name());
        if(nm.endsWith("_ctr.bin"))_aipLoadCtrFile(f);
        else if(nm.endsWith("aerodromes.bin"))_aipLoadAdFile(f);
        f.close();}
    dir.close();
    g_aip_loaded=(g_aip_ctr_cnt>0||g_aip_ad_cnt>0);
    Serial.printf("[AIP] %d CTR/ATZ (%d pts), %d aerodromes\n",g_aip_ctr_cnt,g_aip_pts_cnt,g_aip_ad_cnt);}

// ── AIP overlay draw ──────────────────────────────────────────────────────────
static inline bool latlon_to_screen(int32_t lat_e6,int32_t lon_e6,
    float own_lat,float own_lon,float cos_lat,int hdg,float scale_m,int&sx,int&sy){
    float dlat_m=(lat_e6/1e6f-own_lat)*111319.0f;
    float dlon_m=(lon_e6/1e6f-own_lon)*111319.0f*cos_lat;
    float d2=dlat_m*dlat_m+dlon_m*dlon_m;
    float sm15=scale_m*1.5f;
    if(d2>sm15*sm15)return false;
    float dist=sqrtf(d2);
    float bear=atan2f(dlon_m,dlat_m)*180.0f/(float)M_PI;
    if(bear<0)bear+=360.0f;
    int rb=((int)bear-hdg+360)%360;
    float brd=(float)rb*(float)M_PI/180.0f;
    float dpx=dist*(float)RAD_R/scale_m;
    sx=(int)(RAD_CX+sinf(brd)*dpx);sy=(int)(RAD_CY-cosf(brd)*dpx);
    return true;}

// Cap effectif du radar : north-up auto à l'arrêt. Sous RADAR_STILL_KMH le cap
// GPS (course over ground) n'est pas calculable → on verrouille la rose ET les
// icônes/AIP sur le nord (0°). En mouvement, vrai cap GPS → heading-up.
#define RADAR_STILL_KMH 5
static inline int radarEffHdg(){ return (g_status.spd < RADAR_STILL_KMH) ? 0 : (int)g_status.hdg; }

static void aipDrawCb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_DRAW_MAIN_END)return;
    if(!g_cfg.aip_en||!g_aip_loaded||!g_status.valid||!g_status.gps_fix)return;
    lv_draw_ctx_t*ctx=lv_event_get_draw_ctx(e);
    // Clip all AIP drawing to the radar circle
    lv_draw_mask_radius_param_t cmask;
    lv_area_t carea={RAD_CX-RAD_R,RAD_CY-RAD_R,RAD_CX+RAD_R-1,RAD_CY+RAD_R-1};
    lv_draw_mask_radius_init(&cmask,&carea,LV_RADIUS_CIRCLE,false);
    int16_t mid=lv_draw_mask_add(&cmask,NULL);
    float own_lat=g_status.lat,own_lon=g_status.lon;
    float cos_lat=cosf(own_lat*(float)M_PI/180.0f);
    float scale_m=(float)g_cfg.scale_nm*1852.0f;
    int   hdg=radarEffHdg();   // north-up auto à l'arrêt, cohérent avec le trafic
    // CTR polygons
    lv_draw_line_dsc_t ctr_d,atz_d;
    lv_draw_line_dsc_init(&ctr_d);
    ctr_d.color=lv_color_hex(0x9ca3af);ctr_d.width=1;
    atz_d=ctr_d;atz_d.color=lv_color_hex(0xb0bcc8);
    for(int c=0;c<g_aip_ctr_cnt;c++){
        lv_draw_line_dsc_t&dsc=(g_aip_ctr[c].type_id==13)?atz_d:ctr_d;
        int psx=0,psy=0;bool pok=false;
        uint16_t end=g_aip_ctr[c].pt_start+g_aip_ctr[c].n_pts;
        for(uint16_t pi=g_aip_ctr[c].pt_start;pi<end;pi++){
            int sx,sy;
            bool ok=latlon_to_screen(g_aip_lat[pi],g_aip_lon[pi],
                                     own_lat,own_lon,cos_lat,hdg,scale_m,sx,sy);
            if(ok&&pok){lv_point_t p1={(lv_coord_t)psx,(lv_coord_t)psy},
                                    p2={(lv_coord_t)sx,(lv_coord_t)sy};
                lv_draw_line(ctx,&dsc,&p1,&p2);}
            psx=sx;psy=sy;pok=ok;}}
    // Aerodromes — small amber dot
    lv_draw_rect_dsc_t ad_d;lv_draw_rect_dsc_init(&ad_d);
    ad_d.bg_color=lv_color_hex(0xFBBF24);ad_d.bg_opa=LV_OPA_COVER;
    ad_d.radius=LV_RADIUS_CIRCLE;ad_d.border_width=0;
    for(uint16_t a=0;a<g_aip_ad_cnt;a++){
        uint8_t tid=g_aip_ads[a].type_id;
        // Héliports (7) + hydrobases (10) masqués si ad_heli=OFF
        if(!g_cfg.ad_heli&&(tid==7||tid==10))continue;
        int sx,sy;
        if(latlon_to_screen(g_aip_ads[a].lat_e6,g_aip_ads[a].lon_e6,
                            own_lat,own_lon,cos_lat,hdg,scale_m,sx,sy)){
            lv_area_t ar={(lv_coord_t)(sx-2),(lv_coord_t)(sy-2),
                          (lv_coord_t)(sx+2),(lv_coord_t)(sy+2)};
            lv_draw_rect(ctx,&ad_d,&ar);}}
    lv_draw_mask_free_param(&cmask);
    lv_draw_mask_remove_id(mid);}

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

    // GS deplacee en bas, sous la taille du radar (voir _radar_scale_lbl)
    r_radar_gs=lv_label_create(p);lv_label_set_text(r_radar_gs,"GS ---");
    lv_obj_set_style_text_color(r_radar_gs,TFG(),0);
    lv_obj_set_style_text_font(r_radar_gs,&lv_font_montserrat_14,0);
    lv_obj_align(r_radar_gs,LV_ALIGN_BOTTOM_MID,0,-15);

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
    lv_obj_set_style_bg_opa(ro,LV_OPA_TRANSP,0);lv_obj_set_style_border_color(ro,TFG(),0);
    lv_obj_set_style_border_width(ro,1,0);lv_obj_set_style_shadow_opa(ro,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(ro,0,0);lv_obj_clear_flag(ro,LV_OBJ_FLAG_SCROLLABLE);

    // Inner ring (half scale)
    lv_obj_t*ri=lv_obj_create(p);lv_obj_set_size(ri,RAD_R,RAD_R);
    lv_obj_set_pos(ri,RAD_CX-RAD_R/2,RAD_CY-RAD_R/2);lv_obj_set_style_radius(ri,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_opa(ri,LV_OPA_TRANSP,0);lv_obj_set_style_border_color(ri,TFG(),0);
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
        lv_obj_set_style_line_color(tm,TFG(),0);
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
        lv_obj_set_style_text_color(r_card[ci],TFG(),0);
        lv_obj_set_pos(r_card[ci],RAD_CX-5,RAD_CY-(RAD_R-24)-8);}

    // Scale label — entre le S de la rose et la GS (ordre : S → 4nm → GS XXkt)
    char scl[12];snprintf(scl,12,"%dnm",g_cfg.scale_nm);
    r_radar_scale_lbl=mkLbl(p,scl,TGREY(),&lv_font_montserrat_14,LV_ALIGN_BOTTOM_MID,0,-35);

    // Zoom +/- buttons — flanquent le label scale, mêmes ids que Settings (0=-, 1=+)
    // → reuse cbSetBtn → cfgSave() + updSetPage() (rafraichit aussi r_radar_scale_lbl)
    auto mkZoomBtn = [&](const char* sym, int dx, intptr_t id){
        lv_obj_t* b=lv_obj_create(p);
        lv_obj_set_size(b,34,34);
        lv_obj_align(b,LV_ALIGN_BOTTOM_MID,dx,-28);
        lv_obj_set_style_radius(b,LV_RADIUS_CIRCLE,0);
        lv_obj_set_style_bg_color(b,THDG(),0);lv_obj_set_style_bg_opa(b,LV_OPA_COVER,0);
        lv_obj_set_style_border_color(b,TFG(),0);lv_obj_set_style_border_width(b,1,0);
        lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t* lab=lv_label_create(b);lv_label_set_text(lab,sym);
        lv_obj_set_style_text_color(lab,TFG(),0);
        lv_obj_set_style_text_font(lab,&lv_font_montserrat_20,0);
        lv_obj_center(lab);
        lv_obj_add_event_cb(b,cbSetBtn,LV_EVENT_CLICKED,(void*)id);
    };
    // id 0 = nm-- (zoom IN), id 1 = nm++ (zoom OUT). On mappe "+" sur le zoom IN
    // (pousser + = se rapprocher) et "-" sur le zoom OUT. Settings SCALE inchangé.
    mkZoomBtn("-",-55,1);
    mkZoomBtn("+", 55,0);

    // AIP overlay — transparent layer between grid and traffic icons
    r_aip_layer=lv_obj_create(p);
    lv_obj_set_size(r_aip_layer,480,480);lv_obj_set_pos(r_aip_layer,0,0);
    lv_obj_set_style_bg_opa(r_aip_layer,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(r_aip_layer,0,0);lv_obj_set_style_pad_all(r_aip_layer,0,0);
    lv_obj_set_style_shadow_opa(r_aip_layer,LV_OPA_TRANSP,0);
    lv_obj_clear_flag(r_aip_layer,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(r_aip_layer,aipDrawCb,LV_EVENT_DRAW_MAIN_END,NULL);

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
        lv_obj_set_style_line_color(r_trf_vect[i],TFG(),0);lv_obj_set_style_line_width(r_trf_vect[i],1,0);
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
    r_alert_overlay=lv_obj_create(p);lv_obj_set_size(r_alert_overlay,300,44);
    lv_obj_set_pos(r_alert_overlay,90,356);
    lv_obj_set_style_bg_color(r_alert_overlay,C_RED,0);
    lv_obj_set_style_bg_opa(r_alert_overlay,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(r_alert_overlay,0,0);
    lv_obj_set_style_radius(r_alert_overlay,10,0);lv_obj_set_style_shadow_opa(r_alert_overlay,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(r_alert_overlay,0,0);lv_obj_clear_flag(r_alert_overlay,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);
    r_aov_text=lv_label_create(r_alert_overlay);lv_label_set_text(r_aov_text,"");
    lv_obj_set_style_text_color(r_aov_text,lv_color_hex(0xFFFFFF),0);
    lv_obj_set_style_text_font(r_aov_text,&lv_font_montserrat_16,0);lv_obj_center(r_aov_text);}

// ── Page 2 — Settings ─────────────────────────────────────────────────────────
void updSetPage(){
    char b[16];
    snprintf(b,16,"%dnm",g_cfg.scale_nm); lv_label_set_text(s_scale_v,b);
    snprintf(b,16,"%dft",g_cfg.vfilt_ft); lv_label_set_text(s_vfilt_v,b);
    lv_label_set_text(s_dist_v, g_cfg.dist_nm?"NM":"km");
    lv_label_set_text(s_alt_v,  g_cfg.alt_ft?"ft":"m");
    if(s_spd_v)lv_label_set_text(s_spd_v, g_cfg.spd_kt?"kt":"km/h");
    snprintf(b,16,"%d/16",g_cfg.brightness); lv_label_set_text(s_bright_v,b);
    lv_label_set_text(s_src_v,  kSrcNames[g_cfg.trf_src&3]);
    lv_label_set_text(s_grnd_v, g_cfg.show_grnd?"ON":"OFF");
    lv_label_set_text(s_theme_v,g_cfg.dark?"DARK":"LIGHT");
    lv_label_set_text(s_icon_sz_v,kIconSzNames[g_cfg.icon_sz]);
    lv_label_set_text(s_wifi_v,g_wifi_active?"192.168.4.1":g_cfg.wifi_en?"ON":"OFF");
    if(s_aip_v)lv_label_set_text(s_aip_v,!g_aip_loaded?"NO DATA":g_cfg.aip_en?"ON":"OFF");
    if(s_heli_v)lv_label_set_text(s_heli_v,g_cfg.ad_heli?"ON":"OFF");
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
        // IDs 8/9 (anciens boutons brightness < / >) remplacés par le slider
        case 10:g_cfg.trf_src=(g_cfg.trf_src+3)%4;break;
        case 11:g_cfg.trf_src=(g_cfg.trf_src+1)%4;break;
        case 14:case 15:g_cfg.show_grnd=!g_cfg.show_grnd;break;
        case 12:case 13:g_cfg.dark=!g_cfg.dark;g_rebuildPages=true;break;
        case 16:g_cfg.icon_sz=max((int)g_cfg.icon_sz-1,0);for(int i=0;i<MAX_TRF;i++)lv_img_set_zoom(r_trf_img[i],kIconZoom[g_cfg.icon_sz]);break;
        case 17:g_cfg.icon_sz=min((int)g_cfg.icon_sz+1,2);for(int i=0;i<MAX_TRF;i++)lv_img_set_zoom(r_trf_img[i],kIconZoom[g_cfg.icon_sz]);break;
        case 18:case 19:g_cfg.wifi_en=!g_cfg.wifi_en;
            if(g_cfg.wifi_en)wifiStart();else wifiStop();break;
        case 20:case 21:if(g_aip_loaded){g_cfg.aip_en=!g_cfg.aip_en;
            if(r_aip_layer)lv_obj_invalidate(r_aip_layer);}break;
        case 22:case 23:g_cfg.ad_heli=!g_cfg.ad_heli;
            if(r_aip_layer)lv_obj_invalidate(r_aip_layer);break;
        case 24:case 25:g_cfg.spd_kt=!g_cfg.spd_kt;break;  // tâche F : unité vitesse
    }
    cfgSave();
    if(!g_rebuildPages)updSetPage();}

// Helper: section header brand-blue + underline horizontal
static void mkSetSection(lv_obj_t*p,const char*title,int y){
    mkLbl(p,title,C_BRAND,&lv_font_montserrat_16,LV_ALIGN_TOP_MID,0,y);
    lv_obj_t*hl=lv_obj_create(p);
    lv_obj_set_size(hl,330,1);
    lv_obj_align(hl,LV_ALIGN_TOP_MID,0,y+22);
    lv_obj_set_style_bg_color(hl,C_BRAND,0);lv_obj_set_style_bg_opa(hl,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(hl,0,0);lv_obj_set_style_pad_all(hl,0,0);
    lv_obj_clear_flag(hl,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
}

// Row: LABEL gauche + valeur + boutons < / > droite. Style maquette : fond blanc.
static lv_obj_t* mkSetRow(lv_obj_t*p,const char*k,int y,const char*v,int idn,int idup){
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_14,55,y);
    lv_obj_t*vl=mkLblP(p,v,C_BRAND,&lv_font_montserrat_14,235,y);
    lv_color_t btnbg=lv_color_hex(0xeef2f6);
    lv_obj_t*bd=lv_btn_create(p);lv_obj_set_size(bd,30,22);lv_obj_set_pos(bd,295,y-3);
    lv_obj_set_style_bg_color(bd,btnbg,0);lv_obj_set_style_border_width(bd,0,0);
    lv_obj_set_style_radius(bd,4,0);lv_obj_set_style_shadow_opa(bd,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(bd,0,0);
    lv_obj_add_event_cb(bd,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idn);
    lv_obj_t*ld=lv_label_create(bd);lv_label_set_text(ld,"<");
    lv_obj_set_style_text_color(ld,lv_color_hex(0x0f172a),0);
    lv_obj_set_style_text_font(ld,&lv_font_montserrat_14,0);lv_obj_center(ld);
    lv_obj_t*bu=lv_btn_create(p);lv_obj_set_size(bu,30,22);lv_obj_set_pos(bu,330,y-3);
    lv_obj_set_style_bg_color(bu,btnbg,0);lv_obj_set_style_border_width(bu,0,0);
    lv_obj_set_style_radius(bu,4,0);lv_obj_set_style_shadow_opa(bu,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(bu,0,0);
    lv_obj_add_event_cb(bu,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idup);
    lv_obj_t*lu=lv_label_create(bu);lv_label_set_text(lu,">");
    lv_obj_set_style_text_color(lu,lv_color_hex(0x0f172a),0);
    lv_obj_set_style_text_font(lu,&lv_font_montserrat_14,0);lv_obj_center(lu);
    return vl;}

// Row "EDIT" : LABEL gauche + valeur + bouton EDIT droite (callback custom)
static lv_obj_t* mkSetRowBtn(lv_obj_t*p,const char*k,int y,const char*v,lv_event_cb_t cb){
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_14,55,y);
    lv_obj_t*vl=mkLblP(p,v,C_BRAND,&lv_font_montserrat_14,180,y);
    lv_color_t bg=lv_color_hex(0xeef2f6);
    lv_obj_t*b=lv_btn_create(p);lv_obj_set_size(b,65,22);lv_obj_set_pos(b,295,y-3);
    lv_obj_set_style_bg_color(b,bg,0);lv_obj_set_style_border_width(b,0,0);
    lv_obj_set_style_radius(b,4,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,"EDIT");
    lv_obj_set_style_text_color(l,lv_color_hex(0x0f172a),0);
    lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);
    return vl;}

// Callback du slider brightness — live update du label + panneau, save NVS au release.
// Driver LilyGo : 16 niveaux de backlight (cf. LilyGo_RGBPanel::setBrightness),
// donc le slider est calé sur l'échelle hardware 0-16 directement (pas de mapping).
static void cbBrightSlider(lv_event_t*e){
    lv_event_code_t code=lv_event_get_code(e);
    if(code==LV_EVENT_PRESSED){g_bright_drag=true;return;}
    if(code==LV_EVENT_RELEASED||code==LV_EVENT_PRESS_LOST){g_bright_drag=false;cfgSave();return;}
    if(code!=LV_EVENT_VALUE_CHANGED)return;
    lv_obj_t*sl=lv_event_get_target(e);
    int v=lv_slider_get_value(sl); if(v<0)v=0; if(v>16)v=16;
    g_cfg.brightness=(uint8_t)v;
    panel.setBrightness(g_cfg.brightness);
    if(s_bright_v){char b[8];snprintf(b,8,"%d/16",v);lv_label_set_text(s_bright_v,b);}
}

// Row slider brightness — remplace les boutons < / > par un curseur 0-16 niveaux
static lv_obj_t* mkSetSliderRow(lv_obj_t*p,const char*k,int y,uint8_t val){
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_14,55,y);
    char b[8]; snprintf(b,8,"%d/16",val);
    lv_obj_t*vl=mkLblP(p,b,C_BRAND,&lv_font_montserrat_14,180,y);
    lv_obj_t*sl=lv_slider_create(p);
    lv_obj_set_size(sl,135,10);
    lv_obj_set_pos(sl,225,y+5);
    lv_slider_set_range(sl,0,16);
    lv_slider_set_value(sl,val,LV_ANIM_OFF);
    lv_obj_set_style_bg_color(sl,lv_color_hex(0xe5e7eb),LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sl,LV_OPA_COVER,LV_PART_MAIN);
    lv_obj_set_style_bg_color(sl,C_BRAND,LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(sl,LV_OPA_COVER,LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sl,C_BRAND,LV_PART_KNOB);
    lv_obj_set_style_pad_all(sl,4,LV_PART_KNOB);
    lv_obj_add_event_cb(sl,cbBrightSlider,LV_EVENT_ALL,NULL);
    return vl;
}

// ── Maintenance overlay (Modèle 1 : hotspot + transfert vol) ──────────────────
static lv_obj_t* g_maint_scanlist;   // fwd : annulé ici aussi (enfant de l'overlay)
static void _maint_close(){
    if(!g_maint_ov)return;
    lv_obj_del(g_maint_ov);   // supprime aussi le panneau scan (enfant)
    g_maint_ov=nullptr;g_maint_ssid_ta=nullptr;g_maint_pass_ta=nullptr;g_maint_kb=nullptr;
    g_maint_scanlist=nullptr;}
static void _maint_close_cb(lv_event_t*e){
    if(lv_event_get_code(e)==LV_EVENT_CLICKED)_maint_close();}
static void _maint_upload_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    sendCtl("upload");   // AT-CORE connecte le hotspot + upload → overlay progress auto
    _maint_close();      // referme pour laisser voir l'overlay de progression up_pct
}
static void _maint_save_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    const char*s=lv_textarea_get_text(g_maint_ssid_ta);
    const char*p=lv_textarea_get_text(g_maint_pass_ta);
    if(!s||!s[0])return;                 // SSID obligatoire
    unitSaveHotspot(s,p);                // NVS local (toujours)
    sendWifiCreds(s,p);                  // push BLE vers AT-CORE (si connecté)
    // Feedback honnête : "Sent" si poussé en BLE, sinon juste sauvé localement.
    bool pushed=g_connected&&g_chrCtl&&g_chrCtl->canWrite();
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(l)lv_label_set_text(l,pushed?"Sent":"Saved (offline)");}
static void _maint_test_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    sendCtl("wifitest");   // AT-CORE connecte le hotspot, logue [WIFI] IP=.../FAIL en série
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(l)lv_label_set_text(l,g_connected?"Test sent (see log)":"Offline");}
// Clavier : suit le textarea focalisé, caché tant qu'on ne tape pas. On gère
// FOCUSED (1re entrée) ET CLICKED (re-tap d'un champ déjà focalisé, sinon LVGL
// ne renvoie pas FOCUSED et le clavier ne reviendrait pas après l'avoir fermé).
static void _maint_ta_cb(lv_event_t*e){
    lv_event_code_t c=lv_event_get_code(e);
    if((c!=LV_EVENT_FOCUSED&&c!=LV_EVENT_CLICKED)||!g_maint_kb)return;
    lv_keyboard_set_textarea(g_maint_kb,lv_event_get_target(e));
    lv_obj_clear_flag(g_maint_kb,LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(g_maint_kb);}
static void _maint_kb_cb(lv_event_t*e){
    lv_event_code_t c=lv_event_get_code(e);
    if((c==LV_EVENT_READY||c==LV_EVENT_CANCEL)&&g_maint_kb)
        lv_obj_add_flag(g_maint_kb,LV_OBJ_FLAG_HIDDEN);}

// Scan WiFi (Fix 1) — l'AT-VIEW (ESP32-S3) liste les réseaux 2.4 GHz à proximité.
// g_maint_scanlist est déclaré plus haut (près de _maint_close).
static char      g_scan_ssids[12][33];
static int       g_scan_n=0;
static void _maint_scanpick_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int idx=(int)(intptr_t)lv_event_get_user_data(e);
    if(idx>=0&&idx<g_scan_n&&g_maint_ssid_ta)
        lv_textarea_set_text(g_maint_ssid_ta,g_scan_ssids[idx]);
    if(g_maint_scanlist){lv_obj_del(g_maint_scanlist);g_maint_scanlist=nullptr;}}
static void _maint_scan_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED||g_maint_scanlist)return;
    if(g_maint_kb)lv_obj_add_flag(g_maint_kb,LV_OBJ_FLAG_HIDDEN);
    // Panneau résultats centré, taillé pour le cercle (320 px), scroll vertical.
    g_maint_scanlist=lv_obj_create(g_maint_ov);
    lv_obj_set_size(g_maint_scanlist,320,300);lv_obj_center(g_maint_scanlist);
    lv_obj_set_style_bg_color(g_maint_scanlist,lv_color_hex(0x161b22),0);
    lv_obj_set_style_border_color(g_maint_scanlist,C_BRAND,0);lv_obj_set_style_border_width(g_maint_scanlist,1,0);
    lv_obj_set_style_radius(g_maint_scanlist,8,0);
    lv_obj_set_flex_flow(g_maint_scanlist,LV_FLEX_FLOW_COLUMN);
    lv_obj_t*tt=lv_label_create(g_maint_scanlist);lv_label_set_text(tt,"Scanning 2.4GHz WiFi...");
    lv_obj_set_style_text_color(tt,TGREY(),0);lv_obj_set_style_text_font(tt,&lv_font_montserrat_14,0);
    // Bouton Fermer (idx=-1 → _maint_scanpick_cb ne set pas le SSID, ferme juste) :
    // indispensable si aucun réseau n'est trouvé, sinon le panneau resterait bloqué.
    {lv_obj_t*xb=lv_btn_create(g_maint_scanlist);lv_obj_set_width(xb,120);
     lv_obj_set_style_bg_color(xb,lv_color_hex(0x4b5563),0);lv_obj_set_style_radius(xb,6,0);
     lv_obj_set_style_shadow_opa(xb,LV_OPA_TRANSP,0);
     lv_obj_add_event_cb(xb,_maint_scanpick_cb,LV_EVENT_CLICKED,(void*)(intptr_t)(-1));
     lv_obj_t*l=lv_label_create(xb);lv_label_set_text(l,"Close");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_12,0);lv_obj_center(l);}
    // Si l'AP (Settings WIFI) tourne, NE PAS scanner : passer en STA tuerait l'AP +
    // le WebServer (g_wifi_active resterait incohérent). On refuse proprement.
    if(g_wifi_active){lv_label_set_text(tt,"Disable WIFI (Settings) first");return;}
    lv_refr_now(NULL);   // dessine "Scan..." + Fermer AVANT le scan bloquant (~2-4 s)
    WiFi.mode(WIFI_STA);
    int n=WiFi.scanNetworks();
    g_scan_n=0;
    for(int i=0;i<n&&g_scan_n<12;i++){
        String s=WiFi.SSID(i);
        if(s.length()==0)continue;   // SSID caché
        strlcpy(g_scan_ssids[g_scan_n],s.c_str(),sizeof(g_scan_ssids[0]));
        lv_obj_t*b=lv_btn_create(g_maint_scanlist);lv_obj_set_width(b,280);
        lv_obj_set_style_bg_color(b,lv_color_hex(0x21262d),0);lv_obj_set_style_radius(b,6,0);
        lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
        lv_obj_add_event_cb(b,_maint_scanpick_cb,LV_EVENT_CLICKED,(void*)(intptr_t)g_scan_n);
        char row[52];snprintf(row,sizeof(row),"%s  %ddBm",g_scan_ssids[g_scan_n],(int)WiFi.RSSI(i));
        lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,row);
        lv_obj_set_style_text_color(l,lv_color_hex(0xe6edf3),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_12,0);
        g_scan_n++;
    }
    WiFi.scanDelete();
    WiFi.mode(WIFI_OFF);   // restaure l'état dormant (BLE-only) → wifiStart() repartira proprement OFF→AP
    lv_label_set_text(tt,g_scan_n?"Pick hotspot (2.4GHz):":"No 2.4GHz network (5GHz?)");}

// ── Écran VOLS (WP8) — liste multi-select, transfert, suppression ─────────────
struct VolItem { char fid[20]; char d[12]; char s[6]; char e[6]; uint8_t up; bool sel; lv_obj_t* lbl; lv_obj_t* row; };
static VolItem  g_vols[16];
static int      g_vols_n=0;
static lv_obj_t* g_vols_ov=nullptr;
static lv_obj_t* g_vols_list=nullptr;
static lv_obj_t* g_vols_load=nullptr;   // label "Loading..."
static lv_obj_t* g_vols_xfer=nullptr;   // label du bouton Transferer (N)
static bool     g_vols_loading=false, g_vols_del_armed=false;
static bool     g_vols_xfer_pending=false, g_vols_xfer_seen3=false;  // suivi transfert sur la page
static uint32_t g_vols_t0=0, g_vols_xfer_t0=0;

static void volsClose(){
    if(!g_vols_ov)return;
    lv_obj_del(g_vols_ov);
    g_vols_ov=nullptr;g_vols_list=nullptr;g_vols_load=nullptr;g_vols_xfer=nullptr;
    g_vols_loading=false;g_vols_del_armed=false;g_vols_xfer_pending=false;g_vols_n=0;}
static void _vols_close_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED)volsClose(); }

static void volsUpdXfer(){
    if(!g_vols_xfer)return;
    int n=0; for(int i=0;i<g_vols_n;i++) if(g_vols[i].sel)n++;
    char b[24]; snprintf(b,sizeof(b),"Transfer (%d)",n); lv_label_set_text(g_vols_xfer,b);}

// Remplace la liste par un message d'état (transfert/suppression en cours, on reste
// sur la page). g_vols_load réutilisé comme label, g_vols_n=0 (plus de lignes).
static void volsShowStatus(const char* msg, lv_color_t col){
    if(!g_vols_list)return;
    lv_obj_clean(g_vols_list); g_vols_n=0;
    g_vols_load=lv_label_create(g_vols_list);
    lv_label_set_text(g_vols_load,msg);
    lv_obj_set_style_text_color(g_vols_load,col,0);
    lv_obj_set_style_text_font(g_vols_load,&lv_font_montserrat_16,0);
    volsUpdXfer();}

static void _vols_row_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int i=(int)(intptr_t)lv_event_get_user_data(e);
    if(i<0||i>=g_vols_n||g_vols[i].up)return;
    g_vols[i].sel=!g_vols[i].sel;
    const char* md=strlen(g_vols[i].d)>=10?g_vols[i].d+5:g_vols[i].d;
    char r[48]; snprintf(r,sizeof(r),"[%s] %s %s>%s",g_vols[i].sel?"x":" ",md,g_vols[i].s,g_vols[i].e);
    lv_label_set_text(g_vols[i].lbl,r);
    lv_obj_set_style_bg_color(g_vols[i].row,g_vols[i].sel?C_BRAND:lv_color_hex(0x21262d),0);
    volsUpdXfer();}

// Construit {"cmd":"uploadlist","f":[...]} et l'écrit sur CHR_CONTROL (≤200 B → ≤8 fids).
static void _vols_xfer_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED||!g_chrCtl||!g_chrCtl->canWrite())return;
    char p[200]; int w=snprintf(p,sizeof(p),"{\"cmd\":\"uploadlist\",\"f\":["); int cnt=0;
    for(int i=0;i<g_vols_n;i++){
        if(!g_vols[i].sel)continue;
        int n=snprintf(p+w,sizeof(p)-w,"%s\"%s\"",cnt?",":"",g_vols[i].fid);
        if(w+n>178)break; w+=n; cnt++;
    }
    w+=snprintf(p+w,sizeof(p)-w,"]}");
    if(cnt){
        g_chrCtl->writeValue((uint8_t*)p,strlen(p),false);
        volsShowStatus("Transferring...",C_AMBER);   // on RESTE sur la page (overlay up_pct par-dessus)
        g_vols_xfer_pending=true; g_vols_xfer_seen3=false; g_vols_xfer_t0=millis();
    }
}
// Suppression des .up (transférés) — double-tap de confirmation.
static void _vols_del_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_vols_del_armed){
        g_vols_del_armed=true;
        if(l)lv_label_set_text(l,"Confirm?");
        lv_obj_set_style_bg_color(b,C_RED,0);
        return;
    }
    sendCtl("delflights");   // AT-CORE efface les .up puis re-scanne
    g_vols_del_armed=false;
    volsShowStatus("Deleting...",C_AMBER);   // reste sur la page + recharge à la fin
    g_status.flt_rdy=0; g_vols_loading=true; g_vols_t0=millis();   // attend le re-scan AT-CORE puis volsBuildList
}

// Lit CHR_FLIGHTS, parse le JSON, construit les lignes.
static void volsBuildList(){
    if(!g_chrFl||!g_vols_list)return;
    if(g_vols_load){lv_obj_add_flag(g_vols_load,LV_OBJ_FLAG_HIDDEN);}
    std::string v=g_chrFl->readValue();
    JsonDocument d; if(deserializeJson(d,v.c_str()))return;
    JsonArray arr=d.as<JsonArray>();
    lv_obj_clean(g_vols_list); g_vols_n=0;
    for(JsonObject o:arr){
        if(g_vols_n>=16)break;
        VolItem& it=g_vols[g_vols_n];
        strlcpy(it.fid,o["f"]|"",sizeof(it.fid));
        strlcpy(it.d,o["d"]|"?",sizeof(it.d));
        strlcpy(it.s,o["s"]|"?",sizeof(it.s));
        strlcpy(it.e,o["e"]|"?",sizeof(it.e));
        it.up=o["u"]|0; it.sel=false;
        lv_obj_t*b=lv_btn_create(g_vols_list);lv_obj_set_size(b,270,30);
        lv_obj_set_style_radius(b,6,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
        lv_obj_set_style_bg_color(b,it.up?lv_color_hex(0x161b22):lv_color_hex(0x21262d),0);
        const char* md=strlen(it.d)>=10?it.d+5:it.d;
        char r[52];
        if(it.up) snprintf(r,sizeof(r),"%s %s>%s  sent",md,it.s,it.e);
        else      snprintf(r,sizeof(r),"[ ] %s %s>%s",md,it.s,it.e);
        lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,r);
        lv_obj_set_style_text_color(l,it.up?lv_color_hex(0x6b7280):lv_color_hex(0xe6edf3),0);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);
        it.lbl=l; it.row=b;
        if(!it.up)lv_obj_add_event_cb(b,_vols_row_cb,LV_EVENT_CLICKED,(void*)(intptr_t)g_vols_n);
        g_vols_n++;
    }
    if(g_vols_n==0){
        lv_obj_t*l=lv_label_create(g_vols_list);lv_label_set_text(l,"No flights on SD");
        lv_obj_set_style_text_color(l,TGREY(),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
    }
    volsUpdXfer();}

void mkVolsOverlay(){
    if(g_vols_ov)return;
    g_vols_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_vols_ov,480,480);lv_obj_set_pos(g_vols_ov,0,0);
    lv_obj_set_style_bg_color(g_vols_ov,TBG(),0);lv_obj_set_style_bg_opa(g_vols_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_vols_ov,0,0);lv_obj_set_style_radius(g_vols_ov,0,0);
    lv_obj_set_style_pad_all(g_vols_ov,0,0);lv_obj_clear_flag(g_vols_ov,LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t*tl=lv_label_create(g_vols_ov);lv_label_set_text(tl,"FLIGHTS (UTC)");
    lv_obj_set_style_text_color(tl,C_AMBER,0);lv_obj_set_style_text_font(tl,&lv_font_montserrat_20,0);
    lv_obj_align(tl,LV_ALIGN_TOP_MID,0,26);

    // Liste scrollable (taillée pour le cercle)
    g_vols_list=lv_obj_create(g_vols_ov);
    lv_obj_set_size(g_vols_list,290,238);lv_obj_align(g_vols_list,LV_ALIGN_TOP_MID,0,56);
    lv_obj_set_style_bg_color(g_vols_list,lv_color_hex(0x0d1117),0);
    lv_obj_set_style_border_width(g_vols_list,0,0);lv_obj_set_style_pad_all(g_vols_list,4,0);
    lv_obj_set_flex_flow(g_vols_list,LV_FLEX_FLOW_COLUMN);
    g_vols_load=lv_label_create(g_vols_list);lv_label_set_text(g_vols_load,"Loading...");
    lv_obj_set_style_text_color(g_vols_load,TGREY(),0);lv_obj_set_style_text_font(g_vols_load,&lv_font_montserrat_14,0);

    // Boutons
    lv_obj_t*bx=lv_btn_create(g_vols_ov);lv_obj_set_size(bx,210,34);lv_obj_align(bx,LV_ALIGN_TOP_MID,0,302);
    lv_obj_set_style_bg_color(bx,C_GREEN,0);lv_obj_set_style_radius(bx,8,0);
    lv_obj_set_style_border_width(bx,0,0);lv_obj_set_style_shadow_opa(bx,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bx,_vols_xfer_cb,LV_EVENT_CLICKED,NULL);
    g_vols_xfer=lv_label_create(bx);lv_label_set_text(g_vols_xfer,"Transfer (0)");
    lv_obj_set_style_text_color(g_vols_xfer,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(g_vols_xfer,&lv_font_montserrat_14,0);lv_obj_center(g_vols_xfer);

    lv_obj_t*bd=lv_btn_create(g_vols_ov);lv_obj_set_size(bd,210,30);lv_obj_align(bd,LV_ALIGN_TOP_MID,0,342);
    lv_obj_set_style_bg_color(bd,lv_color_hex(0x4b5563),0);lv_obj_set_style_radius(bd,8,0);
    lv_obj_set_style_border_width(bd,0,0);lv_obj_set_style_shadow_opa(bd,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bd,_vols_del_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bd);lv_label_set_text(l,"Delete sent");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}

    lv_obj_t*bc=lv_btn_create(g_vols_ov);lv_obj_set_size(bc,210,30);lv_obj_align(bc,LV_ALIGN_TOP_MID,0,378);
    lv_obj_set_style_bg_color(bc,lv_color_hex(0x30363d),0);lv_obj_set_style_radius(bc,8,0);
    lv_obj_set_style_border_width(bc,0,0);lv_obj_set_style_shadow_opa(bc,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bc,_vols_close_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bc);lv_label_set_text(l,"Close");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}

    // Demande la liste. On seed flt_rdy=0 localement : l'AT-CORE le met aussi à 0
    // à la réception puis 1 quand la liste est prête. Le poll (loop) attend
    // flt_rdy==1 + ≥1.5 s écoulées (couvre un STATUS périmé encore en transit).
    sendCtl("flights");
    g_status.flt_rdy=0;
    g_vols_loading=true;g_vols_t0=millis();}

static void _open_vols_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(!g_vols_ov)mkVolsOverlay();}

void mkMaintenanceOverlay(){
    if(g_maint_ov)return;
    g_maint_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_maint_ov,480,480);lv_obj_set_pos(g_maint_ov,0,0);
    lv_obj_set_style_bg_color(g_maint_ov,TBG(),0);lv_obj_set_style_bg_opa(g_maint_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_maint_ov,0,0);lv_obj_set_style_radius(g_maint_ov,0,0);
    lv_obj_set_style_pad_all(g_maint_ov,0,0);lv_obj_clear_flag(g_maint_ov,LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t*tl=lv_label_create(g_maint_ov);lv_label_set_text(tl,"MAINTENANCE");
    lv_obj_set_style_text_color(tl,C_AMBER,0);lv_obj_set_style_text_font(tl,&lv_font_montserrat_20,0);
    lv_obj_align(tl,LV_ALIGN_TOP_MID,0,34);

    // Ligne transfert : "Last flight" (rapide) + "Flights" (multi-select, WP8)
    lv_obj_t*bu=lv_btn_create(g_maint_ov);lv_obj_set_size(bu,145,36);
    lv_obj_align(bu,LV_ALIGN_TOP_MID,-78,56);
    lv_obj_set_style_bg_color(bu,C_BRAND,0);lv_obj_set_style_radius(bu,8,0);
    lv_obj_set_style_border_width(bu,0,0);lv_obj_set_style_shadow_opa(bu,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bu,_maint_upload_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bu);lv_label_set_text(l,"Last flight");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}
    lv_obj_t*bv=lv_btn_create(g_maint_ov);lv_obj_set_size(bv,145,36);
    lv_obj_align(bv,LV_ALIGN_TOP_MID,78,56);
    lv_obj_set_style_bg_color(bv,lv_color_hex(0x1f4068),0);lv_obj_set_style_radius(bv,8,0);
    lv_obj_set_style_border_width(bv,0,0);lv_obj_set_style_shadow_opa(bv,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bv,_open_vols_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bv);lv_label_set_text(l,"Flights");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}

    // Champ SSID (gauche) + bouton Scan (droite)
    g_maint_ssid_ta=lv_textarea_create(g_maint_ov);
    lv_textarea_set_one_line(g_maint_ssid_ta,true);
    lv_textarea_set_placeholder_text(g_maint_ssid_ta,"Hotspot SSID");
    lv_textarea_set_text(g_maint_ssid_ta,g_hs_ssid);
    lv_textarea_set_max_length(g_maint_ssid_ta,32);
    lv_obj_set_size(g_maint_ssid_ta,228,36);lv_obj_align(g_maint_ssid_ta,LV_ALIGN_TOP_MID,-56,100);
    lv_obj_add_event_cb(g_maint_ssid_ta,_maint_ta_cb,LV_EVENT_ALL,NULL);
    lv_obj_t*bsc=lv_btn_create(g_maint_ov);lv_obj_set_size(bsc,84,36);
    lv_obj_align(bsc,LV_ALIGN_TOP_MID,150,100);
    lv_obj_set_style_bg_color(bsc,lv_color_hex(0x1f4068),0);lv_obj_set_style_radius(bsc,8,0);
    lv_obj_set_style_border_width(bsc,0,0);lv_obj_set_style_shadow_opa(bsc,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bsc,_maint_scan_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bsc);lv_label_set_text(l,"Scan");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}

    // Champ mot de passe
    g_maint_pass_ta=lv_textarea_create(g_maint_ov);
    lv_textarea_set_one_line(g_maint_pass_ta,true);
    lv_textarea_set_password_mode(g_maint_pass_ta,true);
    lv_textarea_set_placeholder_text(g_maint_pass_ta,"Password");
    lv_textarea_set_text(g_maint_pass_ta,g_hs_pass);
    lv_textarea_set_max_length(g_maint_pass_ta,63);
    lv_obj_set_size(g_maint_pass_ta,340,36);lv_obj_align(g_maint_pass_ta,LV_ALIGN_TOP_MID,0,142);
    lv_obj_add_event_cb(g_maint_pass_ta,_maint_ta_cb,LV_EVENT_ALL,NULL);

    // Bouton Tester le hotspot (diagnostic connexion, sans vol) → BLE {"cmd":"wifitest"}
    lv_obj_t*bt=lv_btn_create(g_maint_ov);lv_obj_set_size(bt,300,34);
    lv_obj_align(bt,LV_ALIGN_TOP_MID,0,182);
    lv_obj_set_style_bg_color(bt,C_CYAN,0);lv_obj_set_style_radius(bt,8,0);
    lv_obj_set_style_border_width(bt,0,0);lv_obj_set_style_shadow_opa(bt,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bt,_maint_test_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bt);lv_label_set_text(l,"Test hotspot");
     lv_obj_set_style_text_color(l,lv_color_hex(0x0d1117),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}

    // Boutons Enregistrer / Fermer
    lv_obj_t*bs=lv_btn_create(g_maint_ov);lv_obj_set_size(bs,160,34);
    lv_obj_align(bs,LV_ALIGN_TOP_MID,-86,220);
    lv_obj_set_style_bg_color(bs,C_GREEN,0);lv_obj_set_style_radius(bs,8,0);
    lv_obj_set_style_border_width(bs,0,0);lv_obj_set_style_shadow_opa(bs,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bs,_maint_save_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bs);lv_label_set_text(l,"Save");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}

    lv_obj_t*bc=lv_btn_create(g_maint_ov);lv_obj_set_size(bc,160,34);
    lv_obj_align(bc,LV_ALIGN_TOP_MID,86,220);
    lv_obj_set_style_bg_color(bc,lv_color_hex(0x4b5563),0);lv_obj_set_style_radius(bc,8,0);
    lv_obj_set_style_border_width(bc,0,0);lv_obj_set_style_shadow_opa(bc,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bc,_maint_close_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bc);lv_label_set_text(l,"Close");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}

    // Aide OTA (non pilotable en BLE : flux AP du portail AT-CORE)
    mkLbl(g_maint_ov,"AT-CORE update: BOOT 6s -> ATCORE-SETUP",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,258);
    mkLbl(g_maint_ov,"AT-VIEW update: WIFI ON -> 192.168.4.1",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,274);

    // Clavier LVGL — TAILLÉ POUR LE CERCLE : 320x175 centré (les coins restent
    // dans le disque 480, contrairement au plein-largeur dont la rangée du bas
    // sortait du cercle). Caché par défaut, suit le textarea focalisé ; les
    // champs (y114/y160) restent visibles au-dessus quand il s'affiche.
    g_maint_kb=lv_keyboard_create(g_maint_ov);
    lv_obj_set_size(g_maint_kb,320,175);lv_obj_align(g_maint_kb,LV_ALIGN_TOP_MID,0,224);
    lv_keyboard_set_textarea(g_maint_kb,g_maint_ssid_ta);
    lv_obj_add_flag(g_maint_kb,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(g_maint_kb,_maint_kb_cb,LV_EVENT_ALL,NULL);}

static void _open_maintenance_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(!g_maint_ov)mkMaintenanceOverlay();}

void buildSettingsPage(){
    lv_obj_t*p=g_pages[2]; char b[16];
    s_pg_idx=0;

    // Force fond blanc (maquette AeroTrace)
    lv_obj_set_style_bg_color(p,lv_color_hex(0xffffff),0);
    lv_obj_set_style_bg_opa(p,LV_OPA_COVER,0);

    // ── Logo A bleu + titre SETTINGS (compactés pour libérer de l'air aux rows)
    lv_obj_t*lA=lv_img_create(p);
    lv_img_set_src(lA,&img_logo_a);
    lv_obj_align(lA,LV_ALIGN_TOP_MID,0,18);
    mkLbl(p,"SETTINGS",lv_color_hex(0x0f172a),&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,82);

    // ── Sub-page containers (transparent, swipe vertical pour basculer)
    for(int i=0;i<2;i++){
        s_pg[i]=lv_obj_create(p);
        lv_obj_set_size(s_pg[i],480,312);lv_obj_set_pos(s_pg[i],0,108);
        lv_obj_set_style_bg_opa(s_pg[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(s_pg[i],0,0);lv_obj_set_style_pad_all(s_pg[i],0,0);
        lv_obj_clear_flag(s_pg[i],LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(s_pg[i],LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_pg[i],swipeCb,LV_EVENT_ALL,NULL);
        if(i!=0)lv_obj_add_flag(s_pg[i],LV_OBJ_FLAG_HIDDEN);}

    // ── Sub-page 0: RADAR + DISPLAY ──────────────────────────────────────────
    {lv_obj_t*sp=s_pg[0];
    mkSetSection(sp,"RADAR",0);
    snprintf(b,16,"%dnm",g_cfg.scale_nm); s_scale_v =mkSetRow(sp,"SCALE",   36,b,0,1);
    snprintf(b,16,"%dft",g_cfg.vfilt_ft); s_vfilt_v =mkSetRow(sp,"V-FILTER",62,b,2,3);
    s_dist_v=mkSetRow(sp,"DIST", 88,g_cfg.dist_nm?"NM":"km",4,5);
    s_alt_v =mkSetRow(sp,"ALT", 114,g_cfg.alt_ft?"ft":"m",6,7);
    s_spd_v =mkSetRow(sp,"SPEED",140,g_cfg.spd_kt?"kt":"km/h",24,25);
    mkSetSection(sp,"DISPLAY",172);
    s_bright_v=mkSetSliderRow(sp,"BRIGHTNESS",208,g_cfg.brightness);
    s_theme_v=mkSetRow(sp,"THEME",234,g_cfg.dark?"DARK":"LIGHT",12,13);}

    // ── Sub-page 1: TRAFFIC + SYSTEM ─────────────────────────────────────────
    {lv_obj_t*sp=s_pg[1];
    mkSetSection(sp,"TRAFFIC",0);
    s_src_v    =mkSetRow(sp,"SOURCE",   36,kSrcNames[g_cfg.trf_src&3],10,11);
    s_grnd_v   =mkSetRow(sp,"GROUNDED", 62,g_cfg.show_grnd?"ON":"OFF",14,15);
    s_icon_sz_v=mkSetRow(sp,"ICONS SIZE",88,kIconSzNames[g_cfg.icon_sz],16,17);
    {const char*aip_v=!g_aip_loaded?"NO DATA":g_cfg.aip_en?"ON":"OFF";
    s_aip_v=mkSetRow(sp,"AIP",114,aip_v,20,21);}
    s_heli_v=mkSetRow(sp,"HELIPORT",140,g_cfg.ad_heli?"ON":"OFF",22,23);
    mkSetSection(sp,"SYSTEM",172);
    {char t[20];snprintf(t,20,"%s %s",g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---");
    s_ac_v=mkSetRowBtn(sp,"AIRCRAFT",208,t,_open_aircraft_cb);}
    s_wifi_v=mkSetRow(sp,"WIFI",234,g_cfg.wifi_en?"ON":"OFF",18,19);
    {char sd_str[12];
     if(g_sd_ok)snprintf(sd_str,12,"%u GB",g_sd_gb);else strlcpy(sd_str,"NO CARD",12);
     mkLblP(sp,"SDCARD (AT-CORE)",lv_color_hex(0x4b5563),&lv_font_montserrat_14,55,260);
     s_sd_v=mkLblP(sp,sd_str,g_sd_ok?C_GREEN:lv_color_hex(0x4b5563),&lv_font_montserrat_14,295,260);}
    mkSetRowBtn(sp,"MAINTENANCE",286,"",_open_maintenance_cb);}

    // ── Footer (toujours visible) : logo AT-VIEW + battery + version
    lv_obj_t*lVw=lv_img_create(p);
    lv_img_set_src(lVw,&img_logo_atview);
    lv_obj_align(lVw,LV_ALIGN_TOP_MID,0,422);
    // Long-press logo AT-VIEW = oublie la pair BLE AT-CORE + reboot.
    // Utile quand on change de carte AT-CORE (MAC différente du nouveau
    // hardware, le filtre par MAC bloque sinon).
    lv_obj_add_flag(lVw,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lVw,_cbForgetPair,LV_EVENT_LONG_PRESSED,NULL);
    r_p2_bat=mkLbl(p,"AT-CORE : ---%",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,448);
    lv_obj_t*ver=mkLbl(p,"v0.7  --  2026-05-14",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,464);
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
// Traffic e.spd_kt = knots (AT-CORE clé "s"). Own g_status.spd = km/h (AT-CORE
// clé "spd" = kt*1.852). Donc conversions différentes : own /3.6, trafic *0.5144.
// Capped at 10s to avoid runaway extrapolation on BLE dropout.
void updateRadarDR(){
    if(!g_traffic.valid||!g_status.valid)return;
    float dt=fminf((float)(millis()-g_traffic.recv_ms)/1000.0f,10.0f);
    float our_spd_ms=(float)g_status.spd/3.6f;   // km/h → m/s (own, pas knots)
    float our_rad=(float)radarEffHdg()*(float)M_PI/180.0f;
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
            int rb=((int)dr_bear-radarEffHdg()+360)%360;
            float brd=(float)rb*(float)M_PI/180.0f;
            float dpx=fminf(dr_dist*(float)RAD_R/scale_m,(float)(RAD_R-8));
            int sx=(int)(RAD_CX+sinf(brd)*dpx);
            int sy=(int)(RAD_CY-cosf(brd)*dpx);
            int rel_hdg=((e.hdg_deg-radarEffHdg())%360+360)%360;
            float hr=(float)rel_hdg*(float)M_PI/180.0f;
            float cs=cosf(hr),sn=sinf(hr);
            lv_color_t col=dr_dist<1000?C_RED:dr_dist<3000?C_AMBER:TFG();
            if(e.type!=r_trf_last_type[i]){
                lv_img_set_src(r_trf_img[i],getAircraftIcon(e.type));
                r_trf_last_type[i]=e.type;}
            // Objet img = 48px, pivot (24,24). On centre TOUJOURS sur (sx,sy) via -24
            // (sinon en taille S/M le centre visuel dérive et le trait paraît décalé).
            lv_obj_set_pos(r_trf_img[i],sx-24,sy-24);
            // set_angle (sens horaire) aligne le nez de l'art nord-up sur +rel_hdg,
            // cohérent avec la trigo écran du vecteur vitesse (même rel_hdg).
            lv_img_set_angle(r_trf_img[i],(int16_t)(rel_hdg*10));
            lv_obj_set_style_img_recolor(r_trf_img[i],col,0);
            lv_obj_clear_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);
            float px_per_nm=(float)RAD_R/(float)g_cfg.scale_nm;
            float vect_px=fmaxf(6.f,fminf((float)e.spd_kt/60.0f*px_per_nm,35.f));
            // Trait fin partant juste DEVANT l'avion (≈ nez = 0.5*demi-icône) pour ne
            // pas se superposer au SVG. Bout = position dans 1 min (vect_px mesuré depuis
            // le CENTRE → distance impérativement = 1 min). pos(0,0) = coords absolues.
            int ih=kIconHalf[g_cfg.icon_sz];
            float nose_r=(float)ih*0.5f;
            r_vect_pts[i][0]={(lv_coord_t)(sx+(int)(nose_r*sn)),(lv_coord_t)(sy-(int)(nose_r*cs))};
            r_vect_pts[i][1]={(lv_coord_t)(sx+(int)(vect_px*sn)),(lv_coord_t)(sy-(int)(vect_px*cs))};
            lv_obj_set_pos(r_trf_vect[i],0,0);
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
    // Tâche F : overlay upload progress (full-screen modal post-vol)
    updUploadOverlay();
    // Refresh live de la ligne diagnostique DB sur page #02 (si auth en cours)
    if(g_auth_ov && g_auth_diag){
        char dbg[48];
        if(g_pilot_cnt>0)
            snprintf(dbg,sizeof(dbg),"DB: %d pilots (%s)",g_pilot_cnt,g_pilots_date[0]?g_pilots_date:"?");
        else
            snprintf(dbg,sizeof(dbg),"DB Firebase non chargee");
        lv_label_set_text(g_auth_diag,dbg);
        lv_obj_set_style_text_color(g_auth_diag,g_pilot_cnt>0?TGREY():C_RED,0);
    }
    // Si le picker est ouvert et la DB pilots vient d'évoluer, repeupler la liste.
    if(g_auth_ov && g_auth_step==0 && g_picker_list
       && g_pilot_cnt != g_picker_last_cnt){
        pickerRefreshList();
    }
    // Page 0 — checks live + batterie AT-CORE
    {
        // AT-CORE : label dynamique selon connexion ("AT-CORE_XXX" ou "AT-CORE")
        char clbl[32];
        if(g_connected && g_peer_name[0]) snprintf(clbl,32,"Connected to %s",g_peer_name);
        else                              snprintf(clbl,32,"AT-CORE");
        updCheckRow(CHK_CORE,clbl,        g_connected);
        // Bluetooth radio : ON dès que BLE est initialisé
        updCheckRow(CHK_BT,  "Bluetooth", g_bootDone);
        // GPS / LTE / ADS-B / OGN — sourcés depuis g_status (vu via AT-CORE)
        bool gps_ok = g_status.valid && g_status.gps_fix;
        bool lte_ok = g_status.valid && g_status.csq>5;
        bool adsb_ok= g_connected && g_status.valid && g_status.adsb_ok;
        bool ogn_ok = g_connected && g_status.valid && g_status.flarm_ok;
        updCheckRow(CHK_GPS, "GPS",                   gps_ok);
        updCheckRow(CHK_LTE, "LTE",                   lte_ok);
        updCheckRow(CHK_ADSB,"ADS-B / ADS-L",         adsb_ok);
        updCheckRow(CHK_OGN, "OGN / FLARM (868Mhz)",  ogn_ok);
        // Batterie AT-CORE (footer page #01 + page Settings)
        const char* bat_txt;
        lv_color_t  bat_col;
        const char* nm = g_peer_name[0]?g_peer_name:"AT-CORE";
        if(g_status.valid && g_status.bat>=0){
            snprintf(b,32,"%s : %d%%%s",nm,g_status.bat,g_status.charging?" " LV_SYMBOL_CHARGE:"");
            bat_col=g_status.charging?C_GREEN:
                    g_status.bat>=50?lv_color_hex(0x0f172a):
                    g_status.bat>=20?C_AMBER:C_RED;
        }else{
            snprintf(b,32,"%s : ---%%",nm);
            bat_col=TGREY();
        }
        bat_txt=b;
        if(r_p0_bat){lv_label_set_text(r_p0_bat,bat_txt);lv_obj_set_style_text_color(r_p0_bat,bat_col,0);}
        if(r_p2_bat){lv_label_set_text(r_p2_bat,bat_txt);lv_obj_set_style_text_color(r_p2_bat,bat_col,0);}
    }
    // Pilote — trigramme ligne 1, prénom+nom ligne 2
    if(r_sess_trig&&r_sess_name){
        if(g_session.valid){
            char t[8]; snprintf(t,sizeof(t),"● %s",g_session.trigram[0]?g_session.trigram:"---");
            lv_label_set_text(r_sess_trig,t);
            bool isOwner=strcmp(g_session.status,"owner")==0;
            lv_obj_set_style_text_color(r_sess_trig,isOwner?C_GREEN:C_AMBER,0);
            lv_label_set_text(r_sess_name,g_session.name[0]?g_session.name:"");
        }else if(g_connected){
            // Pas de session — afficher état pilots pour debug visible
            char t[16];
            if(g_pilot_cnt>0) snprintf(t,sizeof(t),"%dP - code?",g_pilot_cnt);
            else              snprintf(t,sizeof(t),"...");
            lv_label_set_text(r_sess_trig,t);
            lv_obj_set_style_text_color(r_sess_trig,TGREY(),0);
            lv_label_set_text(r_sess_name,"");
        }else{
            lv_label_set_text(r_sess_trig,"");
            lv_label_set_text(r_sess_name,"");}}
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
     // Charging (champ "chg" JSON STATUS) détecté AT-CORE : hausse tension OU float ≥4.13V.
     // Radar : sur secteur = "⚡xx%" vert | sur batterie = "xx%" (rouge si <20%).
     {int bat=(g_status.valid&&g_status.bat>=0)?g_status.bat:
              (g_debug.valid&&g_debug.bat_pct>=0)?g_debug.bat_pct:-1;
       char bb[12];
       if(bat<0){
          // Pas de données
          lv_label_set_text(r_hdr_bat,"--");
          lv_obj_set_style_text_color(r_hdr_bat,PILL_IC_OFF(),0);
      }else if(g_status.charging){
          // Sur secteur / en charge — NIVEAU + éclair, vert (super clair en 1 coup d'œil)
          snprintf(bb,sizeof(bb),LV_SYMBOL_CHARGE "%d%%",bat);
          lv_label_set_text(r_hdr_bat,bb);
          lv_obj_set_style_text_color(r_hdr_bat,C_GREEN,0);
      }else{
          // Sur batterie — niveau seul, PAS d'éclair, rouge si <20%
          snprintf(bb,sizeof(bb),"%d%%",bat);
          lv_label_set_text(r_hdr_bat,bb);
          lv_obj_set_style_text_color(r_hdr_bat,bat<20?C_RED:PILL_IC_ON(),0);}}
     #undef SET_PILL_TXT
     #undef SET_PILL_IMG
     }
    // Auth popup — attendre BLE conn + STATUS valid + min 2s (laisser voir progression #01)
    // Fallback : 10s apres connexion meme si STATUS n'arrive pas
#if !BYPASS_PILOT_AUTH
    if(!g_authShown&&!g_auth_ov&&!g_session.valid&&g_connected&&g_connect_ms){
        uint32_t elapsed = millis() - g_connect_ms;
        bool ready = (g_status.valid && elapsed > 2000) || (elapsed > 10000);
        if(ready){g_authShown=true;mkAuthOverlay();}
    }
#endif
    // Auto-navigate to radar once BLE+GPS ready (one-shot per connection)
    if(!g_autoNavDone&&!g_pair_ov&&g_connected&&g_status.valid&&g_status.gps_fix&&g_page==0){
        g_autoNavDone=true;g_navPending=true;g_navPage=1;}
    // Radar — heading-up en mouvement, north-up auto à l'arrêt (radarEffHdg()).
    // Le cap GPS (course over ground) n'est pas calculable sous RADAR_STILL_KMH :
    // on verrouille alors la rose au nord et la pill affiche "N".
    // Évolution possible : magnétomètre côté AT-CORE pour un cap indépendant
    // de la vitesse (cap magnétique vs cap sol).
    if(g_status.valid){
        if(g_status.spd < RADAR_STILL_KMH) lv_label_set_text(r_radar_hdg,"N " LV_SYMBOL_UP);
        else { snprintf(b,32,"%d°",g_status.hdg);lv_label_set_text(r_radar_hdg,b); }
        if(r_radar_gs){
            // g_status.spd est en km/h (AT-CORE envoie kt*1.852). Bug GS corrigé :
            //   kt   → km/h × 0.539957 ; km/h → valeur directe (avant: km/h étiqueté
            //   "kt", et mode km/h re-multipliait ×1.852 → double conversion).
            if(g_cfg.spd_kt) snprintf(b,32,"GS %dkt",(int)((float)g_status.spd*0.539957f+0.5f));
            else             snprintf(b,32,"GS %dkm/h",g_status.spd);
            lv_label_set_text(r_radar_gs,b);}
        const int cbear[]={0,90,180,270};
        for(int ci=0;ci<4;ci++){
            int rel=((cbear[ci]-radarEffHdg())%360+360)%360;
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
            if(g_alert.co)strcat(ab,"CO  ");if(g_alert.gforce)strcat(ab,"G-FORCE  ");
            if(g_alert.rpm)strcat(ab,"RPM  ");if(g_alert.traffic)strcat(ab,"TRAFFIC");
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

// ── WiFi AP — handlers & control ─────────────────────────────────────────────
static String sdFileListHtml(){
    if(!g_sd_ok)return "<li>SD non montée</li>";
    File dir=SD_MMC.open("/aip");
    if(!dir||!dir.isDirectory())return "<li class='grey'>Répertoire /aip vide</li>";
    String h="";int n=0;
    File f=dir.openNextFile();
    while(f){
        if(!f.isDirectory()){
            String nm=String(f.name());
            int sl=nm.lastIndexOf('/');if(sl>=0)nm=nm.substring(sl+1);
            h+="<li>"+nm+" <span style='color:#6b7280'>("+String(f.size()/1024)+" KB)</span></li>";
            n++;}
        f=dir.openNextFile();}
    dir.close();
    return n?h:"<li class='grey'>Aucun fichier</li>";}

static void handleRoot(){
    String page=F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>AT-VIEW AeroTrace</title><style>"
        "body{font-family:sans-serif;background:#0d1117;color:#e6edf3;max-width:480px;margin:0 auto;padding:16px}"
        "h1{color:#F5A623;margin-bottom:4px}h2{color:#60a5fa;font-size:.9em;margin-top:20px}"
        "form{background:#1f2937;padding:16px;border-radius:8px}"
        "input[type=file]{width:100%;padding:8px;background:#374151;color:#e6edf3;"
        "border:1px solid #4b5563;border-radius:4px;box-sizing:border-box}"
        "input[type=submit]{width:100%;padding:12px;background:#1f4068;color:#fff;"
        "border:none;border-radius:4px;font-size:1em;cursor:pointer;margin-top:8px}"
        "ul{list-style:none;padding:0}li{background:#1f2937;padding:8px 12px;"
        "border-radius:4px;margin:4px 0;font-size:.85em}.grey{color:#6b7280}"
        "</style></head><body>"
        "<h1>AT-VIEW AeroTrace</h1>"
        "<p style='color:#6b7280;font-size:.8em'>SSID : ");
    page+=g_unit_name;
    page+=F("</p><h2>Upload fichier</h2>"
        "<p style='color:#9ca3af;font-size:.8em'>AIP (.bin) → /aip/ &nbsp;|&nbsp; pilots.json → /</p>"
        "<form method='POST' action='/upload' enctype='multipart/form-data'>"
        "<input type='file' name='file'><br>"
        "<input type='submit' value='Envoyer sur la carte SD'></form>"
        "<h2>Firmware (OTA)</h2>"
        "<p style='color:#9ca3af;font-size:.8em'>firmware.bin AT-VIEW &rarr; flash slot OTA + reboot</p>"
        "<form method='POST' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='fw' accept='.bin'><br>"
        "<input type='submit' value='Flasher le firmware'></form>"
        "<h2>Fichiers dans /aip/</h2><ul>");
    page+=sdFileListHtml();
    page+="</ul></body></html>";
    g_webserver->send(200,"text/html",page);}

static File   g_upload_file;
static bool   g_upload_is_pilots=false;
static void handleUploadData(){
    HTTPUpload& u=g_webserver->upload();
    if(u.status==UPLOAD_FILE_START){
        g_upload_is_pilots=(u.filename=="pilots.json");
        String path=g_upload_is_pilots?"/pilots.json":"/aip/"+u.filename;
        g_upload_file=SD_MMC.open(path.c_str(),FILE_WRITE);
        Serial.printf("[WiFi] Upload start: %s\n",path.c_str());
    }else if(u.status==UPLOAD_FILE_WRITE){
        if(g_upload_file)g_upload_file.write(u.buf,u.currentSize);
    }else if(u.status==UPLOAD_FILE_END){
        if(g_upload_file)g_upload_file.close();
        if(g_upload_is_pilots)pilotDBLoad();  // reload immediately
        Serial.printf("[WiFi] Upload done: %u bytes\n",u.totalSize);}}

static void handleUploadDone(){
    g_webserver->send(200,"text/html",
        "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'></head>"
        "<body style='background:#0d1117;color:#22c55e;font-family:sans-serif;padding:20px'>"
        "<h2>&#10003; Upload OK</h2>"
        "<a style='color:#60a5fa' href='/'>&#8592; Retour</a></body></html>");}

// ── OTA firmware AT-VIEW (WP7) ────────────────────────────────────────────────
// Réception du firmware.bin via l'AP (route /update, distincte du /upload AIP→SD).
// Partition default_16MB.csv = 2 slots OTA → flash du slot inactif, pas de migration.
// Reboot différé (g_ota_reboot_ms) pour laisser la réponse HTTP partir.
static volatile uint32_t g_ota_reboot_ms=0;
static void handleOtaDone(){
    bool ok=!Update.hasError();
    g_webserver->sendHeader("Connection","close");
    g_webserver->send(200,"text/html",ok
        ? "<!DOCTYPE html><html><head><meta charset='UTF-8'></head>"
          "<body style='background:#0d1117;color:#22c55e;font-family:sans-serif;padding:20px'>"
          "<h2>&#10003; Firmware OK</h2><p>Redemarrage sur la nouvelle version...</p></body></html>"
        : "<!DOCTYPE html><html><head><meta charset='UTF-8'></head>"
          "<body style='background:#0d1117;color:#ef4444;font-family:sans-serif;padding:20px'>"
          "<h2>Echec flash</h2><p>Firmware inchange. Reessayez.</p></body></html>");
    if(ok)g_ota_reboot_ms=millis();}
static bool s_ota_hdr_ok=false;
static void handleOtaData(){
    HTTPUpload& u=g_webserver->upload();
    if(u.status==UPLOAD_FILE_START){
        s_ota_hdr_ok=false;
        Serial.printf("[OTA] start: %s\n",u.filename.c_str());
        if(!Update.begin(UPDATE_SIZE_UNKNOWN))Serial.println("[OTA] begin FAIL");
    }else if(u.status==UPLOAD_FILE_WRITE){
        // Garde anti-brick : refuse un .bin qui n'est pas une image ESP32-S3
        // (magic 0xE9 @0 + chip_id 9 @12 de l'en-tête esp_image). Évite de flasher
        // le firmware AT-CORE (ESP32, chip_id 0) ou un binaire étranger. 1er chunk.
        if(!s_ota_hdr_ok && u.currentSize>=13){
            s_ota_hdr_ok=true;
            if(u.buf[0]!=0xE9 || u.buf[12]!=0x09){
                Serial.println("[OTA] refuse: pas une image ESP32-S3");
                Update.abort();
            }
        }
        if(!Update.hasError() && Update.write(u.buf,u.currentSize)!=u.currentSize)
            Serial.println("[OTA] write incomplet");
        yield();   // feed WDT éventuel + respiration stack WiFi sur upload multi-MB
    }else if(u.status==UPLOAD_FILE_END){
        if(Update.end(true))Serial.printf("[OTA] OK %u bytes\n",u.totalSize);
        else Serial.printf("[OTA] end FAIL err=%d\n",Update.getError());
    }else if(u.status==UPLOAD_FILE_ABORTED){Update.abort();Serial.println("[OTA] aborte");}}

void wifiStart(){
    WiFi.mode(WIFI_AP);
    WiFi.softAP(g_unit_name,g_wifi_pass);
    MDNS.begin("atview");
    g_webserver=new WebServer(80);
    g_webserver->on("/",HTTP_GET,handleRoot);
    g_webserver->on("/upload",HTTP_POST,handleUploadDone,handleUploadData);
    g_webserver->on("/update",HTTP_POST,handleOtaDone,handleOtaData);   // OTA firmware (WP7)
    g_webserver->begin();
    g_wifi_active=true;
    if(s_wifi_v)lv_label_set_text(s_wifi_v,"192.168.4.1");
    Serial.printf("[WiFi] AP: %s  pass: %s\n",g_unit_name,g_wifi_pass);}

void wifiStop(){
    if(g_webserver){g_webserver->stop();delete g_webserver;g_webserver=nullptr;}
    MDNS.end();
    WiFi.softAPdisconnect(true);WiFi.mode(WIFI_OFF);
    g_wifi_active=false;
    if(s_wifi_v)lv_label_set_text(s_wifi_v,"OFF");
    Serial.println("[WiFi] Stopped");}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup(){
    Serial.begin(115200);
    if(!panel.begin()){while(1){Serial.println("Panel FAIL");delay(1000);}}
    Serial.printf("Touch: %s\n",panel.getTouchModelName());
    // SD card — before beginLvglHelper (factory example order).
    // panel.installSD() tries SDMMC_FREQ_HIGHSPEED (40MHz) by default.
    // If that fails, retry at SDMMC_FREQ_DEFAULT (20MHz) — XL9555 CS stays set.
    delay(50); // card power-on stabilization
    g_sd_ok=panel.installSD();
    if(!g_sd_ok){
        SD_MMC.setPins(39,40,38);
        g_sd_ok=SD_MMC.begin("/sdcard",true,false,SDMMC_FREQ_DEFAULT);
    }
    if(g_sd_ok){
        g_sd_gb=(uint32_t)(SD_MMC.totalBytes()/(1024ULL*1024*1024));
        if(!SD_MMC.exists("/aip"))SD_MMC.mkdir("/aip");
        Serial.printf("[SD] OK %uGB\n",g_sd_gb);
    }else{Serial.println("[SD] No card");}
    beginLvglHelper(panel);
    cfgLoad();acLoad();unitLoad();if(g_sd_ok)aipLoad();
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
    // Auth — load DB + restore cached session; popup shown when BLE+GPS ready
    pilotDBLoad();
    checkOwnerNVS();
    // FORCE encodage appareil : sans immat+type+hex, on ouvre d'office la page
    // AIRCRAFT et on interdit sa fermeture (cf _ac_key_cb d==202). La box ne doit
    // jamais opérer sans identité — sinon rien n'est transmis à SafeSky.
    if(!(g_ac_reg[0]&&g_ac_type[0]&&g_ac_hex[0])) mkAircraftOverlay();
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
    // ── Pairing AT-CORE : tant qu'aucun boîtier n'est lié, overlay de sélection ──
    if(g_paired_mac[0]==0){
        if(!g_pair_ov)pairOverlayShow();
        if(g_bind_confirm)pairShowConfirm();
        else if(g_binding){
            // connexion au candidat ne s'établit pas → abandon, retour liste
            if(g_bind_t0&&now-g_bind_t0>10000){
                if(g_connected&&g_client)g_client->disconnect();
                g_binding=false;g_bind_mac[0]=0;g_bind_name[0]=0;g_bind_t0=0;}
        }else{
            static uint32_t pr=0;if(now-pr>1000){pr=now;pairListRefresh();}}
    }else if(g_pair_ov){
        pairOverlayHide();   // devenu lié (post-bind) → ferme l'overlay
    }
    if(g_rebuildPages){g_rebuildPages=false;rebuildAllPages();}
    if(g_dataUpdated){g_dataUpdated=false;updateAllPages();}
    bool alert=hasAlert();
    {static uint8_t s_prevScale=4;
     if(alert&&!g_alertForced&&!g_inDebug){
         g_prevPage=g_page;g_alertForced=true;
         s_prevScale=g_cfg.scale_nm;
         if(g_cfg.scale_nm!=2){g_cfg.scale_nm=2;
             if(r_radar_scale_lbl){char b[8];snprintf(b,8,"2nm");lv_label_set_text(r_radar_scale_lbl,b);}}
         if(g_page!=1)switchPage(1);
     }else if(!alert&&g_alertForced){
         g_alertForced=false;
         g_cfg.scale_nm=s_prevScale;
         if(r_radar_scale_lbl){char b[12];snprintf(b,12,"%dnm",g_cfg.scale_nm);lv_label_set_text(r_radar_scale_lbl,b);}
         switchPage(g_prevPage);}}
    if(g_navPending){g_navPending=false;switchPage(g_navPage);}
    // Compléter session si code entré avant réception liste pilots, ou SD sans trigram
    if(g_session.valid&&(!g_session.name[0]||!g_session.trigram[0])&&s_session_pc[0]&&g_pilot_cnt>0){
        PilotEntry*pe=pilotFind(s_session_pc);
        if(pe){
            bool hadName=g_session.name[0]!=0;
            strlcpy(g_session.name,pe->name,sizeof(g_session.name));
            strlcpy(g_session.status,pe->status,sizeof(g_session.status));
            strlcpy(g_session.trigram,pe->trigram,sizeof(g_session.trigram));
            g_session.is_owner=(strcmp(pe->status,"owner")==0);
            if(g_session.is_owner){Preferences p;p.begin("auth",false);p.putString("owner",s_session_pc);p.end();}
            if(!hadName)showWelcome(g_session.name, s_instr_name[0]?s_instr_name:nullptr);
            g_dataUpdated=true;}}
    static uint32_t drLast=0;
    if(g_page==1&&now-drLast>=200){drLast=now;updateRadarDR();
        if(g_cfg.aip_en&&r_aip_layer&&g_status.valid)lv_obj_invalidate(r_aip_layer);}
    if(g_wifi_active&&g_webserver)g_webserver->handleClient();
    if(g_ota_reboot_ms&&millis()-g_ota_reboot_ms>1200){Serial.println("[OTA] reboot");delay(200);ESP.restart();}
    // WP8 — liste vols prête : handshake flt_rdy 0→1 (AT-CORE met 0 à la réception
    // de {"cmd":"flights"}, 1 quand la liste est construite) → lit CHR_FLIGHTS.
    if(g_vols_loading&&g_vols_ov){
        if(g_connected&&g_status.flt_rdy==1&&millis()-g_vols_t0>1500){
            g_vols_loading=false;volsBuildList();   // lecture BLE seulement si connecté + liste fraîche
        }else if(millis()-g_vols_t0>12000){
            g_vols_loading=false;if(g_vols_load)lv_label_set_text(g_vols_load,"Timeout - retry");
        }
    }
    // WP8 — suivi transfert : on RESTE sur la page Vols jusqu'à flt_phase 4 (OK) / 5 (fail).
    // seen3 : attendre d'avoir vu UPLOADING (3) avant d'accepter 4/5 (évite un 4 périmé d'un
    // upload précédent). Sur succès → recharge la liste (greys à jour).
    if(g_vols_xfer_pending&&g_vols_ov){
        uint8_t ph=g_status.flt_phase;
        if(ph==3)g_vols_xfer_seen3=true;
        if(g_vols_xfer_seen3&&(ph==4||ph==5)){
            g_vols_xfer_pending=false;
            if(g_vols_load)lv_label_set_text(g_vols_load,ph==4?"Transfer OK":"Transfer failed");
            if(ph==4){ sendCtl("flights"); g_status.flt_rdy=0; g_vols_loading=true; g_vols_t0=millis(); }
        }else if(millis()-g_vols_xfer_t0>90000){
            g_vols_xfer_pending=false;if(g_vols_load)lv_label_set_text(g_vols_load,"Transfer timeout");
        }
    }
    lv_timer_handler();delay(5);}

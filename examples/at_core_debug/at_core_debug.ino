/**
 * AT-VIEW AeroTrace — v0.6
 * LilyGo T-RGB 2.8" | ESP32-S3 | 480×480 circular
 * 3 pages: Status(boot+live) | Radar | Settings  (+hidden Debug)
 * Swipe L/R — long press version → Debug
 * Christophe — 2026-05-04
 */

// Waveshare 2.41" (RM690B0 450×600 portrait → paysage 600×450) : réutilise TOUT
// le layout T4 (→ #define BOARD_T4S3) mais panneau/touch/lvgl par son shim Arduino_GFX
// (→ PANEL_WS241 garde tous les appels spécifiques LilyGo). 4ᵉ cible AT-VIEW.
#ifdef BOARD_WS241
#define BOARD_T4S3
#define PANEL_WS241
#endif

#ifdef BOARD_T4S3            // ── Port test LilyGo T4-S3 AMOLED 2.41" (600×450 rect) ──
#ifdef PANEL_WS241
#include "ws241_shim.h"      // Arduino_GFX RM690B0 + SensorLib FT6336 + beginLvglHelper maison
#include <driver/i2s_std.h>  // DAC I2S PCM5102A (audio) — include EN TÊTE (un #include en plein
                             // milieu du .ino casse le générateur de prototypes Arduino → T4 KO)
#define SD_MMC SD            // SD en SPI (fs::FS partagé) — best-effort sur la 2.41
#else
#include <LilyGo_AMOLED.h>   // lib LilyGo-AMOLED-Series : RM690B0 QSPI + touch CST226SE
#include <LV_Helper.h>       // LV_Helper fourni par la lib AMOLED (même pattern beginLvglHelper)
#include <SD.h>              // SD en SPI sur T4-S3 (pas de slot SD_MMC)
#define SD_MMC SD            // quick&dirty : SDFS et SDMMCFS partagent l'API fs::FS
#endif
#elif defined(BOARD_WS216)   // ── Waveshare ESP32-S3-Touch-AMOLED-2.16 (carré 480×480) ──
#include "ws216_shim.h"      // Arduino_GFX CO5300 + SensorLib CST9220 + beginLvglHelper maison
#define SD_MMC SD            // SD en SPI (comme T4) — fs::FS partagé
#else                        // ── Cible nominale : LilyGo T-RGB circulaire 480×480 ──
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
#endif
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
#include <HTTPClient.h>        // (2026-06-25) cloud-pull OTA AT-VIEW : download depuis Firebase Storage
#include <WiFiClientSecure.h>  //              (HTTPS, setInsecure — public-read firmware/atv/**)
#include <ESPmDNS.h>
#include "esp_mac.h"  // esp_read_mac(ESP_MAC_BT) → nom AT-VIEW = ATV-<MAC>
#include "img_vl3.h"
#include "img_aircraft_icons.h"
#include "img_safesky.h"
#include "img_flarm.h"
#include "img_logos.h"
#include <string>

// ── IMU 6 axes — mouchard G + assiettes (WS216/WS241 seulement) ───────────────────
// Le QMI8658 n'existe QUE sur les Waveshare 2.16/2.41 (ni T4-S3 ni T-RGB) → feature gardée.
// Lit accel+gyro, calcule facteur de charge / G latéral / pitch / roll relatifs au repos
// (auto-zéro au sol), et pousse au boîtier par BLE → écrit dans le CSV du vol (NormAc/LatAc/
// Pitch/Roll). Interprétation/seuils = dashboard après le vol. cf mémoire imu_mouchard_g_attitude.
#if defined(BOARD_WS216) || defined(BOARD_WS241)
#define HAS_IMU 1
#include <SensorQMI8658.hpp>
#ifdef BOARD_WS241
#define IMU_SDA 47
#define IMU_SCL 48
#else   // BOARD_WS216
#define IMU_SDA 15
#define IMU_SCL 14
#endif
#endif

// ── Compat BLE core 2.x / 3.x ────────────────────────────────────────────────────
// Les getters BLE (getManufacturerData / getAddress().toString() / readValue()) renvoient
// std::string en core 2.x (Bluedroid historique) mais Arduino String en core 3.x. Ce helper
// normalise en std::string en PRÉSERVANT les octets (manuf-data binaire → pas de troncature
// sur un éventuel 0x00), pour garder le code aval (size()/[]/== std::string) inchangé.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static inline std::string bleStr(const String& s){ return std::string(s.c_str(), s.length()); }
#else
static inline const std::string& bleStr(const std::string& s){ return s; }
#endif

// ── Version AT-VIEW (T-RGB / T4-S3) ─────────────────────────────────────────────
// Entier incrémenté À CHAQUE flash/release (même discipline que FW_VERSION côté
// AT-CORE). Affiché partout (Welcome, Auth, System) en "ATV v<N>  <date build>" :
// la date (__DATE__) distingue 2 builds du même jour, le numéro suit les releases.
// >>> RÈGLE : bumper VIEW_VERSION d'UN cran à chaque commit qu'on flashe sur l'écran. <<<
//   v1 : socle UI radar (était figé ici malgré ~10 itérations — d'où ce versioning).
//   v2 : icônes trafic L ×1.5 (84px) + immat/Δalt agrandis + vecteur 1-min à l'échelle.
//   v3 : mouvement trafic lissé (base DR par avion → fin du surplace/recul/bond 1 Hz).
//   v4 : easing position trafic (2ᵉ couche → recalage SafeSky invisible entre 2 fixes).
//   v5 : glyphe zoom +/- agrandi (montserrat_40) — bien visible dans le bouton 64px.
//   v6 : chip "Start flight" dispo après un vol terminé (ph≠UPLOADING) — débloque le
//        cas immobile+sans fix où ni START ni STOP n'apparaissaient (impasse multi-leg).
//   v7 : transfert Vols anti-stall (timeout sur absence de progrès, pas délai absolu) —
//        fin du FAUX "Transfer timeout" sur les gros CSV (gzip de plusieurs min).
//   v8 : client BLE recréé à chaque (re)connexion → corrige le bug "écritures perdues
//        après une reconnexion" (Bluedroid gardait un cache handles/conn_id périmé).
//   v9 : page FLIGHTS (Vols) plein 600×450 sur T4-S3 (liste + lignes + boutons élargis,
//        au lieu du layout 480 rond hérité du T-RGB).
//   v10 : page FLIGHTS encore agrandie (T4) — liste plus haute (258), lignes 40px/font16,
//         boutons 46/42px, titre font22 ; WiFi remonté sous le titre.
//   v11 : passe anti-clip (textes/logos plus collés en haut/bas sur T4 — auth/welcome/
//         page0/vols/maint), + affichage du hotspot enregistré (STATUS "wss" du boîtier).
//   v12 : page Status (T4) logos AEROTRACE/ATVIEW agrandis + bloc versions ré-espacé
//         (fin du chevauchement batterie/ATV/ATC) ; message MAJ Maintenance explicite
//         (3 états : MAJ dispo vN / à jour vérifié / pas encore vérifié).
//   v13 : page Status (T4) logos ATVIEW/AEROTRACE descendus (vwY 24→46, atY 62→86) — plus joli.
//   v14 : page Status (T4) tous les textes +~25% (identité 14→18, checks 14→18 + dots 22→28,
//         versions 12→16, espacements adaptés) ; champ SSID Maintenance pré-rempli avec le
//         dernier hotspot validé (NVS écran, fallback STATUS "wss" du boîtier).
//   v15 : (AT-CORE v20) badge "GND" + trafic GRIS quand SafeSky passe en mode éco sol
//         (cadence 60 s) — pastille reste VERTE (SafeSky fonctionne), trafic rafraîchi
//         lentement → conscience situationnelle préservée sans taper le forfait data.
//   v16 : (AT-CORE v22) bouton "Reboot box" (Maintenance, double-tap) → {"cmd":"reboot"}
//         → RST logiciel du boîtier scellé/inaccessible sans accès physique.
//   v17 : placement — "Reboot box" à droite de "Update" (était hors cadre bas T4 450px) ;
//         badge "GND" déplacé à droite du point santé SafeSky.
//   v18 : badge "GND" affiché SEULEMENT si SafeSky vert (ss_ok) → plus de "GND + rouge"
//         trompeur quand le data est down (ex. antenne LTE débranchée).
//   v19 : WS-216 migrée sur pile Waveshare (ESP32 core 3.x pioarduino + GFX 1.6.4) → noir
//         CO5300 enfin profond. + fix conflit SPI : SD isolée sur HSPI (SPI2/FSPI = écran).
//   v20 : (AT-CORE v23) bouton "WiFi Setup" (Maintenance, double-tap) → {"cmd":"portal"} :
//         ouvre le portail WiFi du boîtier (SoftAP ATCORE-SETUP-<box>) + overlay guidant le
//         pilote (SSID/pass/URL) → saisie identité aéronef (callsign/type/hex) au clavier
//         smartphone + creds hotspot + OTA navigateur. Parse STATUS "box" pour le SSID.
//   v21 : (AT-CORE v24) bouton "Update both" (Maintenance, double-tap) → relais ATV-as-STA :
//         ouvre le portail boîtier ({"cmd":"portal"}) PUIS rejoint son AP en STA + garde son
//         updater web (/update) + s'annonce (GET /atv) → la page portail du boîtier pointe
//         vers cet updater. Un seul téléphone/réseau flashe ATC+ATV. Machine d'état relayTick().
#define VIEW_VERSION  "78"   /* BUILD monotone — bump à CHAQUE flash. = version.txt OTA écran (atoi). NE PAS remettre à zéro. v78 : WS-216 glyphes zoom poussés DANS L'ANGLE (décalage vers le coin bas-ext, zone tactile inchangée). v77 : WS-216 zoom = 2 GRANDES zones tactiles de coin (bas-gauche −, bas-droite +), juste le glyphe barres (style T4) centré, EVENT_BUBBLE (swipe préservé) → taps fiables au 1/4 inférieur hors radar. v76 : WS-216 (carré) — boutons zoom +/- dans les coins bas (+ bas-droite, − bas-gauche), agrandis 34→56 px + cible tactile élargie (taps fiables) ; radar RAD_R 175→192 (le carré n'a pas de verre qui clippe). WS-216 only, T-RGB rond inchangé. v75 : WS-241 fine-tuning radar (deltas WS241-only, T4 inchangé) — cluster SafeSky/LTE/GNSS descendu (R_TOP_EXTRA), engrenage+chip remontés (R_GEAR_UP), boutons +/- rapprochés du centre (R_ZOOM_IN). v74 : WS-241 accueil — cluster versions descendu de +15 (au lieu de +30) → ~27 px de marge sous l'ATC (ne colle plus en bas). v73 : WS-241 marges de respiration — RAD_R 208→198 (~20 px de blanc haut/bas, cardinaux N/S ne touchent plus) + bouton zoom "−" réancré SCR_H (ne flotte plus). v72 : WS-241 layout 600×480 — radar recentré (RAD_CY 240/RAD_R 208), pages+overlays+AIP+gear+accueil réancrés via SCR_W/SCR_H board-aware (T4 reste 450). v71 : FIX bande noire bas WS-241 — dalle 2.41 = 600×480 (pas 450), WS241_NATIVE_W/LCD_H 450→480 + UI_OY 0 → canvas 480 remplit pile la hauteur. v70 : ENCODEUR ROTATIF + poussoir (EC11) WS-241 — tourner=zoom radar (page suiv/préc ailleurs), clic=page suivante, appui long=action sheet Start/Stop. A=GPIO38/B=39/SW=40, décodeur quadrature sur ISR. Gated BOARD_WS241, no-op ailleurs. v69 : AUDIO I2S (DAC PCM5102A) TEST sur WS-241 — bip de validation au boot + bouton TEST (BCK=5/LCK=6/DIN=7, SCK→GND ; API i2s_std core 3.x). Gated BOARD_WS241, no-op ailleurs. v68 : SYSTEM = tuiles en grille 2 colonnes (même style que la grille SETTINGS, contour bleu), plus de barres pleine largeur ; SD card en libellé d'état. v67 : SYSTEM = un gros bouton plein large par page (le nom EST sur le bouton, plus de couple label+OPEN). v66 : MENU Settings — fusion CONFIG+DISPLAY en 1 section "CONFIG" scrollable (swipe down ; brightness/theme/scale/vfilt/altdiff/callsign), grille passe à 5 tuiles (6e libre dev futur), SYSTEM = boutons WIFI/FlightLogs/Updates/Diagnostic/Test. v65 : CULLING GÉOGRAPHIQUE AIP dans aipDrawCb — ne dessine que CTR/aérodromes dans la fenêtre radar (own ± portée×1.6), rejet bbox/point en e6 avant projection trig → coût ∝ visible, plus ∝ EU entière → tactile +/- réactif au sol ET en vol (l'AIP bouge en vol, le redraw reste léger). v64 : AIP_MAX_CTR 2048. v63 : AIP embarquée flash. */
// ── Versioning lisible MAJOR.MINOR.BUILD + canal (miroir de l'ATC). ────────────
// VIEW_TRAIN partagé avec l'ATC (même release) ; VIEW_CH : 0=dev 1=rc 2=client.
// Affiché "1.2.38-dev" sur ABOUT (couleur ambre/bleu/vert). version.txt reste = VIEW_VERSION.
#define VIEW_TRAIN  "1.2"
#define VIEW_CH     0
#if   VIEW_CH==0
  #define VIEW_CH_SUFFIX "-dev"
#elif VIEW_CH==1
  #define VIEW_CH_SUFFIX "-rc"
#else
  #define VIEW_CH_SUFFIX ""
#endif
#define VIEW_VSTR     VIEW_TRAIN "." VIEW_VERSION VIEW_CH_SUFFIX   // ex "1.2.38-dev"
#define VIEW_VER_STR  "ATV " VIEW_VSTR "  " __DATE__               // boot banner (ex "ATV 1.2.38-dev  Jun 26 2026")

#ifdef BOARD_T4S3
#ifdef PANEL_WS241
WS241_Panel panel;           // shim Arduino_GFX/SensorLib RM690B0 paysage (ws241_shim.h)
static inline void panelBright(uint8_t v){ panel.setBrightness(v>=16?255:v*17); }
#define lv_font_montserrat_10 lv_font_montserrat_12
// Paysage 600×480 (dalle 2.41 = 480 de haut, pas 450) ; canvas UI 480×480 → remplit pile
// la hauteur. UI_OX=60 centre horizontalement (480 dans 600) ; UI_OY=0 (plus de rognage ni
// de bande noire en bas — corrigé 2026-06-30 après constat bande pleine largeur).
#define UI_OX  60
#define UI_OY  (0)
#else
LilyGo_Class amoled;
#define panel amoled         // les call-sites panel.* (begin via setup dédié) pointent sur l'AMOLED
// Brightness : T-RGB = 16 niveaux hardware, AMOLED = 0-255 → mapping ×17 ici
static inline void panelBright(uint8_t v){ amoled.setBrightness(v>=16?255:v*17); }
// montserrat_10 absente du lv_conf de la lib AMOLED → fallback sur la 12 (texte un poil plus gros)
#define lv_font_montserrat_10 lv_font_montserrat_12
// Recadrage : écran 600×450 paysage, canvas UI 480×480 centré → bandes vides
// 60 px à gauche/droite, 15 px rognés en haut et en bas (UI circulaire : perte négligeable)
#define UI_OX  60
#define UI_OY  (-15)
#endif
#elif defined(BOARD_WS216)
WS216_Panel panel;           // shim Arduino_GFX/SensorLib (ws216_shim.h)
// CO5300 = brightness 0-255 → mapping ×17 depuis l'échelle config 0-16 (comme AMOLED T4)
static inline void panelBright(uint8_t v){ panel.setBrightness(v>=16?255:v*17); }
// Écran CARRÉ 480×480 plein cadre → pas de recadrage (canvas = écran, comme T-RGB)
#define UI_OX  0
#define UI_OY  0
#else
LilyGo_RGBPanel panel;
static inline void panelBright(uint8_t v){ panel.setBrightness(v); }
#define UI_OX  0
#define UI_OY  0
#endif

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
#define BLE_CHR_WIFICRED "6E40000C-B5A3-F393-E0A9-E50E24DCCA9E" // read  : creds WiFi club {s,p} hérités du boîtier (brique 0)
#define BLE_CHR_IMU     "6E40000D-B5A3-F393-E0A9-E50E24DCCA9E"  // write : mouchard G/assiette (IMU écran) → CSV boîtier
#define BLE_CHR_AIP     "6E40000E-B5A3-F393-E0A9-E50E24DCCA9E"  // read  : transfert AIP EU (flash boîtier S3) → RAM écran, chunké (A1)
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
    bool ss_ok;          // échange SafeSky UDP réussi < 15 s (preuve connexion, même sans trafic)
    uint8_t ss_mode;     // (v20) cadence SafeSky AT-CORE : 0=vol · 1=sol/idle (60s) → badge GND + trafic gris
    uint8_t flt_phase;   // 0=fly 1=ended 2=closed 3=uploading 4=done 5=fail (tâche D)
    uint8_t upload_pct;  // 0..100 (tâche D)
    uint8_t flt_rdy;     // WP8 : 1=liste vols prête à lire (CHR_FLIGHTS)
    uint8_t flt_st;      // cycle de vol : 0=sol (pas démarré) 1=en vol 2=arrêt imminent
    uint32_t aip_xfer_len; // (A1) longueur du flux AIP prêt côté boîtier (STATUS "axl", 0=pas prêt)
    uint8_t wst;         // WiFi : 0 idle 1 connexion 2 OK 3 SSID absent 4 échec
    char    wip[16];     // dernière IP WiFi (proof de connexion)
    char    wssid[33];   // SSID hotspot enregistré côté boîtier (STATUS "wss") = dernier validé/en mémoire
    uint8_t ota;         // OTA : 0 idle 1 check 2 download 3 OK(reboot) 4 échec 5 à jour
    uint8_t opct;        // OTA download %
    uint8_t fwv;         // version firmware AT-CORE (BUILD entier)
    char    fws[16];     // version lisible AT-CORE "1.2.36-dev" (STATUS "fws")
    char    fwd[14];     // date de build AT-CORE (__DATE__)
    uint8_t oav;         // MAJ dispo : version cloud si > fwv, sinon 0
    char    box[8];      // box-id boîtier (STATUS "box", ex "CE276D") → SSID portail ATCORE-SETUP-<box>
    };
struct FlightData  { float gforce_z; int co_ppm,rpm,phase; bool valid; };
#define MAX_TRF 5
struct TrafficEntry { char cs[9]; int dist_m,alt_m,bear_deg,hdg_deg,spd_kt,type; bool visible; uint32_t base_ms; float disp_ex,disp_ey; bool disp_init; };
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
static const char*   kCircNames[]  ={"AUTO","CIRC","RTE"};   // override mode alerte (0 auto / 1 circuit / 2 route)
static const uint16_t kIconZoom[]={320,384,448};  // zoom for 60/72/84 px from 48px base (L=84px = 1.5× — visibilité vol 2026-06-08)
static const int8_t  kIconHalf[]={30,36,42};
struct CfgData { uint8_t scale_nm,brightness,trf_src; bool dist_nm,alt_ft,dark,show_grnd,wifi_en,aip_en,ad_heli,spd_kt; int16_t vfilt_ft; uint8_t icon_sz; bool show_cs,show_vdiff; uint8_t circuit_ovr; };  // circuit_ovr : 0=AUTO 1=CIRCUIT(forcé) 2=ROUTE(forcé) — override alerte (en fin de struct → init agrégat inchangé, défaut 0)
// NB : l'init historique {…,false,2000,2} décalait les champs (le 2000 tombait sur le
// bool spd_kt → true, vfilt_ft recevait 2, icon_sz défaut 0). Core 3.x refuse le narrowing
// 2000→bool ; on fige ici EXACTEMENT les valeurs que core 2.x calculait (spd_kt=true,
// vfilt_ft=2, icon_sz=0) pour ne pas changer le comportement existant.
static CfgData     g_cfg={4,16,3,true,true,false,true,false,true,false,true,2,0};   // (juin 2026) défaut thème = LIGHT (champ dark=false) — aligné maquette
// Segmented toggle (UI Settings T4-S3). Types + registres déclarés
// INCONDITIONNELLEMENT (2026-06-08) : les fonctions seg*/updSeg* ne sont pas
// guardées #ifdef BOARD_T4S3 → sur T-RGB le type doit exister quand même (les
// registres restent vides, g_seg_n=0 → boucles no-op). Défini AVANT toute
// fonction pour que le générateur de prototypes Arduino voie le type.
struct SegCtl {
    lv_obj_t *segA, *segB;   // les deux moitiés cliquables
    lv_obj_t *lblA, *lblB;   // labels (recoloration texte actif/inactif)
    bool     *val;           // champ config bool pointé
    bool      aIsTrue;       // segA sélectionné ⇔ *val == aIsTrue
};
static SegCtl g_seg[8];   // 3 Radar (DIST/SPEED/THEME) + 3 Traffic (GROUNDED/AIP/HELI) + 1 System (WIFI)
static int    g_seg_n = 0;
// Segmented multi-options (SOURCE 4, ICONS SIZE 3) — même raison de placement.
struct SegN {
    lv_obj_t *cell[4], *lbl[4];
    uint8_t   n;        // nombre d'options
    uint8_t  *val;      // index courant (champ config uint8_t)
    uint8_t   kind;     // 0=neutre · 1=ICONS (applique le zoom aux icônes trafic)
};
static SegN g_segn[2];
static int  g_segn_n = 0;
static Preferences g_prefs;

static StatusData  g_status  = {};
static FlightData  g_flight  = {};
static TrafficData g_traffic = {};
static uint32_t g_ss_lost_ms = 0;   // millis() de la perte signal SafeSky (0 = signal OK) — vieillissement trafic
static lv_obj_t* r_ss_dot = nullptr; // point santé signal (T4) : vert = UDP OK <10 s, rouge = perte
static lv_obj_t* r_ss_gnd = nullptr; // (v20) badge "GND" : SafeSky en mode éco sol (cadence 60s)
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
                                *g_chrFl=nullptr,  // FLIGHTS read (6E40000B) — liste vols SD (WP8)
                                *g_chrWc=nullptr,  // WIFICRED read (6E40000C) — creds WiFi club hérités (brique 0)
                                *g_chrImu=nullptr, // IMU write (6E40000D) — mouchard G/assiette → CSV boîtier
                                *g_chrAip=nullptr; // AIP read (6E40000E) — transfert AIP EU boîtier S3 → RAM écran
// Pilot list BLE reassembly buffer
static char    g_prx_buf[4096] = {};
static int     g_prx_len       = 0;
static volatile bool g_connected=false, g_doConnect=false, g_doReconnect=false;
static volatile bool g_scanning=false;   // (juin 2026) scan BLE async en cours (anti-relance + reset propre)
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
enum { CHK_CORE=0, CHK_BT=1, CHK_GPS=2, CHK_LTE=3, CHK_SKY=4, CHK_OGN=5 };   // (juin 2026) slot 4 = SafeSky (ex-ADSB)
static lv_obj_t *r_chk_dot[N_CHK]={};   // cercles
static lv_obj_t *r_chk_ico[N_CHK]={};   // ✓ blanc (visible si actif)
static lv_obj_t *r_chk_lbl[N_CHK]={};   // texte
static bool      g_chk_latched[N_CHK]={};  // une fois actif, V reste (reset au disconnect)
static lv_obj_t *r_p0_bat=nullptr;      // "Battery AT-CORE : XX%"
static lv_obj_t *r_p0_atc=nullptr;      // ligne "ATC vN  date" (version AT-CORE live, page #01)

// ── Widget refs — Radar (page 1) ──────────────────────────────────────────────
#ifdef BOARD_T4S3
// ── Layout radar spécifique T4-S3 (écran 600×450 paysage) ────────────────────
// La page radar est PLEIN ÉCRAN (600×450 à (0,0)) → coords radar = coords écran.
// Radar maximisé à droite, annotations agrandies en colonne gauche.
// (juin 2026) Radar CENTRÉ sur l'écran (« mettre le radar bien au milieu »).
// CX=300 = centre du 600×450 → cluster bas + pill cap recentrés (RB_DX/HDG_DX=0).
#define RAD_CX 300          // centre radar écran (= centre du 600 px)
#ifdef PANEL_WS241
#define RAD_CY 240          // WS-241 = dalle 600×480 → centre vertical 240 (profite des 30 px supp)
#define RAD_R  198          // marge de respiration ~20 px haut/bas (cardinaux N/S : 240±(198+12)=±210 → bord à 30, label ~20 → ~20 px de blanc)
#else
#define RAD_CY 225
#define RAD_R  193          // T4-S3 450 px : (juin 2026) réduit pour que S/N (cardinaux extérieurs) tiennent
#endif
#define RLC_X  10           // x colonne annotations gauche (pastilles GPS/LTE + mode SafeSky)
#define RB_DX  0            // cluster scale/GS centré sous le radar (radar au milieu)
#define RB_DY  0
#define HDG_DX 0            // pill cap + overlays centrés sur le radar (= centre écran)
#define CO_SZ  520          // arc CO : Ø extérieur (rayon mi-arc 252 = RAD_R+32)
#define CO_MIR (-1.0f)      // arc CO en bas-GAUCHE (miroir horizontal du bas-droite)
#define PILL_W 64           // pills annotations agrandies
#define PILL_H 40
#define PILL_FONT lv_font_montserrat_20
#define HDG_W  92           // pill heading agrandie
#define HDG_H  36
#define HDG_FONT lv_font_montserrat_20
#define ZOOM_SZ 64          // boutons zoom +/- ergonomie vol (2026-06-05) — coins droits opposés
#define ZOOM_FONT lv_font_montserrat_40   // glyphe +/- bien visible dans le bouton 64px (2026-06-08)
#define CHIP_W 200          // chip Start/End flight agrandi
#define CHIP_H 56
#define CHIP_FONT lv_font_montserrat_20
#define RAD_FONT lv_font_montserrat_20   // scale / GS / cardinales
#else
#define RAD_CX 240
#define RAD_CY 240
#define RAD_R  175
#define RLC_X  0            // (non utilisés sur T-RGB — layout d'origine)
#define RB_DX  0
#define RB_DY  0
#define HDG_DX 0
#define CO_SZ  440
#define CO_MIR (1.0f)
#define PILL_W 52
#define PILL_H 32
#define PILL_FONT lv_font_montserrat_16
#define HDG_W  72
#define HDG_H  28
#define HDG_FONT lv_font_montserrat_16
#define ZOOM_SZ 34
#define ZOOM_FONT lv_font_montserrat_20   // T-RGB : bouton 34px → glyphe inchangé
#define CHIP_W 170
#define CHIP_H 48
#define CHIP_FONT lv_font_montserrat_16
#define RAD_FONT lv_font_montserrat_14
#endif
// WS-216 = carré 480×480 (PAS de verre qui clippe les coins, contrairement au T-RGB rond) :
// radar légèrement agrandi. Le zoom passe par de GRANDES zones tactiles de coin (cf buildRadarPage).
#if defined(BOARD_WS216)
 #undef  RAD_R
 #define RAD_R    192
#endif
// (juin 2026) Décalage radial des cardinaux N/E/S/W vs l'anneau : T4-S3 = À
// L'EXTÉRIEUR (+), T-RGB = à l'intérieur (−24, layout rond d'origine inchangé).
#ifdef BOARD_T4S3
 #define RAD_CARD_OFF  12
#else
 #define RAD_CARD_OFF  (-24)
#endif
// Débord hors mire (2026-06-05, demande vol test) : sur le T4 rectangulaire,
// trafic + AIP se dessinent jusqu'aux BORDS de l'écran (coins ≈ 2× RAD_R du
// centre radar) — le cercle n'est qu'une référence d'échelle. Sur T-RGB rond,
// 1.0 : le verre clippe physiquement de toute façon.
#ifdef BOARD_T4S3
 #define RAD_OVERSCAN 2.1f
 #define AIP_CULL     2.2f
#else
 #define RAD_OVERSCAN 1.0f
 #define AIP_CULL     1.5f
#endif
// Dimensions écran/page board-aware (pour passer le radar/overlays/positions de 450→480 sur
// la Waveshare 2.41 sans toucher au T4-S3 qui reste 600×450). T-RGB/WS216 = 480×480.
#ifdef BOARD_T4S3
 #ifdef PANEL_WS241
  #define SCR_W 600
  #define SCR_H 480
 #else
  #define SCR_W 600
  #define SCR_H 450
 #endif
#else
 #define SCR_W 480
 #define SCR_H 480
#endif
// Deltas d'ajustement radar WS-241 UNIQUEMENT (T4-S3 = 0 → inchangé). Fine-tuning hauteur 480.
#ifdef PANEL_WS241
 #define R_TOP_EXTRA 12   // descend le cluster SafeSky/statut/LTE/GNSS (icône SafeSky était trop haute)
 #define R_GEAR_UP   24   // remonte l'engrenage Settings + chip (étaient trop bas)
 #define R_ZOOM_IN   18   // rapproche les boutons +/- du centre (+ trop haut, − trop bas)
#else
 #define R_TOP_EXTRA 0
 #define R_GEAR_UP   0
 #define R_ZOOM_IN   0
#endif
static lv_obj_t *r_radar_hdg, *r_radar_scale_lbl, *r_radar_gs;
static lv_obj_t *r_radar_ver=nullptr;   // version firmware + date (bas de la page radar)
static lv_obj_t *r_flt_stop=nullptr;    // panneau STOP (fin de vol) — emplacement ex-ADS-B, visible en vol
static lv_obj_t *r_card[4];
static lv_obj_t *r_radar_cs[MAX_TRF],*r_radar_alt[MAX_TRF];
static lv_obj_t *r_trf_img[MAX_TRF],*r_trf_vect[MAX_TRF];
static lv_point_t r_vect_pts[MAX_TRF][2];
static int r_trf_last_type[MAX_TRF];
static lv_obj_t *r_alert_overlay, *r_aov_text;
static lv_obj_t *r_circ_lbl=nullptr;   // chip "AUTO/CIRC/RTE" sur le radar (override mode alerte, tap = cycle)
// Jauge CO : capteur pas câblé → UI compilée out (remettre 1 au montage du capteur).
#define UI_CO_EN 0
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
static lv_obj_t *s_circ_v=nullptr;   // (T-RGB) valeur ALERT MODE (AUTO/CIRC/RTE) dans l'onglet TRAFFIC
static lv_obj_t *s_ac_v,*s_wifi_v,*s_sd_v;
#define S_NPG 3                        // sous-pages Settings : Radar / Traffic / System
static lv_obj_t *s_pg[S_NPG] = {};
static uint8_t   s_pg_idx = 0;
static lv_obj_t *r_p2_bat = nullptr;   // "Battery AT-CORE : XX%" footer settings page (T-RGB)
static lv_obj_t *s_set_title = nullptr; // titre dynamique « SETTINGS / <section> » (T4-S3)
static lv_obj_t *s_set_uline = nullptr; // fine ligne bleue sous le titre (largeur = texte)
static lv_obj_t *s_sys_atcver= nullptr; // page System : version AT-CORE (live BLE)
static lv_obj_t *s_sys_atcbat= nullptr; // page System : batterie AT-CORE (live BLE)
static const char* kSetTab[S_NPG] = {"RADAR","TRAFFIC","SYSTEM"};
// ── Settings refondu (menu grille → 6 sections → popups) — board-INDÉPENDANT depuis le
//    port T-RGB (2026-06-27) : T4 (600×450 rect) ET T-RGB (480×480 rond) via géométrie SETW. ──
static lv_obj_t* s_menu      = nullptr;  // page d'accueil réglages (grille 6 gros boutons)
static lv_obj_t* s_sec[6]    = {};       // conteneurs sections (cachés sauf l'ouvert)
static lv_obj_t* s_back_btn  = nullptr;  // cercle « retour » haut-droite (visible en section)
static int8_t    s_cur_sec   = -1;       // -1 = menu affiché, sinon index section ouverte
// (2026-06-27) FUSION CONFIG+DISPLAY → 1 section "CONFIG" scrollable (swipe down) ;
// libère la 6e tuile de la grille (réservée dev futur, laissée vide).
static const char* kSecName[6]={"CONFIG","TRAFFIC","PILOT","SYSTEM","ABOUT",""};
static lv_obj_t* s_set_acval = nullptr;  // bloc "Active Aircraft" REG/TYP/HEX (en-tête, menu seul)
static lv_obj_t* s_set_aclbl = nullptr;  //   label "Active Aircraft" associé
static void settingsShowMenu();          // (fwd) utilisé par switchPage pour reset à l'entrée
// (2026-06-30) Forward-decls EXPLICITES : après l'ajout du module audio/encodeur, le générateur
// de prototypes Arduino ne couvre plus ces fonctions sur le build T4-S3 (divergence du
// preprocessing selon la carte) → "was not declared". Decls manuelles = robuste (cf
// "auto-prototype fragility" dans CLAUDE.md). PilotEntry/lv_color_t sont déjà définis plus haut.
static int        _parsePilotJSON(const char* json);
void              hideUploadOverlay();
PilotEntry*       pilotFind(const char* code);
void              sendVfilt(int ft);
void              unitSaveHotspot(const char* ssid, const char* pass);
void              unitSaveMac(const char* mac);
static lv_color_t verColor(const char* s);
void              wifiStart();
void              wifiStop();
// Largeur de la zone Settings (coords écran) — pilote la géométrie board-aware des helpers.
#ifdef BOARD_T4S3
  #define SETW 600   // T4-S3 : rectangle plein écran 600 px
#else
  #define SETW 480   // T-RGB : rond 480 px (rows/contrôles centrés dans la bande médiane)
#endif

// ── SD card (AT-VIEW local) ───────────────────────────────────────────────────
static bool     g_sd_ok = false;

// ── AIP overlay data (PSRAM) ─────────────────────────────────────────────────
struct AipCtrEntry { uint16_t pt_start,n_pts; uint8_t type_id; };
#define AIP_MAX_CTR 2048   // toute l'EU = 1098 zones CTR (mesuré 2026-06-27) → 2048 = marge ; 1024 plafonnait
#define AIP_MAX_PTS 24000  // toute l'EU = 13813 points → 24000 suffit (pas besoin de relever)
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
static volatile bool g_aip_pull_req = false;   // (A1) armé à la connexion si le boîtier a CHR_AIP et qu'on n'a pas d'AIP → TaskAipPull
// (2026-06-26) WS241 sans SD : AIP EU EMBARQUÉE dans la flash écran (16 MB S3) → chargée
// direct au boot, instantanée + persistante + AUCUN transfert BLE (le pull curseur corrompait
// les données). Auto-gén : tools/gen_aip_header.py → aip_data.h. Les autres écrans gardent la SD.
#if defined(BOARD_WS241)
#define AIP_EMBEDDED 1
#include "aip_data.h"
#endif

// ── Audio I2S — DAC externe PCM5102A (jack 3.5mm → casque / audio panel) ─────
// Itération 1 (TEST sur WS-241) : valider DAC + câblage + broches via un bip.
// La cadence/hauteur pilotée par le moteur d'alerte (level/dist) viendra après.
// ⚠️ CÂBLAGE (header WS-241, broches réellement sorties) : VIN→3V3 · GND→GND ·
//    BCK→GPIO5 · LCK→GPIO6 · DIN→GPIO7 · SCK→GND (CRITIQUE, sinon silence).
// Core 3.x (pioarduino/IDF5) → nouvelle API i2s_std (include remonté en tête, cf zone includes).
#if defined(BOARD_WS241)
#define AUD_BCK   5
#define AUD_WS    6
#define AUD_DOUT  7
#define AUD_SR    16000
static i2s_chan_handle_t g_aud_tx = nullptr;
static bool g_aud_ok = false;
static void audioInit(){
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    if(i2s_new_channel(&cc,&g_aud_tx,NULL)!=ESP_OK) return;
    // Struct remplie champ par champ : les macros I2S_STD_*_DEFAULT_CONFIG ont un ordre de
    // designated-initializers que le C++ refuse (erreur ext_clk_freq_hz). memset puis champs requis.
    i2s_std_config_t sc; memset(&sc,0,sizeof(sc));
    sc.clk_cfg.sample_rate_hz = AUD_SR;
    sc.clk_cfg.clk_src        = I2S_CLK_SRC_DEFAULT;
    sc.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_256;
    sc.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    sc.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;
    sc.slot_cfg.slot_mode      = I2S_SLOT_MODE_STEREO;
    sc.slot_cfg.slot_mask      = I2S_STD_SLOT_BOTH;
    sc.slot_cfg.ws_width       = 16;
    sc.slot_cfg.ws_pol         = false;
    sc.slot_cfg.bit_shift      = true;          // format Philips/I2S (MSB décalé 1 BCLK)
    sc.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    sc.gpio_cfg.bclk = (gpio_num_t)AUD_BCK;
    sc.gpio_cfg.ws   = (gpio_num_t)AUD_WS;
    sc.gpio_cfg.dout = (gpio_num_t)AUD_DOUT;
    sc.gpio_cfg.din  = I2S_GPIO_UNUSED;
    if(i2s_channel_init_std_mode(g_aud_tx,&sc)!=ESP_OK){ g_aud_tx=nullptr; return; }
    i2s_channel_enable(g_aud_tx);
    g_aud_ok=true;
    Serial.println("[AUD] I2S PCM5102A pret (BCK5/WS6/DIN7, SCK->GND)");
}
// Bip BLOQUANT (ponctuel seulement : boot / bouton test). Sinus 16-bit stéréo.
static void audioBeep(int freq,int ms,int volPct){
    if(!g_aud_ok) return;
    const int N=256; static int16_t buf[N*2];
    float amp=32767.0f*(volPct/100.0f), ph=0.0f, dp=2.0f*3.14159265f*freq/AUD_SR;
    long total=(long)AUD_SR*ms/1000, done=0; size_t wr;
    while(done<total){
        int n=(total-done<N)?(int)(total-done):N;
        for(int i=0;i<n;i++){ int16_t s=(int16_t)(amp*sinf(ph)); ph+=dp; if(ph>6.2831853f)ph-=6.2831853f; buf[2*i]=s; buf[2*i+1]=s; }
        i2s_channel_write(g_aud_tx,buf,n*2*sizeof(int16_t),&wr,100/portTICK_PERIOD_MS);
        done+=n;
    }
}
static void audioTestChime(){ audioBeep(880,120,60); audioBeep(1175,120,60); audioBeep(1568,200,60); }  // 3 bips ascendants = câblage OK
#else
static inline void audioInit(){}
static inline void audioTestChime(){}
#endif

// ── Encodeur rotatif + poussoir (EC11) — nav pages + zoom EN VOL (WS-241) ────
// Bien plus sûr que le tactile en vol. Tourner = ZOOM (radar) / page suiv-préc (ailleurs) ;
// clic court = page suivante ; appui long = action sheet Start/Stop (radar).
// CÂBLAGE : A→GPIO38 · commun→GND · B→GPIO39 · poussoir→GPIO40 + GND. Pull-ups internes.
#if defined(BOARD_WS241)
void cfgSave(); void updSetPage(); static void radarActShow();   // fwd (définis plus bas)
#define ENC_A   38
#define ENC_B   39
#define ENC_SW  40
#define ENC_LONG_MS 600
// Décodeur quadrature plein-pas (Ben Buxton) : 1 évènement par cran, anti-rebond inhérent.
static const uint8_t ENC_TBL[7][4]={
    {0x0,0x2,0x4,0x0},{0x3,0x0,0x1,0x10},{0x3,0x2,0x0,0x0},{0x3,0x2,0x1,0x0},
    {0x6,0x0,0x4,0x0},{0x6,0x5,0x0,0x20},{0x6,0x5,0x4,0x0},
};
static volatile uint8_t g_enc_st=0;
static volatile int     g_enc_delta=0;
static portMUX_TYPE     g_enc_mux=portMUX_INITIALIZER_UNLOCKED;
static void IRAM_ATTR encISR(){
    uint8_t ab=(digitalRead(ENC_A)<<1)|digitalRead(ENC_B);
    g_enc_st=ENC_TBL[g_enc_st&0x0f][ab];
    uint8_t d=g_enc_st&0x30;
    if(d==0x10)g_enc_delta++; else if(d==0x20)g_enc_delta--;
}
static void radarZoom(int dir){   // +1 = zoom in (range--), -1 = zoom out (range++)
    int si=0;for(int i=0;i<7;i++)if(kScaleOpts[i]==g_cfg.scale_nm)si=i;
    if(dir>0)si=(si>0)?si-1:0; else si=(si<6)?si+1:6;
    g_cfg.scale_nm=kScaleOpts[si]; cfgSave(); updSetPage();
}
static void encStep(int dir){
    if(g_page==1) radarZoom(dir);                                  // radar → zoom
    else { g_navPage=(dir>0)?(uint8_t)((g_page+1)%NUM_PAGES)
                            :(uint8_t)((g_page+NUM_PAGES-1)%NUM_PAGES);
           g_navPending=true; }                                    // sinon → page suiv/préc
}
static void encoderInit(){
    pinMode(ENC_A,INPUT_PULLUP); pinMode(ENC_B,INPUT_PULLUP); pinMode(ENC_SW,INPUT_PULLUP);
    attachInterrupt(ENC_A,encISR,CHANGE); attachInterrupt(ENC_B,encISR,CHANGE);
    Serial.println("[ENC] rotary A38/B39/SW40 pret");
}
static void encoderPoll(){        // appelé depuis loop() (contexte LVGL, jamais d'UI en ISR)
    int d; portENTER_CRITICAL(&g_enc_mux); d=g_enc_delta; g_enc_delta=0; portEXIT_CRITICAL(&g_enc_mux);
    while(d>0){ encStep(+1); d--; }
    while(d<0){ encStep(-1); d++; }
    static bool down=false,longFired=false; static uint32_t t0=0;
    bool pressed=(digitalRead(ENC_SW)==LOW); uint32_t now=millis();
    if(pressed && !down){ down=true; longFired=false; t0=now; }
    else if(pressed && down && !longFired && now-t0>=ENC_LONG_MS){ longFired=true; if(g_page==1) radarActShow(); }
    else if(!pressed && down){ down=false; if(!longFired && now-t0>=30){ g_navPage=(uint8_t)((g_page+1)%NUM_PAGES); g_navPending=true; } }
}
#else
static inline void encoderInit(){}
static inline void encoderPoll(){}
#endif

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
    lv_obj_set_pos(p,UI_OX,UI_OY);lv_obj_set_style_bg_color(p,TBG(),0);
    lv_obj_set_style_border_width(p,0,0);lv_obj_set_style_pad_all(p,0,0);
    lv_obj_clear_flag(p,LV_OBJ_FLAG_SCROLLABLE);return p;}
// Tab pill: 52×32 invisible hit-zone, icon floats freely. Returns inner label ref.
lv_obj_t* mkTabPill(lv_obj_t*p,const char*t,int x,int y){
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,PILL_W,PILL_H);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,t);
    lv_obj_set_style_text_color(l,PILL_IC_OFF(),0);lv_obj_set_style_text_font(l,&PILL_FONT,0);
    lv_obj_center(l);return l;}
// LTE pill: 4 drawn signal bars (bottom-aligned), returns dummy label ref for parent lookups
lv_obj_t* mkLTEPill(lv_obj_t*p,int x,int y){
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,PILL_W,PILL_H);lv_obj_set_pos(b,x,y);
    lv_obj_set_style_bg_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(b,0,0);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    // 4 signal bars centered in 52×32 — bar widths 3px, gap 2px, bottom at y=27
    static const int8_t bh[4]={5,8,12,16};
    for(int i=0;i<4;i++){
        r_hdr_lte_b[i]=lv_obj_create(b);lv_obj_set_size(r_hdr_lte_b[i],3,bh[i]);
        lv_obj_set_pos(r_hdr_lte_b[i],(PILL_W-18)/2+i*5,(PILL_H-5)-bh[i]);   // centré qq soit la taille pill
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
    lv_obj_t*b=lv_obj_create(p);lv_obj_set_size(b,PILL_W,PILL_H);lv_obj_set_pos(b,x,y);
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
#ifdef BOARD_T4S3
    const int DOT=28, LX=38, LY=4; const lv_font_t* RF=&lv_font_montserrat_18;   // (v14) checks +~25% (T4)
#else
    const int DOT=22, LX=30, LY=4; const lv_font_t* RF=&lv_font_montserrat_14;
#endif
    lv_obj_t*dot=lv_obj_create(p);
    lv_obj_set_size(dot,DOT,DOT);lv_obj_set_pos(dot,x,y);
    lv_obj_set_style_radius(dot,DOT/2,0);
    lv_obj_set_style_bg_color(dot,C_BRAND,0);lv_obj_set_style_bg_opa(dot,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(dot,0,0);
    lv_obj_set_style_shadow_opa(dot,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(dot,0,0);
    lv_obj_clear_flag(dot,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t*ico=lv_label_create(dot);lv_label_set_text(ico,"");
    lv_obj_set_style_text_color(ico,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(ico,RF,0);
    lv_obj_center(ico);
    r_chk_dot[idx]=dot;r_chk_ico[idx]=ico;
    r_chk_lbl[idx]=mkLblP(p,txt,TGREY(),RF,x+LX,y+LY);
}
void updCheckRow(int idx,const char*txt,bool ok){
    if(!r_chk_dot[idx])return;
    // Latch : une fois actif, le V reste affiche jusqu'au reset BLE
    if(ok) g_chk_latched[idx]=true;
    bool show = g_chk_latched[idx];
    lv_label_set_text(r_chk_ico[idx],show?LV_SYMBOL_OK:"");
    // Texte actif = couleur premier-plan du thème (noir en clair / blanc en sombre), gris sinon
    lv_obj_set_style_text_color(r_chk_lbl[idx],show?TFG():TGREY(),0);
    if(txt)lv_label_set_text(r_chk_lbl[idx],txt);
}

// ── Parsers ───────────────────────────────────────────────────────────────────
void parseStatus(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_status.mode=d["mode"]|0;g_status.gps_sat=d["gps_sat"]|0;g_status.csq=d["csq"]|-1;
    g_status.frames=d["frames"]|0;g_status.alt=d["alt"]|0;g_status.spd=d["spd"]|0;
    g_status.hdg=d["hdg"]|0;g_status.bat=d["bat"]|-1;g_status.lat=d["lat"]|0.0f;g_status.lon=d["lon"]|0.0f;
    g_status.gps_fix=d["gps_fix"]|false;g_status.sd_ok=d["sd_ok"]|false;
    g_status.ss_ok=d["ss"]|false;
    g_status.ss_mode=d["ssm"]|0;   // (v20) 0=vol 1=sol/idle (cadence éco)
    // Santé signal (2026-06-06) : mémorise l'instant de PERTE (ss passe false,
    // = >10 s sans échange UDP côté boîtier). Sert au vieillissement du trafic :
    // gris dès la perte, effacé à perte+20 s (= 30 s réels), cf updateRadarDR.
    if(g_status.ss_ok) g_ss_lost_ms=0;
    else if(g_ss_lost_ms==0) g_ss_lost_ms=millis();
    g_status.flarm_ok=d["flarm"]|false;g_status.adsb_ok=d["adsb"]|false;
    g_status.charging=d["chg"]|false;
    g_status.flt_phase=d["flt_ph"]|0;g_status.upload_pct=d["up_pct"]|0;g_status.flt_rdy=d["flt_rdy"]|0;g_status.flt_st=d["flt_st"]|0;
    g_status.aip_xfer_len=d["axl"]|0u;   // (A1) longueur du flux AIP prêt (handshake pull)
    g_status.wst=d["wst"]|0; strlcpy(g_status.wip,d["wip"]|"",sizeof(g_status.wip));
    strlcpy(g_status.wssid,d["wss"]|"",sizeof(g_status.wssid));   // SSID hotspot enregistré (boîtier)
    g_status.ota=d["ota"]|0; g_status.opct=d["opct"]|0;
    g_status.fwv=d["fwv"]|0; strlcpy(g_status.fwd,d["fwd"]|"",sizeof(g_status.fwd)); g_status.oav=d["oav"]|0;
    strlcpy(g_status.fws,d["fws"]|"",sizeof(g_status.fws));   // version lisible ATC "1.2.36-dev"
    strlcpy(g_status.box,d["box"]|"",sizeof(g_status.box));   // box-id → SSID portail (WiFi Setup)
    // Identité aéronef poussée par l'ATC (saisie au portail web) → l'écran n'a plus de page
    // AIRCRAFT, il mirrore ces valeurs (bloc "Active Aircraft" + radar own-ship).
    // Données seules (pas de LVGL ici : callback BLE) → l'affichage est rafraîchi dans
    // updateAllPages() (main loop) via g_dataUpdated : accueil (p0UpdateAcId) + Active Aircraft.
    { const char* r=d["reg"]|""; const char* t=d["typ"]|""; const char* h=d["hex"]|"";
      if(r[0])strlcpy(g_ac_reg,r,sizeof(g_ac_reg));
      if(t[0])strlcpy(g_ac_type,t,sizeof(g_ac_type));
      if(h[0])strlcpy(g_ac_hex,h,sizeof(g_ac_hex)); }
    g_status.valid=true;g_dataUpdated=true;}
void parseFlight(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    g_flight.gforce_z=d["gf"]|1.0f;g_flight.co_ppm=d["co"]|0;
    g_flight.rpm=d["rpm"]|0;g_flight.phase=d["phase"]|0;
    g_flight.valid=true;g_dataUpdated=true;}
void parseTraffic(const char*j){JsonDocument d;if(deserializeJson(d,j))return;
    // Continuité DR + lissage d'un paquet 1 Hz à l'autre (anti surplace/bond) : l'AT-CORE
    // renvoie le MÊME beacon SafeSky à 1 Hz (~5 s entre 2 vrais fixes). On snapshote l'état
    // AFFICHÉ courant (base_ms + position lissée disp_ex/ey) AVANT d'écraser, et on le ré-
    // injecte par callsign : la DR ne se ré-ancre (base_ms=now) QUE si le fix a vraiment
    // changé (dist/bear/hdg/spd différents, comparaison exacte), et la position lissée est
    // TOUJOURS reprise → mouvement continu (matché sur cs, robuste au tri distance).
    static TrafficEntry prev[MAX_TRF]; int prevN=g_traffic.count;
    for(int k=0;k<prevN;k++) prev[k]=g_traffic.t[k];
    uint32_t now=millis();
    int n=min((int)(d["count"]|0),MAX_TRF);
    for(int i=0;i<n;i++){
        TrafficEntry e={};
        strlcpy(e.cs,d["t"][i]["cs"]|"???",9);
        e.dist_m=d["t"][i]["d"]|0;e.alt_m=d["t"][i]["a"]|0;
        e.bear_deg=d["t"][i]["b"]|0;e.hdg_deg=d["t"][i]["c"]|0;
        e.spd_kt=d["t"][i]["s"]|100;e.visible=d["t"][i]["v"]|true;e.type=d["t"][i]["tp"]|0;
        e.base_ms=now;
        for(int k=0;k<prevN;k++) if(strcmp(prev[k].cs,e.cs)==0){
            e.disp_ex=prev[k].disp_ex;e.disp_ey=prev[k].disp_ey;e.disp_init=prev[k].disp_init; // position lissée reprise
            if(prev[k].dist_m==e.dist_m&&prev[k].bear_deg==e.bear_deg&&
               prev[k].hdg_deg==e.hdg_deg&&prev[k].spd_kt==e.spd_kt)
                e.base_ms=prev[k].base_ms;     // même fix → la DR continue (pas de re-snap)
            break;}
        g_traffic.t[i]=e;}
    g_traffic.count=n;
    g_traffic.valid=true;g_traffic.recv_ms=now;g_dataUpdated=true;}
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
    std::string m=bleStr(d.getManufacturerData());
    if(m.size()<3)return false;
    return (uint8_t)m[0]==0xFF && (uint8_t)m[1]==0xFF && (uint8_t)m[2]==0x01;}

// Insère/actualise un candidat pairing dans g_pcand (appelé depuis le cb scan).
void pcandUpsert(BLEAdvertisedDevice& d){
    if(!g_pcand_mx)return;
    std::string mac=bleStr(d.getAddress().toString());
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
    BLEDevice::getScan()->stop();g_scanning=false;   // scan stoppé → libère le flag (sinon reconnexion future bloquée)
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
        g_scanning=false;   // permet au scan de reconnexion de démarrer
        g_dataUpdated=true;g_doReconnect=true;Serial.println("[BLE] Disconnected");}};
class ATCAdv:public BLEAdvertisedDeviceCallbacks{
    void onResult(BLEAdvertisedDevice dev)override{
        String nm=dev.getName().c_str();
        if(!nm.startsWith("ATC-"))return;   // nom AT-CORE = ATC-<MAC> (ex ATC-CE276D)
        std::string mac=bleStr(dev.getAddress().toString());
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
    // (v8) Client BLE FRAIS à chaque (re)connexion. Réutiliser le même BLEClient Bluedroid
    // garde un cache de services/handles lié à l'ANCIEN conn_id → après une reconnexion les
    // notifications repassent mais les ÉCRITURES (sendCtl : start/stop/upload, acPushBLE…)
    // partent sur le contexte mort et sont silencieusement perdues (bug write-après-reco
    // découvert au bench). On détruit l'ancien client + on null les caractéristiques (la
    // fenêtre de connexion voit g_chrCtl=null → sendCtl no-op sûr), puis découverte propre.
    if(g_client){ delete g_client; g_client=nullptr; }   // destructeur libère le cache services ; gattc_if déjà désenregistré à la déconnexion
    g_svc=nullptr;
    g_chrS=g_chrF=g_chrT=g_chrA=g_chrD=g_chrW=g_chrP=g_chrCfg=g_chrCtl=g_chrFl=g_chrWc=g_chrImu=g_chrAip=nullptr;
    static ATCCB s_cb;   // instance unique (l'ancien new ATCCB() fuyait à chaque reconnexion)
    g_client=BLEDevice::createClient(); g_client->setClientCallbacks(&s_cb);
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
    g_chrWc =g_svc->getCharacteristic(BLE_CHR_WIFICRED);
    g_chrImu=g_svc->getCharacteristic(BLE_CHR_IMU);   // mouchard G/assiette (peut être absent si boîtier < CHR_IMU)
    g_chrAip=g_svc->getCharacteristic(BLE_CHR_AIP);   // transfert AIP (absent si boîtier non-S3 / sans AIP)
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
    // (2026-06) NE PLUS pousser l'identité aéronef à la connexion. L'identité est
    // désormais saisie au PORTAIL WEB du boîtier (source de vérité), et le boîtier la
    // pousse à l'écran via STATUS (reg/typ/hex). L'ancien acPushBLE() ici ÉCRASAIT la
    // valeur du portail par la valeur (souvent périmée) du NVS écran à chaque connexion
    // → « REG/TYP/HEX pas mis à jour après le portail ». Le push écran→boîtier est retiré.
    // (acPushBLE conservée mais plus appelée sur ce chemin ; l'écran n'édite plus l'identité.)
    // (brique 0) Hériter les creds WiFi club du boîtier (CHR_WIFICRED, READ) → zéro saisie
    // côté écran. On lit à la connexion ; si différents du NVS écran, on sauve (g_hs_ssid/pass).
    // → l'OTA "Update ATV" et le wifitest marchent sans re-taper le hotspot.
    if(g_chrWc){
        std::string wc=bleStr(g_chrWc->readValue());
        if(wc.size()>5){
            JsonDocument d;
            if(!deserializeJson(d,wc.c_str())){
                const char* s=d["s"]|""; const char* p=d["p"]|"";
                if(s[0] && (strcmp(s,g_hs_ssid)!=0 || strcmp(p,g_hs_pass)!=0)){
                    unitSaveHotspot(s,p);
                    Serial.printf("[WiFi] creds heritees du boitier: %s\n",s);
                }
            }
        }
    }
    sendVfilt(g_cfg.vfilt_ft);   // (CONFIG) sync le VF= SafeSky du boîtier sur la valeur écran
    // (A1) AIP : si le boîtier expose CHR_AIP (S3 avec AIP embarquée) ET qu'on n'a pas déjà
    // d'AIP (pas de SD chargée → WS241), on arme le pull (TaskAipPull, hors boucle → radar ok).
    if(g_chrAip && !g_aip_loaded) g_aip_pull_req=true;
    return true;}
// (juin 2026) SCAN BLE ASYNCHRONE — cause racine du tactile « lent/fastidieux » :
// l'ancien `s->start(5,false)` BLOQUAIT la boucle 5 s (mesuré loopMax=5007ms) → tous
// les taps pendant le scan étaient perdus (« j'appuie plusieurs fois, parfois ça marche »).
// Variante asynchrone `start(5, cb, false)` : retourne immédiatement, le callback ATCAdv
// arme la connexion quand le boîtier apparaît, scanDoneCb libère le flag à la fin. De plus :
// instance ATCAdv UNIQUE (l'ancien `new ATCAdv()` à chaque scan fuyait de la mémoire).
static void scanDoneCb(BLEScanResults){ g_scanning=false; }
void startScan(){
    if(g_scanning||g_connected||g_doConnect) return;   // pas de scan inutile/concurrent
    static ATCAdv s_atcAdv;                              // singleton (plus de fuite)
    BLEScan*s=BLEDevice::getScan();
    s->setAdvertisedDeviceCallbacks(&s_atcAdv);
    s->setActiveScan(true);
    g_scanning=true;
    if(!s->start(5,scanDoneCb,false)) g_scanning=false;  // ASYNCHRONE → la boucle n'est plus figée
}

// ── Navigation & swipe ────────────────────────────────────────────────────────
void switchPage(uint8_t np){
    if(g_inDebug){lv_obj_add_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);g_inDebug=false;}
    lv_obj_add_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);
    g_page=np;lv_obj_clear_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);
#ifdef BOARD_T4S3
    // (juin 2026) Settings T4 = menu en grille : à chaque entrée sur la page, on
    // repart de la grille (pas de la dernière section ouverte).
    if(np==2) settingsShowMenu();
#endif
}

static lv_coord_t g_swipe_sx=-1, g_swipe_lx=0, g_swipe_sy=0, g_swipe_ly=0;
// Vrai pendant le drag du slider brightness — bloque le swipe horizontal
// qui sinon déclencherait une navigation de page parasite.
static bool g_bright_drag=false;
// Titre dynamique « SETTINGS / <section> » — T4-S3 seulement (s_set_title non nul).
static void setUpdTitle(){
    if(!s_set_title)return;
    char t[28]; snprintf(t,sizeof(t),"SETTINGS / %s",kSetTab[s_pg_idx<S_NPG?s_pg_idx:0]);
    lv_label_set_text(s_set_title,t);
    if(s_set_uline){                       // ligne soulignée recalée sur la largeur du texte
        lv_obj_update_layout(s_set_title);
        lv_obj_set_width(s_set_uline,lv_obj_get_width(s_set_title));
    }
}

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
            // (juin 2026) FIX swipe radar : on prend le point de RELÂCHEMENT réel (pt), pas le
            // dernier PRESSING (g_swipe_lx) — sur le radar le rendu lourd espace les PRESSING,
            // g_swipe_lx restait périmé → delta sous-estimé → swipe raté aléatoirement.
            int dx=(int)pt.x-(int)g_swipe_sx;
            int dy=(int)pt.y-(int)g_swipe_sy;
            if(g_inDebug){
                if(abs(dx)>40){lv_obj_add_flag(g_dbgPage,LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(g_pages[g_page],LV_OBJ_FLAG_HIDDEN);g_inDebug=false;}
            }else if(abs(dy)>abs(dx) && abs(dy)>60 && g_page==2){
                // Swipe vertical sur Settings → bascule sous-page (haut=suivante, bas=précédente).
                // (T4 : s_pg[] non créés → no-op ; la nav se fait par le menu en grille.)
                int np = (dy<0) ? (s_pg_idx+1) : ((int)s_pg_idx-1);
                if(np<0)np=0; if(np>S_NPG-1)np=S_NPG-1;   // clamp (pas de wrap)
                if((uint8_t)np!=s_pg_idx && s_pg[s_pg_idx] && s_pg[np]){
                    lv_obj_add_flag(s_pg[s_pg_idx],LV_OBJ_FLAG_HIDDEN);
                    s_pg_idx=(uint8_t)np;
                    lv_obj_clear_flag(s_pg[s_pg_idx],LV_OBJ_FLAG_HIDDEN);
                    setUpdTitle();}
            }else{
                // (juin 2026) seuil abaissé 60→45 (swipe plus facile)
                if(dx>45){g_navPage=(g_page==0)?NUM_PAGES-1:g_page-1;g_navPending=true;}
                else if(dx<-45){g_navPage=(g_page+1)%NUM_PAGES;g_navPending=true;}}}
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
    g_cfg.trf_src   =0;   // SOURCE retiré de l'UI : trafic toujours SafeSky (SSKY)
    g_cfg.dist_nm   =g_prefs.getBool("dist_nm",true);
    g_cfg.alt_ft    =g_prefs.getBool("alt_ft",true);
    g_cfg.dark      =g_prefs.getBool("dark",false);   // (juin 2026) défaut LIGHT (aligné maquette)
    g_cfg.show_grnd =g_prefs.getBool("show_grnd",true);
    g_cfg.wifi_en   =g_prefs.getBool("wifi_en",false);
    g_cfg.aip_en    =g_prefs.getBool("aip_en",true);
    g_cfg.ad_heli   =g_prefs.getBool("ad_heli",false);
    g_cfg.vfilt_ft  =g_prefs.getShort("vfilt",2000);
    g_cfg.icon_sz   =g_prefs.getUChar("icon_sz",2);
    g_cfg.circuit_ovr=g_prefs.getUChar("circ_ovr",0);   // 0=AUTO 1=CIRCUIT 2=ROUTE
    g_cfg.spd_kt    =g_prefs.getBool("spd_kt",true);    // toujours kt (toggle retiré) — défaut conservé
    g_cfg.dist_nm   =true;                              // (CONFIG) distance radar toujours NM (toggle retiré)
    g_cfg.show_cs   =g_prefs.getBool("show_cs",true);   // decluttering : afficher le callsign
    g_cfg.show_vdiff=g_prefs.getBool("show_vdiff",true);// decluttering : afficher la diff verticale
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
    g_prefs.putUChar("circ_ovr",g_cfg.circuit_ovr);
    g_prefs.putBool("spd_kt",g_cfg.spd_kt);
    g_prefs.putBool("show_cs",g_cfg.show_cs);
    g_prefs.putBool("show_vdiff",g_cfg.show_vdiff);
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

// (CONFIG) Pousse le V-FILTER (ft) au boîtier → VF= SafeSky IN (↓ trafic hors altitude).
// Appelé au changement de V-FILTER et à la connexion (sync ATC sur la valeur écran).
void sendVfilt(int ft){
    if(!g_connected||!g_chrCtl||!g_chrCtl->canWrite())return;
    char p[40];snprintf(p,sizeof(p),"{\"cmd\":\"vfilt\",\"ft\":%d}",ft);
    g_chrCtl->writeValue((uint8_t*)p,strlen(p),false);
    Serial.printf("[BLE] CTRL vfilt ft=%d\n",ft);}

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
    // (2026-06) NE PLUS pousser l'identité au bind : l'écran n'est plus source de l'identité
    // (saisie au portail boîtier). Avant, acPushBLE() ici ÉCRASAIT l'identité du boîtier par
    // celle (vide/périmée) de l'écran à chaque appairage → c'était le clobber principal.
}

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
    lv_obj_set_size(g_pair_ov,480,480);lv_obj_set_pos(g_pair_ov,UI_OX,UI_OY);
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
static void cbRadarLongPress(lv_event_t*e);   // (juin 2026) appui long radar → action sheet Start/Stop
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
        lv_obj_set_style_text_color(g_p0_acid,TFG(),0);  // (juin 2026) suit le thème (accueil sombre OK)
    }else if(g_connected && g_status.valid){
        // Connecté + STATUS reçue mais le boîtier n'a vraiment pas d'identité → warning légitime.
        lv_label_set_text(g_p0_acid,LV_SYMBOL_WARNING " NO AIRCRAFT - SET VIA WIFI SETUP");
        lv_obj_set_style_text_color(g_p0_acid,lv_color_hex(0xD32F2F),0);
    }else{
        // PAS encore connecté/STATUS → on ne SAIT pas si l'aéronef est défini (le boîtier
        // poussera reg via STATUS). Ne PAS afficher le warning rouge envahissant et faux → neutre.
        lv_label_set_text(g_p0_acid,"");
    }
}

void buildStatusPage(){
    lv_obj_t*p=g_pages[0];
    // (juin 2026) Thème GLOBAL : la page accueil suit LIGHT/DARK comme le radar
    // (avant : fond blanc forcé). TBG()=blanc en clair, noir en sombre.
    lv_obj_set_style_bg_color(p,TBG(),0);
    lv_obj_set_style_bg_opa(p,LV_OPA_COVER,0);
    // (v12) Géométrie par carte. T4-S3 : logos PLUS GROS (place dispo sur le 600×450) +
    // bloc versions du bas (batterie/ATV/ATC) ré-espacé pour ne PAS se chevaucher ni
    // coller au bord bas (overlay à UI_OY=-15 → y écran = y local - 15). T-RGB inchangé.
#ifdef BOARD_T4S3
    // WS-241 (dalle 600×480, UI_OY=0) : on descend le cluster bas de +15 (pas +30) pour garder
    // ~27 px de marge sous l'ATC, comme le T4-S3 (qui a son UI_OY=-15). +30 collait (~12 px).
#ifdef PANEL_WS241
    const int VBOT=15;
#else
    const int VBOT=0;
#endif
    const int vwZoom=400, vwY=44, atZoom=480, atY=82, acidY=176,
              batY=394+VBOT, verY=422+VBOT, atcY=422+VBOT;
    const lv_font_t *FID=&lv_font_montserrat_24, *FVER=&lv_font_montserrat_16;   // (juin 2026) identité PLUS GRANDE
    const int chkY0=226, chkDY=33;                                               // (juin 2026) plus d'air sous l'identité + 5 checks (SafeSky)
#else
    const int vwZoom=320, vwY=70, atZoom=384, atY=108, acidY=190, batY=414, verY=432, atcY=450;
    const lv_font_t *FID=&lv_font_montserrat_14, *FVER=&lv_font_montserrat_12;
    const int chkY0=218, chkDY=30;
#endif

    // ── Logos bicolores (A bleu + reste noir) — zoom LVGL pour respecter proportions maquette
    lv_obj_t*lVw=lv_img_create(p);
    lv_img_set_src(lVw,&img_logo_atview);          // 110×22 source
    lv_img_set_zoom(lVw,vwZoom);
    lv_obj_align(lVw,LV_ALIGN_TOP_MID,0,vwY);
    // (juin 2026) Thème sombre : logos bicolores (bleu+noir, pensés fond blanc) → on les
    // recolore en TFG (blanc) en DARK pour rester lisibles ; en LIGHT on garde l'original.
    if(g_dark_theme){lv_obj_set_style_img_recolor(lVw,TFG(),0);lv_obj_set_style_img_recolor_opa(lVw,LV_OPA_COVER,0);}
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
    lv_img_set_zoom(lAt,atZoom);                    // T4 ×1.875 (~450×94) — plus gros
    lv_obj_align(lAt,LV_ALIGN_TOP_MID,0,atY);
    if(g_dark_theme){lv_obj_set_style_img_recolor(lAt,TFG(),0);lv_obj_set_style_img_recolor_opa(lAt,LV_OPA_COVER,0);}

    // ── Identité appareil transmise à SafeSky (sous le logo, centrée).
    g_p0_acid=mkLbl(p,"",TFG(),FID,LV_ALIGN_TOP_MID,0,acidY);
    p0UpdateAcId();

    // ── 6 check rows (cercle bleu + label) — décalées à gauche, plus d'air entre lignes
    const int X = 105;  // colonne cercle (decalee a gauche)
    const int Y0= chkY0;  // 1ere ligne (board-conditionnel : plus bas + plus espacé sur T4)
    const int DY= chkDY;  // espacement vertical (board-conditionnel)
    mkCheckRow(p,CHK_CORE,X,Y0+0*DY,"AT-CORE");
    mkCheckRow(p,CHK_BT,  X,Y0+1*DY,"Bluetooth");
    mkCheckRow(p,CHK_GPS, X,Y0+2*DY,"GPS");
    mkCheckRow(p,CHK_LTE, X,Y0+3*DY,"LTE");
    mkCheckRow(p,CHK_SKY, X,Y0+4*DY,"SafeSky");   // (juin 2026) statut SafeSky sous LTE
    // ADS-B / ADS-L et OGN / FLARM retirés (non poussés pour l'instant).

    // ── Versions ATV (gauche) / ATC (droite), aux bords, colorées par canal.
    // Batterie AT-CORE retirée (non nécessaire). Date de build retirée ici (trop long → se
    // chevauchait au milieu ; la date reste sur ABOUT). r_p0_bat reste nullptr (update no-op).
    mkLbl(p,"ATV " VIEW_VSTR,verColor(VIEW_VSTR),FVER,LV_ALIGN_TOP_LEFT,40,verY);
    r_p0_atc=mkLbl(p,"ATC --",TGREY(),FVER,LV_ALIGN_TOP_RIGHT,-40,atcY);  // rempli live (g_status.fws)
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
    lv_obj_set_size(g_welcome_ov,480,480); lv_obj_set_pos(g_welcome_ov,UI_OX,UI_OY);
    lv_obj_set_style_bg_color(g_welcome_ov,lv_color_hex(0xffffff),0);
    lv_obj_set_style_bg_opa(g_welcome_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_welcome_ov,0,0);
    lv_obj_set_style_radius(g_welcome_ov,0,0);
    lv_obj_clear_flag(g_welcome_ov,LV_OBJ_FLAG_SCROLLABLE);

    // Logo A en haut — lv_obj_align garantit le centrage LVGL
    lv_obj_t*lA=lv_img_create(g_welcome_ov);
    lv_img_set_src(lA,&img_logo_a);
    lv_obj_align(lA,LV_ALIGN_TOP_MID,0,24);   // (clip) marge haut sur T4 (overlay à -15)

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
    mkLbl(g_welcome_ov,VIEW_VER_STR,TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,438);

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
    lv_obj_set_size(g_auth_ov,480,480);lv_obj_set_pos(g_auth_ov,UI_OX,UI_OY);
    lv_obj_set_style_bg_color(g_auth_ov,lv_color_hex(0xffffff),0);
    lv_obj_set_style_bg_opa(g_auth_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_auth_ov,0,0);lv_obj_set_style_radius(g_auth_ov,0,0);
    lv_obj_set_style_shadow_opa(g_auth_ov,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(g_auth_ov,0,0);
    lv_obj_clear_flag(g_auth_ov,LV_OBJ_FLAG_SCROLLABLE);

    // Logo A en haut centre — lv_obj_align garantit le centrage LVGL
    lv_obj_t*lA=lv_img_create(g_auth_ov);
    lv_img_set_src(lA,&img_logo_a);
    lv_obj_align(lA,LV_ALIGN_TOP_MID,0,20);   // (clip) était 12 → y écran -3 sur T4 (overlay à -15)

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
    mkLbl(g_auth_ov,VIEW_VER_STR,TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,438);

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
static uint32_t  g_up_p12_t0   = 0;   // début phase 1/2 (auto-dismiss si pas d'upload auto)
// Init à true → un ph=4 résiduel reçu au boot (état post-upload AT-CORE figé) est ignoré.
// Reset à false dès que ph repasse à 0 (nouveau vol), pour que le prochain cycle 1→2→3→4 affiche bien l'overlay.
static bool      g_up_acked    = true;
// Tap sur l'overlay = fermeture manuelle. Mémorisé pour ne pas réapparaître pendant les
// retries (5→3→5...) ; ré-armé quand ph revient à 0 (nouveau vol) ou 4 (succès à montrer).
static bool      g_up_dismissed= false;

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

    g_up_title=mkLblP(g_up_ov,"FLIGHT UPLOAD",C_AMBER,&lv_font_montserrat_20,118,18);
    g_up_status=mkLblP(g_up_ov,"Flight ended - closing CSV...",lv_color_hex(0xffffff),&lv_font_montserrat_16,40,72);
    lv_obj_set_width(g_up_status,320);
    lv_obj_set_style_text_align(g_up_status,LV_TEXT_ALIGN_CENTER,0);

    g_up_bar=lv_bar_create(g_up_ov);
    lv_obj_set_size(g_up_bar,320,18);lv_obj_set_pos(g_up_bar,40,130);
    lv_bar_set_range(g_up_bar,0,100);lv_bar_set_value(g_up_bar,0,LV_ANIM_OFF);
    lv_obj_set_style_bg_color(g_up_bar,lv_color_hex(0x1f2937),0);
    lv_obj_set_style_bg_color(g_up_bar,C_AMBER,LV_PART_INDICATOR);

    g_up_pct_lbl=mkLblP(g_up_ov,"0%%",lv_color_hex(0xffffff),&lv_font_montserrat_16,178,160);
    mkLblP(g_up_ov,"tap to close",TGREY(),&lv_font_montserrat_12,160,228);

    // Tap n'importe où sur l'overlay → fermeture manuelle (indispensable en phase 5
    // UPLOAD_FAIL qui persiste tant que l'AT-CORE retente l'upload)
    lv_obj_add_event_cb(g_up_ov,[](lv_event_t*e){
        if(lv_event_get_code(e)==LV_EVENT_CLICKED){hideUploadOverlay();g_up_dismissed=true;}
    },LV_EVENT_CLICKED,NULL);
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

    // (2026-06-05) Anti "CSV closed" fantôme : flt_ph 1/2 est un état PERSISTANT
    // côté boîtier (rediffusé tant que l'upload manuel n'a pas eu lieu) → à chaque
    // (re)connexion/reset l'écran le redécouvrait et affichait l'overlay central.
    // On ne montre les phases 1/2 que si la TRANSITION 0→1/2 a été vue en session.
    static uint8_t s_ph_prev  = 0xFF;
    static bool    s_p12_live = false;
    if(g_status.valid){
        if(ph==0) s_p12_live=false;
        else if((ph==1||ph==2) && s_ph_prev==0) s_p12_live=true;
        s_ph_prev = ph;
    } else { s_ph_prev=0xFF; s_p12_live=false; }

    // Phase 0 (FLYING) ou status invalide → hide overlay + reset ack
    if(!g_status.valid || ph == 0){
        if(g_up_ov) hideUploadOverlay();
        g_up_acked = false;
        g_up_dismissed = false;
        return;
    }
    if((ph==1||ph==2) && !s_p12_live){       // état hérité d'avant la connexion → silencieux
        if(g_up_ov) hideUploadOverlay();
        return;
    }

    // Fermé manuellement (tap) → reste caché pendant les retries ; le succès (ph=4)
    // ré-affiche quand même (bonne nouvelle à montrer)
    if(g_up_dismissed && ph != 4){
        if(g_up_ov) hideUploadOverlay();
        return;
    }

    // Phases 1/2 (ended/closed) : en config hotspot l'upload ne démarre PAS tout seul
    // (LTE cloud off, transfert WiFi manuel via Flights). Feedback bref puis on libère
    // l'écran — sinon l'overlay reste figé sur "CSV closed - waiting upload".
    if(ph==1||ph==2){
        if(g_up_p12_t0==0) g_up_p12_t0=millis();
        if(millis()-g_up_p12_t0>5000){ if(g_up_ov) hideUploadOverlay(); return; }
    } else g_up_p12_t0=0;

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
        case 1: msg="Flight ended - closing CSV"; break;
        case 2: msg="CSV closed - waiting upload"; break;
        case 3: msg="Uploading to Firebase..."; break;
        case 4: msg="Transfer OK";
                if(g_up_done_ms==0) g_up_done_ms=millis(); break;
        case 5: msg="Upload failed - tap to close";
                g_up_done_ms=0; break;
    }
    if(g_up_status) lv_label_set_text(g_up_status,msg);
    int barv = (ph<3)?0:pct;   // phases 1/2 = pas d'upload en cours → pas de 100% trompeur
    if(g_up_bar)    lv_bar_set_value(g_up_bar,barv,LV_ANIM_OFF);
    if(g_up_pct_lbl){
        char p[8]; snprintf(p,sizeof(p),"%d%%",barv);
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
    lv_obj_set_size(g_ac_ov,480,480);lv_obj_set_pos(g_ac_ov,UI_OX,UI_OY);
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
    // Nom AT-VIEW = ATV-<MAC> (auto, plus de nom custom) — répertorié par MAC côté flotte.
    { uint8_t mac[6]={}; esp_read_mac(mac,ESP_MAC_BT);
      snprintf(g_unit_name,sizeof(g_unit_name),"ATV-%02X%02X%02X",mac[3],mac[4],mac[5]); }
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

// ── AIP depuis RAM (A1) : transfert BLE boîtier→écran ─────────────────────────
// Parsers buffer (mêmes formats que les versions File : CTR "CTR\0", aérodromes "ADP2").
// memcpy pour les int32 (buffer byte-packé → potentiellement non aligné).
static void _aipLoadCtrBuf(const uint8_t* p,uint32_t len){
    if(len<6||memcmp(p,"CTR\0",4)!=0)return;
    uint32_t o=4; uint16_t cnt; memcpy(&cnt,p+o,2); o+=2;
    for(int i=0;i<cnt&&g_aip_ctr_cnt<AIP_MAX_CTR&&o<len;i++){
        uint8_t nlen=p[o++]; o+=nlen;                 // skip nom
        uint8_t tid=p[o++];
        uint16_t npts; memcpy(&npts,p+o,2); o+=2;
        if(g_aip_pts_cnt+npts>AIP_MAX_PTS){o+=(uint32_t)npts*8;continue;}
        g_aip_ctr[g_aip_ctr_cnt]={g_aip_pts_cnt,npts,tid}; g_aip_ctr_cnt++;
        for(int j=0;j<npts;j++){
            int32_t la,lo; memcpy(&la,p+o,4);o+=4; memcpy(&lo,p+o,4);o+=4;
            g_aip_lat[g_aip_pts_cnt]=la; g_aip_lon[g_aip_pts_cnt]=lo; g_aip_pts_cnt++;}}}
static void _aipLoadAdBuf(const uint8_t* p,uint32_t len){
    if(len<6||memcmp(p,"ADP2",4)!=0)return;
    uint32_t o=4; uint16_t cnt; memcpy(&cnt,p+o,2); o+=2;
    for(int i=0;i<cnt&&g_aip_ad_cnt<AIP_MAX_AD&&o+13<=len;i++){
        memcpy(g_aip_ads[g_aip_ad_cnt].icao,p+o,4); g_aip_ads[g_aip_ad_cnt].icao[4]=0; o+=4;
        int32_t la,lo; memcpy(&la,p+o,4);o+=4; memcpy(&lo,p+o,4);o+=4;
        uint8_t tid=p[o++];
        g_aip_ads[g_aip_ad_cnt].lat_e6=la; g_aip_ads[g_aip_ad_cnt].lon_e6=lo; g_aip_ads[g_aip_ad_cnt].type_id=tid;
        g_aip_ad_cnt++;}}
// Parse le flux concaténé [uint16 nfiles] + ([u8 kind 0=ctr/1=aero][u32 len][len o])×n.
static bool aipLoadFromBuffer(const uint8_t* buf,uint32_t total){
    if(!g_aip_lat){g_aip_lat=(int32_t*)ps_malloc(AIP_MAX_PTS*sizeof(int32_t));
                   g_aip_lon=(int32_t*)ps_malloc(AIP_MAX_PTS*sizeof(int32_t));
                   g_aip_ads=(AipAd*)ps_malloc(AIP_MAX_AD*sizeof(AipAd));}
    if(!g_aip_lat||!g_aip_lon||!g_aip_ads){Serial.println("[AIP] PSRAM alloc failed");return false;}
    g_aip_ctr_cnt=0;g_aip_pts_cnt=0;g_aip_ad_cnt=0;
    if(total<2)return false;
    uint32_t o=0; uint16_t nfiles; memcpy(&nfiles,buf+o,2); o+=2;
    for(int i=0;i<nfiles&&o+5<=total;i++){
        uint8_t kind=buf[o++]; uint32_t L; memcpy(&L,buf+o,4); o+=4;
        if(o+L>total)break;
        if(kind==0)_aipLoadCtrBuf(buf+o,L); else if(kind==1)_aipLoadAdBuf(buf+o,L);
        o+=L;}
    return (g_aip_ctr_cnt>0||g_aip_ad_cnt>0);}

#ifdef AIP_EMBEDDED
// Charge l'AIP depuis la flash écran embarquée (WS241 sans SD) — direct, fiable, instantané.
// Réutilise les parsers buffer sur chaque bin embarqué (pas de transfert, pas de format flux).
static void aipLoadEmbedded(){
    if(!g_aip_lat){g_aip_lat=(int32_t*)ps_malloc(AIP_MAX_PTS*sizeof(int32_t));
                   g_aip_lon=(int32_t*)ps_malloc(AIP_MAX_PTS*sizeof(int32_t));
                   g_aip_ads=(AipAd*)ps_malloc(AIP_MAX_AD*sizeof(AipAd));}
    if(!g_aip_lat||!g_aip_lon||!g_aip_ads){Serial.println("[AIP] PSRAM alloc failed");return;}
    g_aip_ctr_cnt=0;g_aip_pts_cnt=0;g_aip_ad_cnt=0;
    for(int i=0;i<AIP_CTR_N;i++) _aipLoadCtrBuf(AIP_CTR[i].data, AIP_CTR[i].len);
    if(AIP_AERODROMES_LEN) _aipLoadAdBuf(AIP_AERODROMES, AIP_AERODROMES_LEN);
    g_aip_loaded=(g_aip_ctr_cnt>0||g_aip_ad_cnt>0);
    Serial.printf("[AIP] embarquée flash → %d CTR (%d pts) / %d AD\n",g_aip_ctr_cnt,g_aip_pts_cnt,g_aip_ad_cnt);
}
#endif

// Tâche dédiée du pull AIP (les ~340 lectures BLE bloquantes → cette tâche, JAMAIS la boucle/radar).
// Handshake : {"cmd":"aip"} → attend STATUS axl>0 → lit CHR_AIP en boucle (curseur boîtier) → PSRAM
// → aipLoadFromBuffer. g_aip_loaded gate le rendu (false pendant le parse).
static void TaskAipPull(void* pv){
    for(;;){
        if(g_aip_pull_req && g_connected && g_chrAip){
            g_aip_pull_req=false;
            sendCtl("aip");                                  // demande au boîtier de bâtir le flux
            uint32_t total=0,t0=millis();
            while(millis()-t0<8000 && g_connected){ if(g_status.aip_xfer_len>0){total=g_status.aip_xfer_len;break;} vTaskDelay(pdMS_TO_TICKS(150)); }
            if(total && g_connected && g_chrAip){
                uint8_t* buf=(uint8_t*)ps_malloc(total);
                if(buf){
                    uint32_t got=0; int fails=0;
                    while(got<total && g_connected && g_chrAip && fails<6){
                        std::string v=bleStr(g_chrAip->readValue());
                        if(v.empty()){fails++; vTaskDelay(pdMS_TO_TICKS(20)); continue;}
                        uint32_t n=v.size(); if(got+n>total)n=total-got;
                        memcpy(buf+got,v.data(),n); got+=n; fails=0;}
                    if(got>=total){
                        g_aip_loaded=false;                  // gate le rendu pendant le parse
                        bool ok=aipLoadFromBuffer(buf,total);
                        g_aip_loaded=ok;
                        Serial.printf("[AIP] pull %u o → %d CTR / %d AD (%s)\n",total,g_aip_ctr_cnt,g_aip_ad_cnt,ok?"OK":"vide");
                    }else Serial.printf("[AIP] pull incomplet %u/%u\n",got,total);
                    free(buf);
                }else Serial.println("[AIP] PSRAM transfert alloc FAIL");
            }else Serial.println("[AIP] pull: pas de axl (boîtier sans AIP?)");
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ── AIP overlay draw ──────────────────────────────────────────────────────────
static inline bool latlon_to_screen(int32_t lat_e6,int32_t lon_e6,
    float own_lat,float own_lon,float cos_lat,int hdg,float scale_m,int&sx,int&sy){
    float dlat_m=(lat_e6/1e6f-own_lat)*111319.0f;
    float dlon_m=(lon_e6/1e6f-own_lon)*111319.0f*cos_lat;
    float d2=dlat_m*dlat_m+dlon_m*dlon_m;
    float sm15=scale_m*AIP_CULL;   // T4 : 2.2× (couvre les coins de l'écran) | T-RGB : 1.5×
    if(d2>sm15*sm15)return false;
    float dist=sqrtf(d2);
    float bear=atan2f(dlon_m,dlat_m)*180.0f/(float)M_PI;
    if(bear<0)bear+=360.0f;
    int rb=((int)bear-hdg+360)%360;
    float brd=(float)rb*(float)M_PI/180.0f;
    float dpx=dist*(float)RAD_R/scale_m;
    sx=(int)(RAD_CX+sinf(brd)*dpx);sy=(int)(RAD_CY-cosf(brd)*dpx);
    return true;}

// Cap effectif du radar — (2026-06-06, note de vol) la rose devient un
// INDICATEUR D'ENREGISTREMENT :
// - vol NON actif (flt_st==0) → verrouillé NORD-up, capsule "NF" (No Flight)
// - vol ACTIF → heading-up ; à l'arrêt (cap GPS non calculable) on FIGE le
//   dernier cap connu — on ne revient PAS au nord tant que le vol n'est pas
//   terminé → on VOIT d'un coup d'œil que l'enregistrement tourne.
#define RADAR_STILL_KMH 5
static inline int radarEffHdg(){
    static int s_hold=0;
    bool active = g_status.valid && g_status.flt_st>=1;
    if(!active){ s_hold=0; return 0; }                       // NF → nord verrouillé
    if(g_status.spd >= RADAR_STILL_KMH) s_hold=(int)g_status.hdg;
    return s_hold;                                           // arrêt en vol → cap figé
}

static void aipDrawCb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_DRAW_MAIN_END)return;
    if(!g_cfg.aip_en||!g_aip_loaded||!g_status.valid||!g_status.gps_fix)return;
    lv_draw_ctx_t*ctx=lv_event_get_draw_ctx(e);
    // Les coords RAD_* sont en coords PAGE ; le draw ctx est en coords ABSOLUES écran.
    // Offset = origine absolue de la page (0,0 sur T-RGB → no-op ; (60,-15) sur T4-S3).
    lv_area_t pco;lv_obj_get_coords(lv_obj_get_parent(lv_event_get_target(e)),&pco);
    const lv_coord_t ox=pco.x1,oy=pco.y1;
    // Clip du dessin AIP : T-RGB = cercle radar | T4 = PLEIN ÉCRAN (2026-06-05,
    // demande vol test : les contours peuvent déborder de la mire, l'écran rect
    // est entièrement exploitable — le cercle n'est qu'une référence d'échelle).
    lv_draw_mask_radius_param_t cmask;
#ifdef BOARD_T4S3
    lv_area_t carea={(lv_coord_t)(0+ox),(lv_coord_t)(0+oy),
                     (lv_coord_t)(599+ox),(lv_coord_t)(449+oy)};
    lv_draw_mask_radius_init(&cmask,&carea,0,false);   // rectangle = pas de masque rond
#else
    lv_area_t carea={(lv_coord_t)(RAD_CX-RAD_R+ox),(lv_coord_t)(RAD_CY-RAD_R+oy),
                     (lv_coord_t)(RAD_CX+RAD_R-1+ox),(lv_coord_t)(RAD_CY+RAD_R-1+oy)};
    lv_draw_mask_radius_init(&cmask,&carea,LV_RADIUS_CIRCLE,false);
#endif
    int16_t mid=lv_draw_mask_add(&cmask,NULL);
    float own_lat=g_status.lat,own_lon=g_status.lon;
    float cos_lat=cosf(own_lat*(float)M_PI/180.0f);
    float scale_m=(float)g_cfg.scale_nm*1852.0f;
    int   hdg=radarEffHdg();   // north-up auto à l'arrêt, cohérent avec le trafic
    // (2026-06-27) CULLING GÉOGRAPHIQUE : avec toute l'AIP EU embarquée (1098 CTR + 3500 AD),
    // dessiner TOUT à chaque frame bloque la boucle (tactile +/- raté). On ne traite que ce qui
    // est dans la FENÊTRE radar (own ± portée×1.6 de marge coin) → coût ∝ visible (~dizaines),
    // pas total. Rejet rapide en e6 (comparaisons entières) AVANT la projection trig coûteuse.
    float dLatDeg=(g_cfg.scale_nm*1.6f)/60.0f;
    float dLonDeg=dLatDeg/(cos_lat>0.05f?cos_lat:0.05f);
    int32_t wMnLa=(int32_t)((own_lat-dLatDeg)*1e6f), wMxLa=(int32_t)((own_lat+dLatDeg)*1e6f);
    int32_t wMnLo=(int32_t)((own_lon-dLonDeg)*1e6f), wMxLo=(int32_t)((own_lon+dLonDeg)*1e6f);
    // CTR polygons
    lv_draw_line_dsc_t ctr_d,atz_d;
    lv_draw_line_dsc_init(&ctr_d);
    ctr_d.color=C_BRAND;ctr_d.width=2;                       // (juin 2026) AIP bleu AeroTrace, trait 2px
    atz_d=ctr_d;atz_d.color=lv_color_hex(0x9fb3cc);          // ATZ : bleu un peu plus clair (width hérité=2)
    for(int c=0;c<g_aip_ctr_cnt;c++){
        uint16_t st=g_aip_ctr[c].pt_start, end=st+g_aip_ctr[c].n_pts;
        // bbox de la zone (compares entières, pas chères) → skip si hors fenêtre radar
        int32_t mnLa=INT32_MAX,mxLa=INT32_MIN,mnLo=INT32_MAX,mxLo=INT32_MIN;
        for(uint16_t pi=st;pi<end;pi++){ int32_t la=g_aip_lat[pi],lo=g_aip_lon[pi];
            if(la<mnLa)mnLa=la; if(la>mxLa)mxLa=la; if(lo<mnLo)mnLo=lo; if(lo>mxLo)mxLo=lo; }
        if(mxLa<wMnLa||mnLa>wMxLa||mxLo<wMnLo||mnLo>wMxLo)continue;   // hors champ → pas de projection/dessin
        lv_draw_line_dsc_t&dsc=(g_aip_ctr[c].type_id==13)?atz_d:ctr_d;
        int psx=0,psy=0;bool pok=false;
        for(uint16_t pi=st;pi<end;pi++){
            int sx,sy;
            bool ok=latlon_to_screen(g_aip_lat[pi],g_aip_lon[pi],
                                     own_lat,own_lon,cos_lat,hdg,scale_m,sx,sy);
            if(ok&&pok){lv_point_t p1={(lv_coord_t)(psx+ox),(lv_coord_t)(psy+oy)},
                                    p2={(lv_coord_t)(sx+ox),(lv_coord_t)(sy+oy)};
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
        int32_t ala=g_aip_ads[a].lat_e6, alo=g_aip_ads[a].lon_e6;
        if(ala<wMnLa||ala>wMxLa||alo<wMnLo||alo>wMxLo)continue;   // hors fenêtre radar → skip
        int sx,sy;
        if(latlon_to_screen(g_aip_ads[a].lat_e6,g_aip_ads[a].lon_e6,
                            own_lat,own_lon,cos_lat,hdg,scale_m,sx,sy)){
            lv_area_t ar={(lv_coord_t)(sx-2+ox),(lv_coord_t)(sy-2+oy),
                          (lv_coord_t)(sx+2+ox),(lv_coord_t)(sy+2+oy)};
            lv_draw_rect(ctx,&ad_d,&ar);}}
    lv_draw_mask_free_param(&cmask);
    lv_draw_mask_remove_id(mid);}

// ── Page 1 — Radar ────────────────────────────────────────────────────────────
void buildRadarPage(){
    lv_obj_t*p=g_pages[1];
#ifdef BOARD_T4S3
    // Page radar PLEIN ÉCRAN : coords radar = coords écran, et la page masquée
    // invalide tout l'écran (pas de résidus dans les bandes lors des swipes)
    lv_obj_set_size(p,SCR_W,SCR_H);lv_obj_set_pos(p,0,0);   // SCR_H = 480 sur WS-241, 450 sur T4
#endif
    // (juin 2026) Radar NON scrollable : le trafic/AIP en OVERSCAN déborde des bords →
    // si la page est scrollable, un drag horizontal SCROLLE au lieu de déclencher le
    // swipe-nav (cause du « swipe radar capricieux », absent des autres pages qui tiennent
    // dans l'écran). On coupe scroll + scrollbar + élasticité.
    lv_obj_clear_flag(p,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(p,LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(p,LV_DIR_NONE);

    // Heading pill (top centre du radar) — (juin 2026) descendue un peu sur T4-S3
    lv_obj_t*hb=lv_obj_create(p);lv_obj_set_size(hb,HDG_W,HDG_H);
#ifdef BOARD_T4S3
    lv_obj_align(hb,LV_ALIGN_TOP_MID,HDG_DX,46);
#else
    lv_obj_align(hb,LV_ALIGN_TOP_MID,HDG_DX,28);
#endif
    lv_obj_set_style_bg_color(hb,THDG(),0);lv_obj_set_style_bg_opa(hb,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(hb,TFG(),0);lv_obj_set_style_border_width(hb,1,0);
    lv_obj_set_style_radius(hb,14,0);lv_obj_set_style_shadow_opa(hb,LV_OPA_TRANSP,0);
    lv_obj_set_style_pad_all(hb,0,0);lv_obj_clear_flag(hb,LV_OBJ_FLAG_SCROLLABLE);
    r_radar_hdg=lv_label_create(hb);lv_label_set_text(r_radar_hdg,"---°");
    lv_obj_set_style_text_color(r_radar_hdg,TFG(),0);
    lv_obj_set_style_text_font(r_radar_hdg,&HDG_FONT,0);lv_obj_center(r_radar_hdg);

    // GS — (juin 2026) RETIRÉE du radar sur T4-S3 (« GS pas nécessaire »). updateAllPages garde `if(r_radar_gs)`.
#ifdef BOARD_T4S3
    r_radar_gs=nullptr;
#else
    r_radar_gs=lv_label_create(p);lv_label_set_text(r_radar_gs,"GS ---");
    lv_obj_set_style_text_color(r_radar_gs,TFG(),0);
    lv_obj_set_style_text_font(r_radar_gs,&RAD_FONT,0);
    lv_obj_align(r_radar_gs,LV_ALIGN_BOTTOM_MID,RB_DX,-33+RB_DY);   // remonté pour loger la ligne version
#endif

    // Version firmware AT-CORE + date de build — bas de page.
    // (juin 2026) RETIRÉE du radar sur T4-S3 (« pas de n° de version sur cette page »).
    // L'info reste disponible en page Settings/ABOUT. updateAllPages garde `if(r_radar_ver)`.
#ifdef BOARD_T4S3
    r_radar_ver=nullptr;
#else
    r_radar_ver=lv_label_create(p);lv_label_set_text(r_radar_ver,"");
    lv_obj_set_style_text_color(r_radar_ver,TGREY(),0);
    lv_obj_set_style_text_font(r_radar_ver,&lv_font_montserrat_10,0);
    lv_obj_align(r_radar_ver,LV_ALIGN_BOTTOM_MID,RB_DX,-13+RB_DY);
#endif

    // Tab pills 52×32 — outer edge is AT the display circle boundary (8-12px behind bezel).
    // The circular LCD naturally clips the outer rounded corner → flat outer edge = "D" shape.
    // Only the inner rounded end (radius=16 half-circle) is fully visible.
    // Left:  Battery  y_c=96  x=40   SafeSky y_c=134 x=17   FLARM y_c=172 x=2   ADS-B y_c=210 x=-6
    // Right: GPS      y_c=96  x=388  LTE     y_c=134 x=411  WiFi  y_c=172 x=426  BLE   y_c=210 x=434
#ifdef BOARD_T4S3
    // T4-S3 : annotations en colonne verticale à gauche du radar (haut → bas)
    r_hdr_bat  = mkTabPill(p, LV_SYMBOL_CHARGE,    RLC_X, 12);
    r_hdr_sky  = mkImgPill(p, &img_safesky,        RLC_X, 60);
#else
    r_hdr_bat  = mkTabPill(p, LV_SYMBOL_CHARGE,       40, 80);
    r_hdr_sky  = mkImgPill(p, &img_safesky,           17, 118);
#endif
    // Pastilles FLARM et ADS-B retirées du radar (non poussées pour l'instant).
    // À l'emplacement ex-ADS-B (arc gauche) : panneau STOP rouge = fin de vol manuelle,
    // pressable EN VOL (visible seulement quand flt_st==1). Discret, ne masque pas la mire.
    r_flt_stop=lv_btn_create(p);
#ifdef BOARD_T4S3
    // Aligné sur le chip Start (colonne gauche sous BLE, même slot) — gros format vol.
    lv_obj_set_size(r_flt_stop,130,56);
    lv_obj_set_pos(r_flt_stop,RLC_X,300);
#else
    lv_obj_set_size(r_flt_stop,48,40);
    lv_obj_set_pos(r_flt_stop,8,186);
#endif
    lv_obj_set_style_bg_color(r_flt_stop,C_RED,0);lv_obj_set_style_radius(r_flt_stop,10,0);
    lv_obj_set_style_border_color(r_flt_stop,lv_color_hex(0xffffff),0);lv_obj_set_style_border_width(r_flt_stop,2,0);
    lv_obj_set_style_shadow_opa(r_flt_stop,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(r_flt_stop,[](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("stop_flight"); },LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(r_flt_stop);lv_label_set_text(l,"STOP");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
#ifdef BOARD_T4S3
     lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);   // gros bouton colonne gauche
#else
     lv_obj_set_style_text_font(l,&lv_font_montserrat_12,0);
#endif
     lv_obj_center(l);}
    lv_obj_add_flag(r_flt_stop,LV_OBJ_FLAG_HIDDEN);   // caché par défaut (montré en vol)
    r_hdr_flrm = nullptr; r_hdr_adsb = nullptr;
#ifdef BOARD_T4S3
    r_hdr_gps  = mkTabPill(p, LV_SYMBOL_GPS,       RLC_X, 108);
    r_hdr_lte  = mkLTEPill(p, RLC_X, 156);
    r_hdr_wifi = mkTabPill(p, LV_SYMBOL_WIFI,      RLC_X, 204);
    r_hdr_ble  = mkTabPill(p, LV_SYMBOL_BLUETOOTH, RLC_X, 252);
    // Point santé signal (2026-06-06) : en haut de la colonne, à droite de la
    // batterie. VERT = échange SafeSky UDP < 10 s · ROUGE = perte signal —
    // revient vert dès le retour LTE. Le trafic passe gris/disparaît en miroir.
    r_ss_dot=lv_obj_create(p);lv_obj_set_size(r_ss_dot,18,18);
    lv_obj_set_pos(r_ss_dot,RLC_X+PILL_W+10,23);
    lv_obj_set_style_radius(r_ss_dot,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_color(r_ss_dot,C_RED,0);lv_obj_set_style_bg_opa(r_ss_dot,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(r_ss_dot,1,0);lv_obj_set_style_border_color(r_ss_dot,TFG(),0);
    lv_obj_set_style_shadow_opa(r_ss_dot,LV_OPA_TRANSP,0);
    lv_obj_clear_flag(r_ss_dot,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
#else
    r_hdr_gps  = mkTabPill(p, LV_SYMBOL_GPS,         388, 80);
    r_hdr_lte  = mkLTEPill(p, 411, 118);
    r_hdr_wifi = mkTabPill(p, LV_SYMBOL_WIFI,        426, 156);
    r_hdr_ble  = mkTabPill(p, LV_SYMBOL_BLUETOOTH,   434, 194);
#endif

    // (v20) Badge "GND" : SafeSky en mode éco au sol (cadence 60 s côté AT-CORE). La
    // pastille reste VERTE (SafeSky fonctionne) ; le trafic s'affiche en GRIS = rafraîchi
    // lentement → le pilote ne sur-fie pas à une image figée, sans taper le forfait.
    // Caché en vol (ssm=0). Commun T4/T-RGB (RLC_X/PILL_W définis pour les deux).
    r_ss_gnd = lv_label_create(p);
    lv_label_set_text(r_ss_gnd, "GND");
    lv_obj_set_style_text_color(r_ss_gnd, C_AMBER, 0);
    lv_obj_set_style_text_font(r_ss_gnd, &lv_font_montserrat_14, 0);
#ifdef BOARD_T4S3
    lv_obj_set_pos(r_ss_gnd, RLC_X + PILL_W + 32, 24);   // à DROITE du point santé SafeSky (r_ss_dot @ +10,23)
#else
    lv_obj_set_pos(r_ss_gnd, RLC_X + PILL_W + 8, 64);    // T-RGB : pas de point dédié → à côté de la pill SafeSky
#endif
    lv_obj_add_flag(r_ss_gnd, LV_OBJ_FLAG_HIDDEN);

#ifdef BOARD_T4S3
    // ── (juin 2026) STATUT RADAR ───────────────────────────────────────────────
    // HAUT-GAUCHE : icône SafeSky (TEINTE = santé signal, vert/rouge) + à sa droite le
    // STATUT DE VOL (GND au sol / FLT en vol). BAS-GAUCHE : pastilles GPS + LTE (LTE plus
    // bas, demande utilisateur). Masqués : batterie / WiFi / BLE / point santé (redondant).
    // ⚠ mk*Pill renvoient l'objet INTERNE → on déplace/masque la PASTILLE (le parent).
    #define PILL_OF(x) lv_obj_get_parent(x)
    if(r_hdr_bat)  lv_obj_add_flag(PILL_OF(r_hdr_bat),  LV_OBJ_FLAG_HIDDEN);
    if(r_hdr_wifi) lv_obj_add_flag(PILL_OF(r_hdr_wifi), LV_OBJ_FLAG_HIDDEN);
    if(r_hdr_ble)  lv_obj_add_flag(PILL_OF(r_hdr_ble),  LV_OBJ_FLAG_HIDDEN);
    if(r_ss_dot)   lv_obj_add_flag(r_ss_dot,            LV_OBJ_FLAG_HIDDEN);   // r_ss_dot = objet autonome
    // Icône SafeSky 2× en HAUT-GAUCHE (teintée live dans updateAllPages — vert OK / rouge KO)
    if(r_hdr_sky){
        lv_obj_t* sp=PILL_OF(r_hdr_sky);
        lv_obj_clear_flag(sp,LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(sp,52,52);lv_obj_set_pos(sp,RLC_X,8+R_TOP_EXTRA);   // 24×24 ×2 = 48 → pastille 52
        lv_img_set_zoom(r_hdr_sky,512);                          // 256 = 1× → 512 = 2×
        lv_obj_center(r_hdr_sky);
    }
    // Statut de vol, à DROITE de l'icône SafeSky (défaut GND)
    lv_obj_set_pos(r_ss_gnd, RLC_X + 52 + 14, 24+R_TOP_EXTRA);
    lv_obj_set_style_text_font(r_ss_gnd, &lv_font_montserrat_22, 0);
    lv_label_set_text(r_ss_gnd, "GND");
    lv_obj_clear_flag(r_ss_gnd, LV_OBJ_FLAG_HIDDEN);
    // LTE & GPS(GNSS) : CENTRÉS sous l'icône SafeSky (centre x≈36), agrandis manuellement
    // (pas de transform_zoom : il les faisait disparaître). Pastille PILL_W=64 → x=4 = centre 36.
    if(r_hdr_lte){ lv_obj_set_pos(PILL_OF(r_hdr_lte), RLC_X-6, 70+R_TOP_EXTRA);
        // barres plus grandes
        static const int8_t bh2[4]={9,14,19,25}; const int bw=5, sp=7, gx=(PILL_W-(4*bw+3*(sp-bw)))/2;
        for(int i=0;i<4;i++) if(r_hdr_lte_b[i]){
            lv_obj_set_size(r_hdr_lte_b[i],bw,bh2[i]);
            lv_obj_set_pos(r_hdr_lte_b[i],gx+i*sp,(PILL_H-2)-bh2[i]); } }
    if(r_hdr_gps){ lv_obj_set_pos(PILL_OF(r_hdr_gps), RLC_X-6, 120+R_TOP_EXTRA);   // gap SafeSky→LTE = LTE→GPS = 10
        lv_obj_set_style_text_font(r_hdr_gps,&lv_font_montserrat_28,0); }   // symbole GPS plus grand
    #undef PILL_OF
#endif

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

    // Cardinal labels N/E/S/W — position radiale via RAD_CARD_OFF (T4 = à l'extérieur de l'anneau)
    const char*cnames[]={"N","E","S","W"};
    for(int ci=0;ci<4;ci++){
        r_card[ci]=lv_label_create(p);
        lv_label_set_text(r_card[ci],cnames[ci]);
        lv_obj_set_style_text_font(r_card[ci],&RAD_FONT,0);
        lv_obj_set_style_text_color(r_card[ci],TFG(),0);
        // (juin 2026) largeur fixe + texte centré → W (large) ne paraît plus décalé
        lv_obj_set_width(r_card[ci],24);
        lv_obj_set_style_text_align(r_card[ci],LV_TEXT_ALIGN_CENTER,0);
        lv_obj_set_pos(r_card[ci],RAD_CX-12,RAD_CY-(RAD_R+RAD_CARD_OFF)-10);}

    // Scale label — entre le S de la rose et la GS (ordre : S → 4nm → GS XXkt).
    // (juin 2026) T4-S3 : police PLUS GRANDE + couleur premier-plan (« affichage plus
    // clair et grand du NM du 2ème cadran »). T-RGB inchangé.
    char scl[12];snprintf(scl,12,"%dnm",g_cfg.scale_nm);
#ifdef BOARD_T4S3
    r_radar_scale_lbl=mkLbl(p,scl,TFG(),&lv_font_montserrat_32,LV_ALIGN_BOTTOM_MID,RB_DX,-36+RB_DY);   // (juin 2026) descendu
#else
    r_radar_scale_lbl=mkLbl(p,scl,TGREY(),&RAD_FONT,LV_ALIGN_BOTTOM_MID,RB_DX,-53+RB_DY);
#endif

    // Zoom +/- buttons — flanquent le label scale, mêmes ids que Settings (0=-, 1=+)
    // → reuse cbSetBtn → cfgSave() + updSetPage() (rafraichit aussi r_radar_scale_lbl)
    auto mkZoomBtn = [&](const char* sym, int dx, intptr_t id){
        lv_obj_t* b=lv_obj_create(p);
        lv_obj_set_size(b,ZOOM_SZ,ZOOM_SZ);
#ifdef BOARD_T4S3
        // Ergonomie vol (2026-06-05) : "+" coin HAUT-droit, "−" coin BAS-droit —
        // 64 px + zone tactile étendue (+12 px invisible), ~360 px de séparation
        // → impossible à confondre en turbulence. dx = coordonnée Y absolue ici.
        lv_obj_set_pos(b,600-ZOOM_SZ-24,dx);   // (juin 2026) décalé du bord (diagonale 45° via dx aussi)
        // (juin 2026) plus de zone tactile étendue : elle capturait les swipes près du bord droit
#else
        lv_obj_align(b,LV_ALIGN_BOTTOM_MID,RB_DX+dx,-28+RB_DY);
        lv_obj_set_ext_click_area(b,14);       // T-RGB rond : cible élargie
#endif
        lv_obj_set_style_radius(b,LV_RADIUS_CIRCLE,0);
#ifdef BOARD_T4S3
        // (juin 2026) zoom = GLYPHE +/- seul, SANS cercle, plus grand
        lv_obj_set_style_bg_opa(b,LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(b,0,0);
#else
        lv_obj_set_style_bg_color(b,THDG(),0);lv_obj_set_style_bg_opa(b,LV_OPA_COVER,0);
        lv_obj_set_style_border_color(b,TFG(),0);lv_obj_set_style_border_width(b,1,0);
#endif
        lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
#ifdef BOARD_T4S3
        // (juin 2026) +/- DESSINÉS en barres (gros & gras, taille libre vs police).
        {const int BAR=56, TH=10;
         auto mkbar=[&](int w,int h){ lv_obj_t*r=lv_obj_create(b);lv_obj_set_size(r,w,h);lv_obj_center(r);
            lv_obj_set_style_bg_color(r,TFG(),0);lv_obj_set_style_bg_opa(r,LV_OPA_COVER,0);
            lv_obj_set_style_border_width(r,0,0);lv_obj_set_style_radius(r,2,0);lv_obj_set_style_pad_all(r,0,0);
            lv_obj_clear_flag(r,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE); };
         mkbar(BAR,TH);                       // barre horizontale (− et +)
         if(sym[0]=='+') mkbar(TH,BAR);}      // barre verticale (+)
#else
        lv_obj_t* lab=lv_label_create(b);lv_label_set_text(lab,sym);
        lv_obj_set_style_text_color(lab,TFG(),0);
        lv_obj_set_style_text_font(lab,&ZOOM_FONT,0);
        lv_obj_center(lab);
#endif
        lv_obj_add_event_cb(b,cbSetBtn,LV_EVENT_CLICKED,(void*)id);
    };
    // id 0 = nm-- (zoom IN), id 1 = nm++ (zoom OUT). On mappe "+" sur le zoom IN
    // (pousser + = se rapprocher) et "-" sur le zoom OUT. Settings SCALE inchangé.
#ifdef BOARD_T4S3
    mkZoomBtn("+", 24+R_ZOOM_IN,             0);   // haut-droit (descendu de R_ZOOM_IN sur WS-241)
    mkZoomBtn("-", SCR_H-ZOOM_SZ-24-R_ZOOM_IN, 1);   // bas-droit (remonté de R_ZOOM_IN sur WS-241)
#elif defined(BOARD_WS216)
    // Carré 480 : le zoom = 2 GRANDES zones tactiles invisibles dans les coins bas (hors radar
    // rond), avec JUSTE le glyphe +/- (barres, style T4) centré dedans. + = bas-DROITE (id 0,
    // zoom in), − = bas-GAUCHE (id 1, zoom out). EVENT_BUBBLE → le swipe de nav passe toujours.
    {auto mkZoomZone=[&](const char* sym,bool right,intptr_t id){
        lv_obj_t* z=lv_obj_create(p);
        lv_obj_set_size(z,200,180);
        lv_obj_align(z, right?LV_ALIGN_BOTTOM_RIGHT:LV_ALIGN_BOTTOM_LEFT, 0, 0);
        lv_obj_set_style_bg_opa(z,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(z,0,0);
        lv_obj_set_style_shadow_opa(z,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(z,0,0);
        lv_obj_clear_flag(z,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(z,LV_OBJ_FLAG_EVENT_BUBBLE);           // laisse passer le swipe de nav
        lv_obj_add_event_cb(z,cbSetBtn,LV_EVENT_CLICKED,(void*)id);
        const int BAR=54,TH=10;                                // glyphe = barres, POUSSÉ dans l'angle
        const int GOX=(right?50:-50), GOY=48;                  // décalage vers le coin bas-ext (2 barres concentriques → + propre)
        auto bar=[&](int w,int h){ lv_obj_t*r=lv_obj_create(z);lv_obj_set_size(r,w,h);lv_obj_align(r,LV_ALIGN_CENTER,GOX,GOY);
            lv_obj_set_style_bg_color(r,TFG(),0);lv_obj_set_style_bg_opa(r,LV_OPA_COVER,0);
            lv_obj_set_style_border_width(r,0,0);lv_obj_set_style_radius(r,2,0);lv_obj_set_style_pad_all(r,0,0);
            lv_obj_clear_flag(r,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(r,LV_OBJ_FLAG_EVENT_BUBBLE); };
        bar(BAR,TH); if(sym[0]=='+') bar(TH,BAR);              // − = 1 barre, + = croix
     };
     mkZoomZone("-",false,1);   // bas-GAUCHE
     mkZoomZone("+",true, 0);}  // bas-DROITE
#else
    mkZoomBtn("-",-55,1);
    mkZoomBtn("+", 55,0);
#endif

    // AIP overlay — transparent layer between grid and traffic icons
    r_aip_layer=lv_obj_create(p);
#ifdef BOARD_T4S3
    lv_obj_set_size(r_aip_layer,SCR_W,SCR_H);lv_obj_set_pos(r_aip_layer,0,0);   // page plein écran (SCR_H=480 WS-241)
#else
    lv_obj_set_size(r_aip_layer,480,480);lv_obj_set_pos(r_aip_layer,0,0);
#endif
    lv_obj_set_style_bg_opa(r_aip_layer,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(r_aip_layer,0,0);lv_obj_set_style_pad_all(r_aip_layer,0,0);
    lv_obj_set_style_shadow_opa(r_aip_layer,LV_OPA_TRANSP,0);
    lv_obj_clear_flag(r_aip_layer,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(r_aip_layer,aipDrawCb,LV_EVENT_DRAW_MAIN_END,NULL);

    // CO arc gauge — 3 fixed color bands (30° total) in bottom-right quadrant
    // LVGL arc convention: 0°=right(3h), increases CW → compass120°=LVGL30°, compass150°=LVGL60°
    // Each band 10°: green(30-40°) caution(40-50°) danger(50-60°)
    // UI_CO_EN=0 (2026-06-05) : capteur CO pas câblé → jauge/curseur/labels masqués
    // à la compilation. Remettre à 1 quand le capteur sera monté (l'alerte CO BLE,
    // elle, reste active — elle vient de l'AT-CORE).
#if UI_CO_EN
    {
        struct Band{int s,e;lv_color_t c;};
#ifdef BOARD_T4S3
        // Miroir horizontal : quadrant bas-GAUCHE (LVGL 120-150°), vert en haut
        Band bs[]={{140,150,C_GREEN},{130,140,C_ORANGE},{120,130,C_RED}};
#else
        Band bs[]={{30,40,C_GREEN},{40,50,C_ORANGE},{50,60,C_RED}};
#endif
        for(auto&b:bs){
            lv_obj_t*ba=lv_arc_create(p);
            lv_obj_set_size(ba,CO_SZ,CO_SZ);lv_obj_set_pos(ba,RAD_CX-CO_SZ/2,RAD_CY-CO_SZ/2);
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
    lv_obj_set_pos(r_co_ball,(int)(RAD_CX+(CO_SZ/2-8)*0.866f*CO_MIR)-6,(int)(RAD_CY+(CO_SZ/2-8)*0.5f)-6);
    // CO text + ppm — OUTSIDE radar ring (r=175), at arc midpoint (LVGL 45°, r≈190)
    // x=240+190*cos45°=374, y=374 → label anchored just outside the ring
    r_co_text=lv_label_create(p);lv_label_set_text(r_co_text,"CO");
    lv_obj_set_style_text_color(r_co_text,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(r_co_text,&lv_font_montserrat_12,0);
    r_co_val=lv_label_create(p);lv_label_set_text(r_co_val,"");
    lv_obj_set_style_text_font(r_co_val,&lv_font_montserrat_12,0);
#ifdef BOARD_T4S3
    lv_obj_set_pos(r_co_text,160,386);lv_obj_set_pos(r_co_val,154,406);
#else
    lv_obj_set_pos(r_co_text,366,364);lv_obj_set_pos(r_co_val,360,380);
#endif
#else
    r_co_ball=nullptr;r_co_text=nullptr;r_co_val=nullptr;   // CO désactivé (UI_CO_EN=0)
#endif  // UI_CO_EN

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
        lv_obj_set_style_text_font(r_radar_cs[i],&lv_font_montserrat_16,0);   // immat un peu plus grande
        lv_obj_set_style_text_color(r_radar_cs[i],TFG(),0);
        lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
        r_radar_alt[i]=lv_label_create(p);lv_label_set_text(r_radar_alt[i],"");
        lv_obj_set_style_text_font(r_radar_alt[i],&lv_font_montserrat_20,0);  // Δalt bien visible en vol (info anti-collision)
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
    lv_obj_set_style_text_font(r_aov_text,&lv_font_montserrat_16,0);lv_obj_center(r_aov_text);
#ifdef BOARD_T4S3
    // Swipe robuste (T4) : les enfants cliquables du radar (arcs de la mire, boutons
    // zoom des coins droits, chips) capteraient l'appui SANS le transmettre → le swipe
    // horizontal radar→Settings se perdait selon le point de départ. On les fait
    // « bubbler » vers la page → swipeCb voit l'appui où qu'il commence.
    for(uint32_t i=0;i<lv_obj_get_child_cnt(p);i++)
        lv_obj_add_flag(lv_obj_get_child(p,i),LV_OBJ_FLAG_EVENT_BUBBLE);
    // (juin 2026) Appui long sur la mire → action sheet Start/Stop (les enfants bubblent
    // déjà vers la page, donc un long-press n'importe où sur le radar déclenche).
    lv_obj_add_event_cb(p,cbRadarLongPress,LV_EVENT_LONG_PRESSED,NULL);

    // (juin 2026) Bouton SETTINGS bien VISIBLE (cercle bleu + engrenage), créé EN DERNIER
    // (au-dessus du trafic/AIP) et SANS EVENT_BUBBLE → tap 100 % fiable même si le swipe
    // résiste. Coin bas-gauche. C'est la nav de secours demandée.
    {lv_obj_t* gear=lv_btn_create(p);
     lv_obj_set_size(gear,64,64);lv_obj_set_pos(gear,12,SCR_H-64-12-R_GEAR_UP);
     lv_obj_set_style_radius(gear,LV_RADIUS_CIRCLE,0);
     lv_obj_set_style_bg_color(gear,C_BRAND,0);lv_obj_set_style_bg_opa(gear,LV_OPA_COVER,0);
     lv_obj_set_style_border_width(gear,0,0);lv_obj_set_style_shadow_opa(gear,LV_OPA_TRANSP,0);
     lv_obj_set_ext_click_area(gear,20);   // cible élargie (bouton Settings souvent raté)
     lv_obj_add_event_cb(gear,[](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED){ g_navPage=2; g_navPending=true; } },LV_EVENT_CLICKED,NULL);
     lv_obj_t* gl=lv_label_create(gear);lv_label_set_text(gl,LV_SYMBOL_SETTINGS);
     lv_obj_set_style_text_color(gl,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(gl,&lv_font_montserrat_28,0);lv_obj_center(gl);}

    // Toggle rapide MODE ALERTE (au-dessus de l'engrenage) : tap = cycle AUTO→CIRC→RTE.
    // Couleur = état effectif (refresh dans updateRadarDR). Usage opérationnel (bascule en vol).
    {lv_obj_t* cc=lv_btn_create(p);
     lv_obj_set_size(cc,64,36);lv_obj_set_pos(cc,12,SCR_H-64-12-44-R_GEAR_UP);
     lv_obj_set_style_radius(cc,8,0);
     lv_obj_set_style_bg_color(cc,lv_color_hex(0x1f2937),0);lv_obj_set_style_bg_opa(cc,LV_OPA_COVER,0);
     lv_obj_set_style_border_width(cc,0,0);lv_obj_set_style_shadow_opa(cc,LV_OPA_TRANSP,0);
     lv_obj_set_ext_click_area(cc,8);
     lv_obj_add_event_cb(cc,[](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED){ g_cfg.circuit_ovr=(g_cfg.circuit_ovr+1)%3; cfgSave(); } },LV_EVENT_CLICKED,NULL);
     r_circ_lbl=lv_label_create(cc);lv_label_set_text(r_circ_lbl,kCircNames[g_cfg.circuit_ovr]);
     lv_obj_set_style_text_font(r_circ_lbl,&lv_font_montserrat_16,0);lv_obj_center(r_circ_lbl);}
#endif
}

// ── Page 2 — Settings ─────────────────────────────────────────────────────────
#ifdef BOARD_T4S3
static void updSegs();    // fwd : segmented toggles 2-options T4-S3 (défini plus bas)
static void updSegNs();   // fwd : segmented multi-options T4-S3 (défini plus bas)
#endif
void updSetPage(){
    char b[16];
    // Rows nullables (supprimées/remplacées par des segmented sur T4-S3) → toutes gardées.
    if(s_scale_v){snprintf(b,16,"%dnm",g_cfg.scale_nm); lv_label_set_text(s_scale_v,b);}
    if(s_vfilt_v){snprintf(b,16,"%dft",g_cfg.vfilt_ft); lv_label_set_text(s_vfilt_v,b);}
    if(s_dist_v) lv_label_set_text(s_dist_v, g_cfg.dist_nm?"NM":"km");
    if(s_alt_v)  lv_label_set_text(s_alt_v,  g_cfg.alt_ft?"ft":"m");
    if(s_spd_v)  lv_label_set_text(s_spd_v, g_cfg.spd_kt?"kt":"km/h");
    if(s_bright_v){snprintf(b,16,"%d/16",g_cfg.brightness); lv_label_set_text(s_bright_v,b);}
    if(s_src_v)  lv_label_set_text(s_src_v,  kSrcNames[g_cfg.trf_src&3]);
    if(s_grnd_v) lv_label_set_text(s_grnd_v, g_cfg.show_grnd?"ON":"OFF");
    if(s_theme_v)lv_label_set_text(s_theme_v,g_cfg.dark?"DARK":"LIGHT");
#ifdef BOARD_T4S3
    updSegs(); updSegNs();   // refresh des segmented (toggles + multi-options)
#endif
    if(s_icon_sz_v)lv_label_set_text(s_icon_sz_v,kIconSzNames[g_cfg.icon_sz]);
    if(s_circ_v)lv_label_set_text(s_circ_v,kCircNames[g_cfg.circuit_ovr]);
    if(s_wifi_v)lv_label_set_text(s_wifi_v,g_wifi_active?"192.168.4.1":g_cfg.wifi_en?"ON":"OFF");
    if(s_aip_v)lv_label_set_text(s_aip_v,!g_aip_loaded?"NO DATA":g_cfg.aip_en?"ON":"OFF");
    if(s_heli_v)lv_label_set_text(s_heli_v,g_cfg.ad_heli?"ON":"OFF");
    snprintf(b,12,"%dnm",g_cfg.scale_nm); lv_label_set_text(r_radar_scale_lbl,b);
    panelBright(g_cfg.brightness);}

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
        case 26:g_cfg.circuit_ovr=(g_cfg.circuit_ovr+2)%3;break;  // ALERT MODE prev (AUTO/CIRC/RTE)
        case 27:g_cfg.circuit_ovr=(g_cfg.circuit_ovr+1)%3;break;  // ALERT MODE next
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
    panelBright(g_cfg.brightness);
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

// (port T-RGB 2026-06-27) Helpers segment/stepper/popup board-INDÉPENDANTS (étaient T4-only) :
// ════════════════════════════════════════════════════════════════════════════
// SETTINGS T4-S3 — contrôles tactiles agrandis (page 1 « fondamentaux »)
// Demande Christophe 2026-06-07 : réglages exploitables AU DOIGT sur l'AMOLED.
//  • segmented control style iOS (2 options, l'active en bleu plein) → DIST / SPEED / THEME
//  • gros stepper V-FILTER (boutons 58×52, réutilise cbSetBtn ids 2/3)
//  • slider brightness épais (gros knob)
// Réservé T4-S3 ; le T-RGB rond garde les rows compactes mkSetRow.
// ════════════════════════════════════════════════════════════════════════════

// Registre g_seg[]/g_seg_n + struct SegCtl déclarés en tête de fichier (zone types).
static void segApplyStyle(SegCtl &s){
    bool aSel = (*s.val == s.aIsTrue);
    lv_obj_t *on  = aSel? s.segA : s.segB,  *off = aSel? s.segB : s.segA;
    lv_obj_t *onL = aSel? s.lblA : s.lblB,  *offL= aSel? s.lblB : s.lblA;
    lv_obj_set_style_bg_color(on,C_BRAND,0); lv_obj_set_style_bg_opa(on,LV_OPA_COVER,0);
    lv_obj_set_style_text_color(onL,lv_color_hex(0xffffff),0);
    lv_obj_set_style_bg_opa(off,LV_OPA_TRANSP,0);
    lv_obj_set_style_text_color(offL,lv_color_hex(0x4b5563),0);
}
static void updSegs(){ for(int i=0;i<g_seg_n;i++) segApplyStyle(g_seg[i]); }

// tap sur une moitié : data = (segIndex<<1)|side  (side 0=A, 1=B)
static void cbSeg(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int d=(int)(intptr_t)lv_event_get_user_data(e);
    int idx=d>>1, side=d&1;
    if(idx<0||idx>=g_seg_n)return;
    SegCtl &s=g_seg[idx];
    if(s.val==&g_cfg.aip_en && !g_aip_loaded) return;  // AIP non togglable sans données
    bool target = (side==0)? s.aIsTrue : !s.aIsTrue;
    if(*s.val==target)return;
    *s.val=target; cfgSave();
    if(s.val==&g_cfg.dark){ g_rebuildPages=true; return; }   // thème → rebuild complet
    // effets de bord identiques à cbSetBtn (sinon le toggle ne « prend » pas)
    if(s.val==&g_cfg.wifi_en){ if(g_cfg.wifi_en)wifiStart(); else wifiStop(); }
    if((s.val==&g_cfg.aip_en||s.val==&g_cfg.ad_heli) && r_aip_layer) lv_obj_invalidate(r_aip_layer);
    segApplyStyle(s); updSetPage();
}

// Ligne label + segmented (2 options) — réservé T4-S3.
static void mkSegRow(lv_obj_t*p,const char*k,int y,const char*a,const char*b,
                     bool*val,bool aIsTrue){
    if(g_seg_n>=8)return;
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_20,40,y+15);
    const int TW=224,TH=52,TX=SETW-40-TW,HW=(TW-6)/2;   // track à droite (board-aware SETW), 3px pad bords
    lv_obj_t*tr=lv_obj_create(p);
    lv_obj_set_size(tr,TW,TH);lv_obj_set_pos(tr,TX,y);
    lv_obj_set_style_radius(tr,14,0);
    lv_obj_set_style_bg_color(tr,lv_color_hex(0xeef2f6),0);lv_obj_set_style_bg_opa(tr,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(tr,0,0);lv_obj_set_style_pad_all(tr,0,0);
    lv_obj_clear_flag(tr,LV_OBJ_FLAG_SCROLLABLE);
    int idx=g_seg_n;
    lv_obj_t*sa=lv_btn_create(tr);lv_obj_set_size(sa,HW,TH-6);lv_obj_set_pos(sa,3,3);
    lv_obj_set_style_radius(sa,11,0);lv_obj_set_style_shadow_opa(sa,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(sa,0,0);lv_obj_set_style_pad_all(sa,0,0);
    lv_obj_add_event_cb(sa,cbSeg,LV_EVENT_CLICKED,(void*)(intptr_t)((idx<<1)|0));
    lv_obj_t*la=lv_label_create(sa);lv_label_set_text(la,a);
    lv_obj_set_style_text_font(la,&lv_font_montserrat_20,0);lv_obj_center(la);
    lv_obj_t*sb=lv_btn_create(tr);lv_obj_set_size(sb,HW,TH-6);lv_obj_set_pos(sb,3+HW,3);
    lv_obj_set_style_radius(sb,11,0);lv_obj_set_style_shadow_opa(sb,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(sb,0,0);lv_obj_set_style_pad_all(sb,0,0);
    lv_obj_add_event_cb(sb,cbSeg,LV_EVENT_CLICKED,(void*)(intptr_t)((idx<<1)|1));
    lv_obj_t*lb=lv_label_create(sb);lv_label_set_text(lb,b);
    lv_obj_set_style_text_font(lb,&lv_font_montserrat_20,0);lv_obj_center(lb);
    g_seg[idx].segA=sa;g_seg[idx].segB=sb;g_seg[idx].lblA=la;g_seg[idx].lblB=lb;
    g_seg[idx].val=val;g_seg[idx].aIsTrue=aIsTrue;
    g_seg_n++; segApplyStyle(g_seg[idx]);
}

// Gros stepper : label + [−] valeur [+] (réutilise cbSetBtn ids). Renvoie le label valeur.
static lv_obj_t* mkBigStepRow(lv_obj_t*p,const char*k,int y,const char*v,int idn,int idup){
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_20,40,y+15);
    lv_color_t bg=lv_color_hex(0xeef2f6); const int BW=64,BH=52;
    // Bande contrôle commune (board-aware) : alignée aux segmented/slider du dessous.
    lv_obj_t*bd=lv_btn_create(p);lv_obj_set_size(bd,BW,BH);lv_obj_set_pos(bd,SETW-264,y);
    lv_obj_set_style_bg_color(bd,bg,0);lv_obj_set_style_radius(bd,12,0);
    lv_obj_set_style_shadow_opa(bd,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(bd,0,0);lv_obj_set_style_pad_all(bd,0,0);
    lv_obj_add_event_cb(bd,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idn);
    lv_obj_t*ld=lv_label_create(bd);lv_label_set_text(ld,"-");
    lv_obj_set_style_text_color(ld,lv_color_hex(0x0f172a),0);lv_obj_set_style_text_font(ld,&lv_font_montserrat_24,0);lv_obj_center(ld);
    lv_obj_t*vl=mkLblP(p,v,C_BRAND,&lv_font_montserrat_20,SETW-200,y+15);
    lv_obj_set_width(vl,96);lv_obj_set_style_text_align(vl,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_t*bu=lv_btn_create(p);lv_obj_set_size(bu,BW,BH);lv_obj_set_pos(bu,SETW-104,y);
    lv_obj_set_style_bg_color(bu,bg,0);lv_obj_set_style_radius(bu,12,0);
    lv_obj_set_style_shadow_opa(bu,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(bu,0,0);lv_obj_set_style_pad_all(bu,0,0);
    lv_obj_add_event_cb(bu,cbSetBtn,LV_EVENT_CLICKED,(void*)(intptr_t)idup);
    lv_obj_t*lu=lv_label_create(bu);lv_label_set_text(lu,"+");
    lv_obj_set_style_text_color(lu,lv_color_hex(0x0f172a),0);lv_obj_set_style_text_font(lu,&lv_font_montserrat_24,0);lv_obj_center(lu);
    return vl;
}

// Slider brightness épais (gros knob) — réservé T4-S3.
static void mkBigBrightRow(lv_obj_t*p,const char*k,int y,uint8_t val){
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_20,40,y+15);
    lv_obj_t*sl=lv_slider_create(p);
    lv_obj_set_size(sl,224,14);lv_obj_set_pos(sl,SETW-264,y+19);   // bande board-aware (alignée seg/stepper)
    lv_slider_set_range(sl,0,16);lv_slider_set_value(sl,val,LV_ANIM_OFF);
    lv_obj_set_style_bg_color(sl,lv_color_hex(0xe5e7eb),LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sl,LV_OPA_COVER,LV_PART_MAIN);lv_obj_set_style_radius(sl,7,LV_PART_MAIN);
    lv_obj_set_style_bg_color(sl,C_BRAND,LV_PART_INDICATOR);lv_obj_set_style_bg_opa(sl,LV_OPA_COVER,LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sl,C_BRAND,LV_PART_KNOB);
    lv_obj_set_style_pad_all(sl,16,LV_PART_KNOB);     // knob ≈ 46 px → cible doigt
    lv_obj_add_event_cb(sl,cbBrightSlider,LV_EVENT_ALL,NULL);
}

// ── Segmented MULTI-options (SOURCE 4, ICONS SIZE 3) ─────────────────────────
static void segNApply(SegN &s){
    for(int i=0;i<s.n;i++){
        bool on=(*s.val==i);
        lv_obj_set_style_bg_color(s.cell[i],C_BRAND,0);
        lv_obj_set_style_bg_opa(s.cell[i],on?LV_OPA_COVER:LV_OPA_TRANSP,0);
        lv_obj_set_style_text_color(s.lbl[i],on?lv_color_hex(0xffffff):lv_color_hex(0x4b5563),0);
    }
}
static void updSegNs(){ for(int i=0;i<g_segn_n;i++) segNApply(g_segn[i]); }

// tap sur une cellule : data = (segIndex<<3)|cell
static void cbSegN(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int d=(int)(intptr_t)lv_event_get_user_data(e);
    int idx=d>>3, cell=d&7;
    if(idx<0||idx>=g_segn_n)return;
    SegN &s=g_segn[idx];
    if(cell>=s.n || *s.val==cell)return;
    *s.val=(uint8_t)cell; cfgSave();
    if(s.kind==1){ for(int i=0;i<MAX_TRF;i++) lv_img_set_zoom(r_trf_img[i],kIconZoom[g_cfg.icon_sz]); } // ICONS
    segNApply(s); updSetPage();
}

// Ligne label + segmented N options (cellules égales dans la bande 336..560).
static void mkSegRowN(lv_obj_t*p,const char*k,int y,const char*const*opts,int n,uint8_t*val,uint8_t kind){
    if(g_segn_n>=2||n>4)return;
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_20,40,y+15);
    const int TW=224,TH=52,TX=SETW-40-TW,pad=3,cw=(TW-2*pad)/n;
    lv_obj_t*tr=lv_obj_create(p);
    lv_obj_set_size(tr,TW,TH);lv_obj_set_pos(tr,TX,y);
    lv_obj_set_style_radius(tr,14,0);
    lv_obj_set_style_bg_color(tr,lv_color_hex(0xeef2f6),0);lv_obj_set_style_bg_opa(tr,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(tr,0,0);lv_obj_set_style_pad_all(tr,0,0);
    lv_obj_clear_flag(tr,LV_OBJ_FLAG_SCROLLABLE);
    int idx=g_segn_n;
    for(int i=0;i<n;i++){
        lv_obj_t*c=lv_btn_create(tr);lv_obj_set_size(c,cw,TH-6);lv_obj_set_pos(c,pad+i*cw,3);
        lv_obj_set_style_radius(c,11,0);lv_obj_set_style_shadow_opa(c,LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(c,0,0);lv_obj_set_style_pad_all(c,0,0);
        lv_obj_add_event_cb(c,cbSegN,LV_EVENT_CLICKED,(void*)(intptr_t)((idx<<3)|i));
        lv_obj_t*l=lv_label_create(c);lv_label_set_text(l,opts[i]);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_16,0);lv_obj_center(l);
        g_segn[idx].cell[i]=c; g_segn[idx].lbl[i]=l;
    }
    g_segn[idx].n=(uint8_t)n; g_segn[idx].val=val; g_segn[idx].kind=kind;
    g_segn_n++; segNApply(g_segn[idx]);
}

// Ligne label + (valeur optionnelle) + gros bouton d'action (AIRCRAFT / MAINTENANCE).
static lv_obj_t* mkBigBtnRow(lv_obj_t*p,const char*k,int y,const char*v,const char*btxt,lv_event_cb_t cb){
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_20,40,y+15);
    lv_obj_t*vl=nullptr;
    if(v&&v[0]){ vl=mkLblP(p,v,lv_color_hex(0x0f172a),&lv_font_montserrat_20,200,y+15);
                 lv_obj_set_width(vl,150); }
    const int BW=150,BH=52,BX=SETW-40-BW;
    lv_obj_t*b=lv_btn_create(p);lv_obj_set_size(b,BW,BH);lv_obj_set_pos(b,BX,y);
    lv_obj_set_style_bg_color(b,C_BRAND,0);lv_obj_set_style_radius(b,12,0);
    lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_pad_all(b,0,0);
    lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,btxt);
    lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);lv_obj_center(l);
    return vl;
}

// Tuile de navigation = MÊME STYLE que la grille SETTINGS (mkMenuBtn) : contour bleu, rempli
// au press, nom centré. Posée en grille 2 colonnes dans la page SYSTEM (pas de barres pleine
// largeur). Géométrie board-aware identique à mkMenuBtn.
static void mkNavTile(lv_obj_t*p,const char*name,int x,int y,lv_event_cb_t cb){
    lv_obj_t*bt=lv_btn_create(p);
#ifdef BOARD_T4S3
    lv_obj_set_size(bt,252,80);const lv_font_t*MF=&lv_font_montserrat_24;
#else
    lv_obj_set_size(bt,180,64);const lv_font_t*MF=&lv_font_montserrat_18;
#endif
    lv_obj_set_pos(bt,x,y);
    lv_obj_set_style_bg_color(bt,TBG(),0);lv_obj_set_style_bg_opa(bt,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(bt,C_BRAND,0);lv_obj_set_style_border_width(bt,2,0);
    lv_obj_set_style_radius(bt,16,0);lv_obj_set_style_shadow_opa(bt,LV_OPA_TRANSP,0);
    lv_obj_set_style_bg_color(bt,C_BRAND,LV_STATE_PRESSED);
    lv_obj_add_event_cb(bt,cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t*l=lv_label_create(bt);lv_label_set_text(l,name);
    lv_obj_set_style_text_color(l,C_BRAND,0);
    lv_obj_set_style_text_font(l,MF,0);lv_obj_center(l);
}

// ── Maintenance overlay (Modèle 1 : hotspot + transfert vol) ──────────────────
static lv_obj_t* g_maint_scanlist;   // fwd : annulé ici aussi (enfant de l'overlay)
static lv_obj_t* g_maint_upd=nullptr;   // annonce de MAJ firmware (au-dessus du bouton Update)
static lv_obj_t* g_maint_wst=nullptr;   // état WiFi live (wst/wip du STATUS AT-CORE)
// Met à jour l'annonce de MAJ dans Maintenance (appelée à chaque STATUS quand l'écran est ouvert).
static void maintUpdAnnounce(){
    if(!g_maint_upd)return;
    char b[52]; lv_color_t c;
    if(!g_status.valid){ lv_label_set_text(g_maint_upd,""); return; }
    // 3 états distincts pour lever l'ambiguïté du "up to date" : (1) une MAJ existe,
    // (2) check fait à l'instant et déjà à jour (ota==5), (3) pas encore vérifié → on
    // invite à appuyer (le seul vrai check se fait via le bouton Update, qui interroge
    // le cloud ; l'auto-check oav n'a lieu qu'en piggyback d'un transfert).
    if(g_status.oav>g_status.fwv){
        snprintf(b,sizeof(b),"Update to v%d (now v%d)",g_status.oav,g_status.fwv); c=C_AMBER;
    }else if(g_status.ota==5){
        snprintf(b,sizeof(b),"v%d - up to date",g_status.fwv); c=C_GREEN;
    }else{
        snprintf(b,sizeof(b),"v%d - tap Update to check",g_status.fwv); c=TGREY();
    }
    lv_label_set_text(g_maint_upd,b);
    lv_obj_set_style_text_color(g_maint_upd,c,0);
}
// Ligne d'état WiFi sous "Test hotspot" — wst AT-CORE : 0 idle 1 connexion
// 2 OK 3 SSID absent 4 échec. wst est sticky côté AT-CORE (dernier résultat) →
// "WiFi connected" reste affiché après un test/upload réussi.
static void maintWifiStatus(){
    if(!g_maint_wst)return;
    char b[80]; lv_color_t c;
    // SSID enregistré côté BOÎTIER (STATUS "wss") = source de vérité du dernier hotspot
    // validé/en mémoire ; fallback sur le NVS de l'écran si le boîtier n'a rien renvoyé.
    const char* ssid = (g_status.valid && g_status.wssid[0]) ? g_status.wssid
                     : (g_hs_ssid[0] ? g_hs_ssid : "");
    if(!g_status.valid){ b[0]=0; c=TGREY(); }
    else if(!ssid[0]){ strcpy(b,"No hotspot configured"); c=TGREY(); }
    else switch(g_status.wst){
        case 1: snprintf(b,sizeof(b),"%s : connecting...",ssid);      c=C_AMBER; break;
        case 2: snprintf(b,sizeof(b),"%s : connected (%s)",ssid,
                         g_status.wip[0]?g_status.wip:"?");           c=C_GREEN; break;
        case 3: snprintf(b,sizeof(b),"%s : not found",ssid);         c=C_RED;   break;
        case 4: snprintf(b,sizeof(b),"%s : connect failed",ssid);    c=C_RED;   break;
        default: snprintf(b,sizeof(b),"Hotspot: %s",ssid);           c=TGREY(); break;   // dernier validé/en mémoire
    }
    lv_label_set_text(g_maint_wst,b);
    lv_obj_set_style_text_color(g_maint_wst,c,0);
}
static void _maint_close(){
    if(!g_maint_ov)return;
    lv_obj_del(g_maint_ov);   // supprime aussi le panneau scan (enfant)
    g_maint_ov=nullptr;g_maint_ssid_ta=nullptr;g_maint_pass_ta=nullptr;g_maint_kb=nullptr;
    g_maint_scanlist=nullptr;g_maint_upd=nullptr;g_maint_wst=nullptr;}
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
static lv_obj_t* g_vols_wifi=nullptr;   // ligne état WiFi club (SSID + connexion/IP)
static bool     g_vols_loading=false, g_vols_del_armed=false, g_vols_clean_armed=false;
static bool     g_vols_xfer_pending=false, g_vols_xfer_seen3=false;  // suivi transfert sur la page
static uint32_t g_vols_t0=0, g_vols_xfer_t0=0, g_vols_xfer_prog_ms=0;
static uint8_t  g_vols_xfer_lastpct=0, g_vols_xfer_lastph=0xFF;   // anti-stall (v7)

static void volsClose(){
    if(!g_vols_ov)return;
    lv_obj_del(g_vols_ov);
    g_vols_ov=nullptr;g_vols_list=nullptr;g_vols_load=nullptr;g_vols_wifi=nullptr;
    g_vols_loading=false;g_vols_del_armed=false;g_vols_clean_armed=false;g_vols_xfer_pending=false;g_vols_n=0;}

// Ligne d'état WiFi hotspot (page Flights) : SSID configuré + état de connexion + IP.
// Appelée à chaque STATUS quand la page est ouverte → l'utilisateur voit si le box est
// bien connecté et à quel réseau (cause n°1 d'échec : hotspot iPhone éteint).
static void volsUpdWifi(){
    if(!g_vols_wifi)return;
    const char* ss = g_hs_ssid[0]?g_hs_ssid:"(no WiFi)";
    char b[48]; lv_color_t c=TGREY();
    switch(g_status.wst){
        case 1: snprintf(b,sizeof(b),"WiFi: connecting to %s...",ss); c=C_AMBER; break;
        case 2: snprintf(b,sizeof(b),"WiFi OK: %s  %s",ss,g_status.wip); c=C_GREEN; break;
        case 3: snprintf(b,sizeof(b),"WiFi: %s NOT FOUND (off?)",ss); c=C_RED; break;
        case 4: snprintf(b,sizeof(b),"WiFi: %s connection failed",ss); c=C_RED; break;
        default: snprintf(b,sizeof(b),"WiFi: %s (idle)",ss); c=TGREY(); break;
    }
    lv_label_set_text(g_vols_wifi,b);
    lv_obj_set_style_text_color(g_vols_wifi,c,0);}
static void _vols_close_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED)volsClose(); }

// (liste en lecture seule depuis la refonte v33 : plus de compteur "Transfer (N)")

// Remplace la liste par un message d'état (transfert/suppression en cours, on reste
// sur la page). g_vols_load réutilisé comme label, g_vols_n=0 (plus de lignes).
static void volsShowStatus(const char* msg, lv_color_t col){
    if(!g_vols_list)return;
    lv_obj_clean(g_vols_list); g_vols_n=0;
    g_vols_load=lv_label_create(g_vols_list);
    lv_label_set_text(g_vols_load,msg);
    lv_obj_set_style_text_color(g_vols_load,col,0);
    lv_obj_set_style_text_font(g_vols_load,&lv_font_montserrat_16,0);
    // Roue qui tourne — montre que le process tourne en arrière-plan (transfert lent).
    lv_obj_t*sp=lv_spinner_create(g_vols_list,900,55);
    lv_obj_set_size(sp,44,44);
    lv_obj_set_style_arc_color(sp,col,LV_PART_INDICATOR);}

// Arme le suivi de progression d'un transfert (consommé dans le loop : STATUS up_pct/flt_ph).
static void volsArmXferTracking(){
    g_vols_xfer_pending=true; g_vols_xfer_seen3=false; g_vols_xfer_t0=millis();
    g_vols_xfer_prog_ms=millis(); g_vols_xfer_lastpct=0; g_vols_xfer_lastph=0xFF;}

// "Send last" → {"cmd":"upload"} : transfère le vol de la session courante (le dernier volé,
// exclu de la liste SD tant que TaskSD garde son CSV ouvert).
static void _vols_sendlast_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    sendCtl("upload");
    volsShowStatus("Sending last flight...",C_AMBER);
    volsArmXferTracking();
}
// "Send all unsent" → {"cmd":"uploadall"} : scan SD + transfert de tous les non-envoyés (AT-CORE v26).
static void _vols_sendall_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int unsent=0; for(int i=0;i<g_vols_n;i++) if(!g_vols[i].up)unsent++;
    if(!unsent){ volsShowStatus("No unsent flights on SD",TGREY()); return; }
    sendCtl("uploadall");
    volsShowStatus("Sending all unsent...",C_AMBER);
    volsArmXferTracking();
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
// (Clean empty retiré de l'UI : les CSV header-seul sont déjà masqués de la liste et
//  nettoyés automatiquement au boot côté AT-CORE — plus de bouton manuel.)

// Lit CHR_FLIGHTS, parse le JSON, construit les lignes.
static void volsBuildList(){
    if(!g_chrFl||!g_vols_list)return;
    if(g_vols_load){lv_obj_add_flag(g_vols_load,LV_OBJ_FLAG_HIDDEN);}
    std::string v=bleStr(g_chrFl->readValue());
    JsonDocument d; if(deserializeJson(d,v.c_str()))return;
    JsonArray arr=d.as<JsonArray>();
    lv_obj_clean(g_vols_list); g_vols_n=0;
#ifdef BOARD_T4S3
    const int ROWH=40; const lv_font_t* RF=&lv_font_montserrat_16;   // (v10) lignes plus grandes (T4)
#else
    const int ROWH=30; const lv_font_t* RF=&lv_font_montserrat_14;
#endif
    for(JsonObject o:arr){
        if(g_vols_n>=16)break;
        VolItem& it=g_vols[g_vols_n];
        strlcpy(it.fid,o["f"]|"",sizeof(it.fid));
        strlcpy(it.d,o["d"]|"?",sizeof(it.d));
        strlcpy(it.s,o["s"]|"?",sizeof(it.s));
        strlcpy(it.e,o["e"]|"?",sizeof(it.e));
        it.up=o["u"]|0; it.sel=false;
        // Ligne LECTURE SEULE (plus de sélection) : date + heures, "✓ sent" vert si transféré.
        lv_obj_t*b=lv_obj_create(g_vols_list);lv_obj_set_size(b,LV_PCT(100),ROWH);   // pleine largeur liste
        lv_obj_set_style_radius(b,6,0);lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_pad_all(b,0,0);
        lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(b,it.up?lv_color_hex(0x161b22):lv_color_hex(0x21262d),0);
        const char* md=strlen(it.d)>=10?it.d+5:it.d;
        char r[52];
        if(it.up) snprintf(r,sizeof(r),"%s  %s>%s   " LV_SYMBOL_OK " sent",md,it.s,it.e);
        else      snprintf(r,sizeof(r),"%s  %s>%s",md,it.s,it.e);
        lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,r);
        lv_obj_set_style_text_color(l,it.up?C_GREEN:lv_color_hex(0xe6edf3),0);
        lv_obj_set_style_text_font(l,RF,0);lv_obj_center(l);
        it.lbl=l; it.row=b;
        g_vols_n++;
    }
    if(g_vols_n==0){
        lv_obj_t*l=lv_label_create(g_vols_list);lv_label_set_text(l,"No flights on SD");
        lv_obj_set_style_text_color(l,TGREY(),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
    }}

// Bouton de la page Flights (helper factorisé : 4 boutons identiques sauf libellé/couleur/cb).
// Couleur d'une version selon son canal : -dev=ambre, -rc=bleu, sinon (client validé)=vert.
static lv_color_t verColor(const char* s){
    if(!s||!s[0]) return C_BRAND;
    if(strstr(s,"-dev")) return C_AMBER;
    if(strstr(s,"-rc"))  return C_CYAN;
    return C_GREEN;
}
static lv_obj_t* volBtn(lv_obj_t*par,const char*txt,lv_color_t bg,int w,int h,int dx,int y,lv_event_cb_t cb,const lv_font_t*f){
    lv_obj_t*b=lv_btn_create(par);lv_obj_set_size(b,w,h);lv_obj_align(b,LV_ALIGN_TOP_MID,dx,y);
    lv_obj_set_style_bg_color(b,bg,0);lv_obj_set_style_radius(b,8,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,txt);
    lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(l,f,0);lv_obj_center(l);
    return b;}

void mkVolsOverlay(){
    if(g_vols_ov)return;
    // Géométrie par carte. T4-S3 : liste + 4 boutons (Send last / Send all unsent / Delete sent /
    // Close) en 2×2. T-RGB rond : empilés (layout à raffiner au port T-RGB).
#ifdef BOARD_T4S3
    const int OVW=600, OVX=0,     LSTW=564, LSTH=206, LSTY=78;
    const int yTitle=24, yWifi=54;
    const int BW=276, BH=46, dxL=-148, dxR=148, yR1=296, yR2=350;   // 2×2 (T4)
    const int xSL=dxL,ySL=yR1, xSA=dxR,ySA=yR1, xDEL=dxL,yDEL=yR2, xCL=dxR,yCL=yR2;
    const lv_font_t *FTL=&lv_font_montserrat_22, *FBT=&lv_font_montserrat_18, *FB2=&lv_font_montserrat_16;
#else
    const int OVW=480, OVX=UI_OX, LSTW=290, LSTH=180, LSTY=72;
    const int yTitle=26, yWifi=50;
    const int BW=210, BH=32;   // empilés (rond)
    const int xSL=0,ySL=258, xSA=0,ySA=296, xDEL=0,yDEL=334, xCL=0,yCL=372;
    const lv_font_t *FTL=&lv_font_montserrat_20, *FBT=&lv_font_montserrat_14, *FB2=&lv_font_montserrat_14;
#endif
    g_vols_ov=lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_vols_ov,OVW,480);lv_obj_set_pos(g_vols_ov,OVX,UI_OY);
    lv_obj_set_style_bg_color(g_vols_ov,TBG(),0);lv_obj_set_style_bg_opa(g_vols_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_vols_ov,0,0);lv_obj_set_style_radius(g_vols_ov,0,0);
    lv_obj_set_style_pad_all(g_vols_ov,0,0);lv_obj_clear_flag(g_vols_ov,LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t*tl=lv_label_create(g_vols_ov);lv_label_set_text(tl,"FlightLogs (UTC)");
    lv_obj_set_style_text_color(tl,C_AMBER,0);lv_obj_set_style_text_font(tl,FTL,0);
    lv_obj_align(tl,LV_ALIGN_TOP_MID,0,yTitle);

    g_vols_list=lv_obj_create(g_vols_ov);
    lv_obj_set_size(g_vols_list,LSTW,LSTH);lv_obj_align(g_vols_list,LV_ALIGN_TOP_MID,0,LSTY);
    lv_obj_set_style_bg_color(g_vols_list,lv_color_hex(0x0d1117),0);
    lv_obj_set_style_border_width(g_vols_list,0,0);lv_obj_set_style_pad_all(g_vols_list,4,0);
    lv_obj_set_flex_flow(g_vols_list,LV_FLEX_FLOW_COLUMN);
    g_vols_load=lv_label_create(g_vols_list);lv_label_set_text(g_vols_load,"Loading...");
    lv_obj_set_style_text_color(g_vols_load,TGREY(),0);lv_obj_set_style_text_font(g_vols_load,FB2,0);

    // Boutons : Send last (vol courant) · Send all unsent (scan SD) · Delete sent (.up, 2-tap) · Close
    volBtn(g_vols_ov,"Send last",      C_GREEN,                BW,BH,xSL, ySL, _vols_sendlast_cb,FBT);
    volBtn(g_vols_ov,"Send all unsent",C_BRAND,                BW,BH,xSA, ySA, _vols_sendall_cb, FBT);
    volBtn(g_vols_ov,"Delete sent",    lv_color_hex(0x4b5563), BW,BH,xDEL,yDEL,_vols_del_cb,     FB2);
    volBtn(g_vols_ov,"Close",          lv_color_hex(0x30363d), BW,BH,xCL, yCL, _vols_close_cb,   FB2);

    // Ligne d'état WiFi club — visibilité connexion pendant le transfert (sous le titre sur T4).
    g_vols_wifi=lv_label_create(g_vols_ov);
    lv_obj_set_style_text_font(g_vols_wifi,&lv_font_montserrat_14,0);
    lv_obj_align(g_vols_wifi,LV_ALIGN_TOP_MID,0,yWifi);
    volsUpdWifi();

    // Demande la liste. On seed flt_rdy=0 localement : l'AT-CORE le met aussi à 0
    // à la réception puis 1 quand la liste est prête. Le poll (loop) attend
    // flt_rdy==1 + ≥1.5 s écoulées (couvre un STATUS périmé encore en transit).
    sendCtl("flights");
    g_status.flt_rdy=0;
    g_vols_loading=true;g_vols_t0=millis();}

static void _open_vols_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(!g_vols_ov)mkVolsOverlay();}

// ── (2026-06-25) Cloud-pull OTA de l'AT-VIEW : 1-bouton, sans câble ni téléphone ──────
// L'écran télécharge SON firmware depuis Firebase Storage (public-read firmware/atv/<tag>/)
// via le WiFi hotspot/club configuré (g_hs_ssid), flashe (Update) et reboote. Tag = t4s3/trgb
// (binaires distincts). setInsecure (MITM non couvert, acceptable MVP). PAS de rollback :
// écrans actuels récupérables en USB (le rollback viendra avec l'Anders scellé, Phase D).
// Bloquant (LVGL figé pendant le download) → overlay rafraîchi par lv_refr_now().
#define ATV_STORAGE_HOST   "firebasestorage.googleapis.com"
#define ATV_STORAGE_BUCKET "aerotrace-74217.firebasestorage.app"
#ifdef BOARD_T4S3
  #define ATV_OTA_TAG "t4s3"
#else
  #define ATV_OTA_TAG "trgb"
#endif
static lv_obj_t* g_atvota_ov=nullptr,*g_atvota_lbl=nullptr;
static void atvOtaShow(const char* m, lv_color_t c){
    if(!g_atvota_ov){
        g_atvota_ov=lv_obj_create(lv_layer_top());
        lv_obj_set_size(g_atvota_ov,440,180);lv_obj_center(g_atvota_ov);
        lv_obj_set_style_bg_color(g_atvota_ov,lv_color_hex(0x0d1117),0);lv_obj_set_style_bg_opa(g_atvota_ov,LV_OPA_COVER,0);
        lv_obj_set_style_border_color(g_atvota_ov,C_BRAND,0);lv_obj_set_style_border_width(g_atvota_ov,2,0);
        lv_obj_set_style_radius(g_atvota_ov,12,0);lv_obj_clear_flag(g_atvota_ov,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t*t=lv_label_create(g_atvota_ov);lv_label_set_text(t,"AT-VIEW UPDATE");
        lv_obj_set_style_text_color(t,C_BRAND,0);lv_obj_set_style_text_font(t,&lv_font_montserrat_20,0);lv_obj_align(t,LV_ALIGN_TOP_MID,0,22);
        g_atvota_lbl=lv_label_create(g_atvota_ov);lv_obj_set_style_text_font(g_atvota_lbl,&lv_font_montserrat_16,0);lv_obj_align(g_atvota_lbl,LV_ALIGN_CENTER,0,10);
    }
    lv_label_set_text(g_atvota_lbl,m);lv_obj_set_style_text_color(g_atvota_lbl,c,0);
    lv_refr_now(NULL);
}
static void atvOtaHide(){ if(g_atvota_ov){lv_obj_del(g_atvota_ov);g_atvota_ov=nullptr;g_atvota_lbl=nullptr;} }
static void atvCloudOta(){
    char b[64];
    if(!g_hs_ssid[0]){ atvOtaShow("No WiFi configured (Maintenance)",C_RED); delay(2500); atvOtaHide(); return; }
    atvOtaShow("Connecting WiFi...",lv_color_hex(0xffffff));
    WiFi.mode(WIFI_STA); WiFi.begin(g_hs_ssid,(char*)g_hs_pass);
    uint32_t t0=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-t0<15000) delay(200);
    if(WiFi.status()!=WL_CONNECTED){ atvOtaShow("WiFi connect failed",C_RED); delay(2500); atvOtaHide(); WiFi.mode(WIFI_OFF); return; }
    Serial.printf("[ATVOTA] WiFi OK ip=%s rssi=%d heap=%u\n",WiFi.localIP().toString().c_str(),(int)WiFi.RSSI(),(unsigned)ESP.getFreeHeap());
    // Le Bluedroid BLE de l'écran ne laisse que ~40 KB de heap → le TLS handshake échoue
    // (GET=-1). On LIBÈRE le BLE avant le TLS (+50-80 KB). ⚠️ Après ce deinit le BLE est mort
    // → TOUTE sortie doit REBOOTER (le boot ré-init le BLE). On reboote de toute façon en fin d'OTA.
    BLEDevice::deinit(true); delay(150);
    Serial.printf("[ATVOTA] BLE deinit, heap=%u\n",(unsigned)ESP.getFreeHeap());
    WiFiClientSecure client; client.setInsecure(); client.setHandshakeTimeout(20);
    char url[200];
    // 1) version
    snprintf(url,sizeof(url),"https://%s/v0/b/%s/o/firmware%%2Fatv%%2F%s%%2Fversion.txt?alt=media",ATV_STORAGE_HOST,ATV_STORAGE_BUCKET,ATV_OTA_TAG);
    int remote=-1; { HTTPClient http; http.setConnectTimeout(10000); http.setTimeout(15000);
        bool bg=http.begin(client,url); int code=bg?http.GET():-999;
        Serial.printf("[ATVOTA] ver begin=%d GET=%d heap=%u\n",(int)bg,code,(unsigned)ESP.getFreeHeap());
        if(code==200) remote=http.getString().toInt(); http.end(); }
    int local=atoi(VIEW_VERSION);
    if(remote<0){ atvOtaShow("Version check failed - reboot",C_RED); delay(2500); ESP.restart(); }
    if(remote<=local){ snprintf(b,sizeof(b),"Already up to date (v%d) - reboot",local); atvOtaShow(b,C_GREEN); delay(2500); ESP.restart(); }
    // 2) download + flash
    snprintf(b,sizeof(b),"Downloading v%d...",remote); atvOtaShow(b,C_AMBER);
    snprintf(url,sizeof(url),"https://%s/v0/b/%s/o/firmware%%2Fatv%%2F%s%%2Ffirmware.bin?alt=media",ATV_STORAGE_HOST,ATV_STORAGE_BUCKET,ATV_OTA_TAG);
    HTTPClient h2; h2.setConnectTimeout(10000); h2.setTimeout(20000);
    if(!h2.begin(client,url) || h2.GET()!=200){ atvOtaShow("Download failed - reboot",C_RED); h2.end(); delay(2500); ESP.restart(); }
    int total=h2.getSize();
    if(!Update.begin(total>0?total:UPDATE_SIZE_UNKNOWN)){ atvOtaShow("Update.begin failed - reboot",C_RED); h2.end(); delay(2500); ESP.restart(); }
    WiFiClient* st=h2.getStreamPtr(); uint8_t buf[1024]; size_t written=0; bool hok=false; uint32_t last=millis(); int lastpct=-1;
    while(h2.connected() && (total<=0 || (int)written<total)){
        size_t av=st->available();
        if(av){ int n=st->readBytes(buf,av>sizeof(buf)?sizeof(buf):av); if(n<=0)break;
            if(!hok && n>=13){ hok=true; if(buf[0]!=0xE9 || buf[12]!=0x09){ atvOtaShow("Bad image - reboot",C_RED); Update.abort(); h2.end(); delay(2500); ESP.restart(); } }
            if(Update.write(buf,n)!=(size_t)n){ atvOtaShow("Flash write error - reboot",C_RED); Update.abort(); h2.end(); delay(2500); ESP.restart(); }
            written+=n; last=millis();
            if(total>0){ int pct=(int)((uint32_t)written*100/total); if(pct!=lastpct && pct%5==0){ lastpct=pct; snprintf(b,sizeof(b),"Flashing v%d... %d%%",remote,pct); atvOtaShow(b,C_AMBER); } }
        } else { if(millis()-last>20000){ atvOtaShow("Download timeout - reboot",C_RED); Update.abort(); h2.end(); delay(2500); ESP.restart(); } delay(5); }
    }
    h2.end();
    if(!Update.end(true)){ atvOtaShow("Flash failed - reboot",C_RED); delay(2500); ESP.restart(); }
    atvOtaShow("Update OK - rebooting",C_GREEN); delay(2000); ESP.restart();
}
// "Update ATV" (cloud-pull self-OTA) — double-tap de confirmation (reflash + reboot écran).
static bool g_maint_atvota_armed=false;
static void _maint_atvota_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_maint_atvota_armed){ g_maint_atvota_armed=true; if(l)lv_label_set_text(l,"Confirm?"); lv_obj_set_style_bg_color(b,C_AMBER,0); return; }
    g_maint_atvota_armed=false; if(l)lv_label_set_text(l,"Update ATV");
    atvCloudOta();
}

// ── Page UPDATES (SYSTEM) : versions ATC/ATV + MAJ contextuelle ──────────────────
// ATC : bouton [Update] affiché SEULEMENT si une MAJ cloud existe (g_status.oav>fwv),
//       sinon ligne grise "up to date". ATV : bouton toujours présent (auto-check à
//       l'exécution → "Already up to date" si rien de neuf ; le check version ATV côté
//       écran viendra avec la Phase D). MAJ = single tap (l'OTA self-check, no-op si à jour).
static lv_obj_t* g_upd_ov=nullptr;
static lv_obj_t* g_upd_checkbtn=nullptr;   // bouton "Check now" (relabel pendant le check)
static uint8_t   g_upd_shown_oav=0;        // oav au moment du build de la page → rebuild si change
static bool      g_upd_checking=false; static uint32_t g_upd_check_t0=0;
static void showUpdatesPage();             // fwd (rebuild auto depuis updateAllPages)
static void _upd_close_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED&&g_upd_ov){g_upd_checking=false;lv_obj_del(g_upd_ov);g_upd_ov=nullptr;g_upd_checkbtn=nullptr;} }
static void _upd_atc_cb(lv_event_t*e){ if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return; g_upd_checking=false; if(g_upd_ov){lv_obj_del(g_upd_ov);g_upd_ov=nullptr;g_upd_checkbtn=nullptr;} sendCtl("otaupdate"); }
static void _upd_atv_cb(lv_event_t*e){ if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return; g_upd_checking=false; if(g_upd_ov){lv_obj_del(g_upd_ov);g_upd_ov=nullptr;g_upd_checkbtn=nullptr;} atvCloudOta(); }
// "Check now" → force le boîtier à re-vérifier Storage ({"cmd":"otacheck"}) → oav rafraîchi en
// ~5-10 s (WiFi club + GET) → la page se rebuild auto (updateAllPages) quand oav change.
static void _upd_check_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    sendCtl("otacheck");
    g_upd_checking=true; g_upd_check_t0=millis();
    lv_obj_t*b=lv_event_get_target(e),*l=lv_obj_get_child(b,0);
    if(l)lv_label_set_text(l,"Checking..."); lv_obj_set_style_bg_color(b,C_AMBER,0);
}
static void showUpdatesPage(){
    if(g_upd_ov)return;
    g_upd_ov=lv_obj_create(lv_scr_act());
    g_upd_shown_oav=g_status.oav; g_upd_checking=false;   // suivi rebuild auto (Check now)
#ifdef BOARD_T4S3
    lv_obj_set_size(g_upd_ov,600,480);lv_obj_set_pos(g_upd_ov,0,UI_OY);
    const lv_font_t *TF=&lv_font_montserrat_22,*LF=&lv_font_montserrat_20,*BF=&lv_font_montserrat_18;
    const int xLbl=70,xVal=250,bDx=185,BW=150,BH=46,yATC=148,yATV=222;
    const int xCHK=-130,yCHK=400, xCL=130,yCL=400;   // Check now (gauche) · Close (droite)
#else
    lv_obj_set_size(g_upd_ov,480,480);lv_obj_set_pos(g_upd_ov,UI_OX,UI_OY);
    const lv_font_t *TF=&lv_font_montserrat_18,*LF=&lv_font_montserrat_16,*BF=&lv_font_montserrat_14;
    const int xLbl=50,xVal=175,bDx=150,BW=120,BH=40,yATC=148,yATV=210;
    const int xCHK=0,yCHK=334, xCL=0,yCL=378;        // empilés (rond)
#endif
    lv_obj_set_style_bg_color(g_upd_ov,TBG(),0);lv_obj_set_style_bg_opa(g_upd_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_upd_ov,0,0);lv_obj_set_style_radius(g_upd_ov,0,0);
    lv_obj_set_style_pad_all(g_upd_ov,0,0);lv_obj_clear_flag(g_upd_ov,LV_OBJ_FLAG_SCROLLABLE);
    mkLbl(g_upd_ov,"UPDATES",C_AMBER,TF,LV_ALIGN_TOP_MID,0,44);

    // AT-CORE — contextuel (oav>fwv)
    mkLblP(g_upd_ov,"AT-CORE",TFG(),LF,xLbl,yATC+8);
    char ab[40];
    if(g_status.valid && g_status.oav>g_status.fwv){
        snprintf(ab,sizeof(ab),"v%d " LV_SYMBOL_RIGHT " v%d",g_status.fwv,g_status.oav);
        mkLblP(g_upd_ov,ab,C_AMBER,LF,xVal,yATC+8);
        volBtn(g_upd_ov,"Update",C_BRAND,BW,BH,bDx,yATC,_upd_atc_cb,BF);
    }else{
        if(g_status.valid && g_status.fws[0]) snprintf(ab,sizeof(ab),"%s  up to date",g_status.fws);
        else if(g_status.valid)               snprintf(ab,sizeof(ab),"v%d  up to date",g_status.fwv);
        else                                  strlcpy(ab,"v--  (offline)",sizeof(ab));
        mkLblP(g_upd_ov,ab,(g_status.valid&&g_status.fws[0])?verColor(g_status.fws):TGREY(),LF,xVal,yATC+8);
    }

    // AT-VIEW — bouton toujours présent (auto-check à l'exécution)
    mkLblP(g_upd_ov,"AT-VIEW",TFG(),LF,xLbl,yATV+8);
    mkLblP(g_upd_ov,VIEW_VSTR,verColor(VIEW_VSTR),LF,xVal,yATV+8);
    volBtn(g_upd_ov,"Update",C_BRAND,BW,BH,bDx,yATV,_upd_atv_cb,BF);

    // Check now (force re-check Storage côté ATC) + Close
    g_upd_checkbtn=volBtn(g_upd_ov,"Check now",lv_color_hex(0x1f4068),BW,BH,xCHK,yCHK,_upd_check_cb,BF);
    volBtn(g_upd_ov,"Close",lv_color_hex(0x4b5563),BW,BH,xCL,yCL,_upd_close_cb,BF);
}
static void _open_updates_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) showUpdatesPage(); }

// ── Page DIAGNOSTIC (SYSTEM) : état WiFi club/SD + Test WiFi / Reboot box ──
// Reboot box = 2-tap (destructif). Test WiFi = single tap (état visible sur la ligne WiFi live).
// (AP écran retiré : push navigateur legacy, remplacé par le cloud-pull "Update ATV" + USB.)
static lv_obj_t* g_diag_ov=nullptr,*g_diag_wst=nullptr;
static bool g_diag_reboot_armed=false;
static void diagWifiStatus(){
    if(!g_diag_wst)return;
    char b[80]; lv_color_t c;
    const char* ssid=(g_status.valid&&g_status.wssid[0])?g_status.wssid:(g_hs_ssid[0]?g_hs_ssid:"");
    if(!g_status.valid){ strcpy(b,"box offline (no BLE)"); c=TGREY(); }
    else if(!ssid[0]){ strcpy(b,"No club WiFi configured"); c=TGREY(); }
    else switch(g_status.wst){
        case 1: snprintf(b,sizeof(b),"%s : connecting...",ssid); c=C_AMBER; break;
        case 2: snprintf(b,sizeof(b),"%s : connected (%s)",ssid,g_status.wip[0]?g_status.wip:"?"); c=C_GREEN; break;
        case 3: snprintf(b,sizeof(b),"%s : not found",ssid); c=C_RED; break;
        case 4: snprintf(b,sizeof(b),"%s : connect failed",ssid); c=C_RED; break;
        default: snprintf(b,sizeof(b),"club WiFi: %s",ssid); c=TGREY(); break;
    }
    lv_label_set_text(g_diag_wst,b); lv_obj_set_style_text_color(g_diag_wst,c,0);
}
static void _diag_close_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED&&g_diag_ov){lv_obj_del(g_diag_ov);g_diag_ov=nullptr;g_diag_wst=nullptr;} }
static void _diag_test_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("wifitest"); }
static void _diag_reboot_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_diag_reboot_armed){ g_diag_reboot_armed=true; if(l)lv_label_set_text(l,"Confirm?"); lv_obj_set_style_bg_color(b,C_AMBER,0); return; }
    g_diag_reboot_armed=false; if(l)lv_label_set_text(l,"Reboot box"); lv_obj_set_style_bg_color(b,C_ORANGE,0);
    sendCtl("reboot");
}
// Unpair box (2-tap) : casse le lien BLE écran↔boîtier. Si connecté → {"cmd":"unpair"}
// ré-arme le pairing côté boîtier (sinon il refuserait un nouveau lien), puis on oublie
// la MAC côté écran + reboot → cérémonie de pairing. Si boîtier hors-ligne, on ne peut
// que l'oublier localement (le boîtier reste lié/pairable=0 → power-cycle requis) :
// feedback honnête plutôt que laisser croire que c'est fait des 2 côtés.
static bool g_diag_unpair_armed=false;
static void _diag_unpair_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_diag_unpair_armed){ g_diag_unpair_armed=true; if(l)lv_label_set_text(l,"Confirm unpair?"); lv_obj_set_style_bg_color(b,C_AMBER,0); return; }
    g_diag_unpair_armed=false;
    if(g_connected){
        sendCtl("unpair");                              // ré-arme le pairing boîtier
        if(l)lv_label_set_text(l,"Unpairing - reboot");
        delay(200);
    }else{
        if(l)lv_label_set_text(l,"Box offline - local only");
        lv_obj_set_style_bg_color(b,C_RED,0);
        lv_refr_now(NULL); delay(1400);                 // laisse lire l'avertissement
    }
    unitForgetMac();                                    // efface paired_mac côté écran
    delay(400); ESP.restart();
}
static void showDiagPage(){
    g_diag_reboot_armed=false; g_diag_unpair_armed=false;   // ré-arme propre à chaque ouverture
    if(g_diag_ov)return;
    g_diag_ov=lv_obj_create(lv_scr_act());
#ifdef BOARD_T4S3
    lv_obj_set_size(g_diag_ov,600,480);lv_obj_set_pos(g_diag_ov,0,UI_OY);
    const lv_font_t *TF=&lv_font_montserrat_22,*LF=&lv_font_montserrat_18,*BF=&lv_font_montserrat_18;
    const int BW=240,BH=48,yWifi=120,ySD=164;
    const int xTEST=-130,yTEST=232, xREB=130,yREB=232, xUNP=-130,yUNP=300, xCL=130,yCL=300;
#else
    lv_obj_set_size(g_diag_ov,480,480);lv_obj_set_pos(g_diag_ov,UI_OX,UI_OY);
    const lv_font_t *TF=&lv_font_montserrat_18,*LF=&lv_font_montserrat_14,*BF=&lv_font_montserrat_14;
    const int BW=200,BH=36,yWifi=96,ySD=132;
    const int xTEST=0,yTEST=176, xREB=0,yREB=216, xUNP=0,yUNP=256, xCL=0,yCL=296;
#endif
    lv_obj_set_style_bg_color(g_diag_ov,TBG(),0);lv_obj_set_style_bg_opa(g_diag_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_diag_ov,0,0);lv_obj_set_style_radius(g_diag_ov,0,0);
    lv_obj_set_style_pad_all(g_diag_ov,0,0);lv_obj_clear_flag(g_diag_ov,LV_OBJ_FLAG_SCROLLABLE);
    mkLbl(g_diag_ov,"DIAGNOSTIC",C_AMBER,TF,LV_ALIGN_TOP_MID,0,44);

    g_diag_wst=lv_label_create(g_diag_ov);lv_obj_set_style_text_font(g_diag_wst,LF,0);
    lv_obj_align(g_diag_wst,LV_ALIGN_TOP_MID,0,yWifi); diagWifiStatus();

    char sd[24]; if(g_sd_ok)snprintf(sd,sizeof(sd),"SD card: %u GB",g_sd_gb);else strlcpy(sd,"SD card: NO CARD",sizeof(sd));
    mkLbl(g_diag_ov,sd,g_sd_ok?C_GREEN:C_RED,LF,LV_ALIGN_TOP_MID,0,ySD);

    volBtn(g_diag_ov,"Test WiFi",lv_color_hex(0x1f4068),BW,BH,xTEST,yTEST,_diag_test_cb,BF);
    volBtn(g_diag_ov,"Reboot box",C_ORANGE,BW,BH,xREB,yREB,_diag_reboot_cb,BF);
    volBtn(g_diag_ov,"Unpair box",lv_color_hex(0x7c3aed),BW,BH,xUNP,yUNP,_diag_unpair_cb,BF);
    volBtn(g_diag_ov,"Close",lv_color_hex(0x4b5563),BW,BH,xCL,yCL,_diag_close_cb,BF);
}
static void _open_diag_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) showDiagPage(); }

// OTA cloud-pull : double-tap de confirmation (action destructive = reflash + reboot).
static bool g_maint_ota_armed=false;
static void _maint_ota_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_maint_ota_armed){
        g_maint_ota_armed=true;
        if(l)lv_label_set_text(l,"Confirm update?");
        lv_obj_set_style_bg_color(b,C_AMBER,0);
        return;
    }
    g_maint_ota_armed=false;
    if(l)lv_label_set_text(l,"Update firmware");
    sendCtl("otaupdate");   // AT-CORE : connecte hotspot → check version → download → flash
}

// (v22-C) Reboot LOGICIEL du boîtier (scellé/inaccessible) : double-tap → {"cmd":"reboot"}.
// Le SIM7600 n'est pas power-cyclé → almanac GPS préservé → re-fix rapide. Sert de filet
// quand le pilote voit un blocage et ne peut pas atteindre le bouton RST physique.
static bool g_maint_reboot_armed=false;
static void _maint_reboot_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_maint_reboot_armed){
        g_maint_reboot_armed=true;
        if(l)lv_label_set_text(l,"Confirm reboot?");
        lv_obj_set_style_bg_color(b,C_AMBER,0);
        return;
    }
    g_maint_reboot_armed=false;
    if(l)lv_label_set_text(l,"Reboot box");
    sendCtl("reboot");
}

// (v20) WiFi Setup — demande au boîtier d'ouvrir son portail WiFi ({"cmd":"portal"}),
// puis affiche au pilote comment s'y connecter depuis son téléphone (saisie identité
// aéronef au clavier smartphone + creds hotspot + OTA navigateur). Le SSID est
// ATCORE-SETUP-<box> où <box> = box-id reçu en STATUS (aligné v23 boîtier). Le boîtier
// reboote après Save côté portail ; sinon le pilote peut "Reboot box" pour annuler.
static lv_obj_t* g_wifisetup_ov=nullptr;
static void _wifisetup_close_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(g_wifisetup_ov){lv_obj_del(g_wifisetup_ov);g_wifisetup_ov=nullptr;}}
// "Open portal" (single tap) → ouvre le SoftAP ATCORE-SETUP-<box> du boîtier ({"cmd":"portal"}).
// Non destructif (le boîtier ne reboote qu'après Save côté web) → pas de confirm : feedback
// immédiat (bouton vert "Portal open") puis le BLE tombe quand le boîtier passe en AP.
static void _wifisetup_portal_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_connected){ if(l)lv_label_set_text(l,"Box not connected"); lv_obj_set_style_bg_color(b,C_RED,0); return; }   // sinon la commande part dans le vide
    sendCtl("portal");
    if(l)lv_label_set_text(l,"Portal requested");
    lv_obj_set_style_bg_color(b,C_GREEN,0);
}
static void showWifiSetupInfo(){
    if(g_wifisetup_ov)return;
    g_wifisetup_ov=lv_obj_create(lv_scr_act());
#ifdef BOARD_T4S3
    lv_obj_set_size(g_wifisetup_ov,600,480);lv_obj_set_pos(g_wifisetup_ov,0,UI_OY);
    const lv_font_t* TF=&lv_font_montserrat_22; const lv_font_t* BF=&lv_font_montserrat_18;
#else
    lv_obj_set_size(g_wifisetup_ov,480,480);lv_obj_set_pos(g_wifisetup_ov,UI_OX,UI_OY);
    const lv_font_t* TF=&lv_font_montserrat_18; const lv_font_t* BF=&lv_font_montserrat_14;
#endif
    lv_obj_set_style_bg_color(g_wifisetup_ov,TBG(),0);lv_obj_set_style_bg_opa(g_wifisetup_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_wifisetup_ov,0,0);lv_obj_set_style_radius(g_wifisetup_ov,0,0);
    lv_obj_set_style_pad_all(g_wifisetup_ov,0,0);lv_obj_clear_flag(g_wifisetup_ov,LV_OBJ_FLAG_SCROLLABLE);
    // Tout centré (LV_ALIGN_TOP_MID) → reste dans le disque sur l'écran rond (un x fixe
    // à 60 sortirait du cercle en haut) ET sur le rectangle T4.
    char ssid[28];
    snprintf(ssid,sizeof(ssid),"ATCORE-SETUP-%s",
             (g_status.valid&&g_status.box[0])?g_status.box:"----");
    mkLbl(g_wifisetup_ov,"WiFi SETUP",C_AMBER,TF,LV_ALIGN_TOP_MID,0,40);
    mkLbl(g_wifisetup_ov,"1. Tap 'Open portal' below.",TFG(),BF,LV_ALIGN_TOP_MID,0,92);
    mkLbl(g_wifisetup_ov,"2. On your phone, join (~10 s):",TGREY(),BF,LV_ALIGN_TOP_MID,0,132);
    mkLbl(g_wifisetup_ov,ssid,C_GREEN,BF,LV_ALIGN_TOP_MID,0,160);
    mkLbl(g_wifisetup_ov,"password:  ebby-atc",TFG(),BF,LV_ALIGN_TOP_MID,0,188);
    mkLbl(g_wifisetup_ov,"3. Open  http://192.168.4.1",C_CYAN,BF,LV_ALIGN_TOP_MID,0,228);
    mkLbl(g_wifisetup_ov,"Edit callsign / type / hex + club WiFi.",TFG(),BF,LV_ALIGN_TOP_MID,0,268);
    mkLbl(g_wifisetup_ov,"The box reboots after Save.",TGREY(),BF,LV_ALIGN_TOP_MID,0,296);
    // [Open portal] (single tap → {"cmd":"portal"}) + [Close]
    lv_obj_t*bp=lv_btn_create(g_wifisetup_ov);lv_obj_set_size(bp,210,52);
    lv_obj_align(bp,LV_ALIGN_BOTTOM_MID,-112,-30);
    lv_obj_set_style_bg_color(bp,C_BRAND,0);lv_obj_set_style_radius(bp,10,0);
    lv_obj_set_style_border_width(bp,0,0);lv_obj_set_style_shadow_opa(bp,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bp,_wifisetup_portal_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bp);lv_label_set_text(l,"Open portal");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(l,BF,0);lv_obj_center(l);}
    lv_obj_t*b=lv_btn_create(g_wifisetup_ov);lv_obj_set_size(b,150,52);
    lv_obj_align(b,LV_ALIGN_BOTTOM_MID,118,-30);
    lv_obj_set_style_bg_color(b,lv_color_hex(0x4b5563),0);lv_obj_set_style_radius(b,10,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(b,_wifisetup_close_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,"Close");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,BF,0);lv_obj_center(l);}}

static bool g_maint_portal_armed=false;
static void _maint_portal_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_maint_portal_armed){
        g_maint_portal_armed=true;
        if(l)lv_label_set_text(l,"Confirm?");
        lv_obj_set_style_bg_color(b,C_AMBER,0);
        return;
    }
    g_maint_portal_armed=false;
    if(l)lv_label_set_text(l,"WiFi Setup");
    lv_obj_set_style_bg_color(b,C_BRAND,0);
    showWifiSetupInfo();      // page WiFi Setup (instructions + bouton "Open portal" intégré)
}
// Ligne SYSTEM → ouvre la page WiFi Setup (dédiée, instructions + déclencheur portail).
static void _open_wifisetup_cb(lv_event_t*e){
    if(lv_event_get_code(e)==LV_EVENT_CLICKED) showWifiSetupInfo();
}

// ── (v21) Relais "Update both" — flasher ATC + ATV sur un seul réseau WiFi ──────
// L'AT-VIEW demande au boîtier d'ouvrir son portail ({"cmd":"portal"}), puis rejoint
// son AP ATCORE-SETUP-<box> EN STA (pass = constante ATC_PORTAL_PASS), garde son
// serveur web (/update existant) joignable sur ce réseau, et s'annonce au boîtier
// (GET /atv?ip=&v=) → la page portail du boîtier affiche un lien vers l'updater de
// l'AT-VIEW. Un seul téléphone sur l'AP flashe les deux. Machine d'état pilotée par
// relayTick() dans loop() (WiFi.begin non bloquant en boucle).
#define ATC_PORTAL_PASS "ebby-atc"   // = PORTAL_PASS côté AT-CORE (constante compilée)
enum { RLY_IDLE=0, RLY_WAIT_AP, RLY_JOINING, RLY_UP, RLY_FAIL };
static uint8_t  g_relay_state=RLY_IDLE;
static uint32_t g_relay_t0=0;
static char     g_relay_ssid[28]="";
static char     g_relay_ip[16]="";
static lv_obj_t* g_relay_ov=nullptr;
static lv_obj_t* g_relay_status_lbl=nullptr;
static void relayStop();         // fwd (défini près de wifiStart)

static void relayUpdateOverlay(){
    if(!g_relay_status_lbl)return;
    if(g_relay_state==RLY_UP){
        char b[40];snprintf(b,sizeof(b),"Ready - this screen %s",g_relay_ip);
        lv_label_set_text(g_relay_status_lbl,b);
        lv_obj_set_style_text_color(g_relay_status_lbl,C_GREEN,0);
    }else{
        const char* t = g_relay_state==RLY_WAIT_AP?"Opening box WiFi..."
                      : g_relay_state==RLY_JOINING?"Joining box network..."
                      : g_relay_state==RLY_FAIL  ?"Join failed - close & retry":"";
        lv_label_set_text(g_relay_status_lbl,t);
        lv_obj_set_style_text_color(g_relay_status_lbl,g_relay_state==RLY_FAIL?C_RED:C_AMBER,0);
    }}

static void _relay_close_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    relayStop();
    if(g_relay_ov){lv_obj_del(g_relay_ov);g_relay_ov=nullptr;g_relay_status_lbl=nullptr;}}

static void showRelayOverlay(){
    if(g_relay_ov)return;
    g_relay_ov=lv_obj_create(lv_scr_act());
#ifdef BOARD_T4S3
    lv_obj_set_size(g_relay_ov,600,480);lv_obj_set_pos(g_relay_ov,0,UI_OY);
    const lv_font_t* TF=&lv_font_montserrat_22; const lv_font_t* BF=&lv_font_montserrat_18;
#else
    lv_obj_set_size(g_relay_ov,480,480);lv_obj_set_pos(g_relay_ov,UI_OX,UI_OY);
    const lv_font_t* TF=&lv_font_montserrat_18; const lv_font_t* BF=&lv_font_montserrat_14;
#endif
    lv_obj_set_style_bg_color(g_relay_ov,TBG(),0);lv_obj_set_style_bg_opa(g_relay_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_relay_ov,0,0);lv_obj_set_style_radius(g_relay_ov,0,0);
    lv_obj_set_style_pad_all(g_relay_ov,0,0);lv_obj_clear_flag(g_relay_ov,LV_OBJ_FLAG_SCROLLABLE);
    char ssid[28];
    snprintf(ssid,sizeof(ssid),"ATCORE-SETUP-%s",
             (g_status.valid&&g_status.box[0])?g_status.box:"----");
    mkLbl(g_relay_ov,"UPDATE BOTH",C_AMBER,TF,LV_ALIGN_TOP_MID,0,40);
    g_relay_status_lbl=mkLbl(g_relay_ov,"",C_AMBER,BF,LV_ALIGN_TOP_MID,0,82);
    mkLbl(g_relay_ov,"On your phone, join this WiFi:",TGREY(),BF,LV_ALIGN_TOP_MID,0,124);
    mkLbl(g_relay_ov,ssid,C_GREEN,BF,LV_ALIGN_TOP_MID,0,150);
    mkLbl(g_relay_ov,"password:  ebby-atc",TFG(),BF,LV_ALIGN_TOP_MID,0,176);
    mkLbl(g_relay_ov,"Open  http://192.168.4.1",C_CYAN,BF,LV_ALIGN_TOP_MID,0,212);
    mkLbl(g_relay_ov,"Flash AT-CORE on that page,",TFG(),BF,LV_ALIGN_TOP_MID,0,248);
    mkLbl(g_relay_ov,"then tap 'Open AT-VIEW updater'",TFG(),BF,LV_ALIGN_TOP_MID,0,274);
    mkLbl(g_relay_ov,"to flash this screen.",TFG(),BF,LV_ALIGN_TOP_MID,0,300);
    lv_obj_t*b=lv_btn_create(g_relay_ov);lv_obj_set_size(b,200,48);
    lv_obj_align(b,LV_ALIGN_BOTTOM_MID,0,-30);
    lv_obj_set_style_bg_color(b,lv_color_hex(0x4b5563),0);lv_obj_set_style_radius(b,10,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(b,_relay_close_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,"Close (stop WiFi)");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,BF,0);lv_obj_center(l);}
    relayUpdateOverlay();}

static bool g_maint_both_armed=false;
static void _maint_updateboth_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    lv_obj_t*b=lv_event_get_target(e);lv_obj_t*l=lv_obj_get_child(b,0);
    if(!g_maint_both_armed){
        g_maint_both_armed=true;
        if(l)lv_label_set_text(l,"Confirm?");
        lv_obj_set_style_bg_color(b,C_AMBER,0);
        return;
    }
    g_maint_both_armed=false;
    if(l)lv_label_set_text(l,"Update both");
    lv_obj_set_style_bg_color(b,C_BRAND,0);
    if(!(g_status.valid&&g_status.box[0])){showRelayOverlay();return;}  // pas de box-id → affiche quand même les consignes
    snprintf(g_relay_ssid,sizeof(g_relay_ssid),"ATCORE-SETUP-%s",g_status.box);
    sendCtl("portal");                 // boîtier ouvre son AP
    g_relay_state=RLY_WAIT_AP;g_relay_t0=millis();
    showRelayOverlay();                // overlay live (machine d'état dans relayTick)
}

void mkMaintenanceOverlay(){
    if(g_maint_ov)return;
    g_maint_ov=lv_obj_create(lv_scr_act());
#ifdef BOARD_T4S3
    lv_obj_set_size(g_maint_ov,600,480);lv_obj_set_pos(g_maint_ov,0,UI_OY);   // plein écran T4
#else
    lv_obj_set_size(g_maint_ov,480,480);lv_obj_set_pos(g_maint_ov,UI_OX,UI_OY);
#endif
    lv_obj_set_style_bg_color(g_maint_ov,TBG(),0);lv_obj_set_style_bg_opa(g_maint_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_maint_ov,0,0);lv_obj_set_style_radius(g_maint_ov,0,0);
    lv_obj_set_style_pad_all(g_maint_ov,0,0);lv_obj_clear_flag(g_maint_ov,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(g_maint_ov,LV_SCROLLBAR_MODE_OFF);

#ifdef BOARD_T4S3
    // ════ T4-S3 : Maintenance ré-architecturée — 3 groupes logiques, plein 600, tactile ════
    //   HOTSPOT (SSID/Scan/Password/Test/Save)  ·  FLIGHTS (Last/Flights)  ·  FIRMWARE (Update)
    auto mkMBtn=[&](const char*txt,int x,int y,int w,int h,lv_color_t col,lv_color_t tc,lv_event_cb_t cb)->lv_obj_t*{
        lv_obj_t*b=lv_btn_create(g_maint_ov);lv_obj_set_size(b,w,h);lv_obj_set_pos(b,x,y);
        lv_obj_set_style_bg_color(b,col,0);lv_obj_set_style_radius(b,10,0);
        lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(b,0,0);
        lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);
        lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,txt);
        lv_obj_set_style_text_color(l,tc,0);lv_obj_set_style_text_font(l,&lv_font_montserrat_18,0);lv_obj_center(l);
        return b;};
    auto mkMHdr=[&](const char*txt,int y){
        lv_obj_t*l=lv_label_create(g_maint_ov);lv_label_set_text(l,txt);
        lv_obj_set_style_text_color(l,C_BRAND,0);lv_obj_set_style_text_font(l,&lv_font_montserrat_16,0);lv_obj_set_pos(l,30,y);
        lv_obj_t*hl=lv_obj_create(g_maint_ov);lv_obj_set_size(hl,540,1);lv_obj_set_pos(hl,30,y+22);
        lv_obj_set_style_bg_color(hl,C_BRAND,0);lv_obj_set_style_bg_opa(hl,LV_OPA_COVER,0);
        lv_obj_set_style_border_width(hl,0,0);lv_obj_set_style_pad_all(hl,0,0);
        lv_obj_clear_flag(hl,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);};
    // Titre + Close (toujours accessible en haut à droite)
    {lv_obj_t*tl=lv_label_create(g_maint_ov);lv_label_set_text(tl,"MAINTENANCE");
     lv_obj_set_style_text_color(tl,C_AMBER,0);lv_obj_set_style_text_font(tl,&lv_font_montserrat_22,0);
     lv_obj_set_pos(tl,30,22);}
    mkMBtn("Close",455,26,115,44,lv_color_hex(0x4b5563),lv_color_hex(0xffffff),_maint_close_cb);   // (clip) marge haut
    // ── HOTSPOT ───────────────────────────────────────────────────────────────
    mkMHdr("HOTSPOT (phone)",70);
    g_maint_ssid_ta=lv_textarea_create(g_maint_ov);
    lv_textarea_set_one_line(g_maint_ssid_ta,true);
    lv_textarea_set_placeholder_text(g_maint_ssid_ta,"Hotspot SSID");
    // Pré-remplit avec le dernier hotspot validé : NVS écran (g_hs_ssid) sinon SSID
    // enregistré CÔTÉ BOÎTIER (STATUS "wss", exige AT-CORE v17) → on voit toujours le dernier.
    lv_textarea_set_text(g_maint_ssid_ta,
        g_hs_ssid[0]?g_hs_ssid:(g_status.valid?g_status.wssid:""));
    lv_textarea_set_max_length(g_maint_ssid_ta,32);
    lv_obj_set_size(g_maint_ssid_ta,395,46);lv_obj_set_pos(g_maint_ssid_ta,30,102);
    lv_obj_add_event_cb(g_maint_ssid_ta,_maint_ta_cb,LV_EVENT_ALL,NULL);
    mkMBtn("Scan",435,102,135,46,lv_color_hex(0x1f4068),lv_color_hex(0xffffff),_maint_scan_cb);
    g_maint_pass_ta=lv_textarea_create(g_maint_ov);
    lv_textarea_set_one_line(g_maint_pass_ta,true);
    lv_textarea_set_password_mode(g_maint_pass_ta,true);
    lv_textarea_set_placeholder_text(g_maint_pass_ta,"Password");
    lv_textarea_set_text(g_maint_pass_ta,g_hs_pass);
    lv_textarea_set_max_length(g_maint_pass_ta,63);
    lv_obj_set_size(g_maint_pass_ta,540,46);lv_obj_set_pos(g_maint_pass_ta,30,156);
    lv_obj_add_event_cb(g_maint_pass_ta,_maint_ta_cb,LV_EVENT_ALL,NULL);
    mkMBtn("Test",30,210,260,46,C_CYAN,lv_color_hex(0x0d1117),_maint_test_cb);
    mkMBtn("Save",310,210,260,46,C_GREEN,lv_color_hex(0xffffff),_maint_save_cb);
    // ── FLIGHTS ───────────────────────────────────────────────────────────────
    mkMHdr("FLIGHTS",268);
    mkMBtn("Last flight",30,300,260,46,C_BRAND,lv_color_hex(0xffffff),_maint_upload_cb);
    mkMBtn("Flights...",310,300,260,46,lv_color_hex(0x1f4068),lv_color_hex(0xffffff),_open_vols_cb);
    // ── FIRMWARE / SETUP ────────────────────────────────────────────────────────
    mkMHdr("FIRMWARE / SETUP",356);
    g_maint_ota_armed=false;
    // (v21) 4 boutons (130 px) : Update (OTA pull cloud par hotspot) · WiFi Setup (portail
    // boîtier : identité clavier smartphone + OTA navigateur) · Update both (relais STA :
    // flashe ATC+ATV sur un seul réseau) · Reboot box (filet boîtier scellé). Une seule
    // ligne (T4 = 450 px de haut).
    mkMBtn("Update",30,388,130,46,C_BRAND,lv_color_hex(0xffffff),_maint_ota_cb);
    g_maint_portal_armed=false;
    mkMBtn("WiFi Setup",170,388,130,46,C_BRAND,lv_color_hex(0xffffff),_maint_portal_cb);
    g_maint_atvota_armed=false;
    mkMBtn("Update ATV",310,388,130,46,C_BRAND,lv_color_hex(0xffffff),_maint_atvota_cb);   // cloud-pull self-OTA (remplace l'ancien relais "Update both")
    g_maint_reboot_armed=false;
    mkMBtn("Reboot box",450,388,130,46,C_ORANGE,lv_color_hex(0xffffff),_maint_reboot_cb);
    // Labels d'état firmware/wifi : ligne fine sous les 2 boutons (toujours dans le cadre).
    g_maint_upd=lv_label_create(g_maint_ov);
    lv_obj_set_style_text_font(g_maint_upd,&lv_font_montserrat_14,0);lv_obj_set_pos(g_maint_upd,30,440);
    maintUpdAnnounce();
    g_maint_wst=lv_label_create(g_maint_ov);
    lv_obj_set_style_text_font(g_maint_wst,&lv_font_montserrat_14,0);lv_obj_set_pos(g_maint_wst,310,440);
    maintWifiStatus();
    // Clavier — bas d'écran, masqué par défaut, suit le textarea focalisé (au-dessus de lui).
    g_maint_kb=lv_keyboard_create(g_maint_ov);
    lv_obj_set_size(g_maint_kb,600,206);lv_obj_set_pos(g_maint_kb,0,258);
    lv_keyboard_set_textarea(g_maint_kb,g_maint_ssid_ta);
    lv_obj_add_flag(g_maint_kb,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(g_maint_kb,_maint_kb_cb,LV_EVENT_ALL,NULL);
#else
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
    // Pré-remplit avec le dernier hotspot validé : NVS écran (g_hs_ssid) sinon SSID
    // enregistré CÔTÉ BOÎTIER (STATUS "wss", exige AT-CORE v17) → on voit toujours le dernier.
    lv_textarea_set_text(g_maint_ssid_ta,
        g_hs_ssid[0]?g_hs_ssid:(g_status.valid?g_status.wssid:""));
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

    // Ligne y=256 (bande centrale large du cercle) : OTA cloud-pull "Update FW"
    // ({"cmd":"otaupdate"}) à gauche + "WiFi Setup" ({"cmd":"portal"}) à droite.
    // 2 demi-boutons (145) façon Last flight/Flights → pas de bouton en bas du
    // disque (où le cercle se rétrécit → clip). cf géométrie écran rond.
    g_maint_ota_armed=false;
    lv_obj_t*bo=lv_btn_create(g_maint_ov);lv_obj_set_size(bo,145,32);
    lv_obj_align(bo,LV_ALIGN_TOP_MID,-78,256);
    lv_obj_set_style_bg_color(bo,C_BRAND,0);lv_obj_set_style_radius(bo,8,0);
    lv_obj_set_style_border_width(bo,0,0);lv_obj_set_style_shadow_opa(bo,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bo,_maint_ota_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bo);lv_label_set_text(l,"Update FW");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}
    g_maint_portal_armed=false;
    lv_obj_t*bws=lv_btn_create(g_maint_ov);lv_obj_set_size(bws,145,32);
    lv_obj_align(bws,LV_ALIGN_TOP_MID,78,256);
    lv_obj_set_style_bg_color(bws,C_BRAND,0);lv_obj_set_style_radius(bws,8,0);
    lv_obj_set_style_border_width(bws,0,0);lv_obj_set_style_shadow_opa(bws,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(bws,_maint_portal_cb,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(bws);lv_label_set_text(l,"WiFi Setup");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}
    // Ligne y=344 : "Update both" (relais STA → flashe ATC+ATV sur un seul réseau) +
    // "Reboot box" (filet boîtier scellé). 2 demi-boutons dans la bande centrale large.
    g_maint_atvota_armed=false;
    {lv_obj_t*bb=lv_btn_create(g_maint_ov);lv_obj_set_size(bb,145,32);
     lv_obj_align(bb,LV_ALIGN_TOP_MID,-78,344);
     lv_obj_set_style_bg_color(bb,C_BRAND,0);lv_obj_set_style_radius(bb,8,0);
     lv_obj_set_style_border_width(bb,0,0);lv_obj_set_style_shadow_opa(bb,LV_OPA_TRANSP,0);
     lv_obj_add_event_cb(bb,_maint_atvota_cb,LV_EVENT_CLICKED,NULL);   // cloud-pull self-OTA (ex "Update both")
     lv_obj_t*l=lv_label_create(bb);lv_label_set_text(l,"Update ATV");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}
    g_maint_reboot_armed=false;
    {lv_obj_t*br=lv_btn_create(g_maint_ov);lv_obj_set_size(br,145,32);
     lv_obj_align(br,LV_ALIGN_TOP_MID,78,344);
     lv_obj_set_style_bg_color(br,C_ORANGE,0);lv_obj_set_style_radius(br,8,0);
     lv_obj_set_style_border_width(br,0,0);lv_obj_set_style_shadow_opa(br,LV_OPA_TRANSP,0);
     lv_obj_add_event_cb(br,_maint_reboot_cb,LV_EVENT_CLICKED,NULL);
     lv_obj_t*l=lv_label_create(br);lv_label_set_text(l,"Reboot box");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);lv_obj_center(l);}
    // Annonce de MAJ firmware (ambre "Update available: vN" si dispo, gris "up to date" sinon).
    g_maint_upd=lv_label_create(g_maint_ov);
    lv_obj_set_style_text_font(g_maint_upd,&lv_font_montserrat_12,0);
    lv_obj_align(g_maint_upd,LV_ALIGN_TOP_MID,0,292);
    maintUpdAnnounce();   // (fallback AP-OTA par BOUTON BOOT retiré : boîtier scellé, BOOT inaccessible)
    // État WiFi live (vert "WiFi connected (ip)" / ambre connexion / rouge échec) —
    // rafraîchi à chaque STATUS tant que l'écran est ouvert (cf maintWifiStatus).
    g_maint_wst=lv_label_create(g_maint_ov);
    lv_obj_set_style_text_font(g_maint_wst,&lv_font_montserrat_12,0);
    lv_obj_align(g_maint_wst,LV_ALIGN_TOP_MID,0,310);
    maintWifiStatus();


    // Clavier LVGL — TAILLÉ POUR LE CERCLE : 320x175 centré (les coins restent
    // dans le disque 480, contrairement au plein-largeur dont la rangée du bas
    // sortait du cercle). Caché par défaut, suit le textarea focalisé ; les
    // champs (y114/y160) restent visibles au-dessus quand il s'affiche.
    g_maint_kb=lv_keyboard_create(g_maint_ov);
#ifdef BOARD_T4S3
    // Écran rectangulaire : clavier nettement plus grand (480×240 vs 320×175 du cercle)
    lv_obj_set_size(g_maint_kb,480,240);lv_obj_align(g_maint_kb,LV_ALIGN_TOP_MID,0,215);
#else
    lv_obj_set_size(g_maint_kb,320,175);lv_obj_align(g_maint_kb,LV_ALIGN_TOP_MID,0,224);
#endif
    lv_keyboard_set_textarea(g_maint_kb,g_maint_ssid_ta);
    lv_obj_add_flag(g_maint_kb,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(g_maint_kb,_maint_kb_cb,LV_EVENT_ALL,NULL);
#endif
}

static void _open_maintenance_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    if(!g_maint_ov)mkMaintenanceOverlay();}

// ── Page TEST (sous-page SYSTEM) — déclencheurs cycle de vol pour QA / valider Phase A ──
// Les boutons Start/Stop ont été retirés de l'UI normale (cycle 100% auto), mais les
// commandes CHR_CONTROL existent toujours côté firmware. Cette page expose Start/Stop/
// Continue (+ WiFi test / Upload all) pour SIMULER un cycle SANS mouvement au banc :
// Start → attendre qq s → Stop = "atterrissage" → FLT_CLOSED → upload auto (Phase A).
// (port T-RGB 2026-06-27) board-indépendant (géométrie T4 ; sur rond = brut, outil QA).
static lv_obj_t* g_test_ov=nullptr;
static void mkTestOverlay(){
    if(g_test_ov)return;
    g_test_ov=lv_obj_create(lv_scr_act());
    const int OW=600; lv_obj_set_size(g_test_ov,600,480);lv_obj_set_pos(g_test_ov,0,UI_OY);
    lv_obj_set_style_bg_color(g_test_ov,TBG(),0);lv_obj_set_style_bg_opa(g_test_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(g_test_ov,0,0);lv_obj_set_style_radius(g_test_ov,0,0);
    lv_obj_set_style_pad_all(g_test_ov,0,0);lv_obj_clear_flag(g_test_ov,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(g_test_ov,LV_SCROLLBAR_MODE_OFF);
    auto mkTB=[&](const char*txt,int y,lv_color_t col,lv_event_cb_t cb){
        lv_obj_t*b=lv_btn_create(g_test_ov);lv_obj_set_size(b,OW-60,52);lv_obj_set_pos(b,30,y);
        lv_obj_set_style_bg_color(b,col,0);lv_obj_set_style_radius(b,10,0);
        lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(b,0,0);
        lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);
        lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,txt);
        lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_18,0);lv_obj_center(l);};
    {lv_obj_t*tl=lv_label_create(g_test_ov);lv_label_set_text(tl,"TEST");
     lv_obj_set_style_text_color(tl,C_AMBER,0);lv_obj_set_style_text_font(tl,&lv_font_montserrat_22,0);lv_obj_set_pos(tl,30,18);}
    {lv_obj_t*b=lv_btn_create(g_test_ov);lv_obj_set_size(b,115,44);lv_obj_set_pos(b,OW-145,20);
     lv_obj_set_style_bg_color(b,lv_color_hex(0x4b5563),0);lv_obj_set_style_radius(b,10,0);
     lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(b,0,0);
     lv_obj_add_event_cb(b,[](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED){ lv_obj_del(g_test_ov); g_test_ov=nullptr; } },LV_EVENT_CLICKED,NULL);
     lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,"Close");lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_18,0);lv_obj_center(l);}
    {lv_obj_t*h=lv_label_create(g_test_ov);
     lv_label_set_text(h,"Simulate flight (no movement):\nStart -> wait -> Stop = landing -> auto-upload");
     lv_obj_set_style_text_color(h,TGREY(),0);lv_obj_set_style_text_font(h,&lv_font_montserrat_14,0);lv_obj_set_pos(h,30,72);}
    int y=126,DY=62;
    mkTB("Start flight",           y+0*DY,C_GREEN,                [](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("start_flight"); });
    mkTB("Stop flight  (= landing)",y+1*DY,C_RED,                 [](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("stop_flight"); });
    mkTB("Continue flight",        y+2*DY,lv_color_hex(0x4b5563), [](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("continue_flight"); });
    mkTB("WiFi test",              y+3*DY,lv_color_hex(0x1f4068), [](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("wifitest"); });
    mkTB("Upload all",             y+4*DY,C_BRAND,                [](lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("uploadall"); });
}
static void _open_test_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    audioTestChime();                 // bip de validation audio (no-op hors WS-241)
    if(!g_test_ov)mkTestOverlay();}

// ════════════════════════════════════════════════════════════════════════════
// SETTINGS — menu en GRILLE → 6 sections → POPUPS de choix (board-aware via SETW :
// T4 600×450 rect / T-RGB 480×480 rond — port T-RGB 2026-06-27)
//   Page 3 maquette : grille de gros boutons (RADAR/DISPLAY/TRAFFIC/AIRCRAFT/
//   SYSTEM/ABOUT). Page 4 : les choix énumérés (SCALE/V-FILTER/SPEED) ouvrent
//   un popup de gros boutons faciles à cliquer. SCALE = échelle par défaut au
//   boot ; DIST conservé ; ALT supprimé (toujours en ft).
// ════════════════════════════════════════════════════════════════════════════

// ── Popup picker générique (gros boutons de choix) ──────────────────────────
static lv_obj_t* g_pick_ov=nullptr;
static void (*g_pick_apply)(int)=nullptr;
static void pickHide(){ if(g_pick_ov){lv_obj_del(g_pick_ov);g_pick_ov=nullptr;g_pick_apply=nullptr;} }
static void _pick_close_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) pickHide(); }
static void _pick_sel_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int i=(int)(intptr_t)lv_event_get_user_data(e);
    void(*ap)(int)=g_pick_apply;
    pickHide();
    if(ap){ ap(i); cfgSave(); updSetPage(); }   // applique → persiste → rafraîchit labels + radar
}
static void pickShow(const char* title,const char* const* opts,int n,int cur,void(*apply)(int)){
    if(g_pick_ov)return;
    g_pick_apply=apply;
    g_pick_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_pick_ov,600,480);lv_obj_set_pos(g_pick_ov,0,0);
    lv_obj_set_style_bg_color(g_pick_ov,lv_color_hex(0x000000),0);
    lv_obj_set_style_bg_opa(g_pick_ov,LV_OPA_50,0);
    lv_obj_set_style_border_width(g_pick_ov,0,0);lv_obj_set_style_pad_all(g_pick_ov,0,0);
    lv_obj_clear_flag(g_pick_ov,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(g_pick_ov,_pick_close_cb,LV_EVENT_CLICKED,NULL);
    int ph=56+n*60+14; if(ph>466)ph=466;
    lv_obj_t* pan=lv_obj_create(g_pick_ov);
    lv_obj_set_size(pan,360,ph);lv_obj_center(pan);
    lv_obj_set_style_bg_color(pan,TBG(),0);lv_obj_set_style_bg_opa(pan,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(pan,TFG(),0);lv_obj_set_style_border_width(pan,2,0);
    lv_obj_set_style_radius(pan,16,0);lv_obj_set_style_shadow_opa(pan,LV_OPA_TRANSP,0);
    lv_obj_clear_flag(pan,LV_OBJ_FLAG_SCROLLABLE);lv_obj_add_flag(pan,LV_OBJ_FLAG_CLICKABLE);
    mkLblP(pan,title,C_BRAND,&lv_font_montserrat_20,20,14);
    for(int i=0;i<n;i++){
        lv_obj_t* bt=lv_btn_create(pan);
        lv_obj_set_size(bt,320,52);lv_obj_set_pos(bt,20,52+i*60);
        bool sel=(i==cur);
        lv_obj_set_style_bg_color(bt,sel?C_BRAND:lv_color_hex(0xeef2f6),0);
        lv_obj_set_style_radius(bt,12,0);lv_obj_set_style_border_width(bt,0,0);
        lv_obj_set_style_shadow_opa(bt,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(bt,0,0);
        lv_obj_add_event_cb(bt,_pick_sel_cb,LV_EVENT_CLICKED,(void*)(intptr_t)i);
        lv_obj_t* l=lv_label_create(bt);lv_label_set_text(l,opts[i]);
        lv_obj_set_style_text_color(l,sel?lv_color_hex(0xffffff):lv_color_hex(0x0f172a),0);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);lv_obj_center(l);
    }
}

// ── Valeurs + apply/idx des popups énumérés ─────────────────────────────────
static const char* kScaleStr[7]={"1 nm","2 nm","4 nm","8 nm","10 nm","20 nm","40 nm"};
static const int16_t kVfiltOpts[4]={500,1000,1500,2000};
static const char* kVfiltStr[4]={"500 ft","1000 ft","1500 ft","2000 ft"};
static const char* kSpeedStr[2]={"kt","km/h"};
static int  _idxScale(){ for(int i=0;i<7;i++) if(kScaleOpts[i]==g_cfg.scale_nm) return i; return 2; }
static void _applyScale(int i){ if(i>=0&&i<7) g_cfg.scale_nm=kScaleOpts[i]; }
static int  _idxVfilt(){ for(int i=0;i<4;i++) if(kVfiltOpts[i]==g_cfg.vfilt_ft) return i; return 3; }
static void _applyVfilt(int i){ if(i>=0&&i<4){ g_cfg.vfilt_ft=kVfiltOpts[i]; sendVfilt(g_cfg.vfilt_ft); } }  // + push VF= au boîtier
static int  _idxSpeed(){ return g_cfg.spd_kt?0:1; }
static void _applySpeed(int i){ g_cfg.spd_kt=(i==0); }

// ── Ligne « gros bouton valeur → popup » ────────────────────────────────────
struct PopSpec { const char* title; const char* const* opts; int n; int(*idx)(); void(*apply)(int); };
static PopSpec g_pop[6]; static int g_pop_n=0;
static void _pop_open_cb(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
    int i=(int)(intptr_t)lv_event_get_user_data(e);
    if(i<0||i>=g_pop_n)return;
    PopSpec&s=g_pop[i];
    pickShow(s.title,s.opts,s.n,s.idx(),s.apply);
}
static lv_obj_t* mkPopRow(lv_obj_t*p,const char*k,int y,const char*curval,
                          const char* title,const char* const* opts,int n,int(*idx)(),void(*apply)(int)){
    mkLblP(p,k,lv_color_hex(0x4b5563),&lv_font_montserrat_20,40,y+15);
    if(g_pop_n>=6) return nullptr;
    int pi=g_pop_n;
    g_pop[pi].title=title; g_pop[pi].opts=opts; g_pop[pi].n=n; g_pop[pi].idx=idx; g_pop[pi].apply=apply;
    const int BW=224,BH=52,BX=SETW-40-BW;
    lv_obj_t*bt=lv_btn_create(p);lv_obj_set_size(bt,BW,BH);lv_obj_set_pos(bt,BX,y);
    lv_obj_set_style_bg_color(bt,lv_color_hex(0xeef2f6),0);lv_obj_set_style_radius(bt,14,0);
    lv_obj_set_style_border_width(bt,0,0);lv_obj_set_style_shadow_opa(bt,LV_OPA_TRANSP,0);lv_obj_set_style_pad_all(bt,0,0);
    lv_obj_add_event_cb(bt,_pop_open_cb,LV_EVENT_CLICKED,(void*)(intptr_t)pi);
    lv_obj_t*vl=lv_label_create(bt);lv_label_set_text(vl,curval);
    lv_obj_set_style_text_color(vl,lv_color_hex(0x0f172a),0);
    lv_obj_set_style_text_font(vl,&lv_font_montserrat_20,0);lv_obj_center(vl);
    g_pop_n++;
    return vl;
}

// ── Navigation menu ↔ sections ──────────────────────────────────────────────
static void settingsShowMenu(){
    if(!s_menu)return;
    s_cur_sec=-1;
    for(int i=0;i<6;i++) if(s_sec[i]) lv_obj_add_flag(s_sec[i],LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_menu,LV_OBJ_FLAG_HIDDEN);
    if(s_back_btn) lv_obj_add_flag(s_back_btn,LV_OBJ_FLAG_HIDDEN);   // pas de retour sur le menu
    if(s_set_aclbl){ lv_obj_clear_flag(s_set_aclbl,LV_OBJ_FLAG_HIDDEN); }   // bloc Active Aircraft visible (menu)
    if(s_set_acval){ char ac[40]; snprintf(ac,sizeof(ac),"%s / %s / %s",
                       g_ac_reg[0]?g_ac_reg:"---", g_ac_type[0]?g_ac_type:"---", g_ac_hex[0]?g_ac_hex:"------");
                     lv_label_set_text(s_set_acval,ac); lv_obj_clear_flag(s_set_acval,LV_OBJ_FLAG_HIDDEN); }
    if(s_set_title) lv_label_set_text(s_set_title,"SETTINGS");
    if(s_set_uline&&s_set_title){ lv_obj_update_layout(s_set_title); lv_obj_set_width(s_set_uline,lv_obj_get_width(s_set_title)); }
}
static void settingsOpenSection(int i){
    if(i<0||i>=6||!s_sec[i])return;
    s_cur_sec=(int8_t)i;
    if(s_menu) lv_obj_add_flag(s_menu,LV_OBJ_FLAG_HIDDEN);
    for(int k=0;k<6;k++) if(s_sec[k]) lv_obj_add_flag(s_sec[k],LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_sec[i],LV_OBJ_FLAG_HIDDEN);
    if(s_back_btn) lv_obj_clear_flag(s_back_btn,LV_OBJ_FLAG_HIDDEN);   // (juin 2026) cercle retour visible en section
    if(s_set_aclbl) lv_obj_add_flag(s_set_aclbl,LV_OBJ_FLAG_HIDDEN);   // bloc Active Aircraft caché en section (cercle retour à sa place)
    if(s_set_acval) lv_obj_add_flag(s_set_acval,LV_OBJ_FLAG_HIDDEN);
    if(s_set_title) lv_label_set_text(s_set_title,kSecName[i]);        // titre épuré (le retour est le cercle à droite)
    if(s_set_uline&&s_set_title){ lv_obj_update_layout(s_set_title); lv_obj_set_width(s_set_uline,lv_obj_get_width(s_set_title)); }
}
static void _menu_btn_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) settingsOpenSection((int)(intptr_t)lv_event_get_user_data(e)); }
static void _sec_back_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) settingsShowMenu(); }

// Gros bouton de la grille menu (contour bleu, rempli au press).
static void mkMenuBtn(lv_obj_t*parent,int i,int x,int y){
    lv_obj_t*bt=lv_btn_create(parent);
#ifdef BOARD_T4S3
    lv_obj_set_size(bt,252,80);const lv_font_t*MF=&lv_font_montserrat_24;
#else
    lv_obj_set_size(bt,180,64);const lv_font_t*MF=&lv_font_montserrat_18;   // rond : boutons + petits (tiennent dans le cercle)
#endif
    lv_obj_set_pos(bt,x,y);
    lv_obj_set_style_bg_color(bt,TBG(),0);lv_obj_set_style_bg_opa(bt,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(bt,C_BRAND,0);lv_obj_set_style_border_width(bt,2,0);
    lv_obj_set_style_radius(bt,16,0);lv_obj_set_style_shadow_opa(bt,LV_OPA_TRANSP,0);
    lv_obj_set_style_bg_color(bt,C_BRAND,LV_STATE_PRESSED);
    lv_obj_add_event_cb(bt,_menu_btn_cb,LV_EVENT_CLICKED,(void*)(intptr_t)i);
    lv_obj_t*l=lv_label_create(bt);lv_label_set_text(l,kSecName[i]);
    lv_obj_set_style_text_color(l,C_BRAND,0);
    lv_obj_set_style_text_font(l,MF,0);lv_obj_center(l);
}

// (juin 2026) FIX swipe : propage EVENT_BUBBLE à TOUS les descendants (boutons, cellules
// segmented imbriquées…) → un swipe horizontal démarré sur un contrôle remonte au conteneur
// (qui porte swipeCb) au lieu d'être avalé. Sans ça, le menu plein de boutons cassait la
// navigation par swipe. Les taps continuent de marcher (le handler propre du bouton fire quand
// même ; LVGL annule le CLICKED si le doigt quitte le bouton = vrai swipe).
static void bubbleAll(lv_obj_t* o){
    uint32_t n=lv_obj_get_child_cnt(o);
    for(uint32_t i=0;i<n;i++){ lv_obj_t*c=lv_obj_get_child(o,i); lv_obj_add_flag(c,LV_OBJ_FLAG_EVENT_BUBBLE); bubbleAll(c); }
}

void buildSettingsPageT4(lv_obj_t*p){
    char b[20];
    g_seg_n=0; g_segn_n=0; g_pop_n=0; s_cur_sec=-1;
    s_scale_v=nullptr;s_vfilt_v=nullptr;s_dist_v=nullptr;s_alt_v=nullptr;s_spd_v=nullptr;
    s_bright_v=nullptr;s_src_v=nullptr;s_theme_v=nullptr;s_grnd_v=nullptr;s_icon_sz_v=nullptr;
    s_ac_v=nullptr;s_wifi_v=nullptr;s_sd_v=nullptr;s_set_aclbl=nullptr;s_set_acval=nullptr;

    // En-tête + géométrie BOARD-AWARE (T4 600×450 rect / T-RGB 480×480 rond — la grille et
    // les contrôles tiennent dans le cercle). Le RETOUR = cercle bleu ; sur rond il est placé
    // plus bas/centré (les coins du cercle sont clippés).
#ifdef BOARD_T4S3
    const int hLogoX=40,hLogoY=26, hTitX=112,hTitY=38, ulineY=70, SCRH=SCR_H, CY=94;
    const int col0=44,col1=304,r0=29,dyr=109;
#else
    const int hLogoX=30,hLogoY=10, hTitX=100,hTitY=14, ulineY=42, SCRH=480, CY=86;
    const int col0=50,col1=250,r0=20,dyr=92;     // rond : 2×3 boutons 180×64 centrés dans le cercle
#endif
    const int CH=SCRH-CY;
    lv_obj_t*lA=lv_img_create(p);lv_img_set_src(lA,&img_logo_a);
    lv_obj_align(lA,LV_ALIGN_TOP_LEFT,hLogoX,hLogoY);
    s_set_title=mkLblP(p,"SETTINGS",TFG(),&lv_font_montserrat_24,hTitX,hTitY);
    lv_obj_add_flag(s_set_title,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_opa(s_set_title,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(s_set_title,cbDebugLongPress,LV_EVENT_LONG_PRESSED,NULL);
    s_set_uline=lv_obj_create(p);lv_obj_set_size(s_set_uline,120,2);lv_obj_set_pos(s_set_uline,hTitX,ulineY);
    lv_obj_set_style_bg_color(s_set_uline,C_BRAND,0);lv_obj_set_style_bg_opa(s_set_uline,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(s_set_uline,0,0);lv_obj_set_style_pad_all(s_set_uline,0,0);lv_obj_set_style_radius(s_set_uline,1,0);
    lv_obj_clear_flag(s_set_uline,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);

    // Bloc « Active Aircraft » (menu seul ; caché en section). T4 = à droite du titre ;
    // rond = centré sous le titre (les coins sont clippés).
    s_set_aclbl=lv_label_create(p);lv_label_set_text(s_set_aclbl,"Active Aircraft");
    lv_obj_set_style_text_color(s_set_aclbl,lv_color_hex(0x4b5563),0);
    lv_obj_set_style_text_font(s_set_aclbl,&lv_font_montserrat_14,0);
    s_set_acval=lv_label_create(p);
    lv_obj_set_style_text_color(s_set_acval,C_BRAND,0);
    lv_obj_set_style_text_font(s_set_acval,&lv_font_montserrat_20,0);
    {char ac[40];snprintf(ac,sizeof(ac),"%s / %s / %s",
        g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---",g_ac_hex[0]?g_ac_hex:"------");
     lv_label_set_text(s_set_acval,ac);}
#ifdef BOARD_T4S3
    lv_obj_align(s_set_aclbl,LV_ALIGN_TOP_RIGHT,-40,28);
    lv_obj_align(s_set_acval,LV_ALIGN_TOP_RIGHT,-40,46);
#else
    // Rond : titre CENTRÉ (coin haut-gauche clippé par le cercle), logo + soulignement
    // masqués (coins clippés), Active Aircraft centré dessous.
    lv_obj_align(s_set_title,LV_ALIGN_TOP_MID,0,14);
    lv_obj_add_flag(lA,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_set_uline,LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(s_set_aclbl,LV_ALIGN_TOP_MID,0,50);
    lv_obj_align(s_set_acval,LV_ALIGN_TOP_MID,0,66);
#endif

    // Cercle « retour » bleu (‹)
    s_back_btn=lv_btn_create(p);
    lv_obj_set_style_radius(s_back_btn,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_color(s_back_btn,C_BRAND,0);lv_obj_set_style_shadow_opa(s_back_btn,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(s_back_btn,0,0);
    lv_obj_add_event_cb(s_back_btn,_sec_back_cb,LV_EVENT_CLICKED,NULL);
#ifdef BOARD_T4S3
    lv_obj_set_size(s_back_btn,62,62);lv_obj_set_pos(s_back_btn,600-40-62,24);
#else
    lv_obj_set_size(s_back_btn,56,56);lv_obj_align(s_back_btn,LV_ALIGN_BOTTOM_MID,0,-14);  // rond : tout en bas centré (sous le contenu, zone large du cercle)
#endif
    {lv_obj_t*l=lv_label_create(s_back_btn);lv_label_set_text(l,LV_SYMBOL_LEFT);
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_28,0);lv_obj_center(l);}
    lv_obj_add_flag(s_back_btn,LV_OBJ_FLAG_HIDDEN);

    // ── Menu : grille 2×3 ────────────────────────────────────────────────────
    s_menu=lv_obj_create(p);
    lv_obj_set_size(s_menu,SETW,CH);lv_obj_set_pos(s_menu,0,CY);
    lv_obj_set_style_bg_opa(s_menu,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(s_menu,0,0);
    lv_obj_set_style_pad_all(s_menu,0,0);lv_obj_clear_flag(s_menu,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_menu,swipeCb,LV_EVENT_ALL,NULL);   // swipe horizontal (nav page) conservé
    // 5 tuiles : CONFIG / TRAFFIC / PILOT (col 0) · SYSTEM / ABOUT (col 1) ; 6e cellule
    // (col1,r0+2*dyr) laissée VIDE = slot réservé pour un futur réglage.
    {mkMenuBtn(s_menu,0,col0,r0);        mkMenuBtn(s_menu,3,col1,r0);
     mkMenuBtn(s_menu,1,col0,r0+dyr);    mkMenuBtn(s_menu,4,col1,r0+dyr);
     mkMenuBtn(s_menu,2,col0,r0+2*dyr);}

    // ── 6 conteneurs sections (cachés au départ) ────────────────────────────
    for(int i=0;i<6;i++){
        s_sec[i]=lv_obj_create(p);
        lv_obj_set_size(s_sec[i],SETW,CH);lv_obj_set_pos(s_sec[i],0,CY);
        lv_obj_set_style_bg_opa(s_sec[i],LV_OPA_TRANSP,0);lv_obj_set_style_border_width(s_sec[i],0,0);
        lv_obj_set_style_pad_all(s_sec[i],0,0);lv_obj_clear_flag(s_sec[i],LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(s_sec[i],swipeCb,LV_EVENT_ALL,NULL);
        lv_obj_add_flag(s_sec[i],LV_OBJ_FLAG_HIDDEN);
    }

    // 0) CONFIG (fusion ex-CONFIG + ex-DISPLAY) — SCROLLABLE verticalement (swipe down) :
    //    BRIGHTNESS (slider horizontal, en tête : ne gêne pas le scroll vertical) · THEME ·
    //    Default radar scale · Vertical filter (→ VF SafeSky poussé ATC) · Alt difference · Callsign.
    //    6 lignes > hauteur écran → on active le scroll vertical au lieu de tout tasser.
    {lv_obj_t*sp=s_sec[0]; const int Y0=8,DY=72;
     mkBigBrightRow(sp,"BRIGHTNESS",Y0+0*DY,g_cfg.brightness);
     mkSegRow(sp,"THEME",Y0+1*DY,"LIGHT","DARK",&g_cfg.dark,false);
     snprintf(b,sizeof(b),"%d nm",g_cfg.scale_nm);
     s_scale_v=mkPopRow(sp,"DEFAULT RADAR SCALE",Y0+2*DY,b,"DEFAULT RADAR SCALE",kScaleStr,7,_idxScale,_applyScale);
     snprintf(b,sizeof(b),"%d ft",g_cfg.vfilt_ft);
     s_vfilt_v=mkPopRow(sp,"VERTICAL FILTER",Y0+3*DY,b,"VERTICAL FILTER (ft)",kVfiltStr,4,_idxVfilt,_applyVfilt);
     mkSegRow(sp,"ALT DIFFERENCE",Y0+4*DY,"OFF","ON",&g_cfg.show_vdiff,false);
     mkSegRow(sp,"CALLSIGN",Y0+5*DY,"OFF","ON",&g_cfg.show_cs,false);
     // Scroll vertical : le contenu (6 lignes ≈ 420 px) dépasse le conteneur → swipe down.
     // Slider brightness horizontal en tête → drag horizontal = brightness, drag vertical = scroll
     // (orthogonaux, pas de conflit). Le swipe HORIZONTAL de nav page reste géré par swipeCb (bubble).
     lv_obj_add_flag(sp,LV_OBJ_FLAG_SCROLLABLE);
     lv_obj_set_scroll_dir(sp,LV_DIR_VER);
     lv_obj_set_scrollbar_mode(sp,LV_SCROLLBAR_MODE_AUTO);}

    // 1) TRAFFIC : GROUNDED · ICONS · AIP · HELIPORT · ALERT MODE (override circuit)
    {lv_obj_t*sp=s_sec[1]; const int Y0=8,DY=66;
     mkSegRow (sp,"GROUNDED",Y0+0*DY,"OFF","ON",&g_cfg.show_grnd,false);
     mkSegRowN(sp,"ICONS",Y0+1*DY,kIconSzNames,3,&g_cfg.icon_sz,1);
     mkSegRow (sp,"AIP",Y0+2*DY,"OFF","ON",&g_cfg.aip_en,false);
     mkSegRow (sp,"HELIPORT",Y0+3*DY,"OFF","ON",&g_cfg.ad_heli,false);
     mkSegRowN(sp,"ALERT MODE",Y0+4*DY,kCircNames,3,&g_cfg.circuit_ovr,0);}  // AUTO / CIRCUIT forcé / ROUTE forcé

    // 2) PILOT : réservé (réactivé plus tard). L'identité aéronef se configure désormais
    //    via le portail web (champ "Active Aircraft" affiché à droite du menu) → page AIRCRAFT retirée.
    {lv_obj_t*sp=s_sec[2];
     mkLblP(sp,"PILOT",TFG(),&lv_font_montserrat_24,40,8);
     mkLblP(sp,"Coming soon",lv_color_hex(0x4b5563),&lv_font_montserrat_20,40,8+44);}

    // 3) SYSTEM : tuiles en grille 2 colonnes (même style que la grille SETTINGS), une par page.
    //    Géométrie col0/col1/r0/dyr réutilisée de la grille → tuiles alignées à l'identique.
    //    5 tuiles ; la 6e cellule (col1, 3e rangée) porte l'état SD CARD (libellé, pas un bouton).
    {lv_obj_t*sp=s_sec[3];
     mkNavTile(sp,"WIFI",      col0,r0,        _open_wifisetup_cb);   // portail : identité + WiFi club
     mkNavTile(sp,"FlightLogs",col1,r0,        _open_vols_cb);         // liste + Send/Delete
     mkNavTile(sp,"Updates",   col0,r0+dyr,    _open_updates_cb);     // versions ATC/ATV + MAJ
     mkNavTile(sp,"Diagnostic",col1,r0+dyr,    _open_diag_cb);        // WiFi/SD + Test/Reboot
     mkNavTile(sp,"Test",      col0,r0+2*dyr,  _open_test_cb);        // QA : simuler cycle vol
     char sd[16]; if(g_sd_ok)snprintf(sd,sizeof(sd),"SD %u GB",g_sd_gb);else strlcpy(sd,"NO SD CARD",sizeof(sd));
     s_sd_v=mkLblP(sp,sd,g_sd_ok?C_GREEN:lv_color_hex(0x4b5563),&lv_font_montserrat_20,col1+10,r0+2*dyr+26);}

    // 4) ABOUT : versions & batteries (lecture seule, live BLE)
    {lv_obj_t*sp=s_sec[4]; const lv_color_t kcol=lv_color_hex(0x4b5563); const int X2=320,DYr=40;
     mkLblP(sp,"AT-VIEW",kcol,&lv_font_montserrat_20,40,8+0*DYr);
     mkLblP(sp,VIEW_VSTR,verColor(VIEW_VSTR),&lv_font_montserrat_20,X2,8+0*DYr);   // "1.2.38-dev" coloré par canal
     mkLblP(sp,"AT-CORE",kcol,&lv_font_montserrat_20,40,8+1*DYr);
     s_sys_atcver=mkLblP(sp,"v--",C_BRAND,&lv_font_montserrat_20,X2,8+1*DYr);   // rempli live (g_status.fws) dans updateAllPages
     mkLblP(sp,"AT-CORE BATT",kcol,&lv_font_montserrat_20,40,8+2*DYr);
     s_sys_atcbat=mkLblP(sp,"---%",C_BRAND,&lv_font_montserrat_20,X2,8+2*DYr);
     mkLblP(sp,"AT-VIEW BATT",kcol,&lv_font_montserrat_20,40,8+3*DYr);
     mkLblP(sp,"N/A (future)",TGREY(),&lv_font_montserrat_20,X2,8+3*DYr);}

    // (juin 2026) FIX swipe : tous les contrôles bubblent → swipe horizontal de nav page OK
    bubbleAll(s_menu);
    for(int s=0;s<6;s++) bubbleAll(s_sec[s]);

    // Le ‹ retour doit être AU-DESSUS des conteneurs de sections (créés après lui) — sinon,
    // placé dans la zone d'une section (cas rond bas-centre), la section intercepte le tap.
    lv_obj_move_foreground(s_back_btn);

    settingsShowMenu();   // démarre sur la grille
}

void buildSettingsPage(){
    lv_obj_t*p=g_pages[2]; char b[16]; (void)b;
    s_pg_idx=0;
    lv_obj_set_style_bg_color(p,TBG(),0);
    lv_obj_set_style_bg_opa(p,LV_OPA_COVER,0);
    // Settings UNIFIÉ (grille → sections → popups), board-aware via SETW : T4 ET T-RGB y passent.
#ifdef BOARD_T4S3
    lv_obj_set_size(p,SCR_W,SCR_H);lv_obj_set_pos(p,0,0);
#else
    lv_obj_set_size(p,480,480);lv_obj_set_pos(p,0,0);
#endif
    lv_obj_set_scrollbar_mode(p,LV_SCROLLBAR_MODE_OFF);
    buildSettingsPageT4(p);
    return;
    // ⚠️ Legacy round s_pg ci-dessous : INATTEIGNABLE (à supprimer après validation du port rond).
#if 0

    // ── Logo A bleu + titre SETTINGS
    lv_obj_t*lA=lv_img_create(p);
    lv_img_set_src(lA,&img_logo_a);
#ifdef BOARD_T4S3
    // T4-S3 : en-tête sur UNE ligne, alignée à gauche (niveau V-FILTER, x=40) →
    // logo + espace + « SETTINGS » → libère de la hauteur pour les rows tactiles.
    lv_obj_align(lA,LV_ALIGN_TOP_LEFT,40,28);            // logo 56×56 (y=28 : compense l'offset page -15)
    s_set_title=mkLblP(p,"SETTINGS / RADAR",lv_color_hex(0x0f172a),&lv_font_montserrat_24,40+56+16,42);
    // Footer retiré sur T4 → le geste debug caché passe par un appui long sur le titre.
    lv_obj_add_flag(s_set_title,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_opa(s_set_title,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(s_set_title,cbDebugLongPress,LV_EVENT_LONG_PRESSED,NULL);
    // Fine ligne bleue sous le titre (largeur ajustée au texte par setUpdTitle).
    s_set_uline=lv_obj_create(p);
    lv_obj_set_size(s_set_uline,120,2);lv_obj_set_pos(s_set_uline,40+56+16,74);
    lv_obj_set_style_bg_color(s_set_uline,C_BRAND,0);lv_obj_set_style_bg_opa(s_set_uline,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(s_set_uline,0,0);lv_obj_set_style_pad_all(s_set_uline,0,0);
    lv_obj_set_style_radius(s_set_uline,1,0);
    lv_obj_clear_flag(s_set_uline,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    setUpdTitle();   // recale la largeur de la ligne sur le texte courant
#else
    lv_obj_align(lA,LV_ALIGN_TOP_MID,0,18);
    mkLbl(p,"SETTINGS",lv_color_hex(0x0f172a),&lv_font_montserrat_20,LV_ALIGN_TOP_MID,0,82);
#endif

    // ── Sub-page containers (transparent, swipe vertical pour basculer)
    int sph=312, spy=108;
#ifdef BOARD_T4S3
    sph=350; spy=86;   // T4-S3 : en-tête 1 ligne → rows remontées, plus de hauteur tactile
#endif
    for(int i=0;i<S_NPG;i++){
        s_pg[i]=lv_obj_create(p);
        int spw=480, spx=0;
#ifdef BOARD_T4S3
        spw=600; spx=0;   // T4-S3 : les 3 sous-pages exploitent toute la largeur (contrôles tactiles)
#endif
        lv_obj_set_size(s_pg[i],spw,sph);lv_obj_set_pos(s_pg[i],spx,spy);
        lv_obj_set_style_bg_opa(s_pg[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_border_width(s_pg[i],0,0);lv_obj_set_style_pad_all(s_pg[i],0,0);
        lv_obj_clear_flag(s_pg[i],LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(s_pg[i],LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_pg[i],swipeCb,LV_EVENT_ALL,NULL);
        if(i!=0)lv_obj_add_flag(s_pg[i],LV_OBJ_FLAG_HIDDEN);}

    // ── Sub-page 0: RADAR + DISPLAY ──────────────────────────────────────────
#ifdef BOARD_T4S3
    // T4-S3 : page « fondamentaux » repensée pour les doigts (Christophe 2026-06-07).
    // Supprimés : SCALE (réglé sur l'écran radar) + ALT (toujours en feet) → place gagnée.
    // Gardés agrandis : V-FILTER (stepper), DIST/SPEED/THEME (segmented iOS), BRIGHTNESS (slider).
    {lv_obj_t*sp=s_pg[0];
    g_seg_n=0;                                  // reset registre segmented (rebuild safe)
    s_scale_v=nullptr; s_dist_v=nullptr; s_alt_v=nullptr; s_spd_v=nullptr; s_theme_v=nullptr; s_bright_v=nullptr;
    const int Y0=8, DY=72;                       // 5 rows de 52 px, pitch 72 (hauteur libérée par l'en-tête 1 ligne)
    snprintf(b,16,"%dft",g_cfg.vfilt_ft);
    s_vfilt_v=mkBigStepRow(sp,"V-FILTER",Y0+0*DY,b,2,3);
    mkSegRow(sp,"DISTANCE",Y0+1*DY,"NM","KM",   &g_cfg.dist_nm,true);   // NM ⇔ dist_nm=true
    mkSegRow(sp,"SPEED",   Y0+2*DY,"kt","km/h", &g_cfg.spd_kt, true);   // kt ⇔ spd_kt=true
    mkBigBrightRow(sp,"BRIGHTNESS",Y0+3*DY,g_cfg.brightness);
    mkSegRow(sp,"THEME",   Y0+4*DY,"LIGHT","DARK",&g_cfg.dark, false);} // LIGHT ⇔ dark=false
#else
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
#endif

    // ── Sub-page 1: TRAFFIC ──────────────────────────────────────────────────
#ifdef BOARD_T4S3
    // T4-S3 : tout en segmented tactile (SOURCE 4 / ICONS 3 / toggles ON-OFF).
    {lv_obj_t*sp=s_pg[1];
    g_segn_n=0;                 // reset registre segmented multi-options (rebuild safe)
    s_src_v=nullptr; s_grnd_v=nullptr; s_icon_sz_v=nullptr; s_aip_v=nullptr; s_heli_v=nullptr;
    const int Y0=8, DY=72;
    mkSegRowN(sp,"SOURCE",   Y0+0*DY,kSrcNames,4,&g_cfg.trf_src,0);
    mkSegRow (sp,"GROUNDED", Y0+1*DY,"OFF","ON",&g_cfg.show_grnd,false);
    mkSegRowN(sp,"ICONS",    Y0+2*DY,kIconSzNames,3,&g_cfg.icon_sz,1);
    mkSegRow (sp,"AIP",      Y0+3*DY,"OFF","ON",&g_cfg.aip_en,false);   // tap ignoré si NO DATA
    mkSegRow (sp,"HELIPORT", Y0+4*DY,"OFF","ON",&g_cfg.ad_heli,false);}
#else
    {lv_obj_t*sp=s_pg[1];
    mkSetSection(sp,"TRAFFIC",0);
    s_src_v    =mkSetRow(sp,"SOURCE",   36,kSrcNames[g_cfg.trf_src&3],10,11);
    s_grnd_v   =mkSetRow(sp,"GROUNDED", 62,g_cfg.show_grnd?"ON":"OFF",14,15);
    s_icon_sz_v=mkSetRow(sp,"ICONS SIZE",88,kIconSzNames[g_cfg.icon_sz],16,17);
    {const char*aip_v=!g_aip_loaded?"NO DATA":g_cfg.aip_en?"ON":"OFF";
    s_aip_v=mkSetRow(sp,"AIP",114,aip_v,20,21);}
    s_heli_v=mkSetRow(sp,"HELIPORT",140,g_cfg.ad_heli?"ON":"OFF",22,23);
    s_circ_v=mkSetRow(sp,"ALERT MODE",166,kCircNames[g_cfg.circuit_ovr],26,27);}  // override anticollision (AUTO/CIRC/RTE)
#endif

    // ── Sub-page 2: SYSTEM (+ ABOUT : versions & batteries, live BLE) ─────────
#ifdef BOARD_T4S3
    // T4-S3 : actions en gros boutons + toggle ; ABOUT = bloc info lisible (non tactile).
    {lv_obj_t*sp=s_pg[2];
    const lv_color_t kcol=lv_color_hex(0x4b5563);
    s_wifi_v=nullptr;
    {char t[20];snprintf(t,20,"%s %s",g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---");
     s_ac_v=mkBigBtnRow(sp,"AIRCRAFT",4,t,"EDIT",_open_aircraft_cb);}
    mkSegRow(sp,"WIFI AP",64,"OFF","ON",&g_cfg.wifi_en,false);
    mkBigBtnRow(sp,"MAINTENANCE",124,"","OPEN",_open_maintenance_cb);
    // ABOUT — info lisible (font 18), pas de contrôle tactile
    mkLblP(sp,"ABOUT",C_BRAND,&lv_font_montserrat_16,40,186);
    {lv_obj_t*hl=lv_obj_create(sp);lv_obj_set_size(hl,520,1);lv_obj_set_pos(hl,40,208);
     lv_obj_set_style_bg_color(hl,C_BRAND,0);lv_obj_set_style_bg_opa(hl,LV_OPA_COVER,0);
     lv_obj_set_style_border_width(hl,0,0);lv_obj_set_style_pad_all(hl,0,0);
     lv_obj_clear_flag(hl,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);}
    mkLblP(sp,"AT-VIEW",kcol,&lv_font_montserrat_18,40,216);
    mkLblP(sp,"v" VIEW_VERSION "  " __DATE__,C_BRAND,&lv_font_montserrat_18,300,216);
    mkLblP(sp,"AT-CORE",kcol,&lv_font_montserrat_18,40,242);
    s_sys_atcver=mkLblP(sp,"v--",C_BRAND,&lv_font_montserrat_18,300,242);
    mkLblP(sp,"AT-CORE BATT",kcol,&lv_font_montserrat_18,40,268);
    s_sys_atcbat=mkLblP(sp,"---%",C_BRAND,&lv_font_montserrat_18,300,268);
    mkLblP(sp,"AT-VIEW BATT",kcol,&lv_font_montserrat_18,40,294);
    mkLblP(sp,"N/A (future)",TGREY(),&lv_font_montserrat_18,300,294);
    {char sd_str[12];
     if(g_sd_ok)snprintf(sd_str,12,"%u GB",g_sd_gb);else strlcpy(sd_str,"NO CARD",12);
     mkLblP(sp,"SD CARD",kcol,&lv_font_montserrat_18,40,320);
     s_sd_v=mkLblP(sp,sd_str,g_sd_ok?C_GREEN:kcol,&lv_font_montserrat_18,300,320);}}
#else
    {lv_obj_t*sp=s_pg[2];
    const lv_color_t kcol=lv_color_hex(0x4b5563);
    mkSetSection(sp,"SYSTEM",0);
    {char t[20];snprintf(t,20,"%s %s",g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---");
    s_ac_v=mkSetRowBtn(sp,"AIRCRAFT",36,t,_open_aircraft_cb);}
    s_wifi_v=mkSetRow(sp,"WIFI",62,g_cfg.wifi_en?"ON":"OFF",18,19);
    mkSetRowBtn(sp,"MAINTENANCE",88,"",_open_maintenance_cb);
    {char sd_str[12];
     if(g_sd_ok)snprintf(sd_str,12,"%u GB",g_sd_gb);else strlcpy(sd_str,"NO CARD",12);
     mkLblP(sp,"SDCARD (AT-CORE)",kcol,&lv_font_montserrat_14,55,114);
     s_sd_v=mkLblP(sp,sd_str,g_sd_ok?C_GREEN:kcol,&lv_font_montserrat_14,295,114);}
    // ABOUT : versions + batteries (ATV batterie = futur, pas de capteur aujourd'hui)
    mkSetSection(sp,"ABOUT",150);
    mkLblP(sp,"AT-VIEW",kcol,&lv_font_montserrat_14,55,186);
    mkLblP(sp,"v" VIEW_VERSION "  " __DATE__,C_BRAND,&lv_font_montserrat_14,180,186);
    mkLblP(sp,"AT-CORE",kcol,&lv_font_montserrat_14,55,212);
    s_sys_atcver=mkLblP(sp,"v--",C_BRAND,&lv_font_montserrat_14,180,212);
    mkLblP(sp,"AT-CORE BATT",kcol,&lv_font_montserrat_14,55,238);
    s_sys_atcbat=mkLblP(sp,"---%",C_BRAND,&lv_font_montserrat_14,180,238);
    mkLblP(sp,"AT-VIEW BATT",kcol,&lv_font_montserrat_14,55,264);
    mkLblP(sp,"N/A (future)",TGREY(),&lv_font_montserrat_14,180,264);}
#endif

    // ── Footer
#ifdef BOARD_T4S3
    // T4-S3 : pas de footer — versions & batteries vivent dans la page System,
    // le geste debug caché passe par le titre, l'oubli de pairing par la page #01.
    (void)0;
#else
    // logo AT-VIEW + battery + version
    lv_obj_t*lVw=lv_img_create(p);
    lv_img_set_src(lVw,&img_logo_atview);
    lv_obj_align(lVw,LV_ALIGN_TOP_MID,0,422);
    // Long-press logo AT-VIEW = oublie la pair BLE AT-CORE + reboot.
    // Utile quand on change de carte AT-CORE (MAC différente du nouveau
    // hardware, le filtre par MAC bloque sinon).
    lv_obj_add_flag(lVw,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lVw,_cbForgetPair,LV_EVENT_LONG_PRESSED,NULL);
    r_p2_bat=mkLbl(p,"AT-CORE : ---%",TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,448);
    lv_obj_t*ver=mkLbl(p,VIEW_VER_STR,TGREY(),&lv_font_montserrat_12,LV_ALIGN_TOP_MID,0,464);
    lv_obj_add_flag(ver,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_opa(ver,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(ver,cbDebugLongPress,LV_EVENT_LONG_PRESSED,NULL);
#endif
#endif // #if 0 — legacy round s_pg (remplacé par buildSettingsPageT4 unifié)
    }

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
    lv_obj_add_event_cb(g_dbgPage,swipeCb,LV_EVENT_ALL,NULL);
    // Le chip Start/End flight vit sur lv_layer_top et « bubble » ses appuis ici →
    // un swipe démarré sur le chip navigue quand même (swipeCb garde les overlays).
    lv_obj_add_event_cb(lv_layer_top(),swipeCb,LV_EVENT_ALL,NULL);}

// ── Dead reckoning — radar blips (called every loop for smooth movement) ──────
// Traffic e.spd_kt = knots (AT-CORE clé "s"). Own g_status.spd = km/h (AT-CORE
// clé "spd" = kt*1.852). Donc conversions différentes : own /3.6, trafic *0.5144.
// Capped at 10s to avoid runaway extrapolation on BLE dropout.
// ── Moteur d'alerte anticollision (écran) — CPA + bulle proximité, circuit-aware ──
// Modèle TCAS : seuil TEMPS (tCPA = time-to-closest-approach) + DMOD (bulle de proximité).
// Tout gaté par l'écart vertical |Δalt|. En circuit (auto-détecté) : rouge-only + bulle 0,3 nm
// (l'espacement normal du tour de piste <1 nm sinon = alarme permanente). Sort un payload
// {level,clock,dist,Δalt,closing} prêt pour l'audio casque futur.
enum { THREAT_NONE=0, THREAT_ORANGE=1, THREAT_RED=2 };
struct ThreatInfo { uint8_t level; int clock; int dist_m; int dalt_ft; int closing_kt; bool valid; char cs[9]; };
static ThreatInfo g_threat = {};
static uint8_t    g_trf_threat[MAX_TRF] = {};   // niveau par cible (couleur icône radar), index aligné g_traffic.t
static bool       g_circuit_mode = false;
static float      g_field_elev_m = -99999.0f;   // élévation terrain (capture sol) pour l'AGL

// Distance horizontale (nm) à l'aérodrome AIP le plus proche (near-field gate du mode circuit).
static float aipNearestAdNm(){
    if(!g_aip_ads || g_aip_ad_cnt==0 || !g_status.gps_fix) return 9999.0f;
    float best=9999.0f, lat=g_status.lat, lon=g_status.lon, cl=cosf(lat*(float)M_PI/180.0f);
    for(int i=0;i<g_aip_ad_cnt;i++){
        float dlat=(g_aip_ads[i].lat_e6/1e6f-lat)*60.0f;
        float dlon=(g_aip_ads[i].lon_e6/1e6f-lon)*60.0f*cl;
        float d=sqrtf(dlat*dlat+dlon*dlon);
        if(d<best)best=d;
    }
    return best;
}

void alertEngineTick(){
    g_threat = {};
    for(int i=0;i<MAX_TRF;i++) g_trf_threat[i]=THREAT_NONE;
    if(!g_status.valid || !g_status.gps_fix) return;

    float ownGS_kt = g_status.spd/1.852f;            // km/h → kt
    if(ownGS_kt < 5.0f) g_field_elev_m = g_status.alt;   // capture élévation terrain au sol (suit le terrain courant)

    float agl_ft = (g_field_elev_m>-90000.0f) ? ((float)g_status.alt - g_field_elev_m)*3.28084f : 99999.0f;
    // (port T-RGB) Le scan AIP (jusqu'à 5500 aérodromes × sqrt) est LOURD : appelé à 5 Hz sur
    // toutes les pages il affamait le DMA du panneau RGB (lignes horizontales). On le CACHE
    // (≤ tous les 2 s) et on le GATE (uniquement quand un circuit est plausible : bas + vitesse
    // modérée). En croisière haute/au sol → pas de scan, valeur "loin".
    static float s_nearAdNm=9999.0f; static uint32_t s_nearAdT=0;
    bool maybeCircuit = (agl_ft < 2000.0f) && ownGS_kt > 30.0f && ownGS_kt < 170.0f;
    if(maybeCircuit){ if(millis()-s_nearAdT > 2000){ s_nearAdT=millis(); s_nearAdNm=aipNearestAdNm(); } }
    else s_nearAdNm=9999.0f;
    bool autoDet = s_nearAdNm < 5.0f && agl_ft < 1500.0f && ownGS_kt > 40.0f && ownGS_kt < 160.0f;
    // Override manuel (Settings/radar) : 1=CIRCUIT forcé · 2=ROUTE forcé · 0=AUTO (auto-détection)
    g_circuit_mode = (g_cfg.circuit_ovr==1) ? true : (g_cfg.circuit_ovr==2) ? false : autoDet;
    bool circ = g_circuit_mode;

    const float NM = 1852.0f;
    float vOrg=500.0f,        vRed=circ?300.0f:400.0f;          // gate vertical (ft)
    float tOrg=60.0f,         tRed=circ?25.0f:30.0f;            // tCPA (s)
    float dOrg=1.0f*NM,       dRed=circ?0.3f*NM:0.5f*NM;        // dCPA miss (m)
    float bOrg=1.0f*NM,       bRed=circ?0.3f*NM:0.5f*NM;        // bulle proximité (m)

    float ot=g_status.hdg*(float)M_PI/180.0f, ogs=g_status.spd/3.6f;   // own m/s
    float ovx=ogs*sinf(ot), ovy=ogs*cosf(ot);

    for(int i=0;i<g_traffic.count && i<MAX_TRF;i++){
        TrafficEntry&e=g_traffic.t[i];
        if(!e.visible) continue;
        float dalt=fabsf((float)e.alt_m*100.0f);                // |Δalt| ft (alt_m = centaines de ft)
        float br=e.bear_deg*(float)M_PI/180.0f;
        float Rx=e.dist_m*sinf(br), Ry=e.dist_m*cosf(br);       // position relative (m, est/nord)
        float tt=e.hdg_deg*(float)M_PI/180.0f, tgs=e.spd_kt*0.514444f;
        float vrx=tgs*sinf(tt)-ovx, vry=tgs*cosf(tt)-ovy;       // vitesse relative
        float vr2=vrx*vrx+vry*vry;
        float tcpa=(vr2>0.05f)?-(Rx*vrx+Ry*vry)/vr2:-1.0f;      // <=0 → s'éloigne/parallèle
        float dcpa=e.dist_m;
        if(tcpa>0){ float cx=Rx+vrx*tcpa, cy=Ry+vry*tcpa; dcpa=sqrtf(cx*cx+cy*cy); }
        int closing=(int)(sqrtf(vr2)/0.514444f);                // kt

        uint8_t lvl=THREAT_NONE;
        if(!circ && dalt<=vOrg && tcpa>0 && tcpa<=tOrg && dcpa<=dOrg) lvl=THREAT_ORANGE;     // règle TEMPS
        if(dalt<=vRed && tcpa>0 && tcpa<=tRed && dcpa<=dRed)          lvl=THREAT_RED;
        if(!circ && dalt<=vOrg && e.dist_m<=bOrg && lvl<THREAT_ORANGE) lvl=THREAT_ORANGE;    // bulle PROXIMITÉ
        if(dalt<=vRed && e.dist_m<=bRed)                              lvl=THREAT_RED;

        g_trf_threat[i]=lvl;
        if(lvl>g_threat.level){
            int clk=((e.bear_deg-g_status.hdg)%360+360)%360;
            int hr=((clk+15)/30)%12; if(hr==0)hr=12;            // position horaire relative au nez
            g_threat.level=lvl; g_threat.clock=hr; g_threat.dist_m=e.dist_m;
            g_threat.dalt_ft=e.alt_m*100; g_threat.closing_kt=closing;
            g_threat.valid=true; strlcpy(g_threat.cs,e.cs,9);
        }
    }
}

void updateRadarDR(){
    // Chip MODE ALERTE : texte + couleur = état EFFECTIF (CIRC/RTE forcé, ou AUTO actif/inactif).
    if(r_circ_lbl){
        lv_label_set_text(r_circ_lbl,kCircNames[g_cfg.circuit_ovr]);
        lv_color_t cc=g_cfg.circuit_ovr==1?C_CYAN:g_cfg.circuit_ovr==2?C_AMBER
                      :(g_circuit_mode?C_CYAN:lv_color_hex(0x9ca3af));
        lv_obj_set_style_text_color(r_circ_lbl,cc,0);
    }
    if(!g_traffic.valid||!g_status.valid)return;
    // Vieillissement signal (2026-06-06) : ss=false (>10 s sans échange UDP côté
    // boîtier) → trafic GRIS MOYEN, le dead reckoning continue de l'extrapoler
    // sur cap/vitesse dernière connue ; à perte+20 s (= 30 s réels) sans reprise
    // → trafic EFFACÉ. Retour ss=true → couleurs normales immédiates.
    bool ssStale = !g_status.ss_ok || g_status.ss_mode==1;  // (v20) gris aussi en mode sol/idle (ss reste OK, mais trafic dégradé/lent)
    bool ssDead  = ssStale && g_ss_lost_ms && (millis()-g_ss_lost_ms > 20000);
    // dt + compensation du mouvement propre sont désormais PAR AVION (base_ms par
    // entrée, posé dans parseTraffic) : la DR ne se réinitialise plus à chaque paquet
    // BLE 1 Hz, elle court depuis le dernier VRAI fix SafeSky de chaque avion → fin du
    // surplace/recul. Cap d'extrapolation 30 s (phase grise : le trafic suit sa route).
    uint32_t now=millis();
    float our_spd_ms=(float)g_status.spd/3.6f;   // km/h → m/s (own, pas knots)
    float our_rad=(float)radarEffHdg()*(float)M_PI/180.0f;
    char b[32];
    for(int i=0;i<MAX_TRF;i++){
        if(i<g_traffic.count){
            TrafficEntry&e=g_traffic.t[i];
            // dt PROPRE à cet avion : depuis son dernier vrai fix (base_ms), pas depuis
            // le dernier paquet BLE → extrapolation continue, sans re-snap chaque seconde.
            float dt=fminf((float)(now-e.base_ms)/1000.0f,30.0f);
            float our_dx=our_spd_ms*dt*sinf(our_rad);
            float our_dy=our_spd_ms*dt*cosf(our_rad);
            // Filtre ground : masque les aéronefs à vitesse < 20 kt (taxi/stationnement).
            // Seuil 20 kt valable si AT-CORE fournit le champ "s" (spd_kt) dans le JSON TRAFFIC.
            // Si "s" absent, défaut = 100 kt → l'avion reste visible même filtré. Normal.
            float scale_m=(float)g_cfg.scale_nm*1852.0f;
            // (juin 2026) V-FILTER LOCAL : on masque le trafic hors de la bande verticale
            // choisie (±vfilt_ft). e.alt_m = delta en CENTAINES de pieds (AT-CORE = alt_rel_ft/100)
            // → delta_ft = |alt_m|×100. Filtre 100 % AT-VIEW (AT-CORE inchangé).
            bool vfiltOut = (g_cfg.vfilt_ft>0 && abs(e.alt_m)*100 > (int)g_cfg.vfilt_ft);
            if((!g_cfg.show_grnd&&e.spd_kt<20)||(e.dist_m>scale_m*RAD_OVERSCAN)||ssDead||vfiltOut){
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
            // Lissage (2ᵉ couche) : on GARDE la prédiction DR (ex,ny déjà avancés) mais on
            // EASE le saut de recalage au ré-accrochage au lieu de le snapper → recalage
            // invisible. Filtre exponentiel @5 Hz (200 ms) ; latence en régime stable
            // négligeable (delta DR sub-pixel/frame). Au-delà de 1.5 km d'écart (réapparition
            // d'un avion masqué, outlier SafeSky) on snap pour éviter un long glissement.
            const float SM_A=0.45f;
            float gx=ex-e.disp_ex, gy=ny-e.disp_ey;
            if(!e.disp_init || gx*gx+gy*gy>1500.0f*1500.0f){ e.disp_ex=ex; e.disp_ey=ny; e.disp_init=true; }
            else { e.disp_ex+=gx*SM_A; e.disp_ey+=gy*SM_A; }
            ex=e.disp_ex; ny=e.disp_ey;
            // Back to polar
            float dr_dist=sqrtf(ex*ex+ny*ny);
            // Sorti de la zone visible après extrapolation DR → MASQUÉ. La limite
            // inclut l'overscan (T4 : jusqu'aux bords écran ≈ 2× l'échelle ; T-RGB :
            // l'échelle = le cercle, le verre rond clippe le débord d'icône).
            if(dr_dist>scale_m*RAD_OVERSCAN){
                lv_obj_add_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
            } else {
            float dr_bear=atan2f(ex,ny)*180.0f/(float)M_PI;
            if(dr_bear<0.0f)dr_bear+=360.0f;
            // Heading-up projection on screen
            int rb=((int)dr_bear-radarEffHdg()+360)%360;
            float brd=(float)rb*(float)M_PI/180.0f;
            float dpx=dr_dist*(float)RAD_R/scale_m;   // ≤ RAD_R par le test ci-dessus
            int sx=(int)(RAD_CX+sinf(brd)*dpx);
            int sy=(int)(RAD_CY-cosf(brd)*dpx);
            int rel_hdg=((e.hdg_deg-radarEffHdg())%360+360)%360;
            float hr=(float)rel_hdg*(float)M_PI/180.0f;
            float cs=cosf(hr),sn=sinf(hr);
            // Perte signal → gris moyen uniforme ; sinon couleur = NIVEAU DE MENACE (moteur
            // anticollision) au lieu de la distance brute : rouge/ambre seulement si vraie menace
            // (CPA/bulle, gaté vertical), gris-clair sinon → l'œil va droit au danger.
            lv_color_t col=ssStale?lv_color_hex(0x9ca3af)
                          :(g_trf_threat[i]==THREAT_RED?C_RED
                           :g_trf_threat[i]==THREAT_ORANGE?C_AMBER:TFG());
            if(e.type!=r_trf_last_type[i]){
                lv_img_set_src(r_trf_img[i],getAircraftIcon(e.type));
                lv_img_set_zoom(r_trf_img[i],kIconZoom[g_cfg.icon_sz]); // (fix) set_src réinit le zoom → ré-appliquer la taille S/M/L (sinon retour M à l'apparition d'une cible, ex. alerte)
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
            int ih=kIconHalf[g_cfg.icon_sz];
            float nose_r=(float)ih*0.5f;
            // Vecteur "position dans 1 min" À L'ÉCHELLE du radar : kt/60 = nm/min, ×px_per_nm.
            // (FIX 2026-06-08) plus de cap fixe 35px qui le FIGEAIT au zoom (il ne grandissait
            // plus quand on zoomait). Borné seulement au rayon radar (ne sort pas du scope) et
            // planchonné juste devant le nez de l'icône (sinon trait inversé en taille L).
            float vect_px=fmaxf(nose_r+6.f,fminf((float)e.spd_kt/60.0f*px_per_nm,(float)RAD_R));
            // Trait fin partant DEVANT l'avion (nez), bout = position dans 1 min. pos(0,0)=absolu.
            r_vect_pts[i][0]={(lv_coord_t)(sx+(int)(nose_r*sn)),(lv_coord_t)(sy-(int)(nose_r*cs))};
            r_vect_pts[i][1]={(lv_coord_t)(sx+(int)(vect_px*sn)),(lv_coord_t)(sy-(int)(vect_px*cs))};
            lv_obj_set_pos(r_trf_vect[i],0,0);
            lv_line_set_points(r_trf_vect[i],r_vect_pts[i],2);
            lv_obj_clear_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
            if(g_cfg.show_cs){   // (CONFIG) decluttering : callsign optionnel
              lv_obj_set_pos(r_radar_cs[i],sx+ih+6,sy-22);lv_label_set_text(r_radar_cs[i],e.cs);  // à droite de l'icône agrandie (ih=demi-taille)
              lv_obj_set_style_text_color(r_radar_cs[i],
                ssStale?lv_color_hex(0x9ca3af):(e.visible?TFG():C_AMBER),0);
              lv_obj_clear_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
            } else lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);
            if(g_cfg.show_vdiff){ // (CONFIG) decluttering : diff verticale optionnelle
              snprintf(b,32,"%+d",e.alt_m); // already delta in hundreds of feet from AT-CORE
              lv_obj_set_pos(r_radar_alt[i],sx+ih+6,sy+4);lv_label_set_text(r_radar_alt[i],b);
              lv_obj_set_style_text_color(r_radar_alt[i],col,0);
              lv_obj_clear_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
            } else lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);
            } // end else (dans l'échelle post-DR)
            } // end else (not grounded)
        }else{
            lv_obj_add_flag(r_trf_img[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_trf_vect[i],LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(r_radar_cs[i],LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(r_radar_alt[i],LV_OBJ_FLAG_HIDDEN);}}}

// ── Update all live data ──────────────────────────────────────────────────────
// ── Cycle de vol (Start/Stop manuel + countdown d'arrêt) ─────────────────────
// Piloté par g_status.flt_st (0=sol 1=en vol 2=arrêt imminent). Bannière "FLIGHT
// STARTED" au décollage, chip "Start flight" au sol, overlay "stopping in Ns" + Cancel.
static lv_obj_t* g_fb_ov=nullptr;     // bannière FLIGHT STARTED
static uint32_t  g_fb_t0=0;
static lv_obj_t* g_startchip=nullptr; // chip Start/End flight
static uint8_t   g_chip_kind=0;       // 0=aucun 1=Start flight 2=End flight
static lv_obj_t* g_stop_ov=nullptr;   // overlay arrêt imminent
static lv_obj_t* g_stop_lbl=nullptr;
static uint32_t  g_stop_seen_ms=0;
static uint8_t   g_prev_flt_st=0xFF;

static void _startflight_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("start_flight"); }
static void _endflight_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("stop_flight"); }
static void _continueflight_cb(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) sendCtl("continue_flight"); }

static void fbShow(){
    if(g_fb_ov)return;
    g_fb_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_fb_ov,360,120);lv_obj_align(g_fb_ov,LV_ALIGN_CENTER,HDG_DX,0);   // centré sur le radar
    lv_obj_set_style_bg_color(g_fb_ov,lv_color_hex(0x0d1117),0);
    lv_obj_set_style_bg_opa(g_fb_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(g_fb_ov,C_GREEN,0);
    lv_obj_set_style_border_width(g_fb_ov,3,0);
    lv_obj_set_style_radius(g_fb_ov,14,0);
    lv_obj_clear_flag(g_fb_ov,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t*l=lv_label_create(g_fb_ov);lv_label_set_text(l,"FLIGHT STARTED");
    lv_obj_set_style_text_color(l,C_GREEN,0);
    lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);
    lv_obj_center(l);
    g_fb_t0=millis();
}
static void fbHide(){ if(g_fb_ov){lv_obj_del(g_fb_ov);g_fb_ov=nullptr;} }

static void chipShow(uint8_t kind){   // 1=Start flight 2=End flight
    if(g_startchip && g_chip_kind==kind) return;
    if(g_startchip){ lv_obj_del(g_startchip); g_startchip=nullptr; }
    g_chip_kind=kind;
    g_startchip=lv_btn_create(lv_layer_top());
#ifdef BOARD_T4S3
    // (2026-06-05) Start/End flight dans la COLONNE GAUCHE, sous l'icône BLE
    // (y=252+~40) — même emplacement que le panneau STOP (mutuellement exclusifs :
    // chip au sol, STOP en vol) → zone "action vol" unique, gros format tactile.
    lv_obj_set_size(g_startchip,130,56);
    lv_obj_set_pos(g_startchip,RLC_X,300);
#else
    lv_obj_set_size(g_startchip,CHIP_W,CHIP_H);
    lv_obj_align(g_startchip,LV_ALIGN_TOP_MID,HDG_DX,62);   // au-dessus de la mire radar
#endif
    lv_obj_set_style_bg_color(g_startchip,kind==2?C_RED:C_BRAND,0);lv_obj_set_style_radius(g_startchip,24,0);
    lv_obj_set_style_border_width(g_startchip,0,0);lv_obj_set_style_shadow_opa(g_startchip,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(g_startchip,kind==2?_endflight_cb:_startflight_cb,LV_EVENT_CLICKED,NULL);
    lv_obj_add_flag(g_startchip,LV_OBJ_FLAG_EVENT_BUBBLE);   // swipe doit passer même en partant du chip
    lv_obj_t*l=lv_label_create(g_startchip);lv_label_set_text(l,kind==2?"End flight":"Start flight");
    lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(l,&CHIP_FONT,0);lv_obj_center(l);
}
static void chipHide(){ if(g_startchip){lv_obj_del(g_startchip);g_startchip=nullptr;g_chip_kind=0;} }

static void stopShow(){
    if(g_stop_ov)return;
    g_stop_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_stop_ov,420,200);lv_obj_center(g_stop_ov);
    lv_obj_set_style_bg_color(g_stop_ov,lv_color_hex(0x0d1117),0);
    lv_obj_set_style_bg_opa(g_stop_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(g_stop_ov,C_AMBER,0);
    lv_obj_set_style_border_width(g_stop_ov,3,0);
    lv_obj_set_style_radius(g_stop_ov,14,0);
    lv_obj_clear_flag(g_stop_ov,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t*t=lv_label_create(g_stop_ov);lv_label_set_text(t,"Landing detected");
    lv_obj_set_style_text_color(t,C_AMBER,0);
    lv_obj_set_style_text_font(t,&lv_font_montserrat_20,0);
    lv_obj_align(t,LV_ALIGN_TOP_MID,0,22);
    g_stop_lbl=lv_label_create(g_stop_ov);lv_label_set_text(g_stop_lbl,"Stopping flight in 5s");
    lv_obj_set_style_text_color(g_stop_lbl,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(g_stop_lbl,&lv_font_montserrat_16,0);
    lv_obj_align(g_stop_lbl,LV_ALIGN_TOP_MID,0,74);
    lv_obj_t*b=lv_btn_create(g_stop_ov);
    lv_obj_set_size(b,240,54);lv_obj_align(b,LV_ALIGN_BOTTOM_MID,0,-22);
    lv_obj_set_style_bg_color(b,C_GREEN,0);lv_obj_set_style_radius(b,27,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(b,_continueflight_cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t*bl=lv_label_create(b);lv_label_set_text(bl,"Cancel - keep flying");
    lv_obj_set_style_text_color(bl,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(bl,&lv_font_montserrat_16,0);lv_obj_center(bl);
    g_stop_seen_ms=millis();
}
static void stopHide(){ if(g_stop_ov){lv_obj_del(g_stop_ov);g_stop_ov=nullptr;g_stop_lbl=nullptr;} }

// ── (juin 2026) Action sheet Start/Stop sur APPUI LONG du radar ─────────────────
// Le radar n'a plus de bouton Start (vol auto à 15 kt) ni de panneau STOP permanent.
// Un appui long (~400 ms LVGL) sur la mire ouvre une feuille d'action contextuelle :
//   au sol → « Start flight » · en vol → « Stop flight ». Sert surtout aux tests sol.
static lv_obj_t* g_radar_act_ov=nullptr;
static void radarActHide(){ if(g_radar_act_ov){lv_obj_del(g_radar_act_ov);g_radar_act_ov=nullptr;} }
static void _radarAct_start(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED){ sendCtl("start_flight"); radarActHide(); } }
static void _radarAct_stop (lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED){ sendCtl("stop_flight");  radarActHide(); } }
static void _radarAct_close(lv_event_t*e){ if(lv_event_get_code(e)==LV_EVENT_CLICKED) radarActHide(); }
static void radarActShow(){
    if(g_radar_act_ov) return;
    // Backdrop plein écran semi-opaque : tap HORS panneau = fermer.
    g_radar_act_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_radar_act_ov,SCR_W,SCR_H);lv_obj_center(g_radar_act_ov);
    lv_obj_set_style_bg_color(g_radar_act_ov,lv_color_hex(0x000000),0);
    lv_obj_set_style_bg_opa(g_radar_act_ov,LV_OPA_50,0);
    lv_obj_set_style_border_width(g_radar_act_ov,0,0);lv_obj_set_style_radius(g_radar_act_ov,0,0);
    lv_obj_set_style_pad_all(g_radar_act_ov,0,0);
    lv_obj_clear_flag(g_radar_act_ov,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(g_radar_act_ov,_radarAct_close,LV_EVENT_CLICKED,NULL);
    // Panneau central (CLICKABLE → absorbe le tap, ne ferme pas).
    lv_obj_t* pan=lv_obj_create(g_radar_act_ov);
    lv_obj_set_size(pan,300,210);lv_obj_center(pan);
    lv_obj_set_style_bg_color(pan,TBG(),0);lv_obj_set_style_bg_opa(pan,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(pan,TFG(),0);lv_obj_set_style_border_width(pan,2,0);
    lv_obj_set_style_radius(pan,16,0);lv_obj_set_style_shadow_opa(pan,LV_OPA_TRANSP,0);
    lv_obj_clear_flag(pan,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(pan,LV_OBJ_FLAG_CLICKABLE);
    // Bouton principal contextuel : Start (au sol) / Stop (en vol).
    bool flying=(g_status.valid && g_status.flt_st!=0);
    lv_obj_t* b=lv_btn_create(pan);
    lv_obj_set_size(b,256,76);lv_obj_align(b,LV_ALIGN_TOP_MID,0,10);
    lv_obj_set_style_bg_color(b,flying?C_RED:C_GREEN,0);lv_obj_set_style_radius(b,16,0);
    lv_obj_set_style_border_width(b,0,0);lv_obj_set_style_shadow_opa(b,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(b,flying?_radarAct_stop:_radarAct_start,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(b);lv_label_set_text(l,flying?"Stop flight":"Start flight");
     lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_24,0);lv_obj_center(l);}
    // Bouton Close.
    lv_obj_t* c=lv_btn_create(pan);
    lv_obj_set_size(c,256,60);lv_obj_align(c,LV_ALIGN_BOTTOM_MID,0,-10);
    lv_obj_set_style_bg_color(c,TGREY(),0);lv_obj_set_style_radius(c,14,0);
    lv_obj_set_style_border_width(c,0,0);lv_obj_set_style_shadow_opa(c,LV_OPA_TRANSP,0);
    lv_obj_add_event_cb(c,_radarAct_close,LV_EVENT_CLICKED,NULL);
    {lv_obj_t*l=lv_label_create(c);lv_label_set_text(l,"Close");
     lv_obj_set_style_text_color(l,TFG(),0);
     lv_obj_set_style_text_font(l,&lv_font_montserrat_20,0);lv_obj_center(l);}
}
static void cbRadarLongPress(lv_event_t*e){
    if(lv_event_get_code(e)!=LV_EVENT_LONG_PRESSED) return;
    if(g_page!=1) return;                                   // radar uniquement
    // (juin 2026) FIX swipe radar : un appui long pendant un SWIPE (doigt qui a bougé depuis
    // l'appui) ne doit PAS ouvrir l'action sheet, sinon le geste de navigation est avalé.
    lv_indev_t*indev=lv_indev_get_act();
    if(indev){ lv_point_t pt; lv_indev_get_point(indev,&pt);
        if(g_swipe_sx>=0 && (abs((int)pt.x-(int)g_swipe_sx)>22 || abs((int)pt.y-(int)g_swipe_sy)>22)) return; }
    // Pas d'ouverture si un autre overlay/modal est déjà à l'écran.
    if(g_radar_act_ov||g_stop_ov||g_up_ov||g_fb_ov||g_maint_ov||g_vols_ov||g_pair_ov||g_auth_ov) return;
    radarActShow();
}

// Met à jour bannière/chip/overlay selon g_status.flt_st (reçu par BLE STATUS).
void updFlightState(){
    if(!g_status.valid){ fbHide();chipHide();stopHide();g_prev_flt_st=0xFF;return; }
    uint8_t st=g_status.flt_st, ph=g_status.flt_phase;

    if(g_prev_flt_st==0 && st==1) fbShow();           // décollage → bannière
    if(g_fb_ov && millis()-g_fb_t0>4000) fbHide();    // auto-dismiss 4s

    if(st==2){                                        // arrêt imminent → overlay + countdown
        if(!g_stop_ov) stopShow();
        if(g_stop_lbl){
            int left=5-(int)((millis()-g_stop_seen_ms)/1000); if(left<0)left=0;
            char b[32];snprintf(b,sizeof(b),"Stopping flight in %ds",left);
            lv_label_set_text(g_stop_lbl,b);
        }
    } else if(g_stop_ov) stopHide();

    // Chip Start flight (au sol) / End flight (en vol) : vue radar, aucun overlay ouvert.
    // "End flight" force l'arrêt immédiat (stop_flight) — fin de vol quoiqu'il arrive.
    // (v6) Start flight dispo aussi APRÈS un vol terminé, pas seulement en FLYING :
    // sans ça, immobile + sans fix après un leg (escale, GPS perdu sous hangar) → ni
    // START ni STOP → impossible de relancer un vol depuis l'écran (l'auto-restart S3
    // exige taxi >15 kt + fix). On autorise tous les états SAUF UPLOADING(3) en cours
    // (FLYING0/ENDED1/CLOSED2/UPLOADED4/FAIL5). Le handler S3 start_flight régénère le
    // fid + repasse FLYING depuis n'importe quel état terminal.
#ifdef BOARD_T4S3
    // (juin 2026) Radar épuré : plus de chip Start ni panneau STOP permanents.
    // Start/Stop = APPUI LONG sur le radar (action sheet contextuelle, cf cbRadarLongPress).
    // On conserve la bannière FLIGHT STARTED (fbShow) + le countdown atterrissage (stopShow).
    (void)ph;
    if(r_flt_stop) lv_obj_add_flag(r_flt_stop,LV_OBJ_FLAG_HIDDEN);
    chipHide();
#else
    bool fltIdle = (ph != 3);
    bool base = g_connected && fltIdle && g_page==1   // page radar
              && !g_up_ov && !g_stop_ov && !g_fb_ov && !g_maint_ov && !g_vols_ov
              && !g_pair_ov && !g_auth_ov;
    // Au sol : bouton "Start flight" (radar vide). En vol : petit panneau STOP discret
    // (emplacement ex-ADS-B) — plus de gros bouton "End flight" au milieu de la mire.
    bool showStop = (g_connected && st==1 && ph==0 && g_page==1);
    if(r_flt_stop){ if(showStop) lv_obj_clear_flag(r_flt_stop,LV_OBJ_FLAG_HIDDEN);
                    else          lv_obj_add_flag(r_flt_stop,LV_OBJ_FLAG_HIDDEN); }
    if(base && st==0) chipShow(1);   // Start flight (au sol)
    else chipHide();
#endif

    g_prev_flt_st=st;
}

// ── OTA firmware overlay (MAJ par hotspot cloud) ─────────────────────────────
static lv_obj_t* g_ota_ov=nullptr,*g_ota_lbl=nullptr,*g_ota_bar=nullptr;
static uint8_t   g_ota_acked=0;   // état terminal (4/5) acquitté → ne plus ré-afficher (le boîtier le rediffuse en boucle)
static void hideOtaOverlay();     // fwd (utilisé par le tap-to-close de mkOtaOverlay)
static uint32_t  g_ota_done_ms=0;
static void mkOtaOverlay(){
    if(g_ota_ov)return;
    g_ota_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_ota_ov,400,210);lv_obj_center(g_ota_ov);
    lv_obj_set_style_bg_color(g_ota_ov,lv_color_hex(0x0d1117),0);
    lv_obj_set_style_bg_opa(g_ota_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(g_ota_ov,C_BRAND,0);lv_obj_set_style_border_width(g_ota_ov,2,0);
    lv_obj_set_style_radius(g_ota_ov,12,0);lv_obj_clear_flag(g_ota_ov,LV_OBJ_FLAG_SCROLLABLE);
    // Tap pour fermer (états terminaux) — l'overlay restait coincé sur "Already
    // up to date" (ota=5 rediffusé en boucle par le boîtier, cf g_ota_acked).
    lv_obj_add_flag(g_ota_ov,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_ota_ov,[](lv_event_t*e){
        if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return;
        if(g_status.ota>=4)g_ota_acked=g_status.ota;
        hideOtaOverlay();
    },LV_EVENT_CLICKED,NULL);
    lv_obj_t*t=lv_label_create(g_ota_ov);lv_label_set_text(t,"FIRMWARE UPDATE");
    lv_obj_set_style_text_color(t,C_BRAND,0);lv_obj_set_style_text_font(t,&lv_font_montserrat_20,0);
    lv_obj_align(t,LV_ALIGN_TOP_MID,0,22);
    g_ota_lbl=lv_label_create(g_ota_ov);lv_label_set_text(g_ota_lbl,"Checking...");
    lv_obj_set_style_text_color(g_ota_lbl,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(g_ota_lbl,&lv_font_montserrat_16,0);
    lv_obj_align(g_ota_lbl,LV_ALIGN_TOP_MID,0,80);
    g_ota_bar=lv_bar_create(g_ota_ov);lv_obj_set_size(g_ota_bar,320,16);
    lv_obj_align(g_ota_bar,LV_ALIGN_TOP_MID,0,135);
    lv_bar_set_range(g_ota_bar,0,100);lv_bar_set_value(g_ota_bar,0,LV_ANIM_OFF);
    lv_obj_set_style_bg_color(g_ota_bar,lv_color_hex(0x1f2937),0);
    lv_obj_set_style_bg_color(g_ota_bar,C_BRAND,LV_PART_INDICATOR);
}
static void hideOtaOverlay(){ if(g_ota_ov){lv_obj_del(g_ota_ov);g_ota_ov=nullptr;g_ota_lbl=nullptr;g_ota_bar=nullptr;} g_ota_done_ms=0; }
// Phase B (2026-06-25) — prompt « MAJ firmware dispo » : quand le boîtier a détecté oav>fwv
// (check au boot AT-CORE v32) → propose Update now / Later. Distinct de l'overlay progression.
// "Later" arme g_otaav_acked → plus de prompt cette session (re-proposé au prochain boot écran).
static lv_obj_t* g_otaav_ov=nullptr;
static bool g_otaav_acked=false;
void updOtaAvailPrompt(){
    if(!g_status.valid || g_status.oav<=g_status.fwv || g_otaav_acked || g_status.ota!=0){
        if(g_otaav_ov){lv_obj_del(g_otaav_ov);g_otaav_ov=nullptr;}
        return;
    }
    if(g_otaav_ov) return;   // déjà affiché
    g_otaav_ov=lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_otaav_ov,360,200);lv_obj_center(g_otaav_ov);
    lv_obj_set_style_bg_color(g_otaav_ov,lv_color_hex(0x0d1117),0);lv_obj_set_style_bg_opa(g_otaav_ov,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(g_otaav_ov,C_AMBER,0);lv_obj_set_style_border_width(g_otaav_ov,2,0);
    lv_obj_set_style_radius(g_otaav_ov,12,0);lv_obj_clear_flag(g_otaav_ov,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t*t=lv_label_create(g_otaav_ov);lv_label_set_text(t,"FIRMWARE UPDATE");
    lv_obj_set_style_text_color(t,C_AMBER,0);lv_obj_set_style_text_font(t,&lv_font_montserrat_20,0);
    lv_obj_align(t,LV_ALIGN_TOP_MID,0,18);
    char b[48]; snprintf(b,sizeof(b),"v%d available  (now v%d)",g_status.oav,g_status.fwv);
    lv_obj_t*l=lv_label_create(g_otaav_ov);lv_label_set_text(l,b);
    lv_obj_set_style_text_color(l,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(l,&lv_font_montserrat_16,0);
    lv_obj_align(l,LV_ALIGN_TOP_MID,0,66);
    {lv_obj_t*bn=lv_btn_create(g_otaav_ov);lv_obj_set_size(bn,150,54);lv_obj_align(bn,LV_ALIGN_BOTTOM_LEFT,18,-18);
     lv_obj_set_style_bg_color(bn,C_GREEN,0);lv_obj_set_style_radius(bn,10,0);lv_obj_set_style_shadow_opa(bn,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(bn,0,0);
     lv_obj_add_event_cb(bn,[](lv_event_t*e){ if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return; sendCtl("otaupdate"); g_otaav_acked=true; if(g_otaav_ov){lv_obj_del(g_otaav_ov);g_otaav_ov=nullptr;} },LV_EVENT_CLICKED,NULL);
     lv_obj_t*bl=lv_label_create(bn);lv_label_set_text(bl,"Update now");lv_obj_set_style_text_color(bl,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(bl,&lv_font_montserrat_16,0);lv_obj_center(bl);}
    {lv_obj_t*bn=lv_btn_create(g_otaav_ov);lv_obj_set_size(bn,150,54);lv_obj_align(bn,LV_ALIGN_BOTTOM_RIGHT,-18,-18);
     lv_obj_set_style_bg_color(bn,lv_color_hex(0x4b5563),0);lv_obj_set_style_radius(bn,10,0);lv_obj_set_style_shadow_opa(bn,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(bn,0,0);
     lv_obj_add_event_cb(bn,[](lv_event_t*e){ if(lv_event_get_code(e)!=LV_EVENT_CLICKED)return; g_otaav_acked=true; if(g_otaav_ov){lv_obj_del(g_otaav_ov);g_otaav_ov=nullptr;} },LV_EVENT_CLICKED,NULL);
     lv_obj_t*bl=lv_label_create(bn);lv_label_set_text(bl,"Later");lv_obj_set_style_text_color(bl,lv_color_hex(0xffffff),0);lv_obj_set_style_text_font(bl,&lv_font_montserrat_16,0);lv_obj_center(bl);}
}
// MAJ selon g_status.ota (0 idle/1 check/2 download/3 OK reboot/4 fail/5 à jour).
void updOtaOverlay(){
    if(!g_status.valid||g_status.ota==0){ if(g_ota_ov)hideOtaOverlay(); g_ota_acked=0; return; }
    uint8_t s=g_status.ota;
    if(s<4) g_ota_acked=0;                       // nouveau cycle (check/download) → ré-arme
    if((s==4||s==5)&&g_ota_acked==s){            // terminal déjà acquitté (auto 5 s ou tap)
        if(g_ota_ov)hideOtaOverlay();            // → silencieux tant que le boîtier rediffuse
        return;
    }
    if(!g_ota_ov)mkOtaOverlay();
    const char* msg="..."; lv_color_t c=C_BRAND;
    switch(s){
        case 1: msg="Checking version..."; break;
        case 2: msg="Downloading firmware..."; break;
        case 3: msg="Update OK - rebooting"; c=C_GREEN; if(!g_ota_done_ms)g_ota_done_ms=millis(); break;
        case 4: msg="Update failed"; c=C_RED; if(!g_ota_done_ms)g_ota_done_ms=millis(); break;
        case 5: msg="Already up to date"; c=C_GREEN; if(!g_ota_done_ms)g_ota_done_ms=millis(); break;
    }
    if(g_ota_lbl){
        if(s==2){ char b[28]; snprintf(b,sizeof(b),"Downloading %d%%",g_status.opct); lv_label_set_text(g_ota_lbl,b); }
        else lv_label_set_text(g_ota_lbl,msg);
        lv_obj_set_style_text_color(g_ota_lbl,c,0);
    }
    if(g_ota_bar) lv_bar_set_value(g_ota_bar, s==2?g_status.opct:(s>=3?100:0), LV_ANIM_OFF);
    if((s==4||s==5)&&g_ota_done_ms&&millis()-g_ota_done_ms>5000){
        g_ota_acked=s;          // acquitté → ne reviendra pas tant que l'état ne change pas
        hideOtaOverlay();
    }
}

void updateAllPages(){
    char b[32];
    // Tâche F : overlay upload progress (full-screen modal post-vol)
    updUploadOverlay();
    updFlightState();   // bannière FLIGHT STARTED / chip Start flight / overlay arrêt
    updOtaOverlay();    // overlay MAJ firmware (OTA cloud, progression)
    updOtaAvailPrompt(); // (Phase B) prompt "MAJ dispo" au boot : Update now / Later
    // Page UPDATES ouverte : "Check now" → oav rafraîchi par l'ATC → rebuild auto ; sinon timeout.
    if(g_upd_ov){
        if(g_status.oav != g_upd_shown_oav){
            lv_obj_del(g_upd_ov); g_upd_ov=nullptr; g_upd_checkbtn=nullptr; showUpdatesPage();
        } else if(g_upd_checking && millis()-g_upd_check_t0>12000){
            g_upd_checking=false;
            if(g_upd_checkbtn){ lv_obj_t*l=lv_obj_get_child(g_upd_checkbtn,0); if(l)lv_label_set_text(l,"Up to date");
                lv_obj_set_style_bg_color(g_upd_checkbtn,lv_color_hex(0x1f4068),0); }
        }
    }
    if(g_vols_ov) volsUpdWifi();   // ligne état WiFi hotspot dans la page Flights
    p0UpdateAcId();                // accueil : ligne REG/TYPE/HEX (live depuis STATUS ATC)
#ifdef BOARD_T4S3
    if(s_set_acval){ char ac[40]; snprintf(ac,sizeof(ac),"%s / %s / %s",
        g_ac_reg[0]?g_ac_reg:"---",g_ac_type[0]?g_ac_type:"---",g_ac_hex[0]?g_ac_hex:"------");
        lv_label_set_text(s_set_acval,ac); }   // Active Aircraft (Settings) — même source
#endif
    // Version firmware AT-CORE + date de build — bas page radar (l'annonce de MAJ est dans Maintenance)
    if(r_radar_ver||r_p0_atc){
        char vb[40];
        if(g_status.valid && g_status.fwv) snprintf(vb,sizeof(vb),"CORE v%d  %s",g_status.fwv,g_status.fwd);
        else strcpy(vb,"CORE --");
        if(r_radar_ver) lv_label_set_text(r_radar_ver,vb);
        // Accueil (page #01) : "ATC 1.2.38-dev" SANS date (bords, coloré par canal)
        if(r_p0_atc){
            char ab[24];
            if(g_status.valid&&g_status.fws[0]) snprintf(ab,sizeof(ab),"ATC %s",g_status.fws);
            else if(g_status.valid&&g_status.fwv) snprintf(ab,sizeof(ab),"ATC v%d",g_status.fwv);
            else strcpy(ab,"ATC --");
            lv_label_set_text(r_p0_atc,ab);
            lv_obj_set_style_text_color(r_p0_atc, g_status.fws[0]?verColor(g_status.fws):TGREY(), 0);
        }
    }
    // Page ABOUT (T4) : version AT-CORE live "1.2.36-dev  date", couleur par canal
    if(s_sys_atcver){
        char ab[44];
        if(g_status.valid&&g_status.fws[0]) snprintf(ab,sizeof(ab),"%s  %s",g_status.fws,g_status.fwd);
        else if(g_status.valid&&g_status.fwv) snprintf(ab,sizeof(ab),"v%d  %s",g_status.fwv,g_status.fwd);
        else strcpy(ab,"v--");
        lv_label_set_text(s_sys_atcver,ab);
        lv_obj_set_style_text_color(s_sys_atcver, g_status.fws[0]?verColor(g_status.fws):C_BRAND, 0);
    }
    // Annonce de MAJ firmware + état WiFi → page Maintenance (live tant qu'ouverte)
    if(g_maint_ov){ maintUpdAnnounce(); maintWifiStatus(); }
    if(g_diag_ov) diagWifiStatus();   // page DIAGNOSTIC ouverte → état WiFi club live (Test WiFi)
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
        // GPS / LTE — sourcés depuis g_status (vu via AT-CORE). ADS-B / OGN retirés.
        bool gps_ok = g_status.valid && g_status.gps_fix;
        bool lte_ok = g_status.valid && g_status.csq>5;
        updCheckRow(CHK_GPS, "GPS",                   gps_ok);
        updCheckRow(CHK_LTE, "LTE",                   lte_ok);
        updCheckRow(CHK_SKY, "SafeSky", g_connected&&g_status.valid&&g_status.ss_ok);   // (juin 2026)
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
        // Page System (T4) : batterie AT-CORE en % seul (le label « AT-CORE BATT » est à côté)
        if(s_sys_atcbat){
            char sb[16];
            if(g_status.valid&&g_status.bat>=0) snprintf(sb,16,"%d%%%s",g_status.bat,g_status.charging?" " LV_SYMBOL_CHARGE:"");
            else strcpy(sb,"---%");
            lv_label_set_text(s_sys_atcbat,sb);
            lv_obj_set_style_text_color(s_sys_atcbat,bat_col,0);
        }
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
     // Code couleur vol test (2026-06-05) : OK = schéma N&B existant (icône claire),
     // KO = ROUGE (lisible en 1 coup d'œil en vol — "tout sauf rouge" = sain).
     #define SET_PILL_TXT(lbl,act) \
         lv_obj_set_style_text_color(lbl,(act)?PILL_IC_ON():C_RED,0)
     #define SET_PILL_IMG(img,act) \
         lv_obj_set_style_img_recolor(img,(act)?PILL_IC_ON():C_RED,0)
     // GPS
     SET_PILL_TXT(r_hdr_gps, gps_ok);
     // LTE — bars reflect signal level (0 barre = rouge : pas de signal exploitable)
     {int csq=g_status.valid?g_status.csq:0;
      int bars=csq>20?4:csq>14?3:csq>8?2:csq>3?1:0;
      for(int bb=0;bb<4;bb++)
          lv_obj_set_style_bg_color(r_hdr_lte_b[bb],
              bb<bars?PILL_IC_ON():(bars==0?C_RED:PILL_IC_OFF()),0);}
     // WiFi — pas de lien sol permanent : neutre gris (rouge serait un faux négatif)
     lv_obj_set_style_text_color(r_hdr_wifi,PILL_IC_OFF(),0);
     // BLE
     SET_PILL_TXT(r_hdr_ble, g_connected);
     // SafeSky — preuve de connexion bout en bout : échange UDP réussi < 10 s
     // (champ "ss" STATUS, FW ≥ v5). Fallback anciens FW : trafic reçu > 0.
     {bool sky_ok=g_connected&&(g_status.ss_ok||(g_traffic.valid&&g_traffic.count>0));
#ifdef BOARD_T4S3
      // (juin 2026) icône SafeSky = santé signal par TEINTE : vert OK / rouge KO.
      // (v23) + GRIS en mode sol/idle (ss_mode=1) : SafeSky vivant mais beat dégradé/lent
      // (1 beacon/60 s) → l'image n'est pas fraîche. Priorité : KO=rouge, sinon idle=gris, sinon vert.
      lv_color_t skyTint = !sky_ok ? C_RED
                         : (g_status.ss_mode==1 ? lv_color_hex(0x9ca3af) : C_GREEN);
      if(r_hdr_sky){ lv_obj_set_style_img_recolor(r_hdr_sky, skyTint,0);
                     lv_obj_set_style_img_recolor_opa(r_hdr_sky,LV_OPA_COVER,0); }
#else
      SET_PILL_IMG(r_hdr_sky, sky_ok);
#endif
     }
     // Point santé signal : vert = UDP OK · GRIS = mode sol/idle (beat dégradé) · rouge = perte.
     if(r_ss_dot) lv_obj_set_style_bg_color(r_ss_dot,
         !(g_connected&&g_status.valid&&g_status.ss_ok) ? C_RED
         : (g_status.ss_mode==1 ? lv_color_hex(0x9ca3af) : C_GREEN), 0);
#ifdef BOARD_T4S3
     // (juin 2026) Label à droite de l'icône SafeSky = STATUT DE VOL : GND (au sol) / FLT
     // (en vol). La santé SafeSky est portée par la TEINTE de l'icône (ci-dessus), pas ici
     // → plus de « CRZ sans signal » qui n'avait pas de sens.
     if(r_ss_gnd){
         bool flying = g_status.valid && g_status.flt_st!=0;
         lv_label_set_text(r_ss_gnd, flying?"FLT":"GND");
         lv_obj_set_style_text_color(r_ss_gnd, flying?C_GREEN:TFG(), 0);
     }
#else
     // (v18) Badge GND visible UNIQUEMENT si SafeSky fonctionne (ss_ok=vert) ET en mode éco
     // sol (ssm=1). Si SafeSky est ROUGE (data down), on cache GND → pas de "GND + rouge"
     // trompeur (GND laisserait croire que tout va bien alors que le data est mort).
     if(r_ss_gnd){ if(g_connected&&g_status.valid&&g_status.ss_ok&&g_status.ss_mode==1)
              lv_obj_clear_flag(r_ss_gnd,LV_OBJ_FLAG_HIDDEN);
          else lv_obj_add_flag(r_ss_gnd,LV_OBJ_FLAG_HIDDEN); }
#endif
     // FLARM / ADS-B retirés du radar (pastilles supprimées).
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
    // (juin 2026) Bascule auto vers le radar SEULEMENT quand TOUT est OK — « Ne passer
    // sur le radar que si tout est OK, LTE et GPS inclus » : BLE connecté + GPS fix + LTE
    // (csq>5). Tant qu'un manque, on reste sur l'accueil (le pilote peut toujours swiper
    // manuellement). One-shot par connexion.
    if(!g_autoNavDone&&!g_pair_ov&&g_connected&&g_status.valid
       &&g_status.gps_fix&&g_status.csq>5&&g_page==0){
        g_autoNavDone=true;g_navPending=true;g_navPage=1;}
    // Radar — heading-up en mouvement, north-up auto à l'arrêt (radarEffHdg()).
    // Le cap GPS (course over ground) n'est pas calculable sous RADAR_STILL_KMH :
    // on verrouille alors la rose au nord et la pill affiche "N".
    // Évolution possible : magnétomètre côté AT-CORE pour un cap indépendant
    // de la vitesse (cap magnétique vs cap sol).
    if(g_status.valid){
        // Capsule cap = état d'enregistrement : "NF" ambre = pas de vol actif
        // (rose au nord), sinon cap effectif (figé à l'arrêt pendant un vol).
        if(g_status.flt_st==0){
            lv_label_set_text(r_radar_hdg,"NF");
            lv_obj_set_style_text_color(r_radar_hdg,C_AMBER,0);
        } else {
            snprintf(b,32,"%d°",radarEffHdg());lv_label_set_text(r_radar_hdg,b);
            lv_obj_set_style_text_color(r_radar_hdg,TFG(),0);
        }
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
            int r_inner=RAD_R+RAD_CARD_OFF;
            int cx=(int)(RAD_CX+sinf(ra)*(float)r_inner)-12;   // -12 = demi-largeur (labels centrés, juin 2026)
            int cy=(int)(RAD_CY-cosf(ra)*(float)r_inner)-10;
            lv_obj_set_pos(r_card[ci],cx,cy);}}
    // Radar blips — handled by updateRadarDR() called every loop (dead reckoning)
#if UI_CO_EN
    // CO gauge — ball position + ppm label
    if(g_flight.valid){
        int co=g_flight.co_ppm;
        float co_a=(30.0f+fminf((float)co,150.0f)/150.0f*30.0f)*(float)M_PI/180.0f;
        lv_obj_set_pos(r_co_ball,(int)((float)RAD_CX+(float)(CO_SZ/2-8)*cosf(co_a)*CO_MIR)-6,
                       (int)((float)RAD_CY+(float)(CO_SZ/2-8)*sinf(co_a))-6);
        lv_obj_set_style_text_color(r_co_text,lv_color_hex(0x000000),0);
        lv_label_set_text(r_co_val,"");}
#endif
    // Alert overlay : menace trafic (moteur écran, prioritaire), sinon alertes ATC (CO/g/rpm).
    if(g_threat.level!=THREAT_NONE){
        char tb[52]; float dnm=g_threat.dist_m/1852.0f;
        snprintf(tb,sizeof(tb),"TFC %dh  %.1fnm  %+dft  clos %dkt",
                 g_threat.clock,dnm,g_threat.dalt_ft,g_threat.closing_kt);
        lv_label_set_text(r_aov_text,tb);
        lv_obj_set_style_bg_color(r_alert_overlay,g_threat.level==THREAT_RED?C_RED:C_AMBER,0);
        lv_obj_clear_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);
    }else if(g_alert.valid&&(g_alert.co||g_alert.gforce||g_alert.rpm)){
        char ab[48]="";
        if(g_alert.co)strcat(ab,"CO  ");if(g_alert.gforce)strcat(ab,"G-FORCE  ");if(g_alert.rpm)strcat(ab,"RPM");
        lv_label_set_text(r_aov_text,ab);
        lv_obj_set_style_bg_color(r_alert_overlay,C_RED,0);
        lv_obj_clear_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);
    }else{lv_obj_add_flag(r_alert_overlay,LV_OBJ_FLAG_HIDDEN);}
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

bool hasAlert(){return g_alert.valid&&(g_alert.co||g_alert.gforce||g_alert.rpm);}  // trafic = moteur écran (g_threat), pas l'ATC

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

// Crée le WebServer + routes (/, /upload SD, /update OTA) + begin. Partagé par
// l'AP propre (wifiStart) ET le relais STA (relayTick) → mêmes endpoints, l'updater
// web de l'AT-VIEW est joignable dans les deux modes.
static void startWebServer(){
    g_webserver=new WebServer(80);
    g_webserver->on("/",HTTP_GET,handleRoot);
    g_webserver->on("/upload",HTTP_POST,handleUploadDone,handleUploadData);
    g_webserver->on("/update",HTTP_POST,handleOtaDone,handleOtaData);   // OTA firmware (WP7)
    g_webserver->begin();}

void wifiStart(){
    WiFi.mode(WIFI_AP);
    WiFi.softAP(g_unit_name,g_wifi_pass);
    MDNS.begin("atview");
    startWebServer();
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

// (v21) Annonce au boîtier (AP portail, 192.168.4.1) : GET /atv?ip=&v= → le boîtier
// affiche un lien vers cet updater. Fire-and-forget, court timeout.
static void relayAnnounce(){
    if(!g_relay_ip[0])return;
    WiFiClient c;
    if(!c.connect(IPAddress(192,168,4,1),80)){Serial.println("[RLY] announce: no connect");return;}
    c.printf("GET /atv?ip=%s&v=%s HTTP/1.1\r\nHost: 192.168.4.1\r\nConnection: close\r\n\r\n",
             g_relay_ip,VIEW_VERSION);
    uint32_t t0=millis();
    while(c.connected()&&millis()-t0<1500){while(c.available())c.read();delay(5);}
    c.stop();
    Serial.printf("[RLY] announced %s v%s\n",g_relay_ip,VIEW_VERSION);}

// Arrête le relais : serveur web + WiFi OFF, retour BLE-only (le BLE survit). Appelé
// au Close de l'overlay. (Après un self-flash, l'AT-VIEW reboote → état moot.)
static void relayStop(){
    if(g_webserver){g_webserver->stop();delete g_webserver;g_webserver=nullptr;}
    WiFi.disconnect(true);WiFi.mode(WIFI_OFF);
    g_wifi_active=false;g_relay_state=RLY_IDLE;g_relay_ip[0]=0;
    Serial.println("[RLY] stopped");}

// Machine d'état du relais — appelée à chaque loop(). WiFi.begin non bloquant : on
// poll WiFi.status() jusqu'à WL_CONNECTED (ou timeout). Le handleClient() de loop()
// sert l'updater web une fois g_wifi_active.
static void relayTick(){
    if(g_relay_state==RLY_IDLE||g_relay_state==RLY_FAIL)return;
    uint32_t now=millis();
    if(g_relay_state==RLY_UP){
        static uint32_t la=0;
        if(now-la>5000){la=now;relayAnnounce();}   // ré-annonce (au cas où l'AP a redémarré)
        return;
    }
    if(g_relay_state==RLY_WAIT_AP){
        // Laisse le temps au boîtier de lever son AP après {"cmd":"portal"}.
        if(now-g_relay_t0>3500){
            WiFi.mode(WIFI_STA);
            WiFi.begin(g_relay_ssid,(char*)ATC_PORTAL_PASS);   // core attend char* (pas const)
            Serial.printf("[RLY] joining %s\n",g_relay_ssid);
            g_relay_state=RLY_JOINING;g_relay_t0=now;relayUpdateOverlay();
        }
    }else if(g_relay_state==RLY_JOINING){
        if(WiFi.status()==WL_CONNECTED){
            strlcpy(g_relay_ip,WiFi.localIP().toString().c_str(),sizeof(g_relay_ip));
            startWebServer();          // updater web joignable sur l'IP STA
            g_wifi_active=true;        // loop() → handleClient()
            relayAnnounce();
            g_relay_state=RLY_UP;relayUpdateOverlay();
            Serial.printf("[RLY] up, ip=%s\n",g_relay_ip);
        }else if(now-g_relay_t0>20000){
            g_relay_state=RLY_FAIL;relayUpdateOverlay();
            WiFi.disconnect(true);WiFi.mode(WIFI_OFF);
            Serial.println("[RLY] join timeout");
        }
    }}

// ── IMU mouchard (QMI8658, WS216/WS241) ───────────────────────────────────────
#ifdef HAS_IMU
static SensorQMI8658 g_qmi;
static bool  g_imu_ok=false, g_imu_cal=false;
// Repère "repos" auto-zéroté au sol (frame capteur) : d0=verticale(bas), fwd0=avant, right0=droite.
static float g_d0[3]={0,0,1}, g_fwd0[3]={1,0,0}, g_right0[3]={0,1,0};
// Sorties courantes (poussées BLE → CSV boîtier).
static volatile float g_imu_nz=1.0f, g_imu_lat=0, g_imu_lon=0, g_imu_pitch=0, g_imu_roll=0;
// Peak-hold entre 2 pushes (capte les pics G entre deux lignes CSV 4 Hz).
static float g_imu_nz_max=1.0f, g_imu_nz_min=1.0f, g_imu_lat_pk=0, g_imu_gyr_pk=0;
// Snapshot 1 Hz vers la TÂCHE de push BLE (la write Bluedroid bloque ~1-2 s sous charge trafic :
// on la sort de la boucle → elle bloque la tâche, JAMAIS le radar). cf mémoire imu_mouchard.
static volatile float g_imu_tx_nz=1, g_imu_tx_lat=0, g_imu_tx_pitch=0, g_imu_tx_roll=0;
static volatile bool  g_imu_tx_ready=false;

static inline float v3dot(const float*a,const float*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static inline void  v3cross(const float*a,const float*b,float*o){
    o[0]=a[1]*b[2]-a[2]*b[1];o[1]=a[2]*b[0]-a[0]*b[2];o[2]=a[0]*b[1]-a[1]*b[0];}
static inline float v3norm(float*a){float m=sqrtf(v3dot(a,a));if(m>1e-6f){a[0]/=m;a[1]/=m;a[2]/=m;}return m;}

// Fige le repère repos à partir du vecteur gravité mesuré au sol (mount-agnostique).
static void imuCalibrate(const float*a){
    float d[3]={a[0],a[1],a[2]}; if(v3norm(d)<0.5f) return;
    g_d0[0]=d[0];g_d0[1]=d[1];g_d0[2]=d[2];
    // avant = projection de l'axe capteur +X sur le plan horizontal (⊥ d0) ; fallback +Y si dégénéré
    float X[3]={1,0,0}; float k=v3dot(X,g_d0);
    float f[3]={X[0]-k*g_d0[0],X[1]-k*g_d0[1],X[2]-k*g_d0[2]};
    if(v3norm(f)<0.2f){float Y[3]={0,1,0};k=v3dot(Y,g_d0);
        f[0]=Y[0]-k*g_d0[0];f[1]=Y[1]-k*g_d0[1];f[2]=Y[2]-k*g_d0[2];v3norm(f);}
    g_fwd0[0]=f[0];g_fwd0[1]=f[1];g_fwd0[2]=f[2];
    v3cross(g_d0,g_fwd0,g_right0); v3norm(g_right0);
    if(!g_imu_cal){g_imu_cal=true;Serial.println("[IMU] auto-zéro (repère repos figé au sol)");}
}

static void imuInit(){
    if(!g_qmi.begin(Wire,QMI8658_L_SLAVE_ADDRESS,IMU_SDA,IMU_SCL) &&
       !g_qmi.begin(Wire,QMI8658_H_SLAVE_ADDRESS,IMU_SDA,IMU_SCL)){
        Serial.println("[IMU] QMI8658 absent (begin FAIL)");return;}
    g_qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_8G, SensorQMI8658::ACC_ODR_250Hz);
    g_qmi.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS, SensorQMI8658::GYR_ODR_224_2Hz);
    g_qmi.enableAccelerometer(); g_qmi.enableGyroscope();
    g_imu_ok=true;
    Serial.println("[IMU] QMI8658 OK (acc ±8g / gyro ±256dps)");
}

static void imuTick(){
    if(!g_imu_ok) return;
    uint32_t now=millis();
    static uint32_t lastS=0; if(now-lastS<40) return; lastS=now;   // ~25 Hz (allégé : ne pas affamer le radar/trafic)
    float ax,ay,az,gx=0,gy=0,gz=0;
    if(!g_qmi.getAccelerometer(ax,ay,az)) return;
    g_qmi.getGyroscope(gx,gy,gz);
    float a[3]={ax,ay,az};
    float gmag=sqrtf(gx*gx+gy*gy+gz*gz);                 // rotation totale (°/s)
    float amag=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);     // module accel (g)
    // Auto-zéro CONTINU tant qu'au sol + stable → suit le mount, fige dès qu'on bouge/vole.
    bool onGround=(!g_status.valid)||(g_status.flt_st==0);
    if(onGround && gmag<3.0f && fabsf(amag-1.0f)<0.08f) imuCalibrate(a);
    if(!g_imu_cal) return;
    float nz=v3dot(a,g_d0);                              // facteur de charge vertical (g)
    float ah[3]={a[0]-nz*g_d0[0],a[1]-nz*g_d0[1],a[2]-nz*g_d0[2]};
    float lat=v3dot(ah,g_right0), lon=v3dot(ah,g_fwd0);  // G latéral / longitudinal (g)
    float d[3]={a[0],a[1],a[2]}; v3norm(d);
    float pitch=asinf(constrain(-v3dot(d,g_fwd0),-1.f,1.f))*57.2958f;   // accel-only (v1)
    float roll =asinf(constrain( v3dot(d,g_right0),-1.f,1.f))*57.2958f;
    g_imu_nz=nz; g_imu_lat=lat; g_imu_lon=lon; g_imu_pitch=pitch; g_imu_roll=roll;
    if(nz>g_imu_nz_max)g_imu_nz_max=nz; if(nz<g_imu_nz_min)g_imu_nz_min=nz;
    if(fabsf(lat)>fabsf(g_imu_lat_pk))g_imu_lat_pk=lat;
    if(gmag>g_imu_gyr_pk)g_imu_gyr_pk=gmag;
    // Push 4 Hz au boîtier (CHR_IMU) : nz/lat = EXTRÊME peak-hold de l'intervalle (capte les
    // pics G entre 2 lignes CSV), pitch/roll = instantané. Reset du peak-hold après envoi.
    static uint32_t pushT=0;
    if(now-pushT>=1000){pushT=now;
        // Snapshot 1 Hz pour la tâche de push (PAS de write BLE ici → boucle/radar jamais bloqués).
        // nz/lat = extrême peak-hold de l'intervalle (pics G captés), pitch/roll = instantané.
        float nzx=(fabsf(g_imu_nz_max-1.f)>=fabsf(g_imu_nz_min-1.f))?g_imu_nz_max:g_imu_nz_min;
        g_imu_tx_nz=nzx; g_imu_tx_lat=g_imu_lat_pk; g_imu_tx_pitch=pitch; g_imu_tx_roll=roll;
        g_imu_tx_ready=true;
        g_imu_nz_max=g_imu_nz_min=nz; g_imu_lat_pk=0; g_imu_gyr_pk=0;   // ré-arme l'intervalle
    }
    static uint32_t logT=0; if(now-logT>1000){logT=now;
        Serial.printf("[IMU] nz=%.2f lat=%.2f pitch=%.0f roll=%.0f%s\n",
            nz,lat,pitch,roll,(g_connected&&g_chrImu)?" >ATC":"");}
}

// Tâche dédiée du push IMU BLE — la write Bluedroid peut bloquer ~1-2 s sous charge trafic ;
// isolée ici, elle ne bloque QUE cette tâche (prio basse, core 0), jamais la boucle/radar (core 1).
// Lit les globals snapshotés par imuTick (boucle). Aucune I2C/LVGL ici → pas de conflit ressource.
static void TaskImuPush(void* pv){
    char p[80];
    for(;;){
        if(g_imu_tx_ready && g_connected && g_chrImu && g_chrImu->canWrite()){
            g_imu_tx_ready=false;
            int k=snprintf(p,sizeof(p),"{\"nz\":%.2f,\"lat\":%.2f,\"p\":%.1f,\"r\":%.1f}",
                           g_imu_tx_nz,g_imu_tx_lat,g_imu_tx_pitch,g_imu_tx_roll);
            if(k>0) g_chrImu->writeValue((uint8_t*)p,k,false);   // peut bloquer → bloque CETTE tâche
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif // HAS_IMU

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup(){
    Serial.begin(115200);
#if defined(BOARD_T4S3) && defined(PANEL_WS241)
    // Waveshare 2.41 : init RM690B0 + touch FT6336 paysage via le shim (ws241_shim.h)
    if(!panel.begin()){while(1){Serial.println("Panel FAIL");delay(1000);}}
    Serial.printf("Touch: %s\n",panel.getTouchModelName());
    delay(50); // card power-on stabilization
    g_sd_ok=panel.installSD();   // SD best-effort (pins 2.41 à câbler)
#elif defined(BOARD_T4S3)
    // T4-S3 : init AMOLED explicite (pas d'auto-détect), SD différée pour passer par installSD()
    // (true, true) = SD différée (installSD() ci-dessous) + LED STAT du PMU SY6970
    // ÉTEINTE (clignotait rouge en permanence sans batterie — feedback vol 2026-06-05).
    if(!amoled.beginAMOLED_241(true, true)){while(1){Serial.println("Panel FAIL");delay(1000);}}
    Serial.println("Touch: CST226SE (T4-S3)");
    delay(50); // card power-on stabilization
    g_sd_ok=amoled.installSD();   // SD en SPI (MISO 4 / MOSI 2 / SCK 3 / CS 1)
#elif defined(BOARD_WS216)
    // Waveshare 2.16 : init CO5300 + touch CST9220 + SD SPI via le shim (ws216_shim.h)
    if(!panel.begin()){while(1){Serial.println("Panel FAIL");delay(1000);}}
    Serial.printf("Touch: %s\n",panel.getTouchModelName());
    delay(50); // card power-on stabilization
    g_sd_ok=panel.installSD();   // SD en SPI (best-effort : échec non fatal)
#else
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
#endif
    if(g_sd_ok){
        g_sd_gb=(uint32_t)(SD_MMC.totalBytes()/(1024ULL*1024*1024));
        if(!SD_MMC.exists("/aip"))SD_MMC.mkdir("/aip");
        Serial.printf("[SD] OK %uGB\n",g_sd_gb);
    }else{Serial.println("[SD] No card");}
#ifdef HAS_IMU
    imuInit();   // QMI8658 sur le bus I2C déjà ouvert par panel.begin()
    // Push IMU dans une tâche dédiée (core 0, prio basse) → la write BLE bloquante n'affame
    // jamais la boucle/radar (core 1). cf TaskImuPush + mémoire imu_mouchard.
    if(g_imu_ok) xTaskCreatePinnedToCore(TaskImuPush,"imupush",4096,nullptr,1,nullptr,0);
#endif
    // (A1) Pull AIP dans une tâche dédiée (core 0, prio basse) : les lectures BLE bloquantes
    // n'affament jamais la boucle/radar (core 1). Idle tant que g_aip_pull_req non armé.
    xTaskCreatePinnedToCore(TaskAipPull,"aippull",6144,nullptr,1,nullptr,0);
#if defined(BOARD_T4S3) && !defined(PANEL_WS241)
    // (juin 2026) RÉACTIVITÉ TACTILE — cause racine : beginLvglHelper (non-DMA) alloue un
    // buffer PLEIN ÉCRAN de ~540 KB en PSRAM (lente) + flush SYNCHRONE bloquant (pushColors)
    // → la boucle reste figée pendant chaque rendu/transition, le tactile paraît lent/raté.
    // beginLvglHelperDMA : double buffer 1/10 d'écran en RAM INTERNE rapide + flush DMA
    // ASYNCHRONE (pushColorsDMA) → la boucle n'est plus bloquée par le push pixels. C'est
    // la voie « perf » prévue par LilyGo (exemples factory T4). Geometry/rounder inchangés.
    beginLvglHelperDMA(panel);
#else
    beginLvglHelper(panel);   // T-RGB / WS-216 / WS-241 (shim Arduino_GFX)
#endif
    // RÉACTIVITÉ TACTILE — TOUTES cartes : période d'échantillonnage du/des input devices
    // 30 ms (défaut LVGL) → 12 ms. Avant, seul le T4 (branche DMA) l'avait → WS241/WS216/
    // T-RGB restaient à 30 ms → petites cibles (bouton engrenage Settings) ratant des taps
    // quand la boucle est chargée (radar/AIP/alertes). Hors-DMA en bénéficie aussi.
    for(lv_indev_t* d=lv_indev_get_next(NULL); d; d=lv_indev_get_next(d))
        if(d->driver && d->driver->read_timer) lv_timer_set_period(d->driver->read_timer,12);
    // Le canvas 480×480 décalé (UI_OY<0 sur T4-S3) déborde de l'écran → sans ceci,
    // LVGL rend l'écran scrollable et dessine une scrollbar verticale grise au bord
    // droit (pleine hauteur). Toujours OFF : l'UI ne doit jamais scroller l'écran.
    lv_obj_clear_flag(lv_scr_act(),LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(lv_scr_act(),LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(lv_layer_top(),LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(lv_layer_top(),LV_SCROLLBAR_MODE_OFF);
    cfgLoad();acLoad();unitLoad();
#ifdef AIP_EMBEDDED
    aipLoadEmbedded();            // WS241 : AIP depuis la flash écran (instant, persistant, pas de BLE)
#else
    if(g_sd_ok)aipLoad();         // T-RGB/T4/WS216 : AIP depuis la SD
#endif
    g_dark_theme=g_cfg.dark;
    lv_obj_set_style_bg_color(lv_scr_act(),TBG(),0);
    panelBright(g_cfg.brightness);

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
    // (2026-06) Identité aéronef saisie au PORTAIL WEB du boîtier (plus à l'écran) :
    // le boîtier pousse reg/typ/hex en STATUS BLE → affichés live (accueil + Active
    // Aircraft). Plus de saisie FORCÉE ici (l'overlay AIRCRAFT était le dernier reliquat
    // de la page retirée). Si le boîtier n'a pas d'identité, l'accueil montre un hint
    // non-bloquant « set via WiFi Setup » (p0UpdateAcId). Overlay legacy non déclenché.
    audioInit(); audioTestChime();   // WS-241 : init DAC I2S + carillon de validation câblage (no-op ailleurs)
    encoderInit();                   // WS-241 : encodeur rotatif nav/zoom (no-op ailleurs)
    Serial.println("Ready");}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop(){
    uint32_t now=millis();
    encoderPoll();   // WS-241 : encodeur rotatif (zoom/pages) — no-op ailleurs
    if(g_doReconnect){g_doReconnect=false;startScan();}
    if(g_doConnect&&!g_connected){g_doConnect=false;
        if(connectBLE())Serial.println("[BLE] OK");
        else startScan();}   // (juin 2026) plus de delay(2000) bloquant : scan async + throttle 8 s gèrent la cadence
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
    // Throttle du refresh UI : le BLE met g_dataUpdated à chaque STATUS/FLIGHT/TRAFFIC (≈1 Hz
    // chacun → plusieurs fois/s). Sans frein, la boucle passe son temps à redessiner toutes les
    // pages → lv_timer_handler échantillonne le tactile par à-coups (boutons ratés). 120 ms = œil OK.
    {static uint32_t s_lastUI=0;
     if(g_dataUpdated && now-s_lastUI>=120){g_dataUpdated=false;s_lastUI=now;updateAllPages();}}
    bool alert=(g_threat.level==THREAT_RED)||hasAlert();   // menace trafic ROUGE force le radar (orange = discret)
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
#ifdef HAS_IMU
    imuTick();   // mouchard G/assiette : échantillonne ~50 Hz, peak-hold, auto-zéro au sol
#endif
    static uint32_t drLast=0;
    if(now-drLast>=200){drLast=now;
        alertEngineTick();                       // moteur anticollision : toutes pages (force le radar même hors radar)
        if(g_page==1){updateRadarDR();
            if(g_cfg.aip_en&&r_aip_layer&&g_status.valid)lv_obj_invalidate(r_aip_layer);}}
    relayTick();   // (v21) machine d'état du relais "Update both" (STA-join + annonce)
    if(g_wifi_active&&g_webserver)g_webserver->handleClient();
    if(g_ota_reboot_ms&&millis()-g_ota_reboot_ms>1200){Serial.println("[OTA] reboot");delay(200);ESP.restart();}
    // WP8 — liste vols prête : handshake flt_rdy 0→1 (AT-CORE met 0 à la réception
    // de {"cmd":"flights"}, 1 quand la liste est construite) → lit CHR_FLIGHTS.
    if(g_vols_loading&&g_vols_ov){
        if(g_connected&&g_status.flt_rdy==1&&millis()-g_vols_t0>1500){
            g_vols_loading=false;volsBuildList();   // lecture BLE seulement si connecté + liste fraîche
        }else if(millis()-g_vols_t0>30000){   // SD bien remplie = scan lent (16 vols ≈ 13s)
            g_vols_loading=false;if(g_vols_load)lv_label_set_text(g_vols_load,"Timeout - retry");
        }
    }
    // WP8 — suivi transfert : on RESTE sur la page Vols jusqu'à flt_phase 4 (OK) / 5 (fail).
    // On accepte 4/5 si on a vu UPLOADING (3) OU après 8s (l'AT-CORE a forcément consommé
    // g_upl_req via TaskMonitor ~5s et écrit la phase 3 puis le résultat → plus de 4/5 périmé).
    // Ça évite un "timeout" 90s quand l'échec est rapide (ex. hotspot hors de portée).
    if(g_vols_xfer_pending&&g_vols_ov){
        uint8_t ph=g_status.flt_phase;
        uint32_t now=millis(), el=now-g_vols_xfer_t0;
        if(ph==3)g_vols_xfer_seen3=true;
        // (v7) Anti-stall : tant que la phase OU up_pct AVANCE, pas de timeout. Un gros CSV
        // gzippe plusieurs min (up_pct grimpe 2..40, ph reste 3) → l'ancien backstop dur 180s
        // donnait un FAUX "Transfer timeout" alors que ça progressait. On ne timeoute donc que
        // sur une VRAIE absence de progrès (ph + up_pct figés > 2 min) → indépendant de la taille.
        if(ph!=g_vols_xfer_lastph || g_status.upload_pct!=g_vols_xfer_lastpct){
            g_vols_xfer_lastph=ph; g_vols_xfer_lastpct=g_status.upload_pct; g_vols_xfer_prog_ms=now;
        }
        // Progression visible : "Transferring... N%" (up_pct par paliers, MAJ quand un STATUS
        // passe — le BLE est ralenti pendant l'upload WiFi, d'où la roue en complément).
        if(g_vols_load && ph==3){
            char b[28]; snprintf(b,sizeof(b),"Transferring... %d%%",g_status.upload_pct);
            lv_label_set_text(g_vols_load,b);
        }
        if((g_vols_xfer_seen3||el>8000)&&(ph==4||ph==5)){
            g_vols_xfer_pending=false;
            if(g_vols_load)lv_label_set_text(g_vols_load,ph==4?"Transfer OK":"Transfer failed");
            if(ph==4){ sendCtl("flights"); g_status.flt_rdy=0; g_vols_loading=true; g_vols_t0=millis(); }
        }else if(now-g_vols_xfer_prog_ms>120000){
            // 2 min SANS aucun progrès (ni phase ni up_pct) = vrai blocage (hotspot perdu,
            // S3 figé). Plus de backstop absolu → un gros leg qui gzippe longtemps est OK.
            g_vols_xfer_pending=false;if(g_vols_load)lv_label_set_text(g_vols_load,"Transfer timeout");
        }
    }
    lv_timer_handler();delay(5);}

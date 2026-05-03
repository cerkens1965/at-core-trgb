/**
 * AT-CORE AeroTrace Core — DEMO MODE
 * ESP32-S3 — LILYGO T-RGB 2.8" Full Circle
 * Architecture: 5 containers created once, show/hide on nav
 * No lv_obj_clean() — buttons survive page changes
 * Christophe — AT-CORE v0.4-DEMO — 03/05/2026
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

// ── Panel — global like official examples ─────────────────────────────────────
LilyGo_RGBPanel panel;

// ── Colors ────────────────────────────────────────────────────────────────────
#define C_BG     lv_color_hex(0x000000)
#define C_WHITE  lv_color_hex(0xFFFFFF)
#define C_AMBER  lv_color_hex(0xF5A623)
#define C_GREEN  lv_color_hex(0x22c55e)
#define C_BLUE   lv_color_hex(0x60a5fa)
#define C_RED    lv_color_hex(0xef4444)
#define C_ORANGE lv_color_hex(0xf97316)
#define C_GREY   lv_color_hex(0x6b7280)

// ── Pages ─────────────────────────────────────────────────────────────────────
#define NUM_PAGES 5
static lv_obj_t* g_pages[NUM_PAGES];
static uint8_t   g_page = 0;

// ── Navigation callbacks ──────────────────────────────────────────────────────
static void cbPrev(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        lv_obj_add_flag(g_pages[g_page], LV_OBJ_FLAG_HIDDEN);
        g_page = (g_page == 0) ? NUM_PAGES - 1 : g_page - 1;
        lv_obj_clear_flag(g_pages[g_page], LV_OBJ_FLAG_HIDDEN);
        Serial.printf("[NAV] Page %d\n", g_page + 1);
    }
}

static void cbNext(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        lv_obj_add_flag(g_pages[g_page], LV_OBJ_FLAG_HIDDEN);
        g_page = (g_page + 1) % NUM_PAGES;
        lv_obj_clear_flag(g_pages[g_page], LV_OBJ_FLAG_HIDDEN);
        Serial.printf("[NAV] Page %d\n", g_page + 1);
    }
}

// ── Helper: label in page ─────────────────────────────────────────────────────
lv_obj_t* mkLabel(lv_obj_t* parent, const char* txt,
                  lv_color_t col, const lv_font_t* font,
                  lv_align_t align, int ox, int oy)
{
    lv_obj_t* l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_align(l, align, ox, oy);
    return l;
}

// KV row: grey key left, colored value right
void mkKV(lv_obj_t* p, int y, const char* k, const char* v, lv_color_t vc)
{
    lv_obj_t* lk = lv_label_create(p);
    lv_label_set_text(lk, k);
    lv_obj_set_style_text_color(lk, C_GREY, 0);
    lv_obj_set_style_text_font(lk, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(lk, 80, y);

    lv_obj_t* lv = lv_label_create(p);
    lv_label_set_text(lv, v);
    lv_obj_set_style_text_color(lv, vc, 0);
    lv_obj_set_style_text_font(lv, &lv_font_montserrat_16, 0);
    lv_obj_align(lv, LV_ALIGN_TOP_RIGHT, -80, y);
}

// Status dot + label
void mkStatus(lv_obj_t* p, int x, int y, const char* lbl, bool ok)
{
    lv_obj_t* dot = lv_label_create(p);
    lv_label_set_text(dot, "●");
    lv_obj_set_style_text_color(dot, ok ? C_GREEN : C_RED, 0);
    lv_obj_set_style_text_font(dot, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(dot, x, y);

    lv_obj_t* lb = lv_label_create(p);
    lv_label_set_text(lb, lbl);
    lv_obj_set_style_text_color(lb, C_WHITE, 0);
    lv_obj_set_style_text_font(lb, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(lb, x + 20, y);
}

// Page container: full screen, black bg
lv_obj_t* mkPage()
{
    lv_obj_t* p = lv_obj_create(lv_scr_act());
    lv_obj_set_size(p, 480, 480);
    lv_obj_set_pos(p, 0, 0);
    lv_obj_set_style_bg_color(p, C_BG, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    return p;
}

// ── Page 1 — System Status ────────────────────────────────────────────────────
void buildPage1()
{
    lv_obj_t* p = g_pages[0];

    mkLabel(p, "AT-CORE", C_AMBER, &lv_font_montserrat_20, LV_ALIGN_TOP_MID, 0, 55);
    mkLabel(p, "FLIGHT",  C_GREEN, &lv_font_montserrat_20, LV_ALIGN_TOP_MID, 0, 85);

    mkStatus(p, 82,  128, "GPS 11sat", true);
    mkStatus(p, 252, 128, "LTE 18",    true);
    mkStatus(p, 82,  168, "SD 1240fr", true);
    mkStatus(p, 252, 168, "BLE",       true);
    mkStatus(p, 82,  208, "FLARM",     true);
    mkStatus(p, 252, 208, "ADS-B",     true);

    mkLabel(p, "50.5686 / 4.4347", C_WHITE, &lv_font_montserrat_16, LV_ALIGN_TOP_MID, 0, 255);
    mkLabel(p, "1/5", C_GREY, &lv_font_montserrat_16, LV_ALIGN_BOTTOM_MID, 0, -60);
}

// ── Page 2 — Flight Data ──────────────────────────────────────────────────────
void buildPage2()
{
    lv_obj_t* p = g_pages[1];

    mkLabel(p, "CRUISE", C_GREEN, &lv_font_montserrat_22, LV_ALIGN_TOP_MID, 0, 55);

    // ALT left, SPD right big
    mkLabel(p, "820",  C_WHITE, &lv_font_montserrat_32, LV_ALIGN_TOP_LEFT,  88, 95);
    mkLabel(p, "m",    C_GREY,  &lv_font_montserrat_20, LV_ALIGN_TOP_LEFT,  88, 131);
    mkLabel(p, "75",   C_WHITE, &lv_font_montserrat_32, LV_ALIGN_TOP_RIGHT, -108, 95);
    mkLabel(p, "kt",   C_GREY,  &lv_font_montserrat_20, LV_ALIGN_TOP_RIGHT, -108, 131);
    mkLabel(p, "254",  C_WHITE, &lv_font_montserrat_22, LV_ALIGN_TOP_MID,   0, 100);
    mkLabel(p, "HDG",  C_GREY,  &lv_font_montserrat_16, LV_ALIGN_TOP_MID,   0, 128);

    mkKV(p, 178, "G-FORCE", "1.02 G",  C_WHITE);
    mkKV(p, 208, "CO",      "7 ppm",   C_WHITE);
    mkKV(p, 238, "RPM",     "2250",    C_WHITE);

    mkLabel(p, "2/5", C_GREY, &lv_font_montserrat_16, LV_ALIGN_BOTTOM_MID, 0, -60);
}

// ── Page 3 — Traffic ─────────────────────────────────────────────────────────
void buildPage3()
{
    lv_obj_t* p = g_pages[2];

    mkLabel(p, "TRAFFIC", C_BLUE, &lv_font_montserrat_20, LV_ALIGN_TOP_MID, 0, 55);

    // Entry 1 — distant, green
    mkLabel(p, "●", C_GREEN, &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 80, 100);
    mkLabel(p, "OO-LGT", C_WHITE, &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 100, 100);
    mkLabel(p, "2.1km", C_GREEN, &lv_font_montserrat_16, LV_ALIGN_TOP_MID, 0, 100);
    mkLabel(p, "780m",  C_WHITE, &lv_font_montserrat_16, LV_ALIGN_TOP_RIGHT, -80, 100);

    // Entry 2 — invisible, amber
    mkLabel(p, "●", C_AMBER, &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 80, 138);
    mkLabel(p, "3C6517", C_AMBER, &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 100, 138);
    mkLabel(p, "4.8km", C_GREEN, &lv_font_montserrat_16, LV_ALIGN_TOP_MID, 0, 138);
    mkLabel(p, "900m",  C_WHITE, &lv_font_montserrat_16, LV_ALIGN_TOP_RIGHT, -80, 138);

    // Entry 3
    mkLabel(p, "●", C_GREEN, &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 80, 176);
    mkLabel(p, "OO-SKY", C_WHITE, &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 100, 176);
    mkLabel(p, "7.2km", C_GREEN, &lv_font_montserrat_16, LV_ALIGN_TOP_MID, 0, 176);
    mkLabel(p, "650m",  C_WHITE, &lv_font_montserrat_16, LV_ALIGN_TOP_RIGHT, -80, 176);

    mkLabel(p, "white=SafeSky  amber=invisible", C_GREY, &lv_font_montserrat_16, LV_ALIGN_BOTTOM_MID, 0, -80);
    mkLabel(p, "3/5", C_GREY, &lv_font_montserrat_16, LV_ALIGN_BOTTOM_MID, 0, -60);
}

// ── Page 4 — Alerts ──────────────────────────────────────────────────────────
void buildPage4()
{
    lv_obj_t* p = g_pages[3];

    mkLabel(p, "! ALERTS !", C_RED, &lv_font_montserrat_22, LV_ALIGN_TOP_MID, 0, 55);
    mkLabel(p, "No active alert", C_GREEN, &lv_font_montserrat_20, LV_ALIGN_CENTER, 0, -10);
    mkLabel(p, "4/5", C_GREY, &lv_font_montserrat_16, LV_ALIGN_BOTTOM_MID, 0, -60);
}

// ── Page 5 — Debug ────────────────────────────────────────────────────────────
void buildPage5()
{
    lv_obj_t* p = g_pages[4];

    mkLabel(p, "DEBUG SIM7600", C_BLUE, &lv_font_montserrat_16, LV_ALIGN_TOP_MID, 0, 58);

    int y = 90, dy = 26;
    mkKV(p, y, "HB GPS",  "3s",   C_GREEN);  y += dy;
    mkKV(p, y, "HB LTE",  "4s",   C_GREEN);  y += dy;
    mkKV(p, y, "HB SD",   "2s",   C_GREEN);  y += dy;
    mkKV(p, y, "CSQ",     "18",   C_GREEN);  y += dy;
    mkKV(p, y, "HTTP",    "430ms",C_GREEN);  y += dy;
    mkKV(p, y, "CODE",    "200",  C_GREEN);  y += dy;
    mkKV(p, y, "SafeSky", "4s",   C_GREEN);  y += dy;
    mkKV(p, y, "HEAP",    "178kB",C_GREEN);  y += dy;
    mkKV(p, y, "FLT",     "ATC1746271800", C_WHITE);

    mkLabel(p, "5/5", C_GREY, &lv_font_montserrat_16, LV_ALIGN_BOTTOM_MID, 0, -60);
}

// ── Nav buttons — created ONCE on top of everything ───────────────────────────
void createNavButtons()
{
    // PREV — left side, transparent
    lv_obj_t* btnL = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btnL, 80, 240);
    lv_obj_align(btnL, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(btnL,     LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(btnL, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(btnL, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_add_event_cb(btnL, cbPrev, LV_EVENT_ALL, NULL);
    lv_obj_t* la = lv_label_create(btnL);
    lv_label_set_text(la, "<");
    lv_obj_set_style_text_color(la, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(la, &lv_font_montserrat_32, 0);
    lv_obj_center(la);

    // NEXT — right side, transparent
    lv_obj_t* btnR = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btnR, 80, 240);
    lv_obj_align(btnR, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_opa(btnR,     LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(btnR, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(btnR, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_add_event_cb(btnR, cbNext, LV_EVENT_ALL, NULL);
    lv_obj_t* ra = lv_label_create(btnR);
    lv_label_set_text(ra, ">");
    lv_obj_set_style_text_color(ra, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(ra, &lv_font_montserrat_32, 0);
    lv_obj_center(ra);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // Auto-detect touch — same as official examples
    if (!panel.begin()) {
        while (1) { Serial.println("Panel FAIL"); delay(1000); }
    }
    Serial.printf("Touch: %s\n", panel.getTouchModelName());

    beginLvglHelper(panel);

    // Black background
    lv_obj_set_style_bg_color(lv_scr_act(), C_BG, 0);

    // Create all 5 page containers upfront
    for (int i = 0; i < NUM_PAGES; i++) {
        g_pages[i] = mkPage();
        lv_obj_add_flag(g_pages[i], LV_OBJ_FLAG_HIDDEN);  // all hidden
    }

    // Build page content (static data)
    buildPage1();
    buildPage2();
    buildPage3();
    buildPage4();
    buildPage5();

    // Show first page
    lv_obj_clear_flag(g_pages[0], LV_OBJ_FLAG_HIDDEN);

    // Nav buttons on top — created once, never destroyed
    createNavButtons();

    // Brightness at end — same as lv_helloworld
    panel.setBrightness(16);

    Serial.println("DEMO ready — touch < > to navigate");
}

// ── Loop — identical to lv_helloworld ────────────────────────────────────────
void loop()
{
    lv_timer_handler();
    delay(2);
}

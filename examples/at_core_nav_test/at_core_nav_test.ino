/**
 * AT-CORE — Navigation Test
 * Based strictly on lv_helloworld.ino + Touchpad.ino patterns
 * Goal: confirm touch buttons work before adding full UI
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

// ── Same as lv_helloworld — global panel ──────────────────────────────────────
LilyGo_RGBPanel panel;

static uint8_t   g_page      = 0;
static lv_obj_t* g_pageLabel = nullptr;

const char* PAGE_NAMES[5] = {
    "Page 1 / 5 - STATUS",
    "Page 2 / 5 - FLIGHT",
    "Page 3 / 5 - TRAFFIC",
    "Page 4 / 5 - ALERTS",
    "Page 5 / 5 - DEBUG"
};

// ── Callbacks — same pattern as lv_helloworld btn_event_cb ───────────────────
static void cbPrev(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        g_page = (g_page == 0) ? 4 : g_page - 1;
        lv_label_set_text(g_pageLabel, PAGE_NAMES[g_page]);
        lv_obj_center(g_pageLabel);
        Serial.printf("Page -> %d\n", g_page);
    }
}

static void cbNext(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        g_page = (g_page + 1) % 5;
        lv_label_set_text(g_pageLabel, PAGE_NAMES[g_page]);
        lv_obj_center(g_pageLabel);
        Serial.printf("Page -> %d\n", g_page);
    }
}

// ── Setup — identical structure to lv_helloworld ─────────────────────────────
void setup()
{
    Serial.begin(115200);

    // Auto-detect touch model — same as Touchpad.ino and lv_helloworld
    bool rslt = panel.begin();
    if (!rslt) {
        while (1) {
            Serial.println("Error: panel init failed");
            delay(1000);
        }
    }

    // Show touch model name (diagnostic)
    Serial.printf("Touch model: %s\n", panel.getTouchModelName());

    beginLvglHelper(panel);

    // Background black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);

    // Page label — center screen
    g_pageLabel = lv_label_create(lv_scr_act());
    lv_label_set_text(g_pageLabel, PAGE_NAMES[0]);
    lv_obj_set_style_text_color(g_pageLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(g_pageLabel, &lv_font_montserrat_22, 0);
    lv_obj_center(g_pageLabel);

    // Touch debug label — bottom
    lv_obj_t* dbg = lv_label_create(lv_scr_act());
    lv_label_set_text(dbg, "touch test");
    lv_obj_set_style_text_color(dbg, lv_color_hex(0xF5A623), 0);
    lv_obj_set_style_text_font(dbg, &lv_font_montserrat_16, 0);
    lv_obj_align(dbg, LV_ALIGN_BOTTOM_MID, 0, -60);

    // PREV button — left side, visible (not transparent yet)
    lv_obj_t* btnL = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btnL, 80, 200);
    lv_obj_align(btnL, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(btnL, cbPrev, LV_EVENT_ALL, NULL);
    lv_obj_t* la = lv_label_create(btnL);
    lv_label_set_text(la, "<");
    lv_obj_set_style_text_font(la, &lv_font_montserrat_32, 0);
    lv_obj_center(la);

    // NEXT button — right side, visible (not transparent yet)
    lv_obj_t* btnR = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btnR, 80, 200);
    lv_obj_align(btnR, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(btnR, cbNext, LV_EVENT_ALL, NULL);
    lv_obj_t* ra = lv_label_create(btnR);
    lv_label_set_text(ra, ">");
    lv_obj_set_style_text_font(ra, &lv_font_montserrat_32, 0);
    lv_obj_center(ra);

    // Brightness at end — same as lv_helloworld
    panel.setBrightness(16);
}

// ── Loop — identical to lv_helloworld ────────────────────────────────────────
void loop()
{
    // Touch raw poll — diagnostic (shows in Serial)
    static int16_t tx, ty;
    if (panel.getPoint(&tx, &ty)) {
        Serial.printf("Touch X:%d Y:%d\n", tx, ty);
    }

    lv_timer_handler();
    delay(2);
}

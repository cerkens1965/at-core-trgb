/* AUTO-GENERATED — do not edit: run tools/svg2lvgl.py */
#pragma once
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_img_dsc_t img_dot;
extern const lv_img_dsc_t img_glider;
extern const lv_img_dsc_t img_para_glider;
extern const lv_img_dsc_t img_hand_glider;
extern const lv_img_dsc_t img_para_motor;
extern const lv_img_dsc_t img_parachute;
extern const lv_img_dsc_t img_flex_wing_trikes;
extern const lv_img_dsc_t img_light_aircraft;
extern const lv_img_dsc_t img_aircraft;
extern const lv_img_dsc_t img_heavy_aircraft;
extern const lv_img_dsc_t img_helicopter;
extern const lv_img_dsc_t img_gyrocopter;
extern const lv_img_dsc_t img_airship;
extern const lv_img_dsc_t img_ballon;
extern const lv_img_dsc_t img_uav;
extern const lv_img_dsc_t img_pav;
extern const lv_img_dsc_t img_military;

#ifdef __cplusplus
}
#endif

static const lv_img_dsc_t* getAircraftIcon(int type) {
    switch (type) {
        case  0: return &img_dot;
        case  1: return &img_dot;
        case  2: return &img_glider;
        case  3: return &img_para_glider;
        case  4: return &img_hand_glider;
        case  5: return &img_para_motor;
        case  6: return &img_parachute;
        case  7: return &img_flex_wing_trikes;
        case  8: return &img_light_aircraft;
        case  9: return &img_aircraft;
        case 10: return &img_heavy_aircraft;
        case 11: return &img_helicopter;
        case 12: return &img_gyrocopter;
        case 13: return &img_airship;
        case 14: return &img_ballon;
        case 15: return &img_uav;
        case 16: return &img_pav;
        case 17: return &img_military;
        default:   return &img_dot;
    }
}

#!/usr/bin/env python3
"""
Convert SafeSky SVG icons → LVGL 8.x C arrays (RGB565 + Alpha, 24×24 px)
Usage: python3 tools/svg2lvgl.py
Output: examples/at-core-trgb/at_core_debug/img_aircraft_icons.c
         examples/at-core-trgb/at_core_debug/img_aircraft_icons.h
"""

import subprocess
import io
import os
from PIL import Image

ICON_SIZE  = 32
ICONS_DIR  = "public/icons"
OUT_DIR    = "examples/at_core_debug"

# SafeSky beacon_type int → (C name, svg filename)
BEACON_MAP = [
    (0,  "img_dot",              "dot"),
    (1,  "img_dot",              None),           # STATIC_OBJECT → same as dot
    (2,  "img_glider",           "glider"),
    (3,  "img_para_glider",      "para_glider"),
    (4,  "img_hand_glider",      "hand_glider"),
    (5,  "img_para_motor",       "para_motor"),
    (6,  "img_parachute",        "parachute"),
    (7,  "img_flex_wing_trikes", "flex_wing_trikes"),
    (8,  "img_light_aircraft",   "light_aircraft"),
    (9,  "img_aircraft",         "aircraft"),
    (10, "img_heavy_aircraft",   "heavy_aircraft"),
    (11, "img_helicopter",       "helicopter"),
    (12, "img_gyrocopter",       "gyrocopter"),
    (13, "img_airship",          "airship"),
    (14, "img_ballon",           "ballon"),
    (15, "img_uav",              "uav"),
    (16, "img_pav",              "pav"),
    (17, "img_military",         "military"),
]

# Unique icons to generate (skip aliases)
UNIQUE_ICONS = [(name, svg) for _, name, svg in BEACON_MAP if svg is not None]
UNIQUE_ICONS = list({name: svg for name, svg in UNIQUE_ICONS}.items())


def svg_to_white_png(svg_path: str, size: int) -> bytes:
    """Rasterise SVG with rsvg-convert, return PNG bytes."""
    result = subprocess.run(
        ["rsvg-convert", "-w", str(size), "-h", str(size), svg_path],
        capture_output=True, check=True
    )
    return result.stdout


def png_to_lvgl_rgb565a(png_bytes: bytes) -> tuple[list[int], int, int]:
    """Convert PNG bytes to LVGL RGB565+Alpha array (LV_COLOR_16_SWAP 0).
    Returns (data_bytes, width, height).
    Non-transparent pixels are forced to white so LVGL recolor works."""
    img = Image.open(io.BytesIO(png_bytes)).convert("RGBA")
    w, h = img.size
    data = []
    for r, g, b, a in img.getdata():
        # Force visible pixels to white → recolor works perfectly in LVGL
        if a > 0:
            r, g, b = 255, 255, 255
        r5 = r >> 3
        g6 = g >> 2
        b5 = b >> 3
        rgb565 = (r5 << 11) | (g6 << 5) | b5
        data.append(rgb565 & 0xFF)        # low byte
        data.append((rgb565 >> 8) & 0xFF) # high byte
        data.append(a)                    # alpha
    return data, w, h


def format_c_array(name: str, data: list[int], w: int, h: int) -> str:
    rows = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        rows.append("    " + ", ".join(f"0x{b:02X}" for b in chunk) + ",")
    body = "\n".join(rows)
    return (
        f"static const uint8_t {name}_map[] = {{\n"
        f"{body}\n"
        f"}};\n\n"
        f"const lv_img_dsc_t {name} = {{\n"
        f"    .header.cf           = LV_IMG_CF_TRUE_COLOR_ALPHA,\n"
        f"    .header.always_zero  = 0,\n"
        f"    .header.reserved     = 0,\n"
        f"    .header.w            = {w},\n"
        f"    .header.h            = {h},\n"
        f"    .data_size           = {w * h * 3},\n"
        f"    .data                = {name}_map,\n"
        f"}};\n"
    )


def build_lookup_table() -> str:
    """Generate getAircraftIcon(int type) switch."""
    lines = []
    for type_id, c_name, _ in BEACON_MAP:
        lines.append(f"        case {type_id:2d}: return &{c_name};")
    body = "\n".join(lines)
    return (
        "static const lv_img_dsc_t* getAircraftIcon(int type) {\n"
        "    switch (type) {\n"
        f"{body}\n"
        "        default:   return &img_dot;\n"
        "    }\n"
        "}\n"
    )


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(script_dir)
    icons_dir = os.path.join(root, ICONS_DIR)
    out_dir   = os.path.join(root, OUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    c_sections  = []
    h_externs   = []

    generated = set()
    for c_name, svg_stem in UNIQUE_ICONS:
        if c_name in generated:
            continue
        generated.add(c_name)

        svg_path = os.path.join(icons_dir, f"{svg_stem}.svg")
        if not os.path.exists(svg_path):
            print(f"  [SKIP] {svg_path} not found")
            continue

        print(f"  {svg_stem}.svg → {c_name} ({ICON_SIZE}×{ICON_SIZE})")
        png_bytes = svg_to_white_png(svg_path, ICON_SIZE)
        data, w, h = png_to_lvgl_rgb565a(png_bytes)
        c_sections.append(format_c_array(c_name, data, w, h))
        h_externs.append(f"extern const lv_img_dsc_t {c_name};")

    # .c file
    c_text = (
        "/* AUTO-GENERATED — do not edit: run tools/svg2lvgl.py */\n"
        "#include \"img_aircraft_icons.h\"\n\n"
        + "\n".join(c_sections)
    )
    c_path = os.path.join(out_dir, "img_aircraft_icons.c")
    with open(c_path, "w") as f:
        f.write(c_text)

    # .h file
    lookup = build_lookup_table()
    h_text = (
        "/* AUTO-GENERATED — do not edit: run tools/svg2lvgl.py */\n"
        "#pragma once\n"
        "#include <lvgl.h>\n\n"
        "#ifdef __cplusplus\nextern \"C\" {\n#endif\n\n"
        + "\n".join(h_externs) + "\n\n"
        "#ifdef __cplusplus\n}\n#endif\n\n"
        + lookup
    )
    h_path = os.path.join(out_dir, "img_aircraft_icons.h")
    with open(h_path, "w") as f:
        f.write(h_text)

    print(f"\nDone → {c_path}")
    print(f"       {h_path}")


if __name__ == "__main__":
    main()

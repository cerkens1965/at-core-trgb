#!/usr/bin/env python3
"""
Convert AeroTrace brand PNGs → LVGL 8.x C arrays (RGB565 + Alpha)
Usage: python3 tools/png2lvgl_logos.py
Output: examples/at_core_debug/img_logos.c / .h
"""

import io, os
from PIL import Image

LOGOS = [
    ("img_logo_aerotrace", "AerotrAce_AeroTrace.png", 240),
    ("img_logo_atview",    "AerotrAce_AT-VIEW.png",   220),
]

IN_DIR  = "public/logo"
OUT_DIR = "examples/at_core_debug"


def png_to_lvgl_white(path: str, target_w: int) -> tuple[list[int], int, int]:
    img = Image.open(path).convert("RGBA")
    ratio = img.height / img.width
    target_h = max(1, round(target_w * ratio))
    img = img.resize((target_w, target_h), Image.LANCZOS)
    data = []
    for r, g, b, a in img.getdata():
        if a > 0:
            r, g, b = 255, 255, 255
        r5 = r >> 3; g6 = g >> 2; b5 = b >> 3
        rgb565 = (r5 << 11) | (g6 << 5) | b5
        data.append(rgb565 & 0xFF)
        data.append((rgb565 >> 8) & 0xFF)
        data.append(a)
    return data, target_w, target_h


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
        f"    .header.cf          = LV_IMG_CF_TRUE_COLOR_ALPHA,\n"
        f"    .header.always_zero = 0,\n"
        f"    .header.reserved    = 0,\n"
        f"    .header.w           = {w},\n"
        f"    .header.h           = {h},\n"
        f"    .data_size          = {w * h * 3},\n"
        f"    .data               = {name}_map,\n"
        f"}};\n"
    )


def main():
    root     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    in_dir   = os.path.join(root, IN_DIR)
    out_dir  = os.path.join(root, OUT_DIR)

    c_sections, h_externs = [], []

    for c_name, filename, target_w in LOGOS:
        path = os.path.join(in_dir, filename)
        data, w, h = png_to_lvgl_white(path, target_w)
        print(f"  {filename} → {c_name} ({w}×{h})")
        c_sections.append(format_c_array(c_name, data, w, h))
        h_externs.append(f"extern const lv_img_dsc_t {c_name};")

    c_text = (
        "/* AUTO-GENERATED — do not edit: run tools/png2lvgl_logos.py */\n"
        "#include \"img_logos.h\"\n\n"
        + "\n".join(c_sections)
    )
    with open(os.path.join(out_dir, "img_logos.c"), "w") as f:
        f.write(c_text)

    h_text = (
        "/* AUTO-GENERATED — do not edit: run tools/png2lvgl_logos.py */\n"
        "#pragma once\n"
        "#include <lvgl.h>\n\n"
        "#ifdef __cplusplus\nextern \"C\" {\n#endif\n\n"
        + "\n".join(h_externs) + "\n\n"
        "#ifdef __cplusplus\n}\n#endif\n"
    )
    with open(os.path.join(out_dir, "img_logos.h"), "w") as f:
        f.write(h_text)

    print(f"\nDone → {out_dir}/img_logos.c / .h")


if __name__ == "__main__":
    main()

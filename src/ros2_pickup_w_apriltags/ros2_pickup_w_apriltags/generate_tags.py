#!/usr/bin/env python3
"""
generate_tags.py

Generates printable ArUco marker images for each trash category used
by tag_detector_node.py. Run this once to produce PNG files, then
print them and attach to the corresponding trash items.

Usage:
    python3 generate_tags.py

Output:
    tags_output/tag_0_burnable.png
    tags_output/tag_1_pet_bottle.png
    tags_output/tag_2_can.png
    tags_output/tag_3_nonburnable.png

IMPORTANT - printing correctly:
    tag_detector_node.py assumes each marker's BLACK SQUARE is exactly
    MARKER_SIZE meters wide (default 0.05 = 50mm). If you print at the
    wrong scale, every distance reading the detector produces will be
    biased by the same proportion (e.g. print 10% too small -> every
    distance reads 10% too far).

    To print correctly:
      1. Open the PNG in an image viewer or editor that can print at an
         exact physical size (e.g. GIMP, Preview on Mac, or any editor
         with a "print at 100% scale" / "no scaling" / "actual size"
         option).
      2. Do NOT use "fit to page" or "scale to fit" - this will resize
         the marker to whatever the page size allows.
      3. After printing, measure the black square with a ruler. If it's
         not exactly 50mm, either reprint at the correct scale or update
         MARKER_SIZE in tag_detector_node.py to match your actual
         printed size.

    This script bakes in a known DPI (see PRINT_DPI below) and sizes the
    image so that printing it at that DPI with no additional scaling
    produces a marker of exactly MARKER_SIZE_MM. If your printer doesn't
    let you set DPI directly, printing at "actual size" from an image
    viewer should still respect the pixel dimensions correctly - always
    verify with a ruler regardless.
"""

import os
import cv2

# --- Configuration - keep these in sync with tag_detector_node.py ---
CATEGORY_NAMES = {
    0: "burnable",
    1: "pet_bottle",
    2: "can",
    3: "nonburnable",
}

MARKER_SIZE_MM = 50          # must match MARKER_SIZE in tag_detector_node.py
PRINT_DPI = 300              # standard printer resolution; change if needed
WHITE_BORDER_MM = 15         # white margin around the marker, helps detection

OUTPUT_DIR = "tags_output"


def mm_to_px(mm, dpi):
    return int(round((mm / 25.4) * dpi))


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    marker_px = mm_to_px(MARKER_SIZE_MM, PRINT_DPI)
    border_px = mm_to_px(WHITE_BORDER_MM, PRINT_DPI)
    canvas_px = marker_px + 2 * border_px

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for tag_id, name in CATEGORY_NAMES.items():
        marker_img = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)

        # Paste the marker onto a white canvas with a border, so the
        # detector has clean contrast around the marker edges.
        canvas = 255 * cv2.UMat(canvas_px, canvas_px, cv2.CV_8UC1).get()
        canvas[:] = 255
        canvas[border_px:border_px + marker_px, border_px:border_px + marker_px] = marker_img

        filename = os.path.join(OUTPUT_DIR, f"tag_{tag_id}_{name}.png")
        cv2.imwrite(filename, canvas)
        print(f"Wrote {filename}  ({canvas_px}x{canvas_px}px @ {PRINT_DPI} DPI "
              f"-> {MARKER_SIZE_MM}mm marker + {WHITE_BORDER_MM}mm border)")

    print(
        "\nDone. Print each PNG at 100% scale / actual size (NOT 'fit to "
        "page'), then measure the black square with a ruler to confirm "
        f"it's {MARKER_SIZE_MM}mm before attaching to items."
    )


if __name__ == "__main__":
    main()
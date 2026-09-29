#!/usr/bin/env python3
"""
Generates a printable checkerboard calibration pattern for tools/stereo_calibrate.py.

Default (2026-07-27, v2): sized to fill an A3 sheet (297x420mm) as a print-ready PDF
with the correct physical page size embedded, so a print at "Actual Size" on A3 paper
comes out full-size automatically — no more relying on a phone screen's small display
(the earlier phone-based attempt used ~7mm squares, too small for reliable corner
detection; this fills A3 with ~38mm squares, ~5x bigger, and a full sheet is far
easier to tilt at real angles than a handheld phone, which is what earlier caused the
two checkerboard-calibration attempts to diverge — see PEDESTRIAN_DETECTION.md).
9x6 internal corners (10x7 squares, the classic OpenCV default board) by default.

IMPORTANT — printed square size must still be VERIFIED, not trusted, even with a
correctly-sized PDF: printer driver scaling, "fit to page" defaults, or a print shop
misreading the paper size can all still drift the real printed size.
  1. Print the PDF at 100% / "Actual Size" on real A3 paper (NOT "Fit to page"/"Shrink
     to fit" — those will silently rescale it).
  2. Measure one square's edge with a ruler.
  3. Pass the MEASURED value (not the value this script printed) to
     stereo_calibrate.py's --square-size-mm.
  4. Mount the print FLAT and RIGID (tape to a clipboard/board/box) — any curvature,
     sag, or handheld wobble directly corrupts the calibration, and a full A3 sheet
     sags more than a small board if not backed by something stiff.

Run:
    python tools/gen_checkerboard.py
    (writes tools/checkerboard.png + tools/checkerboard.pdf, A3-sized by default;
     --page a4/letter/none, --cols/--rows/--margin-mm/--out to customize)
"""
import argparse
import cv2
import numpy as np
from PIL import Image

PAGE_SIZES_MM = {
    "a3": (297.0, 420.0),
    "a4": (210.0, 297.0),
    "letter": (215.9, 279.4),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cols", type=int, default=10, help="squares across (internal corners = cols-1)")
    ap.add_argument("--rows", type=int, default=7, help="squares down (internal corners = rows-1)")
    ap.add_argument("--page", type=str, default="a3", choices=list(PAGE_SIZES_MM) + ["none"],
                     help="fill this page size (landscape), auto-sizing squares to fit — "
                          "'none' falls back to an explicit --square-size-mm, PNG only, no page")
    ap.add_argument("--margin-mm", type=float, default=12.0,
                     help="blank border kept clear of the board on all sides, in mm — stays "
                          "within a typical printer's safe/printable area")
    ap.add_argument("--dpi", type=int, default=300, help="render resolution — also embedded "
                     "in the PDF page size, so higher isn't just sharper, it must stay consistent")
    ap.add_argument("--square-size-mm", type=float, default=25.0,
                     help="--page none only: explicit square size (a REQUEST, not a guarantee; "
                          "verify after printing)")
    ap.add_argument("--px-per-mm", type=float, default=10.0,
                     help="--page none only: rendering resolution")
    ap.add_argument("--out", type=str, default="tools/checkerboard.png")
    args = ap.parse_args()

    internal_cols, internal_rows = args.cols - 1, args.rows - 1

    if args.page == "none":
        px_per_mm = args.px_per_mm
        sq_px = int(round(args.square_size_mm * px_per_mm))
        margin_px = sq_px  # 1 full square of white border
        board_w_px = args.cols * sq_px
        board_h_px = args.rows * sq_px
        img_w = board_w_px + 2 * margin_px
        img_h = board_h_px + 2 * margin_px
        img = np.full((img_h, img_w), 255, dtype=np.uint8)
        ox, oy = margin_px, margin_px
        square_size_mm = args.square_size_mm
        pdf_path = None
    else:
        page_w_mm, page_h_mm = PAGE_SIZES_MM[args.page]
        # landscape — easier to hold up and matches the camera's ~4:3 aspect better
        page_w_mm, page_h_mm = max(page_w_mm, page_h_mm), min(page_w_mm, page_h_mm)
        avail_w_mm = page_w_mm - 2 * args.margin_mm
        avail_h_mm = page_h_mm - 2 * args.margin_mm
        square_size_mm = min(avail_w_mm / args.cols, avail_h_mm / args.rows)
        px_per_mm = args.dpi / 25.4
        img_w = int(round(page_w_mm * px_per_mm))
        img_h = int(round(page_h_mm * px_per_mm))
        sq_px = square_size_mm * px_per_mm
        board_w_px = args.cols * sq_px
        board_h_px = args.rows * sq_px
        ox = int(round((img_w - board_w_px) / 2.0))
        oy = int(round((img_h - board_h_px) / 2.0))
        img = np.full((img_h, img_w), 255, dtype=np.uint8)
        pdf_path = args.out.rsplit(".", 1)[0] + ".pdf"

    for r in range(args.rows):
        for c in range(args.cols):
            if (r + c) % 2 == 0:
                y0 = oy + int(round(r * sq_px))
                y1 = oy + int(round((r + 1) * sq_px))
                x0 = ox + int(round(c * sq_px))
                x1 = ox + int(round((c + 1) * sq_px))
                img[y0:y1, x0:x1] = 0

    cv2.imwrite(args.out, img)
    print(f"Wrote {args.out}  ({img_w}x{img_h}px, {args.cols}x{args.rows} squares, "
          f"{internal_cols}x{internal_rows} internal corners)")
    print(f"Target square size: {square_size_mm:.2f}mm  "
          f"(=> board pattern is {args.cols*square_size_mm:.0f}x{args.rows*square_size_mm:.0f}mm)")

    if pdf_path:
        dpi = args.dpi
        Image.fromarray(img).convert("RGB").save(pdf_path, "PDF", resolution=dpi)
        page_w_mm, page_h_mm = PAGE_SIZES_MM[args.page]
        page_w_mm, page_h_mm = max(page_w_mm, page_h_mm), min(page_w_mm, page_h_mm)
        print(f"Wrote {pdf_path}  ({page_w_mm:.0f}x{page_h_mm:.0f}mm page, landscape, "
              f"embedded at {dpi} DPI — print at 100%/'Actual Size' on real A3 paper)")

    print("\nAfter printing: MEASURE one printed square with a ruler (do not trust the target "
          "value above) and pass the measured mm to stereo_calibrate.py's --square-size-mm.")
    print("Mount flat and rigid (tape to something stiff) before calibrating.")
    print(f"Tell stereo_calibrate.py the board as: --cols {internal_cols} --rows {internal_rows}")


if __name__ == "__main__":
    main()

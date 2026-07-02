from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parent / "assets" / "colreg_apf_dispatch_flowchart.png"
OUT.parent.mkdir(parents=True, exist_ok=True)

W, H = 2600, 2300
image = Image.new("RGB", (W, H), "white")
draw = ImageDraw.Draw(image)


def font(size: int, bold: bool = False):
    name = "arialbd.ttf" if bold else "arial.ttf"
    return ImageFont.truetype(str(Path("C:/Windows/Fonts") / name), size)


def centered_text(box, text, size=32, bold=False, fill="black"):
    x1, y1, x2, y2 = box
    fnt = font(size, bold)
    lines = text.split("\n")
    heights = []
    widths = []
    for line in lines:
        bb = draw.textbbox((0, 0), line, font=fnt)
        widths.append(bb[2] - bb[0])
        heights.append(bb[3] - bb[1])
    gap = 7
    total_h = sum(heights) + gap * (len(lines) - 1)
    y = y1 + (y2 - y1 - total_h) / 2
    for line, tw, th in zip(lines, widths, heights):
        draw.text((x1 + (x2 - x1 - tw) / 2, y), line, font=fnt, fill=fill)
        y += th + gap


def box(cx, cy, width, height, text, *, fill="white", size=31, bold=False, radius=18):
    b = (cx - width / 2, cy - height / 2, cx + width / 2, cy + height / 2)
    draw.rounded_rectangle(b, radius=radius, fill=fill, outline="black", width=4)
    centered_text(b, text, size=size, bold=bold)
    return b


def oval(cx, cy, width, height, text):
    b = (cx - width / 2, cy - height / 2, cx + width / 2, cy + height / 2)
    draw.ellipse(b, fill="white", outline="black", width=4)
    centered_text(b, text, size=31, bold=True)
    return b


def diamond(cx, cy, width, height, text, size=29):
    pts = [(cx, cy - height / 2), (cx + width / 2, cy), (cx, cy + height / 2), (cx - width / 2, cy)]
    draw.polygon(pts, fill=(235, 235, 235), outline="black")
    draw.line(pts + [pts[0]], fill="black", width=4)
    centered_text((cx - width * 0.34, cy - height * 0.30, cx + width * 0.34, cy + height * 0.30), text, size=size, bold=True)
    return (cx - width / 2, cy - height / 2, cx + width / 2, cy + height / 2)


def arrow(points, label=None, label_xy=None, color="black", width=5):
    draw.line(points, fill=color, width=width, joint="curve")
    a, b = points[-2], points[-1]
    angle = math.atan2(b[1] - a[1], b[0] - a[0])
    size = 25
    left = (b[0] - size * math.cos(angle - 0.5), b[1] - size * math.sin(angle - 0.5))
    right = (b[0] - size * math.cos(angle + 0.5), b[1] - size * math.sin(angle + 0.5))
    draw.polygon([b, left, right], fill=color)
    if label:
        x, y = label_xy
        bb = draw.textbbox((0, 0), label, font=font(26, True))
        pad = 5
        draw.rectangle((x - pad, y - pad, x + bb[2] + pad, y + bb[3] + pad), fill="white")
        draw.text((x, y), label, font=font(26, True), fill=color)


# Section headers.
draw.rounded_rectangle((90, 60, 1660, 1560), radius=28, outline=(150, 150, 150), width=3)
draw.rounded_rectangle((1710, 60, 2510, 1560), radius=28, outline=(150, 150, 150), width=3)
draw.text((125, 78), "A. Candidate evaluation and risk ranking", font=font(35, True), fill=(55, 55, 55))
draw.text((1745, 78), "B. Controller dispatch", font=font(35, True), fill=(55, 55, 55))

# Candidate evaluation lane.
oval(850, 165, 330, 90, "Start")
box(850, 300, 730, 120, "Read own-vessel velocity and\nupdate predicted virtual obstacles")
box(850, 460, 730, 105, "Candidates = LiDAR obstacles + virtual obstacles")
diamond(850, 640, 590, 170, "Unprocessed\ncandidate?")
box(850, 825, 730, 115, "Transform candidate position and velocity\nto the own-vessel body frame")
diamond(850, 1015, 650, 175, "Valid and inside\nactivation sector?")
box(850, 1195, 730, 115, "Classify as head-on, overtaking,\ncrossing, or fallback static obstacle")
box(850, 1370, 790, 125, "Compute TCPA/DCPA and lexicographic score:\n(rule priority, TCPA, DCPA or range)")
diamond(850, 1535, 590, 155, "Better than\ncurrent best?")

# Dispatch lane.
box(2110, 620, 610, 105, "selected_rule = best.rule", fill=(245, 245, 245), bold=True)
diamond(2110, 820, 620, 190, "Which rule\nwas selected?")
box(1870, 1065, 410, 125, "Head-on or overtaking:\nuse overtaking/head-on APF", size=27)
box(2110, 1250, 410, 115, "Crossing:\nuse crossing APF", size=28)
box(2350, 1065, 270, 125, "Other:\nuse default APF", size=26)

# Merge and final control stage.
box(1300, 1740, 760, 115, "Compute APF control command u_cmd", fill=(245, 245, 245), bold=True)
box(1300, 1910, 940, 130, "Record selected controller, encounter mode,\nTCPA and DCPA in state and snapshots")
oval(1300, 2090, 470, 105, "Return u_cmd")

# Main downward path.
arrow([(850, 210), (850, 240)])
arrow([(850, 360), (850, 408)])
arrow([(850, 512), (850, 555)])
arrow([(850, 725), (850, 767)], "Yes", (875, 735))
arrow([(850, 883), (850, 927)])
arrow([(850, 1103), (850, 1137)], "Yes", (875, 1100))
arrow([(850, 1253), (850, 1307)])
arrow([(850, 1433), (850, 1458)])

# Loop when candidate is invalid or not better.
arrow([(525, 1015), (330, 1015), (330, 640), (555, 640)], "No", (345, 970), color=(80, 80, 80))
arrow([(555, 1535), (260, 1535), (260, 640), (555, 640)], "No / next", (275, 1485), color=(80, 80, 80))
arrow([(850, 1613), (850, 1650), (470, 1650), (470, 640), (555, 640)], "Yes / update best", (875, 1610), color=(80, 80, 80))

# End of candidate loop goes to dispatch.
arrow([(1145, 640), (1700, 640), (1805, 640)], "No candidates left", (1290, 595))
arrow([(2110, 673), (2110, 725)])

# Dispatch branches.
arrow([(1800, 820), (1640, 820), (1640, 930), (1870, 930), (1870, 1002)])
arrow([(2110, 915), (2110, 1192)])
arrow([(2420, 820), (2520, 820), (2520, 930), (2350, 930), (2350, 1002)])

# Merge into final control stage.
arrow([(1870, 1128), (1870, 1615), (1300, 1615), (1300, 1682)])
arrow([(2110, 1308), (2110, 1615), (1300, 1615), (1300, 1682)])
arrow([(2350, 1128), (2350, 1615), (1300, 1615), (1300, 1682)])
arrow([(1300, 1798), (1300, 1845)])
arrow([(1300, 1975), (1300, 2037)])

draw.text(
    (130, 2200),
    "Source logic: src/laptop.py — select_colreg_strategy() and compute_apf_control().",
    font=font(27),
    fill=(65, 65, 65),
)

image.save(OUT, dpi=(300, 300))
print(OUT)

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUTPUT_DIR = Path(__file__).resolve().parent / "assets"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
WIDTH, HEIGHT = 3000, 2250
MARGIN, GAP = 115, 80
PANEL_W = (WIDTH - 2 * MARGIN - GAP) // 2
PANEL_H = (HEIGHT - 2 * MARGIN - GAP - 90) // 2


def font(size: int, bold: bool = False):
    filename = "arialbd.ttf" if bold else "arial.ttf"
    path = Path("C:/Windows/Fonts") / filename
    return ImageFont.truetype(str(path), size=size)


def arrow(draw, points, fill, width=8, dashed=False):
    if dashed:
        for a, b in zip(points[:-1], points[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            ux, uy = dx / length, dy / length
            step = 32
            pos = 0
            while pos < length:
                end = min(pos + 18, length)
                draw.line(
                    [(a[0] + ux * pos, a[1] + uy * pos), (a[0] + ux * end, a[1] + uy * end)],
                    fill=fill,
                    width=width,
                )
                pos += step
    else:
        draw.line(points, fill=fill, width=width, joint="curve")
    a, b = points[-2], points[-1]
    angle = math.atan2(b[1] - a[1], b[0] - a[0])
    tip = b
    size = 34
    left = (tip[0] - size * math.cos(angle - 0.48), tip[1] - size * math.sin(angle - 0.48))
    right = (tip[0] - size * math.cos(angle + 0.48), tip[1] - size * math.sin(angle + 0.48))
    draw.polygon([tip, left, right], fill=fill)


def bezier(p0, p1, p2, steps=50):
    points = []
    for i in range(steps + 1):
        t = i / steps
        x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t**2 * p2[0]
        y = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t**2 * p2[1]
        points.append((x, y))
    return points


def ship(draw, x, y, heading, fill, label):
    base = [(0, -52), (-25, 35), (0, 24), (25, 35)]
    angle = math.radians(heading)
    points = []
    for px, py in base:
        rx = px * math.cos(angle) - py * math.sin(angle)
        ry = px * math.sin(angle) + py * math.cos(angle)
        points.append((x + rx, y + ry))
    draw.polygon(points, fill=fill, outline="black")
    box = draw.textbbox((0, 0), label, font=font(23))
    draw.text((x - (box[2] - box[0]) / 2, y + 58), label, fill="black", font=font(23))


def panel(draw, left, top, title, rule):
    draw.rectangle([left, top, left + PANEL_W, top + PANEL_H], outline=(170, 170, 170), width=3)
    title_box = draw.textbbox((0, 0), title, font=font(34, bold=True))
    draw.text((left + (PANEL_W - title_box[2]) / 2, top + 18), title, fill="black", font=font(34, bold=True))
    rule_box = draw.textbbox((0, 0), rule, font=font(24))
    draw.text((left + (PANEL_W - rule_box[2]) / 2, top + 65), rule, fill=(70, 70, 70), font=font(24))
    return left + PANEL_W / 2, top + PANEL_H / 2 + 35


image = Image.new("RGB", (WIDTH, HEIGHT), "white")
draw = ImageDraw.Draw(image)
black, grey, light = (0, 0, 0), (90, 90, 90), (165, 165, 165)

# (a) Head-on.
cx, cy = panel(draw, MARGIN, MARGIN, "(a) Head-on", "Rule 14: both alter course to starboard")
ship(draw, cx, cy + 300, 0, black, "Own vessel")
ship(draw, cx, cy - 300, 180, (185, 185, 185), "Target vessel")
arrow(draw, [(cx, cy + 240), (cx, cy - 210)], light, 5, dashed=True)
arrow(draw, bezier((cx, cy + 240), (cx + 360, cy + 10), (cx + 300, cy - 250)), black, 9)
arrow(draw, bezier((cx, cy - 240), (cx - 360, cy - 10), (cx - 300, cy + 250)), grey, 9)
draw.text((cx + 330, cy - 25), "starboard\nalteration", fill=black, font=font(23))

# (b) Crossing, own vessel gives way.
left = MARGIN + PANEL_W + GAP
cx, cy = panel(draw, left, MARGIN, "(b) Crossing: give-way vessel", "Rules 15-16: avoid crossing ahead; act early and substantially")
ship(draw, cx - 150, cy + 300, 0, black, "Own vessel (give-way)")
ship(draw, cx + 360, cy - 30, 270, (185, 185, 185), "Target (stand-on)")
arrow(draw, [(cx - 150, cy + 240), (cx - 150, cy - 280)], light, 5, dashed=True)
arrow(draw, [(cx + 300, cy - 30), (cx - 430, cy - 30)], grey, 9)
arrow(draw, bezier((cx - 150, cy + 240), (cx + 380, cy + 170), (cx + 260, cy - 210)), black, 9)
draw.text((cx + 50, cy + 145), "pass astern", fill=black, font=font(23))

# (c) Crossing, own vessel stands on.
top = MARGIN + PANEL_H + GAP
cx, cy = panel(draw, MARGIN, top, "(c) Crossing: stand-on vessel", "Rule 17: initially maintain course and speed; monitor give-way action")
ship(draw, cx + 100, cy + 300, 0, black, "Own vessel (stand-on)")
ship(draw, cx - 390, cy - 30, 90, (185, 185, 185), "Target (give-way)")
arrow(draw, [(cx + 100, cy + 240), (cx + 100, cy - 300)], black, 9)
arrow(draw, [(cx - 330, cy - 30), (cx + 430, cy - 30)], light, 5, dashed=True)
arrow(draw, bezier((cx - 330, cy - 30), (cx, cy + 30), (cx + 360, cy + 230)), grey, 9)
draw.text((cx - 315, cy + 115), "target alters to\npass astern", fill=grey, font=font(23))

# (d) Overtaking.
cx, cy = panel(draw, left, top, "(d) Overtaking", "Rule 13: the overtaking vessel keeps out of the way")
ship(draw, cx - 80, cy + 310, 0, black, "Own vessel (overtaking)")
ship(draw, cx - 80, cy - 70, 0, (185, 185, 185), "Vessel being overtaken")
arrow(draw, [(cx - 80, cy - 130), (cx - 80, cy - 315)], grey, 9)
arrow(draw, [(cx - 80, cy + 250), (cx - 80, cy - 300)], light, 5, dashed=True)
arrow(draw, bezier((cx - 80, cy + 250), (cx + 410, cy + 20), (cx + 330, cy - 300)), black, 9)
draw.text((cx + 160, cy + 120), "example passing side;\nRule 13 does not prescribe one", fill=black, font=font(22))

legend = "Solid black: own-vessel action   |   Solid grey: target-vessel action   |   Dashed grey: original course"
legend_box = draw.textbbox((0, 0), legend, font=font(24))
draw.text(((WIDTH - legend_box[2]) / 2, HEIGHT - 62), legend, fill=(50, 50, 50), font=font(24))

png = OUTPUT_DIR / "colreg_encounter_schematic.png"
image.save(png, dpi=(300, 300))
print(png)

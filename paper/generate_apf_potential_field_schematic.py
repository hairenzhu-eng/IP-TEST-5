from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "paper" / "assets" / "apf_potential_field_generation.png"
WIDTH, HEIGHT = 2400, 1060
PANEL_W, PANEL_H = 1070, 790
PANEL_Y = 150
LEFT_X, RIGHT_X = 120, 1210
WORLD = (-1.0, 9.0, -4.2, 4.2)


def font(size, bold=False):
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/calibrib.ttf" if bold else "C:/Windows/Fonts/calibri.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def potential(x, y):
    goal = np.array([8.0, 0.0])
    obstacle = np.array([3.2, 0.8])
    virtual = np.array([4.7, 0.35])
    semi_axes = np.array([2.25, 1.25])
    virtual_axes = np.array([1.7, 0.9])

    distance_goal = np.hypot(x - goal[0], y - goal[1])
    u_att = 0.08 * np.minimum(distance_goal, 5.0) ** 2

    def repulsive(centre, axes, gain):
        level = np.sqrt(((x - centre[0]) / axes[0]) ** 2 + ((y - centre[1]) / axes[1]) ** 2)
        return gain * np.maximum(0.0, 1.0 - level) ** 2

    return (
        u_att
        + repulsive(obstacle, semi_axes, 7.0)
        + repulsive(virtual, virtual_axes, 3.0),
        goal,
        obstacle,
        virtual,
        semi_axes,
        virtual_axes,
    )


def transform(panel_x, x, y):
    xmin, xmax, ymin, ymax = WORLD
    px = panel_x + int((x - xmin) / (xmax - xmin) * PANEL_W)
    py = PANEL_Y + PANEL_H - int((y - ymin) / (ymax - ymin) * PANEL_H)
    return np.array([px, py], dtype=float)


def ellipse_box(panel_x, centre, axes):
    low = transform(panel_x, centre[0] - axes[0], centre[1] + axes[1])
    high = transform(panel_x, centre[0] + axes[0], centre[1] - axes[1])
    return [tuple(low), tuple(high)]


def draw_arrow(draw, start, end, width=4, dashed=False):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length < 1:
        return
    direction = delta / length
    if dashed:
        cursor = 0.0
        while cursor < length - 18:
            a = start + direction * cursor
            b = start + direction * min(cursor + 16, length - 18)
            draw.line([tuple(a), tuple(b)], fill="black", width=width)
            cursor += 27
    else:
        draw.line([tuple(start), tuple(end)], fill="black", width=width)
    normal = np.array([-direction[1], direction[0]])
    head = end - direction * 20
    draw.polygon(
        [tuple(end), tuple(head + normal * 9), tuple(head - normal * 9)],
        fill="black",
    )


def draw_vessel(draw, panel_x, centre, heading_deg=0.0, scale=0.45, fill="white"):
    hull = np.array([[1.25, 0.0], [-0.85, 0.55], [-0.55, 0.0], [-0.85, -0.55]])
    angle = np.deg2rad(heading_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    world_points = hull @ rotation.T * scale + np.asarray(centre)
    pixels = [tuple(transform(panel_x, x, y)) for x, y in world_points]
    draw.polygon(pixels, fill=fill, outline="black", width=3)


def star_points(centre, outer=14, inner=6):
    points = []
    for i in range(10):
        radius = outer if i % 2 == 0 else inner
        angle = -np.pi / 2 + i * np.pi / 5
        points.append((centre[0] + radius * np.cos(angle), centre[1] + radius * np.sin(angle)))
    return points


def label(draw, xy, text, size=24, anchor="mm", bold=False, fill="black"):
    draw.text(tuple(xy), text, font=font(size, bold), fill=fill, anchor=anchor, align="center")


def make_left_panel(canvas, draw, goal, obstacle, virtual, semi_axes, virtual_axes):
    grid_w, grid_h = PANEL_W, PANEL_H
    xs = np.linspace(WORLD[0], WORLD[1], grid_w)
    ys = np.linspace(WORLD[3], WORLD[2], grid_h)
    xx, yy = np.meshgrid(xs, ys)
    u, *_ = potential(xx, yy)
    lo, hi = np.percentile(u, [4, 97])
    scaled = np.clip((u - lo) / max(hi - lo, 1e-9), 0.0, 1.0)
    quantized = np.floor(scaled * 11) / 11
    grey = (246 - 150 * quantized).astype(np.uint8)
    panel = Image.fromarray(grey, mode="L").convert("RGB")
    canvas.paste(panel, (LEFT_X, PANEL_Y))

    draw.rectangle(
        [LEFT_X, PANEL_Y, LEFT_X + PANEL_W, PANEL_Y + PANEL_H],
        outline="black",
        width=2,
    )
    draw.ellipse(ellipse_box(LEFT_X, obstacle, semi_axes), outline="black", width=4)
    draw.ellipse(ellipse_box(LEFT_X, virtual, virtual_axes), outline="black", width=3)

    sample_x = np.linspace(-0.4, 8.4, 13)
    sample_y = np.linspace(-3.5, 3.5, 9)
    eps = 0.04
    for y in sample_y:
        for x in sample_x:
            uxp = potential(x + eps, y)[0]
            uxm = potential(x - eps, y)[0]
            uyp = potential(x, y + eps)[0]
            uym = potential(x, y - eps)[0]
            force = -np.array([(uxp - uxm) / (2 * eps), (uyp - uym) / (2 * eps)])
            n = np.linalg.norm(force)
            if n < 1e-8:
                continue
            start = transform(LEFT_X, x, y)
            end = transform(LEFT_X, x + 0.25 * force[0] / n, y + 0.25 * force[1] / n)
            draw_arrow(draw, start, end, width=2)

    draw_vessel(draw, LEFT_X, obstacle, heading_deg=-12, scale=0.6, fill="#bdbdbd")
    draw_vessel(draw, LEFT_X, virtual, heading_deg=-12, scale=0.4, fill="white")
    draw_vessel(draw, LEFT_X, (0.0, -0.6), heading_deg=5, scale=0.5, fill="white")
    goal_px = transform(LEFT_X, *goal)
    draw.polygon(star_points(goal_px), fill="black")
    draw.line(
        [tuple(transform(LEFT_X, -0.5, -0.9)), tuple(goal_px)],
        fill="black",
        width=2,
    )
    label(draw, transform(LEFT_X, 7.4, 0.55), "look-ahead goal", 23)
    label(draw, transform(LEFT_X, 3.1, 2.55), "LiDAR obstacle and\nPC1/PC2 safety ellipse", 23)
    label(draw, transform(LEFT_X, 5.2, -1.25), "EKF-predicted\nvirtual obstacle", 23)
    label(draw, transform(LEFT_X, 0.0, -1.35), "own vessel", 23)
    label(draw, (LEFT_X + PANEL_W / 2, PANEL_Y - 28), "(a) Combined attractive and repulsive potential", 27, bold=True)


def make_right_panel(draw, goal, obstacle, virtual, semi_axes, virtual_axes):
    draw.rectangle(
        [RIGHT_X, PANEL_Y, RIGHT_X + PANEL_W, PANEL_Y + PANEL_H],
        fill="white",
        outline="black",
        width=2,
    )
    zero_left = transform(RIGHT_X, WORLD[0], 0.0)
    zero_right = transform(RIGHT_X, WORLD[1], 0.0)
    draw.line([tuple(zero_left), tuple(zero_right)], fill="#9a9a9a", width=2)
    draw.ellipse(ellipse_box(RIGHT_X, obstacle, semi_axes), fill="#ececec", outline="black", width=4)
    draw.ellipse(ellipse_box(RIGHT_X, virtual, virtual_axes), outline="black", width=3)

    own = np.array([0.0, -0.4])
    draw_vessel(draw, RIGHT_X, own, heading_deg=0, scale=0.55, fill="white")
    draw_vessel(draw, RIGHT_X, obstacle, heading_deg=-12, scale=0.6, fill="#bdbdbd")
    draw_vessel(draw, RIGHT_X, virtual, heading_deg=-12, scale=0.4, fill="white")

    goal_force = np.array([3.4, 0.3])
    path_force = np.array([0.9, 0.34])
    rep_force = np.array([-0.75, -2.0])
    rule_force = np.array([0.2, -1.15])
    resultant = goal_force + path_force + rep_force + rule_force
    start = transform(RIGHT_X, *own)
    vectors = [
        (goal_force, "F_goal", False, 4, (24, -45)),
        (path_force, "F_path", True, 4, (18, -42)),
        (rep_force, "sum F_rep", False, 4, (18, 22)),
        (rule_force, "F_rule / side lock", True, 4, (38, 30)),
        (resultant, "F_resultant", False, 7, (28, 20)),
    ]
    for vector, text, dashed, width, offset in vectors:
        end = transform(RIGHT_X, *(own + vector))
        draw_arrow(draw, start, end, width=width, dashed=dashed)
        label(draw, end + np.array(offset), text, 22, anchor="lm")

    goal_px = transform(RIGHT_X, *goal)
    draw.polygon(star_points(goal_px), fill="black")
    draw.line(
        [tuple(transform(RIGHT_X, -0.5, -0.75)), tuple(goal_px)],
        fill="black",
        width=2,
    )
    label(draw, transform(RIGHT_X, 7.35, 0.55), "look-ahead goal", 23)
    label(draw, transform(RIGHT_X, 3.2, 2.45), "elliptical repulsive field", 23)
    label(draw, transform(RIGHT_X, 5.2, -1.25), "virtual field", 23)
    label(draw, transform(RIGHT_X, 0.0, -1.3), "own vessel", 23)
    label(draw, (RIGHT_X + PANEL_W / 2, PANEL_Y - 28), "(b) Code-aligned force decomposition", 27, bold=True)


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    canvas = Image.new("RGB", (WIDTH, HEIGHT), "white")
    draw = ImageDraw.Draw(canvas)
    _, goal, obstacle, virtual, semi_axes, virtual_axes = potential(0.0, 0.0)

    label(
        draw,
        (WIDTH / 2, 54),
        "Generation of the size-aware predictive artificial potential field",
        34,
        bold=True,
    )
    make_left_panel(canvas, draw, goal, obstacle, virtual, semi_axes, virtual_axes)
    make_right_panel(draw, goal, obstacle, virtual, semi_axes, virtual_axes)

    label(draw, (LEFT_X + PANEL_W / 2, 990), "body-frame forward / route direction", 25)
    label(draw, (RIGHT_X + PANEL_W / 2, 990), "body-frame forward / route direction", 25)
    canvas.save(OUTPUT, dpi=(300, 300))
    print(OUTPUT)


if __name__ == "__main__":
    main()

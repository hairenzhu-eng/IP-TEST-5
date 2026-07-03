from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "logs" / "generated_figures" / "obstacle_type_regions.png"


def polar_point(radius: float, bearing_deg: float) -> tuple[float, float]:
    """Convert a heading-style bearing into plot coordinates."""
    theta = np.deg2rad(90.0 - bearing_deg)
    return radius * np.cos(theta), radius * np.sin(theta)


def main() -> None:
    output_dir = OUTPUT.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(6.8, 5.8), dpi=220)
    radius = 1.0

    ax.add_patch(Circle((0.0, 0.0), radius, fill=False, edgecolor="black", linewidth=1.2))

    boundary_bearings = [337.5, 22.5, 112.5, 247.5]
    for bearing in boundary_bearings:
        x, y = polar_point(radius, bearing)
        ax.plot([0.0, x], [0.0, y], color="black", linewidth=1.0)

    ship = np.array(
        [
            (-0.12, -0.12),
            (0.12, -0.12),
            (0.06, 0.02),
            (0.00, 0.18),
            (-0.06, 0.02),
        ]
    )
    ax.add_patch(Polygon(ship, closed=True, fill=False, edgecolor="black", linewidth=1.1))

    labels = {
        "A": (0.0, 0.42),
        "B": (-0.52, 0.03),
        "C": (0.58, 0.03),
        "D": (0.0, -0.48),
    }
    for text, (x, y) in labels.items():
        ax.text(x, y, text, ha="center", va="center", fontsize=15, family="serif")

    angle_text = {
        "337.5°": (-0.56, 1.08),
        "22.5°": (0.56, 1.08),
        "247.5°": (-0.92, -0.86),
        "112.5°": (0.88, -0.86),
    }
    for text, (x, y) in angle_text.items():
        ax.text(x, y, text, ha="center", va="center", fontsize=13, family="serif")

    ax.text(0.0, -1.28, "A: head on", ha="center", va="center", fontsize=11)
    ax.text(0.0, -1.42, "B: crossing from port", ha="center", va="center", fontsize=11)
    ax.text(0.0, -1.56, "C: crossing from starboard", ha="center", va="center", fontsize=11)
    ax.text(0.0, -1.70, "D: static obstacle / other", ha="center", va="center", fontsize=11)
    ax.text(
        0.0,
        -1.92,
        "Overtaking in code is checked separately: |bearing|<=67.5° and relative heading/speed must match.",
        ha="center",
        va="center",
        fontsize=9.6,
    )

    ax.set_aspect("equal")
    ax.set_xlim(-1.35, 1.35)
    ax.set_ylim(-2.06, 1.2)
    ax.axis("off")

    fig.savefig(OUTPUT, bbox_inches="tight", pad_inches=0.06)
    plt.close(fig)
    print(OUTPUT)


if __name__ == "__main__":
    main()

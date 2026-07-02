import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pglive.sources.live_plot import LiveLinePlot  # noqa: E402
from show_laptop import PLOT_RATE_HZ, plot_connector  # noqa: E402


def test_plot_updates_are_throttled_and_skip_overlay_auto_range():
    connector = plot_connector(LiveLinePlot(), 10)

    assert connector.plot_timeout == 1 / PLOT_RATE_HZ
    assert connector.ignore_auto_range is True

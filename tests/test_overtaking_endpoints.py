from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_overtaking_motion_endpoints():
    laptop_source = (ROOT / "src" / "laptop-overtaking.py").read_text()
    webots_source = (
        ROOT / "webots" / "controllers" / "webots_robot" / "webots_robot.py"
    ).read_text()

    assert "goal_north, goal_east = 15, 1" in laptop_source
    assert '"stop_x": 10.0' in webots_source
    assert 'target["position"][0] >= target["stop_x"]' in webots_source

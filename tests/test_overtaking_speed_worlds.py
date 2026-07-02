from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_overtaking_speed_worlds_configure_front_obstacle_speed():
    speeds = (("0_2", "0.2"), ("0_4", "0.4"), ("0_6", "0.6"), ("0_8", "0.8"))
    for size in ("small", "large"):
        for speed_label, speed in speeds:
            world = (
                ROOT
                / "webots"
                / "worlds"
                / f"mr_webots_overtaking_{size}_ship_{speed_label}_m_s.wbt"
            ).read_text()
            assert "DEF FRONT_OBSTACLE_ROBOT ShipObstacle" in world
            assert f'customData "front_obstacle_speed={speed}"' in world
            if size == "large":
                assert "scale 2 1.65 1.5" in world
                assert "boundingSize 1.8 0.528 0.24" in world
            else:
                assert "scale 1 1 1" in world

    controller = (
        ROOT / "webots" / "controllers" / "webots_robot" / "webots_robot.py"
    ).read_text()
    assert 'custom_data.startswith("front_obstacle_speed=")' in controller
    assert "acceleration=0.0" in controller

from robotics_sim.preferences import load_preferences


def test_missing_corrupt_or_invalid_preferences_fall_back(tmp_path):
    path = tmp_path / "settings.json"
    assert load_preferences(path) == {}
    for text in ("broken", "[]", "null"):
        path.write_text(text)
        assert load_preferences(path) == {}
    path.write_text('{"label_size": 999, "width": true, "duration": -2, "camera_yaw": NaN, "show_frames": "false", "label_background": false, "label_background_transparency": 101}')
    assert load_preferences(path) == {"label_background": False}

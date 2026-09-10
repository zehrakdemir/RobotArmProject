"""FR3 komut plani + guvenli dry-run."""
from src.robot import FairinoFR3, plan_correction


def test_plan_standing_no_action():
    c = plan_correction("standing_on_wheels", None, (40, 68, 26), tilt_deg=1.0, lean_deg=3.0)
    assert c.action == "none"


def test_plan_standing_but_skewed_reposition():
    c = plan_correction("standing_on_wheels", [10, 20, 30], (40, 68, 26),
                        tilt_deg=12.0, lean_deg=3.0)
    assert c.action == "reposition"
    assert c.rotation_deg == -12.0


def test_plan_tipped_rotate_upright():
    c = plan_correction("tipped_over", [100, 200, 300], (68, 26, 40),
                        tilt_deg=2.0, lean_deg=80.0)
    assert c.action == "rotate_upright"
    assert c.grasp_pose is not None and len(c.grasp_pose) == 6
    assert c.lift_pose is not None and c.lift_pose[2] > c.grasp_pose[2]
    assert 0 < c.rotation_deg <= 90


def test_plan_uncertain_no_action():
    c = plan_correction("uncertain", None, (40, 42, 39), tilt_deg=2.0, lean_deg=20.0)
    assert c.action == "none"


def test_dry_run_never_dispatches():
    fr3 = FairinoFR3(enabled=False)
    assert fr3.connect() is False
    cmd = plan_correction("tipped_over", [100, 200, 300], (68, 26, 40), 2.0, 80.0)
    out = fr3.execute(cmd)
    assert out["dispatched"] is False
    assert out["dry_run"] is True
    assert any("DRY-RUN" in s for s in out["steps"])


def test_command_json_roundtrip():
    cmd = plan_correction("tipped_over", [1, 2, 3], (60, 25, 40), 2.0, 70.0)
    import json

    d = json.loads(cmd.to_json())
    assert d["action"] == "rotate_upright"

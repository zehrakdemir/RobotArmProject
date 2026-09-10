"""OBB uydurma + kavrama plani - numpy/cv2-siz."""
import numpy as np
import pytest

from src import config
from src.grasp import fit_obb, plan_grasp
from src.pose_classify import classify_obb
from tests._synth import box_surface_3d


def test_obb_extents_and_yaw():
    P = box_surface_3d((400, 200, 550), center_mm=(500, 0, 0), yaw_deg=0.0)
    obb = fit_obb(P, ground_z=0.0, up_axis="z")
    assert obb.width_mm == pytest.approx(400, abs=25)
    assert obb.thickness_mm == pytest.approx(200, abs=25)
    assert obb.height_mm == pytest.approx(550, abs=25)
    assert abs(((obb.yaw_deg + 90) % 180) - 90) < 12   # ~0 veya ~180


def test_obb_yaw_rotated():
    P = box_surface_3d((400, 200, 550), yaw_deg=30.0)
    obb = fit_obb(P, ground_z=0.0, up_axis="z")
    # uzun eksen ~30 (veya 210) derece
    assert min(abs(obb.yaw_deg - 30), abs(obb.yaw_deg - 210), abs(obb.yaw_deg + 150)) < 12


def test_grasp_infeasible_when_too_thick():
    P = box_surface_3d((400, 200, 550))       # kalinlik 200 > gripper 80
    plan = plan_grasp(P)
    assert plan.feasible is False
    assert "max" in plan.reason.lower() or "kavranamaz" in plan.reason.lower()


def test_grasp_feasible_thin_case():
    P = box_surface_3d((400, 60, 550), center_mm=(500, 0, 0), yaw_deg=0.0)
    plan = plan_grasp(P)
    assert plan.feasible is True
    assert plan.opening_mm == pytest.approx(60 + 2 * config.GRASP_CLEARANCE_MM, abs=15)
    # kavrama yuksekligi ~ tabandan %50 * 550
    assert plan.grasp_point[2] == pytest.approx(0.5 * 550, abs=40)
    # kapanma ekseni ~ kalinlik ekseni (y), yatay
    assert abs(plan.close_axis[2]) < 0.2
    assert plan.rpy_deg[0] == 180.0


def test_grasp_yaw_follows_thickness_axis():
    P = box_surface_3d((400, 60, 550), yaw_deg=25.0)
    plan = plan_grasp(P)
    # kalinlik ekseni uzun eksene dik -> yaw ~ 25+90
    assert plan.feasible
    d = min(abs(plan.yaw_deg - 115), abs(plan.yaw_deg + 65), abs(plan.yaw_deg - 295))
    assert d < 15


def test_classify_obb_upright_vs_flat():
    up = fit_obb(box_surface_3d((400, 200, 550)), ground_z=0, up_axis="z")
    flat = fit_obb(box_surface_3d((550, 400, 200)), ground_z=0, up_axis="z")
    assert classify_obb(up).label == "standing_on_wheels"
    assert classify_obb(flat).label == "tipped_over"

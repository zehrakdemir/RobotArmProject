"""Yan yatik -> dik konuma getirme plani - numpy/cv2-siz."""
import numpy as np
import pytest

from src.align import plan_tipup
from src.grasp import fit_obb
from src.robot import build_tipup
from tests._synth import box_surface_3d


def _flat_cloud():
    # buyuk yuz yerde: 550 x 400 taban, 200 dikey (kalinlik)
    return box_surface_3d((550, 400, 200), center_mm=(600, 100, 0), yaw_deg=0.0)


def test_tipup_basic_geometry():
    P = _flat_cloud()
    plan = plan_tipup(P, base_origin_xy=(0.0, 0.0))
    assert plan.feasible
    # itme yonu yatay ve birim
    assert abs(plan.push_dir[2]) < 1e-6
    assert np.hypot(plan.push_dir[0], plan.push_dir[1]) == pytest.approx(1.0, abs=1e-3)
    # donme uzun kenar (X) etrafinda -> itme KISA eksen (Y) boyunca
    assert abs(plan.push_dir[1]) > 0.9
    # robottan (orijin) UZAGA
    assert float(np.dot(plan.push_dir[:2], [600.0, 100.0])) > 0
    # temas noktasi ust kesime yakin (~0.8 * 200)
    assert plan.contact_point[2] == pytest.approx(0.8 * 200, abs=40)
    # dik olunca beklenen yukseklik ~ kisa yatay kenar (400)
    assert plan.expected_upright_height_mm == pytest.approx(400, abs=60)


def test_tipup_rejects_upright_cloud():
    P = box_surface_3d((400, 200, 550))       # zaten dik
    plan = plan_tipup(P)
    assert plan.feasible is False


def test_build_tipup_program_steps():
    P = _flat_cloud()
    plan = plan_tipup(P)
    prog = build_tipup(plan)
    assert prog.action == "tipup" and prog.feasible
    kinds = [k for k, _, _ in prog.steps]
    assert "gripper" in kinds and kinds.count("movel") >= 4

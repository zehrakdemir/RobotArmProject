"""
Olcum cekirdegi - MEDYAN tabanli en/boy, derinlik, 3B egim.
cv2 gerektirmeyen sayisal yardimcilar test edilir.
"""
import numpy as np
import pytest

from src.measurement import _depth_span, _lean_angle, _median_span
from tests._synth import box_point_cloud


def test_median_width_height():
    # real_d=0 -> fronto-paralel on yuz, temiz en/boy
    pts, mask = box_point_cloud(real_w=40.0, real_h=60.0, real_d=0.0)
    w_u, w_n = _median_span(pts, mask, "row")
    h_u, h_n = _median_span(pts, mask, "col")
    assert w_u == pytest.approx(40.0, abs=1.5)
    assert h_u == pytest.approx(60.0, abs=1.5)
    assert w_n > 50 and h_n > 50


def test_depth_span_percentile():
    pts, mask = box_point_cloud(real_w=8.0, real_h=8.0, real_d=30.0)
    d_u = _depth_span(pts, mask)
    # en yakin/uzak %10 medyan farki ~ 0.9 * gercek derinlik
    assert 20.0 <= d_u <= 30.0


@pytest.mark.parametrize("ang", [0.0, 15.0, 30.0, 45.0])
def test_lean_angle(ang):
    pts, mask = box_point_cloud(real_w=20.0, real_h=70.0, real_d=0.0, lean_deg=ang)
    assert _lean_angle(pts, mask) == pytest.approx(ang, abs=6.0)


def test_empty_mask_safe():
    pts = np.full((50, 50, 3), np.nan)
    mask = np.zeros((50, 50), bool)
    assert _median_span(pts, mask, "row") == (0.0, 0)
    assert _depth_span(pts, mask) == 0.0
    assert _lean_angle(pts, mask) == 0.0

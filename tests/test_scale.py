"""Olcek kalibrasyonu - cv2/model gerektirmez."""
import numpy as np
import pytest

from src import config
from src.measurement import OrientedRect, RawMeasurement, TopReference
from src.scale import ScaleError, compute_scale


def _raw(w=0.5, h=0.9, d=0.25):
    rect = OrientedRect((100, 100), (120, 200), 0.0, np.zeros((4, 2)), 0.0, False)
    top = TopReference((100, 40), None, False)
    return RawMeasurement(w, h, d, 100, 100, rect, top, 3.0, 5000)


def test_scale_width():
    cfg = config.RunConfig(known_width_cm=40.0, known_dimension="width")
    assert compute_scale(_raw(w=0.5), cfg) == pytest.approx(80.0)


def test_scale_height_axis():
    cfg = config.RunConfig(known_width_cm=56.0, known_dimension="height")
    assert compute_scale(_raw(h=0.8), cfg) == pytest.approx(70.0)


def test_to_metric_consistency():
    cfg = config.RunConfig(known_width_cm=40.0, known_dimension="width")
    m = _raw(w=0.5, h=0.9, d=0.25).to_metric(compute_scale(_raw(w=0.5), cfg))
    assert (m.width_cm, m.height_cm, m.depth_cm) == pytest.approx((40.0, 72.0, 20.0))


def test_zero_measure_raises():
    with pytest.raises(ScaleError):
        compute_scale(_raw(w=0.0), config.RunConfig(known_width_cm=40.0))


def test_bad_axis_raises():
    with pytest.raises(ScaleError):
        compute_scale(_raw(), config.RunConfig(known_width_cm=40.0, known_dimension="hacim"))

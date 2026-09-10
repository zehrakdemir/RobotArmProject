"""Dik / yatik durus siniflandirmasi."""
import numpy as np

from src import config
from src.measurement import OrientedRect, RawMeasurement, TopReference
from src.pose_classify import classify


def _metric(w, h, d, tilt=0.0, lean=0.0):
    rect = OrientedRect((0, 0), (0, 0), 0.0, np.zeros((4, 2)), tilt,
                        abs(tilt) > config.TILT_THRESHOLD_DEG)
    top = TopReference((0, 0), None, False)
    return RawMeasurement(w, h, d, 50, 50, rect, top, lean, 4000).to_metric(1.0)


def test_standing_on_wheels():
    r = classify(_metric(40, 68, 26, tilt=1.0, lean=3.0))
    assert r.label == "standing_on_wheels"
    assert r.needs_robot is False


def test_tipped_on_back():
    # sirt uzeri: dikey uzanim EN KUCUK
    r = classify(_metric(68, 26, 40, tilt=2.0, lean=82.0))
    assert r.label == "tipped_over"
    assert r.needs_robot is True


def test_tipped_on_side():
    # yan yatmis: en > boy
    r = classify(_metric(68, 40, 26, tilt=3.0, lean=85.0))
    assert r.label == "tipped_over"


def test_uncertain_when_borderline():
    # neredeyse kup, egim orta -> net esik yok
    r = classify(_metric(40, 42, 39, tilt=2.0, lean=20.0))
    assert r.label == "uncertain"
    assert r.needs_robot is False


def test_confidence_range():
    for m in (_metric(40, 70, 25, lean=2), _metric(70, 25, 40, lean=80)):
        c = classify(m).confidence
        assert 0.0 <= c <= 1.0

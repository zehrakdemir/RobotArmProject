"""Zemin kaldirma + nesne ayiklama - numpy (scipy varsa daha iyi)."""
import numpy as np

from src.segment import fit_plane_ransac, isolate_object, remove_ground


def _floor_plus_box(seed=0):
    rng = np.random.default_rng(seed)
    # zemin: genis, UZAK (z buyuk), ~1000 nokta
    fx = rng.uniform(-50, 50, 1400); fy = rng.uniform(-50, 50, 1400)
    fz = 40 + rng.normal(0, 0.3, 1400)
    floor = np.c_[fx, fy, fz]
    # kutu: kucuk, YAKIN (z kucuk), ~400 nokta
    bx = rng.uniform(-8, 8, 500); by = rng.uniform(-6, 6, 500)
    bz = 28 + rng.normal(0, 0.3, 500)
    box = np.c_[bx, by, bz]
    return np.vstack([floor, box]), len(floor), len(box)


def test_plane_ransac_finds_floor():
    pts, nf, nb = _floor_plus_box()
    nrm, d, inl = fit_plane_ransac(pts, thresh=1.5)
    assert inl.sum() >= 0.7 * nf          # zemin noktalarinin cogu
    assert inl[:nf].mean() > 0.9          # zemin bloklari inlier
    assert inl[nf:].mean() < 0.2          # kutu degil


def test_remove_ground_keeps_box():
    pts, nf, nb = _floor_plus_box()
    keep = remove_ground(pts, thresh_frac=0.02)
    assert keep[nf:].mean() > 0.8         # kutunun cogu kaldi
    assert keep[:nf].mean() < 0.2         # zeminin cogu gitti


def test_isolate_object_pipeline():
    pts, nf, nb = _floor_plus_box()
    cols = np.zeros((len(pts), 3), np.uint8)
    out, oc, info = isolate_object(pts, cols, drop_ground=True, denoise=True,
                                   keep_largest=True)
    assert 0.6 * nb <= len(out) <= nb * 1.4
    assert oc.shape[0] == len(out)
    assert info["giris"] == len(pts) and info["cikis"] == len(out)
    # kalan noktalarin merkezi kutu bolgesinde (z ~ 28)
    assert 26 < np.median(out[:, 2]) < 30


def test_isolate_no_ground_when_object_dominates():
    # tek duz nesne, zemin yok -> hicbir sey silinmemeli
    rng = np.random.default_rng(1)
    pts = np.c_[rng.uniform(-10, 10, 800), rng.uniform(-10, 10, 800),
               20 + rng.normal(0, 0.2, 800)]
    keep = remove_ground(pts, 0.02)
    assert keep.mean() > 0.9

"""
Mutlak metrik olcek kalibrasyonu.

Monokuler iki goruntuden yalnizca baseline YONU bilinir, BUYUKLUGU degil.
Bu yuzden tum 3B olculer bir "baseline birimi" cinsindendir. Kullanici
valizin GERCEK bir olcusunu (varsayilan: genislik) santimetre cinsinden
girer; o eksenin baseline-birimi karsiligina bolerek olcek carpanini
buluruz:

    scale [cm / birim] = bilinen_olcu_cm / olculen_olcu_birim
"""
from __future__ import annotations

import logging

from . import config
from .measurement import RawMeasurement

logger = logging.getLogger(__name__)


class ScaleError(RuntimeError):
    pass


def compute_scale(raw: RawMeasurement, run_cfg: config.RunConfig) -> float:
    dim = (run_cfg.known_dimension or "width").lower()
    known_cm = float(run_cfg.known_width_cm)
    if known_cm <= 0:
        raise ScaleError("Bilinen gercek olcu > 0 olmali.")

    measured = {
        "width": raw.width_u,
        "en": raw.width_u,
        "height": raw.height_u,
        "boy": raw.height_u,
        "depth": raw.depth_u,
        "derinlik": raw.depth_u,
    }.get(dim)

    if measured is None:
        raise ScaleError(f"Bilinmeyen olcu ekseni: {dim!r} (width|height|depth)")
    if measured <= 1e-9:
        raise ScaleError(
            f"'{dim}' ekseni baseline biriminde olculemedi (deger ~0). "
            "Disparite kalitesi dusuk olabilir; fotograflari veya "
            "SGBM/WLS parametrelerini gozden gecirin."
        )

    scale = known_cm / measured
    logger.info(
        "Olcek: bilinen %s = %.1f cm | olculen = %.5f birim | scale = %.5f cm/birim",
        dim, known_cm, measured, scale,
    )
    return scale

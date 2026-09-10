"""Arayuz icin gorsel bindirimler (overlay)."""
from __future__ import annotations

import numpy as np

from .measurement import MetricMeasurement


def overlay_detection(image_bgr: np.ndarray, mask: np.ndarray, score: float,
                      weights: str) -> np.ndarray:
    import cv2

    out = image_bgr.copy()
    color = np.array([0, 200, 0], dtype=np.uint8)
    m = mask.astype(bool)
    out[m] = (0.55 * out[m] + 0.45 * color).astype(np.uint8)
    contours, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, contours, -1, (0, 255, 0), 2)
    ys, xs = np.where(m)
    if xs.size:
        cv2.putText(out, f"suitcase {score:.2f} [{weights}]",
                    (int(xs.min()), max(20, int(ys.min()) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return out


def overlay_measurement(image_bgr: np.ndarray, m: MetricMeasurement) -> np.ndarray:
    """Rektifiye sol goruntu uzerine: yonelimli kutu, 4 kose (x,y), tepe referansi."""
    import cv2

    out = image_bgr.copy()
    corners = m.rect.corners_px.astype(int)
    cv2.polylines(out, [corners.reshape(-1, 1, 2)], True, (0, 165, 255), 2)

    labels = ["TL", "TR", "BR", "BL"]
    for (x, y), lab in zip(corners, labels):
        cv2.circle(out, (int(x), int(y)), 5, (255, 0, 255), -1)
        cv2.putText(out, f"{lab} ({int(x)},{int(y)})", (int(x) + 6, int(y) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

    # tepe (kavrama) referansi - "x y koordinati valizin ustune"
    u, v = m.top_ref.pixel
    cv2.drawMarker(out, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
    cv2.putText(out, f"TEPE (x={u}, y={v})", (int(u) + 8, int(v) + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    txt = (f"en={m.width_cm:.1f}cm  boy={m.height_cm:.1f}cm  "
           f"derinlik={m.depth_cm:.1f}cm  yamukluk={m.rect.tilt_deg:.1f}deg")
    cv2.rectangle(out, (0, 0), (min(out.shape[1], 760), 30), (0, 0, 0), -1)
    cv2.putText(out, txt, (6, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return out

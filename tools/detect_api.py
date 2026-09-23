#!/usr/bin/env python3
"""
VIZYON API KOPRUSU (port 8502) - Huseyin'in movecobot-web "Pick & place
(camera)" ozelligi icin. Kendi tespit/kalibrasyon koduzumu (bu repo), onun
src/fairino_cobot/vizyon.py istemcisinin bekledigi sozlesmeyle disariya acar.

ROBOTA HICBIR SEKILDE DOKUNMAZ - sadece kamera + olcum/tespit. Robot hareketi
movecobot-web'in kendi tarafinda (ROS2/MoveIt) kalir.

Sozlesme (src/fairino_cobot/vizyon.py + webapp/static/app.js ile senkron -
degistirilirse orada da guncellenmeli):
  GET  /health  -> {"ok": bool, "handeye": bool}
  GET  /slots   -> {"slots": [[x,y,z,rx,ry,rz], ...]}       (config.PLACE_SLOTS)
  POST /detect  -> {"ok": bool, "err": str?, "objects": [
        {"dims_cm": [b,e,d], "graspable": bool, "reason": str, "posture": str,
         "volume_l": float,
         "box": {"center_mm": [x,y,z],
                 "size_mm": [uzun_yatay, kalinlik, dikey],   # OBB.extents sirasi
                 "yaw_deg": float},
         "grasp": {"point_mm": [x,y,z], "rpy_deg": [rx,ry,rz]} | null}, ...]}
  Tum konumlar ROBOT TABAN cercevesinde (mm) - hand-eye (base_T_cam.yaml)
  gecerli olmali, yoksa "handeye": false doner ama /detect yine de calisir
  (kamera cercevesinde donebilir, arayuz bunu ayrica uyarir).

Calistirma (host.docker.internal ile kameraya (cam_web.py, host, 8600)
ulasir - pnp_web/server.py ile ayni desen):
    docker compose exec -d valiz-olcum python -m tools.detect_api
Arayuz: http://localhost:8502 (movecobot-web VISION_API_URL varsayilani ile
otomatik eslesir - host.docker.internal:8502)
"""
from __future__ import annotations

import cv2
from fastapi import FastAPI

from src import config
from src.calib import calibration_status
from src.multi import detect_objects

from tools import pick_and_place as pnp

app = FastAPI(title="detect_api")


@app.get("/health")
def health() -> dict:
    st = calibration_status()
    return {"ok": True, "handeye": bool(st["base_T_cam"]["var"])}


@app.get("/slots")
def slots() -> dict:
    return {"slots": [list(s) for s in config.PLACE_SLOTS]}


def _nesne_sozlugu(obj) -> dict:
    """DetectedObject -> vizyon.py/app.js'in bekledigi sozluk. box.size_mm
    OBB.extents ile AYNI SIRADA (uzun-yatay, kalinlik/kapanma-yonu, dikey) -
    dims_cm'in aksine buyukten kucuge SIRALANMAMIS, eksenler korunuyor."""
    obb = obj.obb
    g = obj.grasp
    out = {
        "dims_cm": list(obj.dims_cm),
        "posture": obj.posture,
        "volume_l": round(float(obj.volume_l), 3),
        "graspable": bool(g and g.feasible),
        "reason": (g.reason if g else "plan yok"),
        "box": {
            "center_mm": [round(float(v), 1) for v in obb.center],
            "size_mm": [round(float(v), 1) for v in obb.extents],
            "yaw_deg": round(float(obb.yaw_deg), 1),
        },
        "grasp": None,
    }
    if g and g.feasible:
        # AYNI duzeltme pickcobot'ta da uygulaniyor (tools/pick_and_place.py) -
        # src/grasp.py'nin ham yaw'i ROS2/MoveIt'e dogrudan verilirse ~46
        # derece hatali cikiyor, kok nedeni bulunamadi, deneysel duzeltme.
        corrected_rz = g.yaw_deg + pnp.YAW_CORRECTION_DEG
        out["grasp"] = {
            "point_mm": [round(float(v), 1) for v in g.grasp_point],
            "rpy_deg": [g.rpy_deg[0], g.rpy_deg[1], round(corrected_rz, 1)],
        }
    return out


@app.post("/detect")
def detect() -> dict:
    try:
        left, right = pnp.capture_pair()
        imgL, imgR = cv2.imread(left), cv2.imread(right)
        if imgL is None or imgR is None:
            return {"ok": False, "err": "kamera goruntusu okunamadi"}
        objs = detect_objects(imgL, imgR, mode="nesne")
        return {"ok": True, "objects": [_nesne_sozlugu(o) for o in objs]}
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "err": str(e)}


def main() -> int:
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8502)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

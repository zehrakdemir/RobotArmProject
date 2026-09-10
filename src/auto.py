"""
OTOMATIK COK-NESNELI HUCRE - gercek zamanli dongu.

Akis (tur = bir nesne):
  1. cam_web'den cift kare al
  2. detect_objects -> masadaki tum kutular, HACIMCE sirali
  3. En buyuk KAVRANABILIR nesneyi sec
  4. build_pick_place -> FR3'e gonder (bir sonraki bos slota)
  5. tekrar (sahne degisti; kalan en buyuk)
  Nesne kalmayinca dur.

Gorus: base_T_cam (hand-eye) YOKSA pozlar kamera cercevesindedir -> kol
oynar ama nesnenin tam yerine gitmez. Gercek is icin once hand-eye.

CLI:  docker compose run --rm valiz-olcum python -m tools.auto_run
"""
from __future__ import annotations

import glob
import logging
import os
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import config
from .calib import BaseTransform, StereoRig
from .multi import DetectedObject, detect_objects, draw_overlay
from .robot import FairinoFR3, build_pick_place

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------- kamera
def grab_pair(timeout: float = 25.0):
    """Host cam_web'e ÇİFT KAYDET dedirt; JSON yanitindaki yollari oku.
    Doner (imgL, imgR, ad)."""
    import cv2
    import json as _json
    import urllib.request

    req = urllib.request.Request(config.CAM_WEB_URL.rstrip("/") + "/snap", method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        js = _json.loads(r.read().decode("utf-8", "replace"))
    if not js.get("ok"):
        raise RuntimeError(f"cam_web /snap: {js}")
    # cam_web Windows yollari doner ("C:/...\\left_..png"); basename'i SNAP_DIR ile birlestir
    lname = os.path.basename(js["left"].replace("\\", "/"))
    rname = os.path.basename(js["right"].replace("\\", "/"))
    lp = os.path.join(config.SNAP_DIR, lname)
    rp = os.path.join(config.SNAP_DIR, rname)
    for _ in range(20):
        if os.path.exists(lp) and os.path.exists(rp):
            L, R = cv2.imread(lp), cv2.imread(rp)
            if L is not None and R is not None:
                return L, R, lname
        time.sleep(0.3)
    raise RuntimeError(f"cam_web dosyalari SNAP_DIR'de yok: {lname}")


def grab_frames(n: int = 3):
    """Sabit sahneden n L/R cifti al (bulut birlestirme icin). Doner (frames, son_ad)."""
    n = max(1, int(n))
    frames = []
    name = ""
    for i in range(n):
        if i:
            time.sleep(1.1)                 # cam_web dosya adi saniye cozunurluklu
        L, R, name = grab_pair()
        frames.append((L, R))
    return frames, name


# ---------------------------------------------------------------- durum
@dataclass
class AutoState:
    running: bool = False
    placed: int = 0                       # yerlestirilen nesne sayisi
    last_cycle: dict = field(default_factory=dict)
    objects: list = field(default_factory=list)   # son taramadaki DetectedObject.as_dict
    overlay_png: bytes = b""                       # son tarama: cerceveli rektifiye SOL kare
    plan: list = field(default_factory=list)      # kuru mod: tum nesneler icin sira plani
    events: list = field(default_factory=list)
    done: bool = False
    error: str = ""

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.events.append(f"[{ts}] {msg}")
        self.events[:] = self.events[-200:]
        logger.info(msg)


# ---------------------------------------------------------------- orkestra
class AutoCell:
    def __init__(self, dry_run: bool = False, place_slots: Optional[list] = None):
        self.dry_run = dry_run
        self.slots = [list(s) for s in (place_slots or config.PLACE_SLOTS)]
        self.rig = StereoRig.load()
        try:
            self.base = BaseTransform.load()
            self.have_he = True
        except FileNotFoundError:
            self.base = BaseTransform.identity()
            self.have_he = False
        self.fr3 = FairinoFR3(enabled=not dry_run)
        self.state = AutoState()

    def _overlay(self, left_img, objs):
        try:
            import cv2
            img = draw_overlay(left_img, objs, rig=self.rig, pick_index=0)
            ok, buf = cv2.imencode(".png", cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            if ok:
                self.state.overlay_png = buf.tobytes()
        except Exception as exc:  # noqa: BLE001
            logger.debug("overlay olusmadi: %s", exc)

    # -- tek tur --
    def step(self) -> dict:
        s = self.state
        if s.placed >= len(self.slots):
            s.done = True
            s.log(f"Tum slotlar dolu ({len(self.slots)}). Bitti.")
            return {"done": True}

        try:
            frames, nm = grab_frames(config.AUTO_MERGE_FRAMES)
        except Exception as exc:  # noqa: BLE001
            s.error = str(exc); s.log(f"KAMERA HATASI: {exc}")
            return {"error": s.error}
        objs = detect_objects(None, None, rig=self.rig, base_T_cam=self.base,
                              max_objects=config.AUTO_MAX_OBJECTS, frames=frames)
        s.objects = [o.as_dict() for o in objs]
        self._overlay(frames[-1][0], objs)
        if not objs:
            s.log(f"{nm}: masada nesne yok - bekleniyor.")
            return {"empty": True, "objects": []}

        graspable = [o for o in objs if o.grasp and o.grasp.feasible]
        s.log(f"{nm}: {len(objs)} nesne "
              f"(boylar: {', '.join(f'{o.dims_cm[0]}' for o in objs)} cm) "
              f"- {len(graspable)} kavranabilir.")
        if not graspable:
            s.log("Kavranabilir nesne yok (kalinlik/parmak/duz-yuz). Duruyor.")
            return {"blocked": True, "objects": s.objects}

        target = graspable[0]                       # EN BUYUK kavranabilir
        slot = self.slots[s.placed]
        prog = build_pick_place(target.grasp, place_pose=slot)
        s.log(f"-> SIRA {s.placed+1}: en buyuk {target.dims_cm} cm "
              f"({target.volume_l:.1f} L) -> slot {s.placed+1} {slot[:3]}")

        if self.dry_run or not self.fr3.enabled:
            for k, a, note in prog.steps:
                s.log(f"   [DRY] {note}")
            out = {"dry_run": True, "steps": len(prog.steps)}
        else:
            if not self.fr3._connected:
                self.fr3.connect()
            out = self.fr3.run_program(prog)
            s.log(f"   FR3: dispatched={out.get('dispatched')} "
                  f"kol_oynadi={out.get('kol_oynadi')} hata={out.get('error') or '-'}")
            if out.get("error"):
                s.error = out["error"]
                return {"robot_error": out["error"], "cycle": out}

        s.placed += 1
        s.last_cycle = {"nesne": target.as_dict(), "slot": slot, "cikti": out}
        return {"placed": s.placed, "cycle": s.last_cycle, "objects": s.objects}

    # -- KURU: tek tarama, tum nesneler icin sira planla (robota gitmez) --
    def plan_all(self) -> list[dict]:
        s = self.state
        s.running = True
        try:
            frames, nm = grab_frames(config.AUTO_MERGE_FRAMES)
        except Exception as exc:  # noqa: BLE001
            s.error = str(exc); s.log(f"KAMERA HATASI: {exc}"); s.running = False
            return []
        objs = detect_objects(None, None, rig=self.rig, base_T_cam=self.base,
                              max_objects=config.AUTO_MAX_OBJECTS, frames=frames)
        s.objects = [o.as_dict() for o in objs]
        self._overlay(frames[-1][0], objs)
        s.log(f"{nm}: {config.AUTO_MERGE_FRAMES} kare birlestirildi -> {len(objs)} nesne, "
              "buyukten kucuge sira:")
        plan = []
        for i, o in enumerate(objs):
            if not (o.grasp and o.grasp.feasible):
                s.log(f"   {i+1}. {o.dims_cm} cm — ATLA ({o.grasp.reason if o.grasp else '-'})")
                continue
            slot = self.slots[len(plan) % len(self.slots)]
            prog = build_pick_place(o.grasp, place_pose=slot)
            s.log(f"   {i+1}. {o.dims_cm} cm ({o.volume_l:.1f} L) -> slot {len(plan)+1} {slot[:3]} "
                  f"[{len(prog.steps)} adim]")
            plan.append({"nesne": o.as_dict(), "slot": slot, "program": prog.to_dict()})
        s.log(f"PLAN HAZIR: {len(plan)} nesne yerlestirilecek (kuru — robota gitmedi).")
        s.plan = plan
        s.running = False
        s.done = True
        return plan

    # -- GERCEK: surekli dongu (robot fiziksel nesneyi kaldirir, sahne degisir) --
    def run(self, stop: Optional[Callable[[], bool]] = None,
            on_update: Optional[Callable[[AutoState], None]] = None, max_cycles: int = 50):
        stop = stop or (lambda: False)
        if self.dry_run or not self.fr3.enabled:
            self.plan_all()                         # kuru = tek tarama + tam plan
            return
        self.state.running = True
        self.state.log(f"BASLADI (hand-eye={'var' if self.have_he else 'YOK'}, "
                       f"slot={len(self.slots)})")
        if not self.have_he:
            self.state.log("UYARI: base_T_cam yok - pozlar KAMERA cercevesinde. "
                           "Kol oynar ama nesne yerine gitmez. tools/handeye_solve.py")
        empties = 0
        try:
            for _ in range(max_cycles):
                if stop():
                    self.state.log("DURDURULDU."); break
                r = self.step()
                if on_update:
                    on_update(self.state)
                if r.get("done"):
                    break
                if r.get("error") or r.get("robot_error"):
                    self.state.log("Hata nedeniyle durdu."); break
                if r.get("empty"):
                    empties += 1
                    if empties >= 5:
                        self.state.log("5 tur ust uste bos - masa temiz kabul edildi. Bitti.")
                        self.state.done = True; break
                    time.sleep(max(config.AUTO_LOOP_PAUSE_S, 2.0))
                elif r.get("blocked"):
                    break
                else:
                    empties = 0
                    time.sleep(config.AUTO_LOOP_PAUSE_S)
        finally:
            self.state.running = False
            self.state.log(f"BITTI. Yerlestirilen: {self.state.placed}")

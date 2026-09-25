#!/usr/bin/env python3
"""
PICKCOBOT web arayuzu: gorme (bu repo) + ROS2/MoveIt kol kontrolu
(movecobot-web) tek bir sayfada. tools/pick_and_place.py'nin ayni guvenli
adimlarini (kucuk jog'lar, hata kontrolu, yonelim-secimi) HTTP endpoint'leri
olarak sunar - kullanici her fazi (yaklas / in / kavra / kaldir / tasi /
birak) kendi onayiyla, tek tikla tetikler; hicbir adim otomatik zincirlenmez.

Calistirma (konteyner icinde, host.docker.internal ile kameraya ve ROS2
webapp'e ulasir):
    docker compose exec -d valiz-olcum python -m tools.pnp_web.server
Arayuz: http://localhost:8090
"""
from __future__ import annotations

import io
import json
import time
import traceback
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, Response
from pydantic import BaseModel

from src import config
from src.calib import calibration_status
from src.cell import run_cell
from src.multi import detect_objects, draw_overlay

from tools import pick_and_place as pnp

app = FastAPI(title="pickcobot")

# ---------------------------------------------------------------------------
# Global oturum durumu (tek kullanicili yerel arac - basit tutuluyor)
#
# Olcum/plan bilgisi (S.plan, S.used_az, S.place_xyz/rpy) ayrica bind-mount'lu
# snaps/ klasorune JSON olarak yazilir - bu surec (docker cp + restart ile
# kod guncellemesi, ya da container'in kendisi) yeniden baslatilinca "once Olc"
# adimini tekrarlamaya gerek kalmaz (2026-09'da defalarca yasandi).
# ---------------------------------------------------------------------------
_SESSION_PATH = Path(getattr(config, "SNAP_DIR", "snaps")) / "pnp_session.json"
_PERSIST_FIELDS = ("plan", "left_path", "right_path", "used_az", "place_xyz", "place_rpy")


class Session:
    def __init__(self) -> None:
        self.plan: Optional[dict] = None
        self.left_path: Optional[str] = None
        self.right_path: Optional[str] = None
        self.used_az: Optional[float] = None
        self.overlay_png: Optional[bytes] = None
        self.objects: list = []   # son detect_objects() ciktisi - kalici DEGIL,
                                   # sadece "birden fazla nesne varsa hangisini
                                   # sec" akisi icin bellekte tutuluyor.
        self.log: list[str] = []
        self.place_xyz: tuple[float, float, float] = (-300.0, 100.0, 150.0)
        self.place_rpy: tuple[float, float, float] = (180.0, 0.0, 0.0)
        # True ise poz-tabanli adimlar (yaklas/birakma noktasi - onizlemesi
        # OLAN adimlar) sadece planlayip durur, movecobot'un KENDI arayuzunden
        # (ayni sunucu-tarafi onizleme orada da gorunur) elle onaylanmasi
        # beklenir. Jog-tabanli adimlarda (in/kaldir/vs.) onizleme
        # movecobot'ta hic yok, bu bayraktan etkilenmezler.
        self.manual_confirm: bool = False

    def add(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.append(f"[{ts}] {msg}")
        self.log = self.log[-200:]

    def save(self) -> None:
        try:
            _SESSION_PATH.parent.mkdir(parents=True, exist_ok=True)
            data = {k: getattr(self, k) for k in _PERSIST_FIELDS}
            _SESSION_PATH.write_text(json.dumps(data), encoding="utf-8")
        except Exception:  # noqa: BLE001
            pass  # kalicilik bir kolayliktir - basarisiz olursa oturum yine calisir

    @classmethod
    def load(cls) -> "Session":
        s = cls()
        try:
            data = json.loads(_SESSION_PATH.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            return s
        if data.get("place_xyz"):
            data["place_xyz"] = tuple(data["place_xyz"])
        if data.get("place_rpy"):
            data["place_rpy"] = tuple(data["place_rpy"])
        for k in _PERSIST_FIELDS:
            if k in data:
                setattr(s, k, data[k])
        return s


S = Session.load()


def _result(ok: bool, msg: str, **extra: Any) -> dict:
    S.add(msg)
    out = {"ok": ok, "message": msg, "log": S.log[-30:]}
    out.update(extra)
    return out


def _plan_from_grasp(g, obb) -> dict:
    corrected_rz = g.yaw_deg + pnp.YAW_CORRECTION_DEG
    # Arayüzde her zaman uzunluk sırasıyla göster: boy >= en >= derinlik.
    dims_mm = sorted((float(obb.width_mm), float(obb.height_mm),
                      float(obb.thickness_mm)), reverse=True)
    return {
        "approach": g.approach_point.tolist(),
        "grasp": g.grasp_point.tolist(),
        "lift": g.lift_point.tolist(),
        "rz": corrected_rz,
        "raw_yaw": g.yaw_deg,
        "opening_mm": g.opening_mm,
        "height_mm": float(obb.height_mm),
        "grasp_z": float(g.grasp_point[2]),
        "dimensions_mm": dims_mm,
    }


def _redraw_overlay(pick_index: int) -> None:
    """S.left_path'ten yeniden okuyup S.objects'i, secili nesne (pick_index,
    DetectedObject.rank ile ayni 0-tabanli) yesil vurgulanacak sekilde ciz."""
    if not S.left_path or not S.objects:
        return
    try:
        imgL = cv2.imread(S.left_path)
        overlay = draw_overlay(imgL, S.objects, pick_index=pick_index)
        ok_enc, buf = cv2.imencode(".png", overlay)
        S.overlay_png = buf.tobytes() if ok_enc else None
    except Exception:  # noqa: BLE001
        pass


def _objects_summary() -> list[dict]:
    return [{"num": o.rank + 1, "dims_cm": o.dims_cm, "hacim_L": round(o.volume_l, 2),
             "kavranabilir": bool(o.grasp and o.grasp.feasible)} for o in S.objects]


# plan_grasp()'in varsayilani (GRASP_HEIGHT_FRACTION=0.5) cismin ORTASINDAN
# kavrar. pick_and_place.py CLI aracinda, 2026-09-18'de bir carpismadan sonra
# tepeye yakin kavramanin (yukseklik-10mm) daha guvenli oldugu kanitlanmisti;
# 2026-09-23'te ince bir cisimle bu tekrar dogrulandi (45mm elle zorlanip
# calisti, guvenlik tabani 40mm'nin hemen ustunde). pickcobot'un otomatik
# yolu da varsayilan olarak bunu kullanir - GRASP_HEIGHT_FRACTION Streamlit
# tarafinda hala 0.5 kalir, sadece burada (ROS2/ pickcobot) degistiriliyor.
GRASP_TOP_OFFSET_MM = 10.0
# ROS2/MoveIt tarafinda TCP'nin masaya ~40mm'den fazla yaklasmasini engelleyen
# sabit bir guvenlik marji var (movecobot.py, gripper+carpisma kutusu). Kisa
# nesnelerde "yukseklik-10mm" bu tabanin altina dusup GOAL_STATE_INVALID
# hatasi veriyordu (2026-09-23, tekrarlayan canli olay) - taban burada
# zorlanarak onleniyor.
GRASP_Z_FLOOR_MM = 42.0


def _default_grasp_z(obb) -> float:
    return max(GRASP_Z_FLOOR_MM, float(obb.height_mm) - GRASP_TOP_OFFSET_MM)


def _apply_grasp_z_override(g, obb, grasp_z_mm: float):
    """Kullanicinin elle girdigi kavrama yuksekligini (tabandan mm) mevcut plana
    uygular - approach/lift noktalarini plan_grasp() ile AYNI formulle (bkz.
    src/grasp.py:plan_grasp) yeniden hesaplar, boylece nokta bulutunu tekrar
    islemeye gerek kalmaz."""
    ax_idx = {"x": 0, "y": 1, "z": 2}[config.UP_AXIS.lower()]
    up = obb.up_axis
    new_gp = g.grasp_point.copy()
    new_gp[ax_idx] = obb.ground_z + float(grasp_z_mm)
    g.grasp_point = new_gp
    g.approach_point = new_gp - g.approach_axis * config.GRASP_APPROACH_MM
    g.lift_point = new_gp + up * config.GRASP_LIFT_MM
    depth_from_top = obb.height_mm - float(grasp_z_mm)
    if depth_from_top > config.GRIPPER_FINGER_LEN_MM:
        g.reason += (f" UYARI: parmak boyu {config.GRIPPER_FINGER_LEN_MM:.0f}mm, "
                     f"tepeden kavrama derinligi {depth_from_top:.0f}mm - kisa kalabilir.")
    return g


def _safe(fn, *a, **kw) -> dict:
    try:
        r = fn(*a, **kw)
        return r if isinstance(r, dict) else {"ok": True, "result": r}
    except Exception as e:  # noqa: BLE001
        tb = traceback.format_exc(limit=6)
        return _result(False, f"HATA: {e}", traceback=tb)


# ---------------------------------------------------------------------------
# Olcum
# ---------------------------------------------------------------------------
class MeasureBody(BaseModel):
    grasp_z_override: Optional[float] = None
    # Sol goruntudeki secim: x, y, genislik, yukseklik (tam kamera pikseli).
    # Verilirse YOLO/en-buyuk-kume yerine sadece bu alan olculur.
    roi_xywh: Optional[tuple[int, int, int, int]] = None


def _capture_into_session() -> tuple[np.ndarray, np.ndarray]:
    """Ayni L/R ciftini oturumda sakla; ROI secimi ekrandaki kareye ait olur."""
    left, right = pnp.capture_pair()
    S.left_path, S.right_path = left, right
    imgL, imgR = cv2.imread(left), cv2.imread(right)
    if imgL is None or imgR is None:
        raise RuntimeError("kamera goruntusu okunamadi")
    return imgL, imgR


@app.post("/api/capture")
def capture() -> dict:
    """Olcumden once bir stereo cifti al; kullanici sol karede ROI cizer."""
    def _do():
        _capture_into_session()
        return _result(True, "Kare alindi. Sigara paketini fareyle cerceveleyin.")
    return _safe(_do)


@app.post("/api/measure")
def measure(body: MeasureBody = MeasureBody()) -> dict:
    def _do():
        imgL, imgR = _capture_into_session()
        S.overlay_png = None

        roi = body.roi_xywh
        if roi is not None:
            x, y, w, h = (int(v) for v in roi)
            H, W = imgL.shape[:2]
            if w < 8 or h < 8 or x < 0 or y < 0 or x + w > W or y + h > H:
                return _result(False, "Gecersiz secim alani; paketi tamamen cevreleyin.")
            roi = (x, y, w, h)
            # ROI verildiyse eski (YOLO tabanli/tek nesne) akis kullanilir.
            probe = run_cell(imgL, imgR, posture_override="standing_on_wheels", roi_xywh=roi)
            if probe.obb is None:
                return _result(False, f"Olcum basarisiz: {probe.message}")
            height_mm = probe.obb.height_mm
            grasp_z = body.grasp_z_override
            if grasp_z is None:
                grasp_z = max(10.0, height_mm - 10.0)
            res = run_cell(imgL, imgR, posture_override="standing_on_wheels",
                            grasp_z_mm=grasp_z, roi_xywh=roi)
            if res.grasp is None or not res.grasp.feasible:
                return _result(False, f"Kavrama plani yok: {res.message}")
            g, obb = res.grasp, res.obb
            message = res.message
        else:
            # Otomatik: YOLO/ozel-sinif GEREKMEZ - masa duzlemi + 3B kumeleme
            # (src/multi.py DETECT_MODE=nesne). Herhangi bir duzgun kutu/paket
            # icin calisir; en buyuk kumeyi (rank 0) alir.
            objs = detect_objects(imgL, imgR, mode="nesne")
            if not objs:
                return _result(False, "Olcum basarisiz: masada nesne bulunamadi.")
            S.objects = objs
            obj = objs[0]
            g, obb = obj.grasp, obj.obb
            # Hangi bolgeyi/nesneyi olctugunu GORMEDEN guvenmeyin - yanlis nesne
            # (orn. karton/masa) olcumu bu overlay olmadan fark edilmesi gunler
            # surmustu (2026-09 oturumu). Kontur cizimi basarisiz olsa bile
            # olcumu engellemesin diye ayri try/except.
            _redraw_overlay(pick_index=0)
            if g is None or not g.feasible:
                return _result(False, f"Kavrama plani yok: {obj.as_dict().get('kavrama_sebep')}")
            z_target = body.grasp_z_override if body.grasp_z_override is not None else _default_grasp_z(obb)
            g = _apply_grasp_z_override(g, obb, z_target)
            message = (f"boy {obj.dims_cm[0]:.1f} · en {obj.dims_cm[1]:.1f} · "
                       f"derinlik {obj.dims_cm[2]:.1f} cm (yaw {obj.yaw_deg:.0f}°, "
                       f"{obj.n_points} nokta)")

        S.plan = _plan_from_grasp(g, obb)
        S.used_az = None
        S.save()
        selected = "secili alan" if roi is not None else "otomatik (nesne modu)"
        return _result(True, f"Olcum tamam ({selected}): {message}", plan=S.plan,
                       has_overlay=S.overlay_png is not None, objects=_objects_summary())

    return _safe(_do)


class SelectObjectBody(BaseModel):
    num: int   # gorseldeki "#N" etiketiyle ayni (1-tabanli)
    grasp_z_override: Optional[float] = None


@app.post("/api/select_object")
def select_object(body: SelectObjectBody) -> dict:
    """Otomatik tespit birden fazla nesne bulduysa (orn. masadaki baska bir
    esya/kalibrasyon tahtasi yanlislikla 'en buyuk nesne' sayilirsa), kullanici
    overlay'deki numaraya bakip DOGRU nesneyi burada elle secer - olcumu
    TEKRARLAMADAN (ayni fotograf/tespit uzerinde) plani gunceller."""
    def _do():
        if not S.objects:
            return _result(False, "Once olcum yapin.")
        matches = [o for o in S.objects if o.rank == body.num - 1]
        if not matches:
            return _result(False, f"Nesne #{body.num} bulunamadi (toplam {len(S.objects)} nesne).")
        obj = matches[0]
        g, obb = obj.grasp, obj.obb
        if g is None or not g.feasible:
            return _result(False, f"Nesne #{body.num} icin kavrama plani yok: "
                                   f"{obj.as_dict().get('kavrama_sebep')}")
        z_target = body.grasp_z_override if body.grasp_z_override is not None else _default_grasp_z(obb)
        g = _apply_grasp_z_override(g, obb, z_target)
        _redraw_overlay(pick_index=obj.rank)
        S.plan = _plan_from_grasp(g, obb)
        S.used_az = None
        S.save()
        message = (f"boy {obj.dims_cm[0]:.1f} · en {obj.dims_cm[1]:.1f} · "
                   f"derinlik {obj.dims_cm[2]:.1f} cm (yaw {obj.yaw_deg:.0f}°)")
        return _result(True, f"Nesne #{body.num} secildi: {message}", plan=S.plan,
                       has_overlay=S.overlay_png is not None, objects=_objects_summary())

    return _safe(_do)


@app.get("/api/session")
def session_state() -> dict:
    """Sayfa yeniden acildiginda (ya da sunucu yeniden baslatilip S.plan diskten
    geri yuklendiginde) onceki olcumu goster - "once Olc" tekrarini gerektirmesin."""
    return {"ok": True, "plan": S.plan, "has_overlay": S.overlay_png is not None,
            "has_photo": bool(S.left_path), "objects": _objects_summary()}


@app.get("/api/photo/{side}")
def photo(side: str) -> Response:
    if side == "overlay":
        if S.overlay_png is None:
            return Response(status_code=404)
        return Response(content=S.overlay_png, media_type="image/png")
    path = S.left_path if side == "left" else S.right_path
    if not path or not Path(path).exists():
        return Response(status_code=404)
    data = Path(path).read_bytes()
    return Response(content=data, media_type="image/png")


# ---------------------------------------------------------------------------
# Kalibrasyon durumu - "olcum neden garip cikti" sorusuna ilk bakilacak yer.
# ---------------------------------------------------------------------------
@app.get("/api/calib_status")
def calib_status() -> dict:
    try:
        st = calibration_status()
        st["measure_scale"] = float(getattr(config, "MEASURE_SCALE", 1.0))
        st["yaw_correction_deg"] = pnp.YAW_CORRECTION_DEG
        return {"ok": True, "result": st}
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "result": None, "error": str(e)}


# ---------------------------------------------------------------------------
# Robot durumu (proxy)
# ---------------------------------------------------------------------------
@app.get("/api/robot_state")
def robot_state() -> dict:
    try:
        st = pnp._get("/api/state")
        return {"ok": True, "result": st}
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "result": None, "error": str(e)}


class ManualConfirmBody(BaseModel):
    enabled: bool


@app.post("/api/manual_confirm")
def set_manual_confirm(body: ManualConfirmBody) -> dict:
    S.manual_confirm = body.enabled
    return {"ok": True, "manual_confirm": S.manual_confirm}


class SpeedBody(BaseModel):
    speed_pct: float
    accel_pct: float


@app.post("/api/speed")
def set_speed(body: SpeedBody) -> dict:
    def _do():
        r = pnp._post("/api/settings", {"speed_pct": body.speed_pct, "accel_pct": body.accel_pct})
        return _result(True, f"Hiz %{r.get('speed_pct')} / Ivme %{r.get('accel_pct')} olarak ayarlandi.",
                       settings=r)
    return _safe(_do)


# ---------------------------------------------------------------------------
# Adimlar - her biri kullanicinin tek tikiyla tetiklenir, hicbiri otomatik
# zincirlenmez. Her adim kendi ici guvenli (kucuk jog / hata kontrolu / az
# eklem hareketi secimi) ama BASKA ADIMI cagirmaz.
# ---------------------------------------------------------------------------
@app.post("/api/step/gripper_activate")
def step_gripper_activate() -> dict:
    def _do():
        pnp.ensure_gripper_active(False)
        return _result(True, "Gripper reaktive edildi ve acildi.")

    return _safe(_do)


@app.post("/api/step/approach")
def step_approach() -> dict:
    def _do():
        if S.plan is None:
            return _result(False, "Once olcum yapin.")
        ax, ay, az = S.plan["approach"]
        gx, gy, gz = S.plan["grasp"]
        rz = S.plan["rz"]
        used_az = None
        for candidate_az in np.arange(az, gz + 29.0, -20.0):
            candidate_az = float(candidate_az)
            ok = pnp.move_pose(ax, ay, candidate_az, 180.0, 0.0, rz, False,
                                prefer_lin=False, label=f"yaklasma (z={candidate_az:.0f})",
                                auto_run=not S.manual_confirm)
            if ok:
                used_az = candidate_az
                break
        if used_az is None:
            return _result(False, "Yaklasma noktasina hicbir yukseklikte ulasilamadi.")
        S.used_az = used_az
        S.save()
        if S.manual_confirm:
            return _result(True, f"Yaklasma planlandi (z={used_az:.1f}) - movecobot'tan "
                                  f"onaylayip calistirin.")
        return _result(True, f"Yaklasma tamam (z={used_az:.1f}).")

    return _safe(_do)


@app.post("/api/step/descend")
def step_descend() -> dict:
    def _do():
        if S.plan is None or S.used_az is None:
            return _result(False, "Once yaklasma adimini tamamlayin.")
        gz = S.plan["grasp"][2]
        pnp.jog_dz(gz - S.used_az, False, label="kavramaya inis", max_step=500.0)
        return _result(True, f"Kavrama yuksekligine inildi (z={gz:.1f}).")

    return _safe(_do)


@app.post("/api/step/grip")
def step_grip() -> dict:
    return _safe(lambda: _result(True, "Kavrama denendi.", **pnp.gripper_grip(False)))


@app.post("/api/step/lift")
def step_lift() -> dict:
    def _do():
        if S.plan is None:
            return _result(False, "Plan yok.")
        gz = S.plan["grasp"][2]
        lz = S.plan["lift"][2]
        pnp.jog_dz(lz - gz, False, label="kaldirma", max_step=500.0)
        return _result(True, f"Kaldirildi (z={lz:.1f}).")

    return _safe(_do)


class PlaceBody(BaseModel):
    x: float
    y: float
    z: float
    rx: float = 180.0
    ry: float = 0.0
    rz: float = 0.0


@app.post("/api/step/goto_place")
def step_goto_place(body: PlaceBody) -> dict:
    def _do():
        S.place_xyz = (body.x, body.y, body.z)
        S.place_rpy = (body.rx, body.ry, body.rz)
        S.save()
        ok = pnp.move_pose(body.x, body.y, body.z, body.rx, body.ry, body.rz,
                            False, prefer_lin=False, label="birakma noktasi",
                            auto_run=not S.manual_confirm)
        if not ok:
            return _result(False, "Birakma noktasina ulasilamadi (tekillik/IK).")
        if S.manual_confirm:
            return _result(True, "Birakma noktasi planlandi - movecobot'tan onaylayip calistirin.")
        return _result(True, "Birakma noktasina ulasildi.")

    return _safe(_do)


class LowerBody(BaseModel):
    drop_mm: float = 60.0


class AutoLowerBody(BaseModel):
    # drop_mm verilirse eskisi gibi SABIT bir mesafe iner. Verilmezse (varsayilan),
    # kavrama yuksekligine (S.plan["grasp_z"]) + margin_mm kadar YUKARIDA duracak
    # sekilde otomatik hesaplanir - "birakmaya in cok yukarida duruyordu, kavradigi
    # yuksekligin biraz ustunde dursun" (2026-09-23 canli geri bildirimi).
    drop_mm: Optional[float] = None
    margin_mm: float = 1.0


@app.post("/api/step/lower")
def step_lower(body: AutoLowerBody = AutoLowerBody()) -> dict:
    def _do():
        if body.drop_mm is not None:
            drop = body.drop_mm
            target_msg = f"sabit {drop:.1f}mm"
        else:
            if S.plan is None:
                return _result(False, "Once olcum yapin (kavrama yuksekligi bilinmiyor).")
            st = pnp._get("/api/state")
            cur_z = st["tcp_pose"]["z"]
            target_z = S.plan["grasp_z"] + body.margin_mm
            drop = max(0.0, cur_z - target_z)
            target_msg = f"z={target_z:.1f} (kavrama+{body.margin_mm:.1f})"
        pnp.jog_dz(-drop, False, label="birakmaya inis", max_step=500.0)
        return _result(True, f"Birakma yuksekligine inildi ({target_msg}).")

    return _safe(_do)


@app.post("/api/step/release")
def step_release() -> dict:
    def _do():
        pnp.gripper_release(False)
        return _result(True, "Gripper acildi.")

    return _safe(_do)


@app.post("/api/step/retreat")
def step_retreat(body: LowerBody = LowerBody()) -> dict:
    def _do():
        pnp.jog_dz(body.drop_mm, False, label="geri cekilme", max_step=500.0)
        return _result(True, "Geri cekildi.")

    return _safe(_do)


@app.post("/api/step/neutral")
def step_neutral() -> dict:
    def _do():
        pnp.move_joints([0.0, -90.0, 90.0, -90.0, -90.0, 0.0], False)
        return _result(True, "Notr pozisyona donuldu.")

    return _safe(_do)


@app.post("/api/run_all")
def run_all(body: PlaceBody) -> dict:
    """Tum akisi TEK cagriyla, sirayla calistirir - herhangi bir adim basarisiz
    olursa (ok=False) HEMEN durur, sonrakileri calistirmaz. Olcum onceden
    yapilmis olmali (S.plan dolu)."""
    sequence = [
        ("gripper_activate", lambda: step_gripper_activate()),
        ("approach", lambda: step_approach()),
        ("descend", lambda: step_descend()),
        ("grip", lambda: step_grip()),
        ("lift", lambda: step_lift()),
        ("goto_place", lambda: step_goto_place(body)),
        ("lower", lambda: step_lower(AutoLowerBody())),
        ("release", lambda: step_release()),
        ("retreat", lambda: step_retreat(LowerBody())),
        ("neutral", lambda: step_neutral()),
    ]
    results = {}
    for name, fn in sequence:
        r = fn()
        results[name] = r
        if not r.get("ok"):
            return _result(False, f"'{name}' adiminda durdu: {r.get('message')}",
                            steps=results)
    return _result(True, "Tum akis basariyla tamamlandi.", steps=results)


# ---------------------------------------------------------------------------
# Tek sayfa arayuz
# ---------------------------------------------------------------------------
_PAGE = """<!doctype html>
<html lang="tr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>pickcobot</title>
<style>
  :root {
    --bg: #0f1115; --panel: #171a21; --border: #262b36; --text: #e6e8ec;
    --muted: #8b93a3; --accent: #4c8dff; --ok: #3ecf8e; --err: #ef5a5a;
    --warn: #e0a940;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--text);
    font-family: -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    font-size: 14px;
  }
  header {
    padding: 16px 24px; border-bottom: 1px solid var(--border);
    display: flex; align-items: center; justify-content: space-between;
  }
  header h1 { font-size: 16px; font-weight: 600; margin: 0; letter-spacing: 0.2px; }
  #robot-badge { font-size: 12px; color: var(--muted); }
  #robot-badge.ok { color: var(--ok); }
  #robot-badge.err { color: var(--err); }
  main {
    display: grid; grid-template-columns: 360px 1fr 340px; gap: 0;
    height: calc(100vh - 53px);
  }
  section { padding: 20px; overflow-y: auto; }
  section + section { border-left: 1px solid var(--border); }
  h2 {
    font-size: 12px; text-transform: uppercase; letter-spacing: 0.6px;
    color: var(--muted); font-weight: 600; margin: 0 0 12px;
  }
  .card {
    background: var(--panel); border: 1px solid var(--border); border-radius: 8px;
    padding: 14px; margin-bottom: 14px;
  }
  button {
    background: var(--panel); color: var(--text); border: 1px solid var(--border);
    border-radius: 6px; padding: 9px 14px; font-size: 13px; cursor: pointer;
    width: 100%; margin-bottom: 8px; text-align: left; font-family: inherit;
    transition: border-color 0.15s, background 0.15s;
  }
  button:hover:not(:disabled) { border-color: var(--accent); }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
  button.danger { border-color: var(--err); color: var(--err); }
  .row { display: flex; gap: 8px; }
  .row button { margin-bottom: 0; }
  .snap-wrap { position: relative; width: 100%; display: none; margin-bottom: 8px; }
  img.snap { display: block; width: 100%; border-radius: 6px; border: 1px solid var(--border); }
  table { width: 100%; border-collapse: collapse; font-size: 12px; }
  td { padding: 4px 0; color: var(--muted); }
  td.val { color: var(--text); text-align: right; font-variant-numeric: tabular-nums; }
  #log {
    font-family: ui-monospace, "SF Mono", Consolas, monospace; font-size: 11.5px;
    white-space: pre-wrap; line-height: 1.5; color: var(--muted);
  }
  #log .err { color: var(--err); }
  input[type=number] {
    background: var(--bg); border: 1px solid var(--border); color: var(--text);
    border-radius: 5px; padding: 6px 8px; width: 100%; font-family: inherit; font-size: 12px;
  }
  label { font-size: 11px; color: var(--muted); display: block; margin-bottom: 3px; }
  .field { margin-bottom: 8px; }
  .status-line { font-size: 12px; color: var(--muted); margin-bottom: 10px; min-height: 16px; }
  .status-line.ok { color: var(--ok); }
  .status-line.err { color: var(--err); }
</style>
</head>
<body>
<header>
  <h1>pickcobot</h1>
  <div style="display:flex;align-items:center;gap:14px">
    <span id="robot-badge">robot: -</span>
    <div style="display:flex;align-items:center;gap:6px">
      <label style="margin:0">Hiz %</label>
      <input type="number" id="speed-pct" value="10" min="1" max="100" style="width:60px">
      <label style="margin:0">Ivme %</label>
      <input type="number" id="accel-pct" value="10" min="1" max="100" style="width:60px">
      <button onclick="setSpeed()">Uygula</button>
    </div>
  </div>
</header>
<main>
  <section id="col-measure">
    <h2>Olcum</h2>
    <div class="card">
      <div class="field">
        <label>Kavrama yuksekligi (mm, tabandan - bos birakilirsa otomatik)</label>
        <input type="number" id="grasp-z-override" placeholder="otomatik">
      </div>
      <button class="primary" onclick="doMeasure()">Olc</button>
      <div class="status-line" id="measure-status"></div>
      <div class="snap-wrap" id="snap-wrap">
        <img class="snap" id="snap-left" alt="Sol kamera karesi">
      </div>
      <div id="object-select" style="display:none;margin-bottom:8px">
        <label>Birden fazla nesne bulundu - dogru olani secin:</label>
        <div id="object-buttons" style="display:flex;gap:6px;flex-wrap:wrap"></div>
      </div>
      <table id="measure-table"></table>
    </div>
  </section>

  <section id="col-steps">
    <h2>Adimlar</h2>
    <div class="card">
      <div class="field">
        <label style="display:flex;align-items:center;gap:6px;cursor:pointer">
          <input type="checkbox" id="manual-confirm" onchange="setManualConfirm()">
          Yaklas / Birakma noktasi: sadece planla, movecobot'tan onayla (otomatik calistirma)
        </label>
      </div>
      <div class="field">
        <label>Birakma X / Y / Z (mm)</label>
        <div class="row">
          <input type="number" id="place-x" value="-300">
          <input type="number" id="place-y" value="100">
          <input type="number" id="place-z" value="150">
        </div>
      </div>
      <div class="field">
        <label>Birakma RX / RY / RZ (derece)</label>
        <div class="row">
          <input type="number" id="place-rx" value="180">
          <input type="number" id="place-ry" value="0">
          <input type="number" id="place-rz" value="0">
        </div>
      </div>
      <button class="primary" onclick="runAll()">Tum akisi calistir (al -&gt; tasi -&gt; birak)</button>
    </div>
    <div class="status-line" id="step-status"></div>
    <h2>Tek tek (elle kontrol)</h2>
    <div class="card">
      <button onclick="step('gripper_activate')">1. Gripper ac</button>
      <button onclick="step('approach')">2. Yaklas</button>
      <button onclick="step('descend')">3. Kavrama yuksekligine in</button>
      <button onclick="step('grip')">4. Kavra</button>
      <button onclick="step('lift')">5. Kaldir</button>
      <button onclick="gotoPlace()">6. Birakma noktasina git</button>
      <button onclick="step('lower')">7. Birakmaya in</button>
      <button onclick="step('release')">8. Birak (gripper ac)</button>
      <button onclick="step('retreat')">9. Geri cekil</button>
      <button onclick="step('neutral')">10. Notre don</button>
    </div>
  </section>

  <section id="col-log">
    <h2>Kalibrasyon</h2>
    <div class="card">
      <table id="calib-table"></table>
    </div>
    <h2>Durum / Gunluk</h2>
    <div class="card" id="plan-card" style="display:none">
      <table id="plan-table"></table>
    </div>
    <div id="log"></div>
  </section>
</main>

<script>
async function post(url, body) {
  const r = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: body ? JSON.stringify(body) : undefined,
  });
  return r.json();
}
async function get(url) {
  const r = await fetch(url);
  return r.json();
}

async function setManualConfirm() {
  const enabled = document.getElementById("manual-confirm").checked;
  await post("/api/manual_confirm", {enabled: enabled});
}

async function setSpeed() {
  const speed = parseFloat(document.getElementById("speed-pct").value);
  const accel = parseFloat(document.getElementById("accel-pct").value);
  const r = await post("/api/speed", {speed_pct: speed, accel_pct: accel});
  renderLog(r.log || []);
}

function setStatus(id, ok, msg) {
  const el = document.getElementById(id);
  el.textContent = msg;
  el.className = "status-line " + (ok ? "ok" : "err");
}

function renderLog(lines) {
  const el = document.getElementById("log");
  el.innerHTML = lines.map(l => {
    const cls = /HATA|ok=False|basarisiz|erisilemedi/i.test(l) ? "err" : "";
    return `<div class="${cls}">${l.replace(/[<>]/g, c => c === '<' ? '&lt;' : '&gt;')}</div>`;
  }).join("");
  el.scrollTop = el.scrollHeight;
}

function renderPlan(p) {
  if (!p) return;
  document.getElementById("plan-card").style.display = "block";
  const rows = [
    ["boy (mm)", p.dimensions_mm?.[0]?.toFixed(1)],
    ["en (mm)", p.dimensions_mm?.[1]?.toFixed(1)],
    ["derinlik (mm)", p.dimensions_mm?.[2]?.toFixed(1)],
    ["yukseklik (mm)", p.height_mm?.toFixed(1)],
    ["kavrama z (mm)", p.grasp_z?.toFixed(1)],
    ["ham aci", p.raw_yaw?.toFixed(1)],
    ["duzeltilmis rz", p.rz?.toFixed(1)],
    ["acilma (mm)", p.opening_mm?.toFixed(1)],
    ["yaklasma", p.approach?.map(v => v.toFixed(1)).join(", ")],
    ["kavrama", p.grasp?.map(v => v.toFixed(1)).join(", ")],
    ["kaldirma", p.lift?.map(v => v.toFixed(1)).join(", ")],
  ];
  document.getElementById("plan-table").innerHTML =
    rows.map(([k, v]) => `<tr><td>${k}</td><td class="val">${v ?? "-"}</td></tr>`).join("");
}

function renderObjects(objects) {
  const wrap = document.getElementById("object-select");
  const el = document.getElementById("object-buttons");
  if (!objects || objects.length < 2) {
    wrap.style.display = "none";
    el.innerHTML = "";
    return;
  }
  wrap.style.display = "block";
  el.innerHTML = "";
  for (const o of objects) {
    const b = document.createElement("button");
    const tag = o.kavranabilir ? "" : " [kavranamaz]";
    b.textContent = "#" + o.num + " (" + o.dims_cm.join("x") + "cm)" + tag;
    b.onclick = () => selectObject(o.num);
    el.appendChild(b);
  }
}

async function selectObject(num) {
  setStatus("measure-status", true, "nesne #" + num + " seciliyor...");
  const r = await post("/api/select_object", {num: num});
  setStatus("measure-status", r.ok, r.message);
  renderLog(r.log || []);
  if (r.plan) renderPlan(r.plan);
  renderObjects(r.objects);
  const img = document.getElementById("snap-left");
  const side = r.has_overlay ? "overlay" : "left";
  img.src = "/api/photo/" + side + "?t=" + Date.now();
}

async function doMeasure() {
  setStatus("measure-status", true, "olculuyor...");
  const zRaw = document.getElementById("grasp-z-override").value;
  const body = zRaw === "" ? {} : {grasp_z_override: parseFloat(zRaw)};
  const r = await post("/api/measure", body);
  setStatus("measure-status", r.ok, r.message);
  renderLog(r.log || []);
  if (r.plan) renderPlan(r.plan);
  renderObjects(r.objects);
  const img = document.getElementById("snap-left");
  document.getElementById("snap-wrap").style.display = "block";
  const side = r.has_overlay ? "overlay" : "left";
  img.src = "/api/photo/" + side + "?t=" + Date.now();
}

async function step(name) {
  setStatus("step-status", true, name + " calisiyor...");
  const r = await post("/api/step/" + name, {});
  setStatus("step-status", r.ok, r.message);
  renderLog(r.log || []);
}

async function gotoPlace() {
  setStatus("step-status", true, "birakma noktasina gidiliyor...");
  const r = await post("/api/step/goto_place", placeBody());
  setStatus("step-status", r.ok, r.message);
  renderLog(r.log || []);
}

function placeBody() {
  return {
    x: parseFloat(document.getElementById("place-x").value),
    y: parseFloat(document.getElementById("place-y").value),
    z: parseFloat(document.getElementById("place-z").value),
    rx: parseFloat(document.getElementById("place-rx").value),
    ry: parseFloat(document.getElementById("place-ry").value),
    rz: parseFloat(document.getElementById("place-rz").value),
  };
}

async function runAll() {
  const b = placeBody();
  const confirmed = confirm(
    `Robot GERCEKTEN hareket edecek: al -> tasi -> birak.` + String.fromCharCode(10, 10) +
    `Birakma noktasi: x=${b.x} y=${b.y} z=${b.z}  rx=${b.rx} ry=${b.ry} rz=${b.rz}` + String.fromCharCode(10, 10) +
    `Devam edilsin mi?`
  );
  if (!confirmed) return;
  setStatus("step-status", true, "tum akis calisiyor...");
  const r = await post("/api/run_all", b);
  setStatus("step-status", r.ok, r.message);
  renderLog(r.log || []);
}

async function loadCalibStatus() {
  const el = document.getElementById("calib-table");
  try {
    const r = await get("/api/calib_status");
    if (!r.ok || !r.result) { el.innerHTML = "<tr><td>kalibrasyon durumu okunamadi</td></tr>"; return; }
    const s = r.result;
    const rows = [
      ["stereo kalibrasyon", s.stereo.var
        ? `var - RMS ${s.stereo.rms_px?.toFixed?.(2)}px (${s.stereo.boyut?.join("x")})`
        : "yok"],
      ["el-goz (hand-eye)", s.base_T_cam.var
        ? `var - artik hata ${s.base_T_cam.residual_mm?.toFixed?.(1)}mm`
        : "yok"],
      ["MEASURE_SCALE", s.measure_scale?.toFixed?.(4)],
      ["YAW_CORRECTION_DEG", s.yaw_correction_deg?.toFixed?.(2) + "° (deneysel)"],
    ];
    el.innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td class="val">${v}</td></tr>`).join("");
  } catch (e) {
    el.innerHTML = "<tr><td>kalibrasyon durumu okunamadi</td></tr>";
  }
}
loadCalibStatus();

async function loadSession() {
  try {
    const r = await get("/api/session");
    if (!r.ok || !r.plan) return;
    renderPlan(r.plan);
    renderObjects(r.objects);
    setStatus("measure-status", true, "onceki olcum geri yuklendi (sunucu yeniden baslatildi).");
    if (r.has_photo) {
      const img = document.getElementById("snap-left");
      document.getElementById("snap-wrap").style.display = "block";
      img.src = "/api/photo/" + (r.has_overlay ? "overlay" : "left") + "?t=" + Date.now();
    }
  } catch (e) { /* sessiz - sadece kolaylik */ }
}
loadSession();

async function pollRobot() {
  try {
    const r = await get("/api/robot_state");
    const badge = document.getElementById("robot-badge");
    if (r.ok && r.result) {
      const st = r.result;
      const fault = st.fault && st.fault.main_code !== 0;
      badge.className = fault ? "err" : "ok";
      const tcp = st.tcp_pose || {};
      badge.textContent = `robot: ${fault ? "HATA " + st.fault.main_code + "/" + st.fault.sub_code : "hazir"}  ` +
        `x=${tcp.x?.toFixed?.(0)} y=${tcp.y?.toFixed?.(0)} z=${tcp.z?.toFixed?.(0)}`;
    } else {
      badge.className = "err";
      badge.textContent = "robot: baglanti yok";
    }
  } catch (e) {
    document.getElementById("robot-badge").textContent = "robot: baglanti yok";
  }
}
pollRobot();
setInterval(pollRobot, 2000);
</script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return _PAGE


def main() -> int:
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8090)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

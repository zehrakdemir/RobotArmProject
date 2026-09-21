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
from src.cell import run_cell
from src.multi import detect_objects

from tools import pick_and_place as pnp

app = FastAPI(title="pickcobot")

# ---------------------------------------------------------------------------
# Global oturum durumu (tek kullanicili yerel arac - basit tutuluyor)
# ---------------------------------------------------------------------------
class Session:
    def __init__(self) -> None:
        self.plan: Optional[dict] = None
        self.left_path: Optional[str] = None
        self.right_path: Optional[str] = None
        self.used_az: Optional[float] = None
        self.log: list[str] = []
        self.place_xyz: tuple[float, float, float] = (-300.0, 100.0, 150.0)
        self.place_rpy: tuple[float, float, float] = (180.0, 0.0, 0.0)

    def add(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.append(f"[{ts}] {msg}")
        self.log = self.log[-200:]


S = Session()


def _result(ok: bool, msg: str, **extra: Any) -> dict:
    S.add(msg)
    out = {"ok": ok, "message": msg, "log": S.log[-30:]}
    out.update(extra)
    return out


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
            obj = objs[0]
            g, obb = obj.grasp, obj.obb
            if g is None or not g.feasible:
                return _result(False, f"Kavrama plani yok: {obj.as_dict().get('kavrama_sebep')}")
            message = (f"boy {obj.dims_cm[0]:.1f} · en {obj.dims_cm[1]:.1f} · "
                       f"derinlik {obj.dims_cm[2]:.1f} cm (yaw {obj.yaw_deg:.0f}°, "
                       f"{obj.n_points} nokta)")

        corrected_rz = g.yaw_deg + pnp.YAW_CORRECTION_DEG
        # Arayüzde her zaman uzunluk sırasıyla göster: boy >= en >= derinlik.
        dims_mm = sorted((float(obb.width_mm), float(obb.height_mm),
                          float(obb.thickness_mm)), reverse=True)
        S.plan = {
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
        S.used_az = None
        selected = "secili alan" if roi is not None else "otomatik (nesne modu)"
        return _result(True, f"Olcum tamam ({selected}): {message}", plan=S.plan)

    return _safe(_do)


@app.get("/api/photo/{side}")
def photo(side: str) -> Response:
    path = S.left_path if side == "left" else S.right_path
    if not path or not Path(path).exists():
        return Response(status_code=404)
    data = Path(path).read_bytes()
    return Response(content=data, media_type="image/png")


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
                                prefer_lin=False, label=f"yaklasma (z={candidate_az:.0f})")
            if ok:
                used_az = candidate_az
                break
        if used_az is None:
            return _result(False, "Yaklasma noktasina hicbir yukseklikte ulasilamadi.")
        S.used_az = used_az
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
        ok = pnp.move_pose(body.x, body.y, body.z, body.rx, body.ry, body.rz,
                            False, prefer_lin=False, label="birakma noktasi")
        if not ok:
            return _result(False, "Birakma noktasina ulasilamadi (tekillik/IK).")
        return _result(True, "Birakma noktasina ulasildi.")

    return _safe(_do)


class LowerBody(BaseModel):
    drop_mm: float = 60.0


@app.post("/api/step/lower")
def step_lower(body: LowerBody) -> dict:
    def _do():
        pnp.jog_dz(-body.drop_mm, False, label="birakmaya inis", max_step=500.0)
        return _result(True, "Birakma yuksekligine inildi.")

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
        ("lower", lambda: step_lower(LowerBody())),
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
  <span id="robot-badge">robot: -</span>
</header>
<main>
  <section id="col-measure">
    <h2>Olcum</h2>
    <div class="card">
      <button class="primary" onclick="doMeasure()">Olc</button>
      <div class="status-line" id="measure-status"></div>
      <div class="snap-wrap" id="snap-wrap">
        <img class="snap" id="snap-left" alt="Sol kamera karesi">
      </div>
      <table id="measure-table"></table>
    </div>
  </section>

  <section id="col-steps">
    <h2>Adimlar</h2>
    <div class="card">
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

async function doMeasure() {
  setStatus("measure-status", true, "olculuyor...");
  const r = await post("/api/measure", {});
  setStatus("measure-status", r.ok, r.message);
  renderLog(r.log || []);
  if (r.plan) renderPlan(r.plan);
  const img = document.getElementById("snap-left");
  document.getElementById("snap-wrap").style.display = "block";
  img.src = "/api/photo/left?t=" + Date.now();
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
  setStatus("step-status", true, "tum akis calisiyor...");
  const r = await post("/api/run_all", placeBody());
  setStatus("step-status", r.ok, r.message);
  renderLog(r.log || []);
}

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

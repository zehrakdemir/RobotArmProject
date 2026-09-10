"""movecobot web arayuzunun HTTP/WebSocket sunucusu.

Calistirma:
    movecobot-web                 # 0.0.0.0:8088
    movecobot-web --port 9000

Container'da bu, scripts/start_all.sh tarafindan fairino_launch ile birlikte
otomatik baslatilir.

GUVENLIK NOTU: bu arayuz GERCEK BIR ROBOTU HAREKET ETTIRIR ve kimlik
dogrulamasi YOKTUR. Yalnizca guvendigin bir agda ac. Varsayilan olarak
tum arayuzlere baglanir cunku container icinden disari acilmasi gerekiyor;
kisitlamak icin --host 127.0.0.1 kullan.
"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from fairino_cobot import paths
from fairino_cobot.webapp.session import Busy, RobotSession

STATIC = Path(__file__).resolve().parent / "static"

app = FastAPI(title="movecobot web", docs_url="/api/docs")
oturum = RobotSession()


# ---------------------------------------------------------------------------
# Istek govdeleri
# ---------------------------------------------------------------------------

class JointHedef(BaseModel):
    joints_deg: list[float] = Field(..., min_length=6, max_length=6)
    speed_pct: float | None = None
    accel_pct: float | None = None


class JointFark(BaseModel):
    deltas_deg: list[float] = Field(..., min_length=6, max_length=6)


class TcpYerel(BaseModel):
    dx_mm: float = 0.0
    dy_mm: float = 0.0
    dz_mm: float = 0.0


class TcpMutlak(BaseModel):
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    prefer_lin: bool = True


class GripperIstek(BaseModel):
    action: str  # open | close | grip | move
    position: int = 0
    speed: int = 50
    force: int = 50


class OynatIstek(BaseModel):
    speed_pct: float | None = None
    accel_pct: float | None = None
    grip_speed: int | None = None
    grip_force: int | None = None


# ---------------------------------------------------------------------------
# Yardimcilar
# ---------------------------------------------------------------------------

def _hazir() -> None:
    if not oturum.ready:
        raise HTTPException(
            status_code=503,
            detail=oturum.startup_error or "Robot oturumu henuz hazir degil.",
        )


def _is_baslat(kind: str, label: str, fn) -> dict[str, Any]:
    _hazir()
    try:
        job = oturum.run_job(kind, label, fn)
    except Busy as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return job.as_dict()


# ---------------------------------------------------------------------------
# Durum
# ---------------------------------------------------------------------------

@app.get("/api/state")
def durum() -> dict[str, Any]:
    return oturum.telemetry()


@app.get("/api/gripper/status")
def gripper_durum() -> dict[str, Any]:
    _hazir()
    return oturum.gripper_status()


@app.websocket("/ws/state")
async def ws_durum(ws: WebSocket) -> None:
    """Canli telemetri. Arayuz saniyede birkac kez guncelleniyor; okuma
    onbellekli durum paketinden geldigi icin robota ek yuk bindirmiyor."""
    await ws.accept()
    try:
        while True:
            veri = await asyncio.to_thread(oturum.telemetry)
            await ws.send_text(json.dumps(veri, default=str))
            await asyncio.sleep(0.3)
    except (WebSocketDisconnect, RuntimeError):
        pass


# ---------------------------------------------------------------------------
# Kol hareketleri
# ---------------------------------------------------------------------------

@app.post("/api/arm/joints")
def eklem_hedefi(istek: JointHedef) -> dict[str, Any]:
    def calistir(job):
        job.log.append(f"Eklem hedefi: {istek.joints_deg}")
        return oturum.arm.move_to_joints(
            istek.joints_deg,
            confirm=False,
            speed_pct=istek.speed_pct,
            accel_pct=istek.accel_pct,
        )

    return _is_baslat("arm", "Eklem hedefine git", calistir)


@app.post("/api/arm/jog")
def eklem_jog(istek: JointFark) -> dict[str, Any]:
    def calistir(job):
        job.log.append(f"Eklem farki: {istek.deltas_deg}")
        return oturum.arm.move_relative_joints(istek.deltas_deg, confirm=False)

    return _is_baslat("arm", "Eklem jog", calistir)


@app.post("/api/arm/tcp_local")
def tcp_yerel(istek: TcpYerel) -> dict[str, Any]:
    def calistir(job):
        job.log.append(f"TCP yerel: dx={istek.dx_mm} dy={istek.dy_mm} dz={istek.dz_mm}")
        return oturum.arm.move_tcp_local(
            istek.dx_mm, istek.dy_mm, istek.dz_mm, confirm=False
        )

    return _is_baslat("arm", "TCP yerel hareket", calistir)


@app.post("/api/arm/pose")
def tcp_mutlak(istek: TcpMutlak) -> dict[str, Any]:
    def calistir(job):
        job.log.append(f"TCP hedefi: {istek.x},{istek.y},{istek.z}")
        return oturum.arm.move_to_pose_base(
            istek.x, istek.y, istek.z, istek.rx, istek.ry, istek.rz,
            confirm=False, prefer_lin=istek.prefer_lin,
        )

    return _is_baslat("arm", "TCP pozuna git", calistir)


@app.post("/api/arm/preview")
def onizleme(istek: JointHedef) -> dict[str, Any]:
    """Hareket ETMEDEN, verilen eklem acilarinin TCP karsiligini hesaplar.
    Joint modunda nereye gidilecegini once gormek icin (movecobot'taki FK
    onizlemesinin ayni)."""
    _hazir()
    try:
        return {"tcp_pose": oturum.arm.predict_tcp_pose(istek.joints_deg)}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------

@app.post("/api/gripper")
def gripper(istek: GripperIstek) -> dict[str, Any]:
    if oturum.gripper is None:
        raise HTTPException(status_code=503, detail="Gripper yok / hazirlanamadi.")

    def calistir(job):
        g = oturum.gripper
        job.log.append(f"Gripper {istek.action} (hiz={istek.speed} kuvvet={istek.force})")
        if istek.action == "open":
            g.open(speed=istek.speed, force=istek.force)
        elif istek.action == "close":
            g.close(speed=istek.speed, force=istek.force)
        elif istek.action == "grip":
            # Adaptif kavrama: nesne buyuklugunu bilmeye gerek yok, basinctan
            # anliyor. Kayitlarda da pozisyon degil "kavra" eylemi saklaniyor.
            g.grip(speed=istek.speed, force=istek.force)
        elif istek.action == "move":
            g.move(istek.position, speed=istek.speed, force=istek.force)
        else:
            raise ValueError(f"bilinmeyen eylem: {istek.action}")
        return {"success": True}

    return _is_baslat("gripper", f"Gripper: {istek.action}", calistir)


# ---------------------------------------------------------------------------
# Kayitli hareketler
# ---------------------------------------------------------------------------

@app.get("/api/moves")
def hareketler() -> dict[str, Any]:
    from fairino_cobot.movecobot import load_moves

    moves = load_moves()
    ozet = []
    for ad, kayit in moves.items():
        if isinstance(kayit, dict) and kayit.get("type") == "pick_place":
            ozet.append({"name": ad, "type": "pick_place", "steps": 2,
                         "speed_pct": kayit.get("speed_pct"),
                         "accel_pct": kayit.get("accel_pct"),
                         "saved_at": kayit.get("saved_at")})
        else:
            adimlar = kayit.get("steps", []) if isinstance(kayit, dict) else kayit
            ozet.append({"name": ad, "type": "sequence", "steps": len(adimlar),
                         "speed_pct": kayit.get("speed_pct") if isinstance(kayit, dict) else None,
                         "accel_pct": kayit.get("accel_pct") if isinstance(kayit, dict) else None,
                         "saved_at": kayit.get("saved_at") if isinstance(kayit, dict) else None})
    return {"moves": sorted(ozet, key=lambda m: m["name"]), "file": str(paths.moves_file())}


@app.post("/api/moves/{ad}/play")
def oynat(ad: str, istek: OynatIstek) -> dict[str, Any]:
    from fairino_cobot.movecobot import load_moves, replay_sequence, run_pick_place

    moves = load_moves()
    if ad not in moves:
        raise HTTPException(status_code=404, detail=f"'{ad}' diye bir kayit yok.")
    kayit = moves[ad]

    def calistir(job):
        job.log.append(f"'{ad}' oynatiliyor")
        if isinstance(kayit, dict) and kayit.get("type") == "pick_place":
            cfg = dict(kayit)  # kopya - kayitli degerler bozulmasin
            for anahtar, deger in (
                ("speed_pct", istek.speed_pct), ("accel_pct", istek.accel_pct),
                ("grip_speed", istek.grip_speed), ("grip_force", istek.grip_force),
            ):
                if deger is not None:
                    cfg[anahtar] = deger
            return run_pick_place(oturum.arm, oturum.gripper, cfg, sor=False)
        adimlar = kayit.get("steps", []) if isinstance(kayit, dict) else kayit
        return replay_sequence(oturum.arm, oturum.gripper, adimlar, oturum.robot_ip)

    return _is_baslat("replay", f"Oynat: {ad}", calistir)


@app.delete("/api/moves/{ad}")
def sil(ad: str) -> dict[str, Any]:
    from fairino_cobot.movecobot import load_moves, save_moves, sync_bashrc_aliases

    moves = load_moves()
    if ad not in moves:
        raise HTTPException(status_code=404, detail=f"'{ad}' diye bir kayit yok.")
    del moves[ad]
    save_moves(moves)
    try:
        sync_bashrc_aliases(moves)
    except Exception:
        pass  # container'da .bashrc olmayabilir - kayit silindi, yeter
    return {"deleted": ad}


# ---------------------------------------------------------------------------
# Guvenlik
# ---------------------------------------------------------------------------

@app.post("/api/stop")
def acil_dur() -> dict[str, Any]:
    """Acil durdurma. Hareket kilidini beklemez."""
    _hazir()
    return oturum.stop()


@app.post("/api/fault/clear")
def fault_temizle() -> dict[str, Any]:
    """Robot hatalarini temizler.

    DIKKAT: ResetAllError() GLOBAL bir resettir ve kolun servo oturumunu
    duesurur - sonrasinda ros2_control akisi kopar ve launch'in yeniden
    baslatilmasi gerekir. Bu, arayuzde de acikca uyariliyor.
    """
    _hazir()
    try:
        with oturum.arm.sdk_lock:
            rc = oturum.arm.sdk.ResetAllError()
        return {
            "rc": rc,
            "warning": "Servo oturumu duestu - fairino_launch yeniden baslatilmali.",
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) from e


# ---------------------------------------------------------------------------
# Statik dosyalar
# ---------------------------------------------------------------------------

@app.get("/")
def kok() -> FileResponse:
    return FileResponse(STATIC / "index.html")


app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")


@app.on_event("startup")
async def baslat() -> None:
    # Robot baglantisi uzun surer; sunucu hemen ayaga kalksin ki arayuz
    # "hazirlaniyor" ekranini gosterebilsin.
    asyncio.get_running_loop().run_in_executor(None, oturum.start)


@app.on_event("shutdown")
async def kapat() -> None:
    oturum.shutdown()


def main() -> None:
    import uvicorn

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")  # noqa: S104 - container'dan disari acilmali
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--speed", type=float, default=10.0, help="varsayilan hiz %%")
    ap.add_argument("--accel", type=float, default=10.0, help="varsayilan ivme %%")
    args = ap.parse_args()

    oturum.speed_pct = args.speed
    oturum.accel_pct = args.accel

    print(f"movecobot web arayuzu: http://localhost:{args.port}")
    print("UYARI: kimlik dogrulamasi YOK, gercek robotu hareket ettirir.")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()

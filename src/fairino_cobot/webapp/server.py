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
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from fairino_cobot import paths
from fairino_cobot.webapp.session import Busy, RobotSession

STATIC = Path(__file__).resolve().parent / "static"

app = FastAPI(title="movecobot web", docs_url="/api/docs")
oturum = RobotSession()


@app.middleware("http")
async def onbellek_kapat(request: Request, call_next) -> Response:
    """Arayuz dosyalari (/, /static/) her acilista sunucuya sorulsun.

    NEDEN: imaj yenilenip container yeniden olusturulunca tarayici eski
    app.js'i onbellekten kullanabiliyor. Degismeyen dosya yine 304 ile doner.
    """
    cevap = await call_next(request)
    yol = request.url.path
    if yol == "/" or yol.startswith("/static/"):
        cevap.headers["Cache-Control"] = "no-cache"
    return cevap


# ---------------------------------------------------------------------------
# Istek govdeleri
# ---------------------------------------------------------------------------

class JointHedef(BaseModel):
    joints_deg: list[float] = Field(..., min_length=6, max_length=6)
    speed_pct: float | None = None
    accel_pct: float | None = None


class JointFark(BaseModel):
    deltas_deg: list[float] = Field(..., min_length=6, max_length=6)


class TcpFark(BaseModel):
    """TCP icin goreceli oteleme. Hangi cerceveye gore oldugunu ENDPOINT
    belirler: /api/arm/tcp_local arac eksenleri, /api/arm/tcp_base base
    (dunya) eksenleri."""

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


class Ayarlar(BaseModel):
    """Ayarlar penceresi. Aralik arayuzdeki diger yuzde alanlariyla ayni
    (1-100); _plan_and_execute de olcegi 0.01-1.0'a kirpiyor."""

    speed_pct: float = Field(..., ge=1, le=100)
    accel_pct: float = Field(..., ge=1, le=100)


class PPPoz(BaseModel):
    """Pick & place kaydinda alma (pick) ya da birakma (place) pozu."""

    etiket: str = Field(..., pattern="^(pick|place)$")
    approach_mm: float = Field(..., gt=0)


class PPKaydet(BaseModel):
    """Pick & place taslagini kayit olarak yazma istegi."""

    name: str
    approach_mm: float = Field(..., gt=0)
    grip_speed: int = Field(..., ge=1, le=100)
    grip_force: int = Field(..., ge=1, le=100)
    object_d_mm: float = Field(..., gt=0)
    object_h_mm: float = Field(..., gt=0)
    overwrite: bool = False


class OnizlemeIstek(BaseModel):
    """Yol onizlemesi: hareket isteginin AYNISI, robot hareket etmeden
    planlanir. values: pose -> x,y,z,rx,ry,rz (base, mm/derece); tcp_local ve
    tcp_base -> dx,dy,dz (mm); joints -> 6 mutlak aci; jog -> 6 aci farki."""

    kind: Literal["pose", "tcp_local", "tcp_base", "joints", "jog"]
    values: list[float] = Field(..., min_length=3, max_length=6)
    prefer_lin: bool = True


class OnizlemeCalistir(BaseModel):
    """Onizlemedeki bir secenegi calistirma istegi. Yol DEGIL, yalnizca
    onizleme numarasi ve secenek numarasi (1'den baslar) gelir."""

    id: int
    option: int


# ---------------------------------------------------------------------------
# Yardimcilar
# ---------------------------------------------------------------------------

def _hazir() -> None:
    if not oturum.ready:
        raise HTTPException(
            status_code=503,
            detail=oturum.startup_error or "Robot session is not ready yet.",
        )


def _is_baslat(kind: str, label: str, fn) -> dict[str, Any]:
    _hazir()
    try:
        job = oturum.run_job(kind, label, fn)
    except Busy as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return job.as_dict()


def _kol_hizi() -> tuple[float, float]:
    """Ayarlar penceresindeki kol hiz/ivmesi. Istek aninda okunur; suren
    bir hareketi degistirmez."""
    a = oturum.ayarlar
    return a["speed_pct"], a["accel_pct"]


# ---------------------------------------------------------------------------
# Durum
# ---------------------------------------------------------------------------

@app.get("/api/state")
def durum() -> dict[str, Any]:
    return oturum.telemetry()


# ---------------------------------------------------------------------------
# Ayarlar
# ---------------------------------------------------------------------------
# Eklem jog, TCP jog, mutlak TCP pozu ve mutlak eklem hedefi bu degerleri
# kullanir. Kayitli hareketler (/api/moves) ve Gripper paneli (/api/gripper)
# KULLANMAZ: onlar kendi alanlarini gonderiyor.

@app.get("/api/settings")
def ayarlari_oku() -> dict[str, Any]:
    return dict(oturum.ayarlar)


@app.post("/api/settings")
def ayarlari_kaydet(istek: Ayarlar) -> dict[str, Any]:
    """Robota bir sey gondermez; yalnizca sonraki isteklerde kullanilir."""
    oturum.ayarlar = istek.model_dump()
    return dict(oturum.ayarlar)


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
    hiz, ivme = _kol_hizi()
    # Istekte hiz/ivme verilmisse o gecerli; arayuz gondermiyor.
    if istek.speed_pct is not None:
        hiz = istek.speed_pct
    if istek.accel_pct is not None:
        ivme = istek.accel_pct

    def calistir(job):
        job.log.append(f"Joint target: {istek.joints_deg} (speed {hiz:g}%, accel {ivme:g}%)")
        # Girilen acilara aynen; once eklem siniri kontrolu.
        return oturum.arm.move_to_joints_absolute(
            istek.joints_deg,
            confirm=False,
            speed_pct=hiz,
            accel_pct=ivme,
        )

    return _is_baslat("arm", "Move to joint target", calistir)


@app.post("/api/arm/jog")
def eklem_jog(istek: JointFark) -> dict[str, Any]:
    hiz, ivme = _kol_hizi()

    def calistir(job):
        job.log.append(f"Joint delta: {istek.deltas_deg} (speed {hiz:g}%, accel {ivme:g}%)")
        return oturum.arm.move_relative_joints(
            istek.deltas_deg, confirm=False, speed_pct=hiz, accel_pct=ivme
        )

    return _is_baslat("arm", "Joint jog", calistir)


@app.post("/api/arm/tcp_local")
def tcp_yerel(istek: TcpFark) -> dict[str, Any]:
    """TCP'nin KENDI eksenlerine gore goreceli hareket - eksenler aletle
    birlikte doner."""
    hiz, ivme = _kol_hizi()

    def calistir(job):
        job.log.append(f"TCP tool: dx={istek.dx_mm} dy={istek.dy_mm} dz={istek.dz_mm} "
                       f"(speed {hiz:g}%, accel {ivme:g}%)")
        return oturum.arm.move_tcp_local(
            istek.dx_mm, istek.dy_mm, istek.dz_mm, confirm=False,
            speed_pct=hiz, accel_pct=ivme,
        )

    return _is_baslat("arm", "Tool jog", calistir)


@app.post("/api/arm/tcp_base")
def tcp_baz(istek: TcpFark) -> dict[str, Any]:
    """TCP'yi BASE (dunya) eksenlerine gore goreceli hareket ettirir -
    eksenler SABIT, alet nasil durursa dursun +Z yukaridir."""
    hiz, ivme = _kol_hizi()

    def calistir(job):
        job.log.append(f"TCP base: dx={istek.dx_mm} dy={istek.dy_mm} dz={istek.dz_mm} "
                       f"(speed {hiz:g}%, accel {ivme:g}%)")
        return oturum.arm.move_tcp_base(
            istek.dx_mm, istek.dy_mm, istek.dz_mm, confirm=False,
            speed_pct=hiz, accel_pct=ivme,
        )

    return _is_baslat("arm", "Base jog", calistir)


@app.post("/api/arm/pose")
def tcp_mutlak(istek: TcpMutlak) -> dict[str, Any]:
    hiz, ivme = _kol_hizi()

    def calistir(job):
        job.log.append(f"TCP target: {istek.x},{istek.y},{istek.z} (speed {hiz:g}%, accel {ivme:g}%)")
        return oturum.arm.move_to_pose_base(
            istek.x, istek.y, istek.z, istek.rx, istek.ry, istek.rz,
            confirm=False, prefer_lin=istek.prefer_lin,
            speed_pct=hiz, accel_pct=ivme,
        )

    return _is_baslat("arm", "Move to TCP pose", calistir)


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
# Yol onizlemesi ("Preview before moving", kullanici istegi 15 Eylul)
# ---------------------------------------------------------------------------
# Hareket istegi robot HAREKET ETMEDEN planlanir; secenekler (robotun IK
# duruslari, LIN/PTP/OMPL) guvenlik kontrolleriyle birlikte arayuzde
# gosterilir. Kullanici bir secenegi secince YALNIZCA o plan yurutulur:
# tarayici yol gondermez, sunucu sakladigi plani calistirir. Kayitli
# hareketler ve pick & place bu kapsamda degil.

ONIZLEME_DEGER_SAYISI = {"pose": 6, "tcp_local": 3, "tcp_base": 3, "joints": 6, "jog": 6}


@app.post("/api/preview")
def yollari_onizle(istek: OnizlemeIstek) -> dict[str, Any]:
    beklenen = ONIZLEME_DEGER_SAYISI[istek.kind]
    if len(istek.values) != beklenen:
        raise HTTPException(
            status_code=400,
            detail=f"'{istek.kind}' needs {beklenen} values, got {len(istek.values)}.",
        )
    hiz, ivme = _kol_hizi()

    def calistir(job):
        # Eski onizleme, yenisi planlanirken calistirilamasin.
        oturum.onizleme = None
        job.log.append(f"Plan paths: {istek.kind} {istek.values} (speed {hiz:g}%, accel {ivme:g}%)")
        sonuc = oturum.arm.yol_secenekleri(istek.kind, istek.values, prefer_lin=istek.prefer_lin,
                                           speed_pct=hiz, accel_pct=ivme)
        planlar = sonuc.pop("_planlar", [])
        oturum.onizleme_kaydet(sonuc, planlar)
        secenekler = sonuc.get("options", [])
        job.log.append(f"{len(secenekler)} options, {sum(1 for s in secenekler if s['ok'])} can be run")
        if not sonuc.get("success"):
            return {"success": False, "reason": sonuc.get("reason"), "details": sonuc.get("details")}
        return {"success": True}

    return _is_baslat("preview", "Plan paths", calistir)


@app.get("/api/preview")
def onizleme_oku() -> dict[str, Any]:
    """Son onizleme (plan nesneleri olmadan). Yoksa bos sozluk."""
    o = oturum.onizleme
    return o["ozet"] if o else {}


@app.delete("/api/preview")
def onizleme_sil() -> dict[str, Any]:
    oturum.onizleme = None
    return {}


@app.post("/api/preview/run")
def onizleme_calistir(istek: OnizlemeCalistir) -> dict[str, Any]:
    """Onizlemedeki bir secenegi calistirir. ROBOTU HAREKET ETTIRIR (arayuz
    once onay ister). Plan yeniden uretilmez; kol onizlemeden sonra oynadiysa
    CobotArm.onizlenen_yolu_yurut hareket etmeden reddeder."""
    _hazir()
    o = oturum.onizleme
    if o is None:
        raise HTTPException(status_code=404, detail="No path preview. Plan the paths first.")
    if o["id"] != istek.id:
        raise HTTPException(status_code=409, detail="This preview is out of date. Plan the paths again.")
    secenekler = o["ozet"].get("options", [])
    if not 1 <= istek.option <= len(secenekler):
        raise HTTPException(status_code=404, detail=f"No option {istek.option} in this preview.")
    ozet = secenekler[istek.option - 1]
    plan = o["planlar"][istek.option - 1]
    if not ozet["ok"] or plan is None:
        raise HTTPException(
            status_code=409,
            detail=f"Option {istek.option} was rejected ({ozet.get('reason') or 'no plan'}); "
                   "it cannot be run.",
        )
    baslangic = o["ozet"]["start_joints"]

    def calistir(job):
        job.log.append(f"Run previewed path {istek.option}: {ozet['label']} ({ozet['method']})")
        return oturum.arm.onizlenen_yolu_yurut(plan, baslangic)

    cevap = _is_baslat("arm", f"Run previewed path {istek.option}", calistir)
    # Tek kullanimlik: ayni plan ikinci kez yurutulmez (kol artik baska yerde).
    oturum.onizleme = None
    return cevap


# ---------------------------------------------------------------------------
# 3D gorunum icin robot modeli
# ---------------------------------------------------------------------------

def _paket_klasoru(paket: str) -> str | None:
    """ROS paketinin kurulum (share) klasoru; bulunamazsa None."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory(paket)
    except Exception:
        return None


def _robot_modeli() -> dict[str, Any]:
    """MoveIt'in KULLANDIGI URDF'ten model ozeti; oturum boyunca bir kez."""
    if oturum.model_onbellek is None:
        from fairino_cobot.robot_modeli import model_paketi

        kol = oturum.arm
        urdf = getattr(kol, "robot_description_urdf", None)
        if not urdf:
            raise HTTPException(status_code=503, detail="Robot model (URDF) is not available.")
        ozet, dosyalar = model_paketi(urdf, _paket_klasoru)
        ozet["joint_names"] = list(kol.joint_names)
        ozet["tip_link"] = kol.tip_link
        ozet["tcp_offset_m"] = [float(v) for v in kol.tcp_offset_m]
        ozet["gripper_box"] = kol.gripper_kutusu
        oturum.model_onbellek = {"ozet": ozet, "dosyalar": dosyalar}
    return oturum.model_onbellek


@app.get("/api/model")
def model_oku() -> dict[str, Any]:
    _hazir()
    return _robot_modeli()["ozet"]


@app.get("/api/model/mesh/{sira}")
def mesh_oku(sira: int) -> FileResponse:
    """Yalnizca model ozetinde listelenen dosyalar, sira numarasiyla (yol yok)."""
    _hazir()
    dosyalar = _robot_modeli()["dosyalar"]
    if not 0 <= sira < len(dosyalar):
        raise HTTPException(status_code=404, detail="No such mesh.")
    return FileResponse(dosyalar[sira], media_type="model/stl")


# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------

@app.post("/api/gripper")
def gripper(istek: GripperIstek) -> dict[str, Any]:
    if oturum.gripper is None:
        raise HTTPException(status_code=503, detail="No gripper, or the gripper could not be initialized.")

    def calistir(job):
        g = oturum.gripper
        job.log.append(f"Gripper {istek.action} (speed={istek.speed} force={istek.force})")
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
            raise ValueError(f"unknown action: {istek.action}")
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
        raise HTTPException(status_code=404, detail=f"No saved move named '{ad}'.")
    kayit = moves[ad]

    def calistir(job):
        job.log.append(f"Playing '{ad}'")
        # onay_iste=False: web'de terminal yok; onay arayuz penceresinden alindi.
        if isinstance(kayit, dict) and kayit.get("type") == "pick_place":
            cfg = dict(kayit)  # kopya - kayitli degerler bozulmasin
            for anahtar, deger in (
                ("speed_pct", istek.speed_pct), ("accel_pct", istek.accel_pct),
                ("grip_speed", istek.grip_speed), ("grip_force", istek.grip_force),
            ):
                if deger is not None:
                    cfg[anahtar] = deger
            tamam = run_pick_place(oturum.arm, oturum.gripper, cfg, sor=False, onay_iste=False)
        else:
            adimlar = kayit.get("steps", []) if isinstance(kayit, dict) else kayit
            tamam = replay_sequence(oturum.arm, oturum.gripper, adimlar, oturum.robot_ip,
                                    onay_iste=False)
        # Ikisi de True/False donuyor; run_job yalnizca {"success": ...}
        # sozlugunu anliyor, digerini BASARI sayiyordu.
        if tamam:
            return {"success": True}
        return {"success": False, "reason": "replay_failed",
                "details": "playback did not complete - see the container log for details."}

    return _is_baslat("replay", f"Play: {ad}", calistir)


# ---------------------------------------------------------------------------
# Pick & place kaydi
# ---------------------------------------------------------------------------
# Terminaldeki record_pick_and_place ile AYNI akis ve AYNI kayit bicimi: kol
# jog ile kavrama pozuna getirilir, poz kaydedilir; kol TCP'nin kendi Z
# ekseninde approach_mm GERI cekilip on-poz kaydedilir. Ayni sey birakma icin.
# Taslak oturumda tutulur (oturum.pp_taslak), kaydedilince bosalir.

PP_POZ_ANAHTARLARI = ("pick", "pre_pick", "place", "pre_place")


@app.get("/api/pickplace/draft")
def pp_taslak_oku() -> dict[str, Any]:
    return dict(oturum.pp_taslak)


@app.delete("/api/pickplace/draft")
def pp_taslak_sil() -> dict[str, Any]:
    oturum.pp_taslak = {}
    return {}


@app.post("/api/pickplace/pose")
def pp_poz_kaydet(istek: PPPoz) -> dict[str, Any]:
    """Kolun SU ANKI eklem acilarini alma/birakma pozu olarak kaydeder, sonra
    TCP'nin kendi Z ekseninde approach_mm GERI cekilip on-pozu kaydeder.

    Geri cekilme ROBOTU HAREKET ETTIRIR (arayuz once onay ister). Geri
    cekilemezse taslak degismez; terminaldeki gibi yaklasma mesafesi
    kucultulup tekrar denenir."""
    onceki = oturum.pp_taslak.get("approach_mm")
    if onceki is not None and onceki != istek.approach_mm:
        raise HTTPException(
            status_code=409,
            detail=(f"Approach distance differs from the first pose ({onceki} mm). "
                    "Use the same value or reset the draft."),
        )
    hiz, ivme = _kol_hizi()

    def calistir(job):
        kol = oturum.arm
        poz = list(kol.get_current_state()["joints_deg"])
        job.log.append(f"{istek.etiket} pose: {poz}")
        sonuc = kol.move_tcp_local(0, 0, -istek.approach_mm, confirm=False,
                                   speed_pct=hiz, accel_pct=ivme)
        if not sonuc.get("success"):
            return sonuc
        # Varis dogrulamasinin yoklayarak okudugu acilar (bayat okumaya karsi);
        # yoksa taze oku.
        son = sonuc.get("final_state") or kol.get_current_state()
        on_poz = list(son["joints_deg"])
        job.log.append(f"{istek.etiket} approach pose: {on_poz}")
        oturum.pp_taslak = {**oturum.pp_taslak, "approach_mm": istek.approach_mm,
                            istek.etiket: poz, f"pre_{istek.etiket}": on_poz}
        return sonuc

    return _is_baslat("arm", f"Pick & place: {istek.etiket} pose", calistir)


@app.post("/api/pickplace/save")
def pp_kaydet(istek: PPKaydet) -> dict[str, Any]:
    """Taslagi terminaldeki record_pick_and_place ile ayni bicimde kayit
    dosyasina yazar. Robota bir sey gondermez."""
    from fairino_cobot.movecobot import load_moves, save_moves, sync_bashrc_aliases, valid_move_name

    t = oturum.pp_taslak
    eksik = [k for k in PP_POZ_ANAHTARLARI if k not in t]
    if eksik:
        raise HTTPException(
            status_code=400,
            detail="Save the pick and place poses first (missing: " + ", ".join(eksik) + ").",
        )
    if t["approach_mm"] != istek.approach_mm:
        raise HTTPException(
            status_code=409,
            detail=f"Approach distance was {t['approach_mm']} mm when the poses were saved.",
        )
    ok, hata = valid_move_name(istek.name)
    if not ok:
        raise HTTPException(status_code=400, detail=f"Invalid name: {hata}")
    moves = load_moves()
    if istek.name in moves and not istek.overwrite:
        raise HTTPException(status_code=409, detail=f"'{istek.name}' already exists.")

    hiz, ivme = _kol_hizi()
    moves[istek.name] = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "type": "pick_place",
        "pick_joints": t["pick"],
        "place_joints": t["place"],
        # On-pozlar: kavrama/birakma noktasinin approach_mm kadar gerisi.
        "pre_pick_joints": t["pre_pick"],
        "pre_place_joints": t["pre_place"],
        "approach_mm": istek.approach_mm,
        "grip_speed": istek.grip_speed,
        "grip_force": istek.grip_force,
        "object_dh_mm": [istek.object_d_mm, istek.object_h_mm],
        # Terminal kaydi o anki oturum hizini yaziyor; web'de o, Ayarlar penceresi.
        "speed_pct": hiz,
        "accel_pct": ivme,
    }
    save_moves(moves)
    try:
        sync_bashrc_aliases(moves)
    except Exception:
        pass  # container'da .bashrc olmayabilir - kayit yazildi, yeter
    oturum.pp_taslak = {}
    return {"saved": istek.name}


@app.delete("/api/moves/{ad}")
def sil(ad: str) -> dict[str, Any]:
    from fairino_cobot.movecobot import load_moves, save_moves, sync_bashrc_aliases

    moves = load_moves()
    if ad not in moves:
        raise HTTPException(status_code=404, detail=f"No saved move named '{ad}'.")
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
def hareketi_durdur() -> dict[str, Any]:
    """Suren kol hareketini durdurur (MoveIt yurutmesini iptal eder).
    Hareket kilidini beklemez. Acil stop degildir."""
    _hazir()
    return oturum.stop()


# Fault temizleme web'den YAPILMAZ: ResetAllError() global bir reset, kolun
# servo oturumunu dusurur ve ros2_control calisirken hata geri gelir. Fault
# olursa robotun kendi WebApp'inde Clear, sonra container yeniden baslatilir.


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

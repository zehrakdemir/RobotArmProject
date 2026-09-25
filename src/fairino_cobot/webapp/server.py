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
import math
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


class PPSonKaydet(BaseModel):
    """Son calistirilan onizlenmis pick & place zincirini kaydetme istegi."""

    name: str
    overwrite: bool = False


class PPPozdaCalistir(BaseModel):
    """Kayitli bir hareket olmadan, DISARIDAN (kamera/vision) gelen yaklasim
    pozlarindan MTC pick&place calistirma istegi (17 Eylul).

    GUVENLIK: bu yol tek-tek hareketlerin (_ik_adaylariyla_git) gectigi
    zemin/self-collision post-kontrolu, tekil korumasi ve SDK varis
    dogrulamasindan GECMEZ - MTC kendi Cartesian planlamasini yapar, ekstra
    kontrol yoktur (bkz. GUNCEL_DURUM.md 9-27). Kullanici bilerek onayladi."""

    pick_approach: list[float] = Field(..., min_length=6, max_length=6)
    place_approach: list[float] = Field(..., min_length=6, max_length=6)
    approach_mm: float = Field(..., gt=0)
    grip_speed: int = Field(..., ge=1, le=100)
    grip_force: int = Field(..., ge=1, le=100)
    object_d_mm: float = Field(..., gt=0)
    object_h_mm: float = Field(..., gt=0)
    # On planlamada nesnenin genisligi (kavrama yonunde, mm). Gercekte gripper
    # kapaninca olculuyor; verilmezse tam strok (GRIPPER_STROKE_M) alinir.
    object_w_mm: float | None = Field(None, gt=0)
    # Kavrama noktasi nesnenin UST YUZUNUN kac mm altinda (panel "Grasp depth
    # below top"). Carpisma kutusunu dogru yere koymak icin gerekli: kutu
    # TCP'de ORTALANIRSA kod "nesne tam ortasindan kavranmis" varsayar, oysa
    # nesnenin buyuk kismi kavrama noktasinin ALTINDA (bkz.
    # safety.kavranan_nesne_kutusu_merkezi). Verilmezse eski (ortadan)
    # davranis - kayitli hareketlerde derinlik bilinmiyor.
    grasp_depth_mm: float | None = Field(None, gt=0)
    # Kac kol durusu icin tam zincir planlanip aralarindan EN KISA yol
    # secilsin (23 Eylul). 1 = eski davranis (ilk uygun cift). Her artis
    # fazladan bir tam zincir planlamasi (3 MTC gorevi) demek.
    max_options: int = Field(3, ge=1, le=6)
    # True: yalnizca zinciri planla ve secilen ciftleri logla; robot hareket
    # etmez, gripper'a dokunulmaz.
    sadece_planla: bool = False


class PPOnizleme(BaseModel):
    """Vision koprusunden gelen bir nesne icin pick & place ONIZLEMESI
    istegi (bkz. webapp Pick & place bolumu). PPPozdaCalistir'den farki:
    hareket etmez, tum zinciri (points/times) doner ve mtc_yolunu_denetle
    ile zemin/self-collision kontrolunden GECIRIR - artik tum yol
    noktalari elde oldugu icin bu bedava (bkz. o fonksiyonun docstring'i).

    STANDART KAVRAMA/BIRAKMA TANIMI (25 Eylul, MTC demo'su gibi): pozlar
    TCP'nin degil KAVRAMA NOKTASININ (parmak uclarinin ortasi) - sunucu
    MTC'ye ik_frame olarak parmak ucunu verir (CobotArm.kavrama_ik_cercevesi),
    istemci TCP-parmak ucu farkini hic bilmez. Birakma NESNENIN yeriyle
    verilir: place_pose z = nesnenin konacagi YUZEYIN yuksekligi (zemin 0);
    nesne yuksekligi ve birakma payi sunucuda eklenir
    (tcp_cercevesi.birakma_noktasi)."""

    # Parmak uclarinin kavramadaki konumu (taban, mm).
    grasp_point: list[float] = Field(..., min_length=3, max_length=3)
    grasp_rpy: list[float] = Field(..., min_length=3, max_length=3)
    # x, y = nesnenin birakilacagi yer, z = uzerine konacagi yuzey (mm).
    # rx/ry/rz kullanilmaz: birakma yonelimi kavramanin yonelimi.
    place_pose: list[float] = Field(..., min_length=6, max_length=6)
    # Parmak uclarinin duz inis/kalkis mesafesi (kavrama noktasina kadar).
    approach_mm: float = Field(..., gt=0, le=200)
    grip_speed: int = Field(..., ge=1, le=100)
    grip_force: int = Field(..., ge=1, le=100)
    object_w_mm: float | None = Field(None, gt=0)
    object_d_mm: float = Field(..., gt=0)
    object_h_mm: float = Field(..., gt=0, le=160)
    # Parmak uclari nesnenin UST YUZUNUN kac mm altinda (panel "Grasp depth
    # below top"). Hem birakma yuksekligi hem carpisma kutusunun yeri buna
    # bagli: kutu kavrama noktasinda ORTALANIRSA kod "nesne tam ortasindan
    # kavranmis" varsayar, oysa nesnenin buyuk kismi kavrama noktasinin
    # ALTINDA (bkz. safety.kavranan_nesne_kutusu_merkezi).
    grasp_depth_mm: float = Field(..., gt=0)
    # Kac farkli kol yolu planlanip secenek olarak sunulsun (23 Eylul).
    # Her secenek AYRI bir tam zincir planlamasi (3 MTC gorevi) demek -
    # arttirmak onizlemeyi uzatir. Ust sinir kasitli olarak dusuk.
    max_options: int = Field(3, ge=1, le=6)


class PPOnizlemeCalistir(BaseModel):
    """Onizlenen pick & place zincirini calistirma istegi. option 1'den
    baslar (OnizlemeCalistir ile ayni sozlesme); verilmezse en kisa yol
    (1. secenek)."""

    id: int
    option: int = Field(1, ge=1)


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


def _aci_norm(a: float) -> float:
    """Aciyi [-180, 180) araligina getirir."""
    return (a + 180.0) % 360.0 - 180.0


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
        # "Show safety layers" icin (yalnizca gorunum): zemin kutusu ve
        # planlayiciya verilen padding. Degerler sahneye eklenenlerin aynisi.
        ozet["safety"] = {
            "floor_box": getattr(kol, "zemin_kutusu", None),
            "padding_m": getattr(kol, "padding_m", None),
        }
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
        if isinstance(kayit, dict) and kayit.get("type") in ("pick_place", "pick_place_chain"):
            ozet.append({"name": ad, "type": kayit["type"], "steps": 2,
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
    if isinstance(kayit, dict) and kayit.get("type") == "pick_place_chain":
        return _kayitli_zinciri_onizle(ad, kayit, istek)

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


def _kayitli_zinciri_onizle(ad: str, kayit: dict[str, Any], istek: OynatIstek) -> dict[str, Any]:
    """Kayitli "pick_place_chain" oynatmasi = ROBOT HAREKET ETMEDEN planlama.
    Zincir, kayittaki pre_pick/pre_place duruslari ve olculerle kolun O ANKI
    yerinden planlanir, yeni bir planla ayni kontrollerden (mtc_yolunu_denetle)
    gecer ve pick & place onizlemesi olarak saklanir; robot yalnizca Path
    preview'da "Run" (/api/pickplace/preview/run) ile hareket eder.

    Eski "pick_place" kayitlarinin yolu (run_pick_place) KULLANILMAZ: o yol
    nesne kutusunu TCP'ye gore koyuyor ve son iniste nesne-zemin izni
    vermiyor; yuzeye birakilan nesnede iniste (nesne tutulurken) planlama
    dusebilir.

    Hiz/ivme ve gripper hiz/kuvveti: Saved moves panelindeki alanlar doluysa
    onlar, bossa kayittakiler (diger kayitli hareketlerle ayni kural)."""
    def secim(panel, kayitli, varsayilan):
        if panel is not None:
            return panel
        return kayitli if kayitli is not None else varsayilan

    def calistir(job):
        arm = oturum.arm
        pp = oturum.pp
        if pp is None:
            return _pp_basarisiz("mtc_init_failed",
                                 "MTC was not initialized at startup (see server log).")
        v_hiz, v_ivme = _kol_hizi()
        hiz = secim(istek.speed_pct, kayit.get("speed_pct"), v_hiz)
        ivme = secim(istek.accel_pct, kayit.get("accel_pct"), v_ivme)
        pp.hizlari_guncelle(hiz, ivme)
        boyut = [float(v) for v in kayit["object_size_mm"]]
        derinlik = float(kayit["grasp_depth_mm"])
        kavrama_ofseti = arm.kavrama_ik_cercevesi()[0]
        job.log.append(f"Saved pick & place '{ad}': planning from the current arm pose "
                       "(the robot does not move)")
        return _pp_zincirleri_planla_ve_sakla(
            job, arm, pp, [(list(kayit["pre_pick_joints"]), list(kayit["pre_place_joints"]))],
            boyut, kavrama_ofseti, float(kayit["approach_mm"]), derinlik,
            {"grip_speed": int(secim(istek.grip_speed, kayit.get("grip_speed"), 50)),
             "grip_force": int(secim(istek.grip_force, kayit.get("grip_force"), 5)),
             "speed_pct": hiz, "accel_pct": ivme,
             "target_tcp": kayit.get("target_tcp"),
             "saved_name": ad,
             "object": _pp_nesne_bilgisi(arm, boyut, kavrama_ofseti, derinlik,
                                         kayit.get("place_surface_mm"),
                                         kayit.get("place_point_mm"))})

    return _is_baslat("preview", "Plan pick & place chain", calistir)


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


@app.post("/api/pickplace/run_at_pose")
def pp_pozda_calistir(istek: PPPozdaCalistir) -> dict[str, Any]:
    """Disaridan (kamera/vision) gelen yaklasim pozlarindan MTC pick&place
    calistirir - kayitli hareket gerekmez. bkz. PPPozdaCalistir docstring'i
    icin guvenlik notu.

    Bu yolda ONIZLEME YOK (cagrildigi anda hareket eder), ama cift secimi
    /api/pickplace/preview ile AYNI: birkac kol durusu icin tum zincir
    planlanip TOPLAM EKLEM YOLU en kisa olan secilir (23 Eylul)."""
    from fairino_cobot.ik_secimi import uygun_ciftleri_bul
    from fairino_cobot.movecobot import GRIPPER_STROKE_M, run_pick_place
    from fairino_cobot.mtc_yolu import asamalari_birlestir, toplam_eklem_yolu_deg

    def calistir(job):
        arm = oturum.arm
        gripper = oturum.gripper

        _, sirali_pick, red_pick = arm._ik_adaylari(*istek.pick_approach)
        if red_pick is not None or not sirali_pick:
            return {"success": False, "reason": "pick_ik_failed",
                    "details": red_pick.get("details") if red_pick else "no candidates"}

        _, sirali_place, red_place = arm._ik_adaylari(*istek.place_approach)
        if red_place is not None or not sirali_place:
            return {"success": False, "reason": "place_ik_failed",
                    "details": red_place.get("details") if red_place else "no candidates"}

        pick_adaylari = [a for a, _ in sirali_pick]
        place_adaylari = [a for a, _ in sirali_place]
        job.log.append(f"IK candidates: {len(pick_adaylari)} pick, {len(place_adaylari)} place")

        # Kavrama genisligi gercekte gripper kapaninca olculuyor. Verilmediyse
        # on planlamada run_pick_place'in gripper konumu okunamayinca (0)
        # kullandigi deger alinir: tam strok.
        obj_w = istek.object_w_mm if istek.object_w_mm else GRIPPER_STROKE_M * 1000
        boyut = [obj_w, istek.object_d_mm, istek.object_h_mm]
        yaklasma = istek.approach_mm

        # PickPlace OTURUM BASI TEK SEFERLIK kuruldu (bkz. session.py
        # _baglanmayi_dene) - burada YENIDEN KURULMAZ (kaynak sizdirip
        # SEGFAULT riski tasir, bkz. pickplace_olustur docstring'i).
        pp = oturum.pp
        if pp is None:
            return {"success": False, "reason": "mtc_init_failed",
                    "details": "MTC was not initialized at startup (see server log)."}
        # Hiz/ivme = Ayarlar penceresi (ust cubuk), diger hareketlerle ayni.
        # arm.speed_pct acilis varsayilanidir, pencere onu degistirmez.
        hiz, ivme = _kol_hizi()
        pp.hizlari_guncelle(hiz, ivme)

        # Tum zincir (yaklas, kaldir, tasi, in, geri cekil) robot HAREKET
        # ETMEDEN ve gripper kapanmadan ONCE planlanir. Eskiden bastan sona
        # planlanabilen ILK cift calistiriliyordu; "ilk uygun" = "en kisa"
        # DEGIL (21 Eylul: pick j1=163 ile place j1=-54 secilmisti). Simdi
        # max_options kadar cift planlanip en kisasi secilir.
        zincirler: list[list[Any]] = []

        def zincir_planla(p, q):
            if not pp.zinciri_onceden_planla(p, q, yaklasma, boyut,
                                             arm.tcp_offset_m, istek.grasp_depth_mm):
                return False
            zincirler.append(list(pp.son_zincir))
            return True

        ciftler, deneme = uygun_ciftleri_bul(
            pick_adaylari, place_adaylari,
            lambda p: pp.alma_planla(p, yaklasma),
            zincir_planla,
            en_fazla=istek.max_options,
            en_fazla_zincir_denemesi=4 * istek.max_options,
        )
        job.log.append(f"pre-planning: {deneme['alma']} approach + {deneme['zincir']} chain plans "
                       f"-> {len(ciftler)} feasible pair(s)")
        if not ciftler:
            return {"success": False, "reason": "no_feasible_pick_place_pair",
                    "details": ("no pick/place arm configuration pair could be planned end to "
                                "end. Robot did not move.")}

        # EN KISA YOL: yolun toplam eklem donusu (bkz. toplam_eklem_yolu_deg).
        maliyetler = []
        for sira, zincir in enumerate(zincirler):
            noktalar, _zamanlar, _bilgi = asamalari_birlestir(
                [pp.cozum_asamalari(cozum) for _, cozum in zincir], bosluk_s=0.8)
            maliyet = toplam_eklem_yolu_deg(noktalar)
            maliyetler.append(maliyet)
            job.log.append(f"pair {sira + 1}: total joint travel {maliyet:.0f} deg "
                           f"(pre_pick j1={ciftler[sira][0][0]:.1f})")
        en_iyi = min(range(len(ciftler)), key=lambda i: maliyetler[i])
        pre_pick, pre_place = ciftler[en_iyi]
        job.log.append(f"chosen: pair {en_iyi + 1} ({maliyetler[en_iyi]:.0f} deg total)")
        job.log.append(f"pre_pick_joints: {[round(v, 2) for v in pre_pick]}")
        job.log.append(f"pre_place_joints: {[round(v, 2) for v in pre_place]}")
        if istek.sadece_planla:
            job.log.append("plan only: the robot was not moved.")
            return {"success": True}

        cfg = {
            "pre_pick_joints": pre_pick,
            "pre_place_joints": pre_place,
            "approach_mm": yaklasma,
            "grip_speed": istek.grip_speed,
            "grip_force": istek.grip_force,
            "object_dh_mm": [istek.object_d_mm, istek.object_h_mm],
            "kavrama_derinligi_mm": istek.grasp_depth_mm,
        }
        # DIKKAT: run_pick_place asamalari YENIDEN PLANLAR (secilen ciftin
        # eklem hedeflerinden). MTC plan sonucu deterministik olmadigi icin
        # (bkz. GUNCEL_DURUM 7, 21 Eylul) yurutulen yol, maliyeti yukarida
        # olculen yolun AYNISI olmayabilir - hedef duruslar ayni, ara yol
        # farkli olabilir. Onizlenen yolun AYNEN yurutulmesi isteniyorsa
        # /api/pickplace/preview + /preview/run kullanilir.
        ok = run_pick_place(arm, gripper, cfg, sor=False, onay_iste=False, pp=pp)
        return {"success": bool(ok)}

    return _is_baslat("pickplace", "MTC pick & place at given pose", calistir)


@app.get("/api/vision/info")
def vizyon_bilgi() -> dict[str, Any]:
    """Vision API'sinin durumu (el-goz kalibreli mi, tespit modu) + hazir
    birakma slotlari (bkz. fairino_cobot.vizyon). Robota dokunmaz."""
    from fairino_cobot import vizyon
    try:
        return vizyon.bilgi()
    except vizyon.VizyonHatasi as e:
        raise HTTPException(status_code=503, detail=str(e)) from e


@app.post("/api/vision/detect")
def vizyon_algila() -> dict[str, Any]:
    """Kameradan (stereo + geometrik tespit) nesneleri ister. ROBOTA
    DOKUNMAZ - motion_lock ALINMAZ (bkz. vizyon.py modul notu), bu yuzden
    jog/onizleme/pick&place ile CAKISMAZ. Senkron: FastAPI'nin kendi thread
    pool'unda calisir (plain def), WebSocket telemetriyi bloklamaz. Stereo+
    tespit bir kac saniye surebilir - arayuz "Detect" butonunu bu sure
    disable eder."""
    from fairino_cobot import vizyon
    try:
        return vizyon.algila()
    except vizyon.VizyonHatasi as e:
        raise HTTPException(status_code=503, detail=str(e)) from e


def _pp_basarisiz(reason: str, details: str) -> dict[str, Any]:
    """Pick & place planlamasi basarisiz: sonuc da KAYDEDILIR (Path preview'daki
    "success: false de kaydedilir" deseni, bkz. yollari_onizle) - kullanici
    sonucu panelde gorsun, yalnizca genel log'da degil."""
    oturum.pp_onizleme_kaydet(
        {"success": False, "reason": reason, "details": details, "options": []},
        [], None)
    return {"success": False, "reason": reason, "details": details}


def _pp_zincirleri_planla_ve_sakla(job, arm, pp, ciftler, boyut, kavrama_ofseti,
                                   yaklasma_mm, derinlik_mm, bilgi: dict[str, Any]) -> dict[str, Any]:
    """Verilen (pre_pick, pre_place) eklem ciftleri icin TAM zinciri robot
    HAREKET ETMEDEN planlar, her birini mtc_yolunu_denetle'den (zemin +
    self-collision, tasinan nesne dahil) gecirir, en kisa yol basta olacak
    sekilde siralayip pick & place onizlemesi olarak saklar.

    Yeni plan (pp_onizle) ve kayitli zincirin oynatilmasi (oynat) AYNI yoldan
    gecer: kayittan oynatmada da once onizleme, sonra /preview/run.

    bilgi: ozete aynen eklenen alanlar (grip_speed, grip_force, speed_pct,
    accel_pct, target_tcp, object, saved_name...)."""
    from fairino_cobot.mtc_yolu import asamalari_birlestir, tasima_araligi, toplam_eklem_yolu_deg
    from fairino_cobot.yol_onizleme import secenek_ozeti

    # Her uygun cift bulundugunda O ANIN zinciri saklanir: pp.son_zincir
    # bir sonraki planlamada UZERINE YAZILIR, sonradan okunamaz.
    planlanan: list[tuple[list[float], list[float], list[Any]]] = []
    for pre_pick, pre_place in ciftler:
        job.log.append(f"  chain: pre_pick j1={pre_pick[0]:.1f}, pre_place j1={pre_place[0]:.1f}")
        if pp.zinciri_onceden_planla(pre_pick, pre_place, yaklasma_mm, boyut,
                                     kavrama_ofseti, derinlik_mm):
            planlanan.append((pre_pick, pre_place, list(pp.son_zincir)))
    if not planlanan:
        return _pp_basarisiz("no_feasible_pick_place_pair",
                             "MTC found no pick & place solution for this object and place "
                             "pose. Robot did not move.")

    kol_baslangici = arm.get_current_state()["joints_deg"]
    # (maliyet, ozet, zincir) - maliyet = yolun toplam eklem donusu.
    adaylar: list[tuple[float, dict[str, Any], list[Any]]] = []
    for pre_pick, pre_place, zincir in planlanan:
        asamalar = [pp.cozum_asamalari(cozum) for _, cozum in zincir]
        noktalar, zamanlar, asama_bilgisi = asamalari_birlestir(asamalar, bosluk_s=0.8)
        # Tasinan nesne de kontrole giriyor (zemin + self-collision):
        # kutu, kavrama derinligine gore parmak uclarinin ALTINA
        # yerlestirilir (MTC sahnesindeki kutuyla ayni yer).
        problem = arm.mtc_yolunu_denetle(
            noktalar, {"boyut_mm": boyut,
                       "kavrama_derinligi_mm": derinlik_mm,
                       "ofset_m": kavrama_ofseti,
                       "tasima_araligi": tasima_araligi(asama_bilgisi)}) if noktalar else None
        # Yolun GERCEK baslangici: zincirin ilk gorevi CurrentState'ten
        # (kolun plan anindaki durusu) basliyor - pre_pick "on-poza git"
        # asamasinin HEDEFI. Eskiden buraya pre_pick veriliyordu; hem
        # "en cok donen eklem" yanlis noktaya gore olculuyordu hem de
        # /preview/run'daki bayatlik kontrolu kolu pre_pick'te sanip
        # neredeyse her zaman preview_stale veriyordu.
        yol_baslangici = noktalar[0] if noktalar else list(kol_baslangici)
        maliyet = toplam_eklem_yolu_deg(noktalar)
        ozet = secenek_ozeti(len(adaylar) + 1, "", "MTC", yol_baslangici, noktalar,
                             zamanlar, problem, noktalar[-1] if noktalar else None)
        ozet["stages"] = asama_bilgisi
        ozet["start_joints"] = [round(float(v), 3) for v in yol_baslangici]
        ozet["pre_pick_joints"] = [round(float(v), 2) for v in pre_pick]
        ozet["pre_place_joints"] = [round(float(v), 2) for v in pre_place]
        ozet["total_deg"] = None if not math.isfinite(maliyet) else round(maliyet, 1)
        ozet["grasp_rz"] = None
        adaylar.append((maliyet, ozet, zincir))

    # EN KISA YOL BASA: once calistirilabilir olanlar (reddedilen bir yol
    # kisa olsa da secilemez), sonra toplam eklem yolu kucukten buyuge.
    adaylar.sort(key=lambda a: (not a[1]["ok"], a[0]))
    secenekler = []
    for sira, (_, ozet, _) in enumerate(adaylar, start=1):
        ozet["index"] = sira
        ozet["label"] = ("Shortest chain" if sira == 1 and ozet["ok"]
                         else f"Alternative chain {sira}")
        secenekler.append(ozet)
    for s in secenekler:
        job.log.append(f"option {s['index']}: {'runnable' if s['ok'] else 'REJECTED - ' + str(s['reason'])}"
                       f" · total {s['total_deg']} deg · {s['duration_s']} s")

    no = oturum.pp_onizleme_kaydet(
        {"success": True, "kind": "pickplace",
         "start_joints": secenekler[0]["start_joints"],
         "options": secenekler,
         # Kayit (/api/pickplace/save_last) icin: calistirilan zincirin
         # yeniden planlanabilmesi bunlarla + secenegin pre_pick/pre_place'i.
         "approach_mm": yaklasma_mm, "grasp_depth_mm": derinlik_mm,
         **bilgi},
        [z for _, _, z in adaylar], pp)
    job.log.append(f"stored as preview #{no}")
    return {"success": True}


def _pp_nesne_bilgisi(arm, boyut, kavrama_ofseti, derinlik_mm, yuzey_mm, birakma_mm) -> dict[str, Any]:
    """3D gorunum icin (yalnizca gorsel): nesnenin kavranip tasinmasi ve
    birakma yeri. Kutu PLANLAYICININ kutusu: MTC ile ayni fonksiyon
    (safety.kavranan_nesne_kutusu_merkezi), uc linkin eksenlerinde."""
    from fairino_cobot.movecobot import GRIPPER_STROKE_M
    from fairino_cobot.safety import kavranan_nesne_kutusu_merkezi

    return {
        "size_mm": [float(v) for v in boyut],
        "tip_link": arm.tip_link,
        "grip_point_tip_m": [float(v) for v in kavrama_ofseti],
        "center_tip_m": [float(v) for v in kavranan_nesne_kutusu_merkezi(
            kavrama_ofseti, boyut[2], derinlik_mm)],
        "grasp_depth_mm": derinlik_mm,
        "place_surface_mm": None if yuzey_mm is None else [float(v) for v in yuzey_mm],
        "place_point_mm": None if birakma_mm is None else [float(v) for v in birakma_mm],
        "stroke_mm": GRIPPER_STROKE_M * 1000.0,
    }


@app.post("/api/pickplace/preview")
def pp_onizle(istek: PPOnizleme) -> dict[str, Any]:
    """Tespit edilen bir nesne icin TAM pick & place zincirini (yaklas ->
    kavra -> kaldir -> tasi -> in -> birak -> geri cekil) robot HAREKET
    ETMEDEN planlar ve 3D onizleme icin (points/times) doner.

    run_at_pose'un aksine burada TUM yol noktalari elde oldugu icin
    mtc_yolunu_denetle ile zemin/self-collision kontrolunden de gecirilir -
    ihlal varsa secenek 'ok': False doner (yol yine gosterilir, calistirilamaz).

    BIRDEN FAZLA SECENEK (23 Eylul, kullanici istegi): farkli kol duruslari
    icin en cok max_options tam zincir planlanir ve TOPLAM EKLEM YOLU'na gore
    siralanir - 1. secenek EN KISA yoldur, arayuz onu secili getirir. Diger
    secenekler listede durur, kullanici baskasini secebilir.
    """
    from fairino_cobot.movecobot import GRIPPER_STROKE_M
    from fairino_cobot.tcp_cercevesi import birakma_noktasi

    def calistir(job):
        arm = oturum.arm

        if istek.grasp_depth_mm >= istek.object_h_mm:
            return _pp_basarisiz("invalid_grasp_depth",
                                 f"Grasp depth {istek.grasp_depth_mm:g} mm must be smaller than the "
                                 f"object height {istek.object_h_mm:g} mm.")
        obj_w = istek.object_w_mm if istek.object_w_mm else GRIPPER_STROKE_M * 1000
        boyut = [obj_w, istek.object_d_mm, istek.object_h_mm]
        # MTC'nin ik_frame'i = parmak uclari; kutu ve derinlik de ona gore.
        kavrama_cercevesi = arm.kavrama_ik_cercevesi()
        kavrama_ofseti = kavrama_cercevesi[0]
        birakma = birakma_noktasi(istek.place_pose[:3], istek.object_h_mm, istek.grasp_depth_mm)
        job.log.append(f"place: surface z={istek.place_pose[2]:.1f} mm -> fingertips "
                       f"z={birakma[2]:.1f} mm")

        # PickPlace OTURUM BASI TEK SEFERLIK kuruldu (bkz. session.py
        # _baglanmayi_dene) - burada YENIDEN KURULMAZ. "Plan pick & place"
        # butonuna hizli art arda basmak eskiden her tikta yeni bir rclcpp
        # dugumu aciyordu; eskileri temizlenmeden birikince class_loader
        # SEGFAULT'a yol acti (22 Eylul canli olayi) - bkz. pickplace_olustur
        # docstring'i.
        pp = oturum.pp
        if pp is None:
            return _pp_basarisiz("mtc_init_failed",
                                 "MTC was not initialized at startup (see server log).")
        # Hiz/ivme = Ayarlar penceresi (ust cubuk), diger hareketlerle ayni.
        # arm.speed_pct acilis varsayilanidir, pencere onu degistirmez.
        hiz, ivme = _kol_hizi()
        pp.hizlari_guncelle(hiz, ivme)

        # STANDART MTC SECIMI (24 Eylul, kullanici istegi): kol duruslarini
        # robotun SDK IK'sindan toplayip ciftlemek yerine MTC'nin kendi
        # yontemi - tek gorev, iki gripper yonu (paralel gripper rz ve
        # rz+180 ayni kavrama) x MoveIt IK cozumleri, Connect ile planlama,
        # maliyete (yol uzunlugu) gore sirali cozumler. Bkz.
        # pickplace.standart_ciftleri_bul. Birakma yonu kavramayla ayni
        # iki yonden secilir (nesne kavrandigi yonelimle birakilir).
        rz = istek.grasp_rpy[2]
        rz_listesi = [_aci_norm(rz), _aci_norm(rz + 180.0)]
        adaylar_std = pp.standart_ciftleri_bul(
            istek.grasp_point, birakma, rz_listesi, istek.approach_mm, boyut,
            kavrama_cercevesi, istek.grasp_depth_mm, en_fazla=istek.max_options)
        job.log.append(f"standard MTC: {len(adaylar_std)} cheapest arm configuration pair(s) "
                       f"(gripper rz {rz_listesi[0]:.1f} / {rz_listesi[1]:.1f})")
        for _, _, maliyet in adaylar_std:
            job.log.append(f"  MTC cost {maliyet:.3f}")

        return _pp_zincirleri_planla_ve_sakla(
            job, arm, pp, [(p, q) for p, q, _ in adaylar_std], boyut, kavrama_ofseti,
            istek.approach_mm, istek.grasp_depth_mm,
            {"grip_speed": istek.grip_speed, "grip_force": istek.grip_force,
             "speed_pct": hiz, "accel_pct": ivme,
             # 3D gorunumdeki yesil hedef: kavrama noktasi (parmak uclari).
             "target_tcp": dict(zip(("x", "y", "z", "rx", "ry", "rz"),
                                    [*istek.grasp_point, *istek.grasp_rpy])),
             "object": _pp_nesne_bilgisi(arm, boyut, kavrama_ofseti, istek.grasp_depth_mm,
                                         istek.place_pose[:3], birakma)})

    return _is_baslat("preview", "Plan pick & place chain", calistir)


@app.get("/api/pickplace/preview")
def pp_onizleme_oku() -> dict[str, Any]:
    """Son pick & place onizlemesi (Task/Solution nesneleri olmadan). Yoksa
    bos sozluk."""
    o = oturum.pp_onizleme
    return o["ozet"] if o else {}


@app.delete("/api/pickplace/preview")
def pp_onizleme_sil() -> dict[str, Any]:
    oturum.pp_onizleme = None
    return {}


def _pp_kayit_adayi(genel: dict[str, Any], secenek: dict[str, Any]) -> dict[str, Any] | None:
    """Onizlenen zincirin bir seceneginden kayit ("pick_place_chain")
    icerigi. Baslangic durusu KAYDEDILMEZ (kullanici istegi, 25 Eylul):
    oynatmada zincir kolun O ANKI yerinden yeniden planlanir. Gerekli alan
    eksikse (eski onizleme) None."""
    nesne = genel.get("object") or {}
    try:
        return {
            "pre_pick_joints": list(secenek["pre_pick_joints"]),
            "pre_place_joints": list(secenek["pre_place_joints"]),
            "approach_mm": float(genel["approach_mm"]),
            "grasp_depth_mm": float(genel["grasp_depth_mm"]),
            "object_size_mm": [float(v) for v in nesne["size_mm"]],
            "grip_speed": int(genel["grip_speed"]),
            "grip_force": int(genel["grip_force"]),
            "speed_pct": genel.get("speed_pct"),
            "accel_pct": genel.get("accel_pct"),
            # Yalnizca 3D gorunum icin (oynatmada planlamayi etkilemez).
            "target_tcp": genel.get("target_tcp"),
            "place_surface_mm": nesne.get("place_surface_mm"),
            "place_point_mm": nesne.get("place_point_mm"),
        }
    except (KeyError, TypeError, ValueError):
        return None


@app.get("/api/pickplace/last")
def pp_son_calisan_oku() -> dict[str, Any]:
    """Kaydedilebilir son pick & place calismasi; yoksa bos sozluk."""
    return dict(oturum.pp_son_calisan or {})


@app.post("/api/pickplace/save_last")
def pp_son_calisani_kaydet(istek: PPSonKaydet) -> dict[str, Any]:
    """Son BASARIYLA calistirilan onizlenmis pick & place zincirini kayitli
    hareket olarak yazar ("pick_place_chain"). Robota bir sey gondermez.

    Oynatma (/api/moves/{ad}/play) robotu DOGRUDAN hareket ettirmez: zincir
    ayni pre_pick/pre_place duruslariyla kolun o anki yerinden yeniden
    planlanir, ayni kontrollerden gecer ve Path preview'da gosterilir;
    robot yalnizca "Run" ile hareket eder."""
    from fairino_cobot.movecobot import load_moves, save_moves, valid_move_name

    son = oturum.pp_son_calisan
    if not son:
        raise HTTPException(status_code=404,
                            detail="No completed pick & place run to save. Run a planned one first.")
    ok, hata = valid_move_name(istek.name)
    if not ok:
        raise HTTPException(status_code=400, detail=f"Invalid name: {hata}")
    moves = load_moves()
    if istek.name in moves and not istek.overwrite:
        raise HTTPException(status_code=409, detail=f"'{istek.name}' already exists.")
    kayit = {k: v for k, v in son.items() if k != "ran_at"}
    moves[istek.name] = {"created": datetime.now().isoformat(timespec="seconds"),
                         "type": "pick_place_chain", **kayit}
    save_moves(moves)
    return {"saved": istek.name}


@app.post("/api/pickplace/preview/run")
def pp_onizlemeyi_calistir(istek: PPOnizlemeCalistir) -> dict[str, Any]:
    """Onizlenen pick & place zincirinin SECILEN secenegini AYNEN calistirir
    (yeniden planlamaz, bkz. pickplace.zinciri_calistir). ROBOTU HAREKET
    ETTIRIR (arayuz once onay ister). Gripper: ac -> [zincir 1: yaklas] ->
    kavra -> [zincir 2: kaldir+tasi+in] -> ac -> [zincir 3: ayir+geri cekil].

    Hangi secenek calistirilacagi TARAYICIDAN DEGIL sunucudan gelir: istek
    yalnizca onizleme id'si + secenek numarasi tasir, yol sunucuda saklidir
    (onizleme_calistir ile ayni sozlesme)."""
    from fairino_cobot.gripper_controller import GripperError
    from fairino_cobot.yol_onizleme import ONIZLEME_BASLANGIC_TOLERANSI_DEG, baslangic_farki

    _hazir()
    o = oturum.pp_onizleme
    if o is None:
        raise HTTPException(status_code=404, detail="No pick & place preview. Plan it first.")
    if o["id"] != istek.id:
        raise HTTPException(status_code=409, detail="This preview is out of date. Plan it again.")
    secenekler = o["ozet"].get("options") or []
    if not secenekler:
        raise HTTPException(status_code=409, detail=o["ozet"].get("details")
                            or o["ozet"].get("reason") or "No plan to run.")
    if not 1 <= istek.option <= len(secenekler):
        raise HTTPException(status_code=404,
                            detail=f"No option {istek.option} in this plan "
                                   f"({len(secenekler)} available).")
    ozet = secenekler[istek.option - 1]
    if not ozet["ok"]:
        raise HTTPException(
            status_code=409,
            detail=f"This plan was rejected ({ozet.get('reason') or 'no plan'}); it cannot be run.")
    baslangic = ozet["start_joints"]
    zincir = o["zincirler"][istek.option - 1]
    pp = o["pp"]
    g_speed, g_force = o["ozet"]["grip_speed"], o["ozet"]["grip_force"]
    # Basariyla biterse kaydedilebilsin (/api/pickplace/save_last).
    kayit_adayi = _pp_kayit_adayi(o["ozet"], ozet)

    def calistir(job):
        oturum.pp_son_calisan = None
        sonuc = _pp_zinciri_yurut(job)
        if sonuc.get("success") and kayit_adayi is not None:
            oturum.pp_son_calisan = {**kayit_adayi,
                                     "ran_at": datetime.now().isoformat(timespec="seconds")}
        return sonuc

    def _pp_zinciri_yurut(job):
        arm = oturum.arm
        gripper = oturum.gripper
        with arm.sdk_lock:
            simdiki = list(arm.sdk.GetActualJointPosDegree()[1])
        fark = baslangic_farki(baslangic, simdiki)
        if fark > ONIZLEME_BASLANGIC_TOLERANSI_DEG:
            return {"success": False, "reason": "preview_stale",
                    "details": (f"the arm moved {fark:.1f} degrees since the plan was made "
                                f"(limit {ONIZLEME_BASLANGIC_TOLERANSI_DEG:g}). Plan again - "
                                f"robot did not move.")}

        job.log.append(f"Run previewed pick & place chain (option {istek.option}/{len(secenekler)})")
        try:
            gripper.move(position=0, speed=g_speed, force=g_force)
        except GripperError as e:
            job.log.append(f"WARNING: gripper open failed: {e}")
        if not pp.zinciri_calistir(0, zincir):
            return {"success": False, "reason": "chain_failed", "details": "stage 1/3 (approach)"}

        try:
            sonuc = gripper.grip(speed=g_speed, force=g_force)
            job.log.append(f"gripped={sonuc['gripped']} position={sonuc['position_pct']}")
        except GripperError as e:
            return {"success": False, "reason": "grip_failed", "details": str(e)}

        if not pp.zinciri_calistir(1, zincir):
            return {"success": False, "reason": "chain_failed", "details": "stage 2/3 (transfer)"}

        try:
            gripper.move(position=0, speed=g_speed, force=g_force)
        except GripperError as e:
            job.log.append(f"WARNING: gripper open failed: {e}")

        if not pp.zinciri_calistir(2, zincir):
            return {"success": False, "reason": "chain_failed", "details": "stage 3/3 (retreat)"}
        return {"success": True}

    cevap = _is_baslat("pickplace", "Run previewed pick & place chain", calistir)
    # Tek kullanimlik: ayni zincir ikinci kez yurutulmez (kol artik baska yerde).
    oturum.pp_onizleme = None
    return cevap


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

"""Web arayuzundeki 3D kol gorunumu icin robot modeli (SAF, ROS gerektirmez).

Model, MoveIt'in KULLANDIGI URDF metninden (CobotArm.robot_description_urdf)
cikarilir; ayri bir kopya tutulmaz. Boylece gorunum planlayicinin modeliyle
ayni kalir. Mesh dosyalari (STL) ROS paketinin kurulum klasorunden sunulur;
robot PC'sinde internet olmayabilir.

GUVENLIK: tarayici dosya YOLU gondermez, yalnizca bu modulun urettigi sira
numarasini ister. Yol URDF'ten gelir ve paket klasorunun disina cikamaz.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Callable
from pathlib import Path


def _sayilar(metin: str | None, varsayilan: list[float]) -> list[float]:
    if not metin:
        return list(varsayilan)
    return [float(v) for v in metin.split()]


def _carpisma_gorselle_ayni(link: ET.Element) -> bool:
    """Linkin carpisma geometrisi gorsel mesh'inin AYNISI mi (ayni dosya,
    olcek ve origin)? Gorunum link padding kabugunu yalnizca bu linklere
    cizer: MoveIt padding'i carpisma geometrisine uygular, gorunum ise gorsel
    mesh'i yukler. Carpisma geometrisi olmayan (gripper modeli gibi yalnizca
    gorsel) linkler False."""
    gorsel, carpisma = link.find("visual"), link.find("collision")
    if gorsel is None or carpisma is None:
        return False
    gm, cm = gorsel.find("geometry/mesh"), carpisma.find("geometry/mesh")
    if gm is None or cm is None or not gm.get("filename"):
        return False
    if gm.get("filename") != cm.get("filename"):
        return False
    if _sayilar(gm.get("scale"), [1.0] * 3) != _sayilar(cm.get("scale"), [1.0] * 3):
        return False
    go, co = gorsel.find("origin"), carpisma.find("origin")
    for anahtar in ("xyz", "rpy"):
        a = _sayilar(go.get(anahtar) if go is not None else None, [0.0] * 3)
        b = _sayilar(co.get(anahtar) if co is not None else None, [0.0] * 3)
        if a != b:
            return False
    return True


def urdf_ozeti(urdf_metni: str) -> dict:
    """URDF'ten linklerin gorsel mesh'i ve eklemlerin kinematik zinciri.

    Birimler URDF'teki gibi: metre, radyan. rpy sabit eksenli X-Y-Z (URDF
    tanimi): R = Rz(yaw) * Ry(pitch) * Rx(roll).
    """
    kok = ET.fromstring(urdf_metni)
    linkler = []
    for link in kok.findall("link"):
        kayit = {"name": link.get("name"), "mesh_uri": None, "xyz": [0.0, 0.0, 0.0],
                 "rpy": [0.0, 0.0, 0.0], "scale": [1.0, 1.0, 1.0], "rgba": None}
        gorsel = link.find("visual")
        if gorsel is not None:
            mesh = gorsel.find("geometry/mesh")
            if mesh is not None:
                kayit["mesh_uri"] = mesh.get("filename")
                kayit["scale"] = _sayilar(mesh.get("scale"), [1.0, 1.0, 1.0])
            origin = gorsel.find("origin")
            if origin is not None:
                kayit["xyz"] = _sayilar(origin.get("xyz"), [0.0, 0.0, 0.0])
                kayit["rpy"] = _sayilar(origin.get("rpy"), [0.0, 0.0, 0.0])
            renk = gorsel.find("material/color")
            if renk is not None:
                kayit["rgba"] = _sayilar(renk.get("rgba"), [1.0, 1.0, 1.0, 1.0])
        kayit["collision_mesh_is_visual"] = _carpisma_gorselle_ayni(link)
        linkler.append(kayit)

    eklemler = []
    for eklem in kok.findall("joint"):
        origin = eklem.find("origin")
        eksen = eklem.find("axis")
        # Gripper parmaklari (prismatic + mimic): 3D gorunum acikligi limitten
        # ve mimic iliskisinden hesaplar.
        limit = eklem.find("limit")
        mimic = eklem.find("mimic")
        eklemler.append({
            "limit": None if limit is None else {
                "lower": float(limit.get("lower", 0.0)), "upper": float(limit.get("upper", 0.0))},
            "mimic": None if mimic is None else {
                "joint": mimic.get("joint"),
                "multiplier": float(mimic.get("multiplier", 1.0)),
                "offset": float(mimic.get("offset", 0.0))},
            "name": eklem.get("name"),
            "type": eklem.get("type"),
            "parent": eklem.find("parent").get("link"),
            "child": eklem.find("child").get("link"),
            "xyz": _sayilar(origin.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0]),
            "rpy": _sayilar(origin.get("rpy") if origin is not None else None, [0.0, 0.0, 0.0]),
            "axis": _sayilar(eksen.get("xyz") if eksen is not None else None, [1.0, 0.0, 0.0]),
        })
    return {"name": kok.get("name"), "links": linkler, "joints": eklemler}


def parmak_acikligi_m(urdf_metni: str, eklem: str, konum_yuzde: float) -> float | None:
    """Gripper'in okudugu konumdan (0 = tam acik, 100 = tam kapali;
    gripper_controller.move) parmak ekleminin degeri (m). Eklem URDF'te yoksa
    ya da limiti yoksa None. Deger limit araligina kirpilir."""
    for j in urdf_ozeti(urdf_metni)["joints"]:
        if j["name"] == eklem and j["limit"]:
            alt, ust = j["limit"]["lower"], j["limit"]["upper"]
            oran = min(max((100.0 - float(konum_yuzde)) / 100.0, 0.0), 1.0)
            return alt + oran * (ust - alt)
    return None


def paket_uri_coz(uri: str | None) -> tuple[str, str] | None:
    """package://paket/goreli/yol -> (paket, goreli/yol). Baska bicim (file://,
    mutlak yol, bos) None: yalnizca ROS paketlerinden dosya sunulur."""
    on = "package://"
    if not uri or not uri.startswith(on):
        return None
    paket, _, yol = uri[len(on):].partition("/")
    if not paket or not yol:
        return None
    return paket, yol


def paket_ici_dosya(paket_klasoru: str | Path, goreli_yol: str) -> Path | None:
    """Paket klasorunun ICINDEKI dosya; '..' ile disari cikiyorsa None."""
    kok = Path(paket_klasoru).resolve()
    hedef = (kok / goreli_yol).resolve()
    if kok not in hedef.parents:
        return None
    return hedef


def model_paketi(urdf_metni: str,
                 paket_klasoru_bul: Callable[[str], str | None]) -> tuple[dict, list[Path]]:
    """Arayuze gidecek model ozeti ve sunulabilecek mesh dosyalari.

    paket_klasoru_bul: paket adi -> kurulum (share) klasoru ya da None
    (container'da ament_index; testte sahte). Her linkin "mesh" alani
    /api/model/mesh/<sira> adresi ya da None (bulunamadi / STL degil).
    """
    ozet = urdf_ozeti(urdf_metni)
    dosyalar: list[Path] = []
    for link in ozet["links"]:
        link["mesh"] = None
        coz = paket_uri_coz(link.pop("mesh_uri"))
        if coz is None:
            continue
        klasor = paket_klasoru_bul(coz[0])
        if not klasor:
            continue
        yol = paket_ici_dosya(klasor, coz[1])
        if yol is None or yol.suffix.lower() != ".stl" or not yol.is_file():
            continue
        link["mesh"] = f"/api/model/mesh/{len(dosyalar)}"
        dosyalar.append(yol)
    return ozet, dosyalar

"""Web yol onizlemesi HTTP katmani. ROBOT, ROS2 veya SDK GEREKTIRMEZ.

Test edilen: onizleme robotu hareket ettirmiyor, plan nesneleri arayuze
gitmiyor, yalnizca guncel ve kontrolden gecmis secenek calistiriliyor,
calistirma sakli plani (yeniden planlamadan) kola veriyor ve tek kullanimlik.
"""

import json
import threading
import time

import pytest

server = pytest.importorskip(
    "fairino_cobot.webapp.server",
    reason="webapp bagimliliklari (fastapi) yok",
)

URDF = """<?xml version="1.0"?>
<robot name="deneme_robot">
  <link name="base_link">
    <visual><geometry><mesh filename="package://desc/meshes/base_link.STL"/></geometry></visual>
  </link>
  <link name="wrist3_link">
    <visual><geometry><mesh filename="package://desc/meshes/yok.STL"/></geometry></visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="wrist3_link"/><axis xyz="0 0 1"/>
  </joint>
</robot>
"""


class SahtePlan:
    """plan_result yerine gecer. Arayuze (JSON) sizmamali."""


def _secenek(sira, ok, reason=None):
    return {"index": sira, "label": f"Option {sira}", "method": "PTP", "ok": ok,
            "reason": reason, "details": None, "max_joint": "j1", "max_deg": 1.0,
            "duration_s": 1.0, "goal_joints": [0.0] * 6, "points": [[0.0] * 6], "times": [0.0]}


class SahteKol:
    """CobotArm yerine gecer. move_* metodu YOK: onizleme robotu hareket
    ettirmeye kalkarsa AttributeError ile is basarisiz olur."""

    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
    tip_link = "wrist3_link"
    tcp_offset_m = (0.0, 0.0, 0.2)
    gripper_kutusu = {"link": "wrist3_link", "xyz": [0.0, 0.0, 0.16], "size": [0.095, 0.06, 0.13]}
    robot_description_urdf = URDF

    def __init__(self):
        self.cagrilar = []
        self.stop_requested = threading.Event()
        self.planlar = [SahtePlan(), None]

    def yol_secenekleri(self, tur, degerler, prefer_lin=True, speed_pct=None, accel_pct=None):
        self.cagrilar.append(("secenekler", tur, list(degerler), prefer_lin, speed_pct, accel_pct))
        return {"success": True, "kind": tur, "start_joints": [1.0] * 6,
                "options": [_secenek(1, True), _secenek(2, False, "floor_collision")],
                "_planlar": list(self.planlar)}

    def onizlenen_yolu_yurut(self, plan, baslangic):
        self.cagrilar.append(("yurut", plan, list(baslangic)))
        return {"success": True}


@pytest.fixture
def kol(monkeypatch):
    sahte = SahteKol()
    monkeypatch.setattr(server.oturum, "arm", sahte)
    monkeypatch.setattr(server.oturum, "ready", True)
    monkeypatch.setattr(server.oturum, "ayarlar", {"speed_pct": 20.0, "accel_pct": 15.0})
    monkeypatch.setattr(server.oturum, "onizleme", None)
    monkeypatch.setattr(server.oturum, "model_onbellek", None)
    return sahte


def _isi_bekle():
    for _ in range(50):
        j = server.oturum.job_dict()
        if j and not j["running"]:
            return j
        time.sleep(0.05)
    raise AssertionError("is bitmedi")


def _onizle(kol):
    server.yollari_onizle(server.OnizlemeIstek(kind="tcp_base", values=[0, 0, -10]))
    assert _isi_bekle()["ok"] is True
    return server.onizleme_oku()


def test_onizleme_robotu_hareket_ettirmiyor_ve_plan_nesnesi_arayuze_gitmiyor(kol):
    ozet = _onizle(kol)

    assert kol.cagrilar == [("secenekler", "tcp_base", [0.0, 0.0, -10.0], True, 20.0, 15.0)], \
        "yalnizca planlama cagrilmali, Ayarlar penceresinin hiziyla"
    assert "_planlar" not in ozet
    json.dumps(ozet)  # SahtePlan icerseydi TypeError
    assert ozet["id"] == server.oturum.onizleme["id"]
    assert server.oturum.telemetry()["preview_id"] == ozet["id"]


def test_deger_sayisi_yanlissa_reddediliyor(kol):
    with pytest.raises(server.HTTPException) as hata:
        server.yollari_onizle(server.OnizlemeIstek(kind="pose", values=[1, 2, 3]))

    assert hata.value.status_code == 400
    assert kol.cagrilar == []


def test_calistirma_sakli_plani_aynen_veriyor_ve_tek_kullanimlik(kol):
    ozet = _onizle(kol)

    server.onizleme_calistir(server.OnizlemeCalistir(id=ozet["id"], option=1))
    assert _isi_bekle()["ok"] is True

    son = kol.cagrilar[-1]
    assert son[0] == "yurut"
    assert son[1] is kol.planlar[0], "yeniden planlanmadan AYNI plan yurutulmeli"
    assert son[2] == [1.0] * 6, "onizlemenin baslangic acilari iletilmeli (tolerans kontrolu icin)"
    assert server.oturum.onizleme is None

    with pytest.raises(server.HTTPException) as hata:
        server.onizleme_calistir(server.OnizlemeCalistir(id=ozet["id"], option=1))
    assert hata.value.status_code == 404, "ayni plan ikinci kez calistirilmamali"


def test_reddedilen_secenek_calistirilamiyor(kol):
    ozet = _onizle(kol)

    with pytest.raises(server.HTTPException) as hata:
        server.onizleme_calistir(server.OnizlemeCalistir(id=ozet["id"], option=2))

    assert hata.value.status_code == 409
    assert all(c[0] != "yurut" for c in kol.cagrilar)
    assert server.oturum.onizleme is not None, "reddedilen istek onizlemeyi silmemeli"


def test_eski_onizleme_numarasi_reddediliyor(kol):
    ozet = _onizle(kol)

    with pytest.raises(server.HTTPException) as hata:
        server.onizleme_calistir(server.OnizlemeCalistir(id=ozet["id"] + 1, option=1))

    assert hata.value.status_code == 409
    assert all(c[0] != "yurut" for c in kol.cagrilar)


@pytest.mark.parametrize("secenek", [0, 3, -1])
def test_olmayan_secenek_reddediliyor(kol, secenek):
    ozet = _onizle(kol)

    with pytest.raises(server.HTTPException) as hata:
        server.onizleme_calistir(server.OnizlemeCalistir(id=ozet["id"], option=secenek))

    assert hata.value.status_code == 404


def test_onizleme_yokken_calistirma_reddediliyor(kol):
    with pytest.raises(server.HTTPException) as hata:
        server.onizleme_calistir(server.OnizlemeCalistir(id=1, option=1))

    assert hata.value.status_code == 404


def test_model_yalnizca_listelenen_mesh_dosyalarini_sunuyor(kol, monkeypatch, tmp_path):
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "base_link.STL").write_bytes(b"solid x")
    monkeypatch.setattr(server, "_paket_klasoru", lambda ad: str(tmp_path) if ad == "desc" else None)

    model = server.model_oku()

    assert [link["mesh"] for link in model["links"]] == ["/api/model/mesh/0", None]
    assert model["tcp_offset_m"] == [0.0, 0.0, 0.2]
    assert model["joint_names"] == SahteKol.joint_names and model["tip_link"] == "wrist3_link"
    assert model["gripper_box"]["size"] == [0.095, 0.06, 0.13]
    assert server.mesh_oku(0).path == (tmp_path / "meshes" / "base_link.STL").resolve()
    for sira in (1, -1):
        with pytest.raises(server.HTTPException) as hata:
            server.mesh_oku(sira)
        assert hata.value.status_code == 404

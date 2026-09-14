"""Web arayuzu HTTP katmani testleri.

ROBOT, ROS2 veya SDK GEREKTIRMEZ: endpoint fonksiyonlari dogrudan cagriliyor
(HTTP istemcisi de gerekmiyor, httpx bagimliligi yok) ve kolun yerine cagri
kaydeden bir sahte nesne konuyor.

Burada test edilen sey robot degil, ISTEGIN DOGRU YERE GITTIGI.
"""

import time

import pytest

server = pytest.importorskip(
    "fairino_cobot.webapp.server",
    reason="webapp bagimliliklari (fastapi) yok",
)


class SahteKol:
    """CobotArm yerine gecer, hangi cagrinin geldigini kaydeder."""

    def __init__(self):
        self.cagrilar = []
        self.hizlar = []  # her cagrinin (speed_pct, accel_pct) degeri
        self.speed_pct = 10.0  # CobotArm'in oturum varsayilani
        self.accel_pct = 10.0

    def move_tcp_base(self, dx, dy, dz, confirm=False, speed_pct=None, accel_pct=None):
        self.cagrilar.append(("base", dx, dy, dz))
        self.hizlar.append((speed_pct, accel_pct))
        return {"success": True}

    def move_tcp_local(self, dx, dy, dz, confirm=False, speed_pct=None, accel_pct=None):
        self.cagrilar.append(("local", dx, dy, dz))
        self.hizlar.append((speed_pct, accel_pct))
        return {"success": True}

    def move_relative_joints(self, deltas, confirm=False, speed_pct=None, accel_pct=None):
        self.cagrilar.append(("jog", *deltas))
        self.hizlar.append((speed_pct, accel_pct))
        return {"success": True}

    def move_to_joints(self, joints, confirm=False, speed_pct=None, accel_pct=None):
        self.cagrilar.append(("joints", *joints))
        self.hizlar.append((speed_pct, accel_pct))
        return {"success": True}

    def move_to_joints_absolute(self, joints, confirm=False, speed_pct=None, accel_pct=None):
        # Web'den mutlak eklem hedefi girilen acilara aynen (sinir kontrolu ile) gider.
        self.cagrilar.append(("joints", *joints))
        self.hizlar.append((speed_pct, accel_pct))
        return {"success": True}

    def move_to_pose_base(self, x, y, z, rx, ry, rz, confirm=False, prefer_lin=False,
                          speed_pct=None, accel_pct=None):
        self.cagrilar.append(("pose", x, y, z))
        self.hizlar.append((speed_pct, accel_pct))
        return {"success": True}


class SahteGripper:
    def __init__(self):
        self.cagrilar = []

    def open(self, speed, force):
        self.cagrilar.append(("open", speed, force))


@pytest.fixture
def kol(monkeypatch):
    sahte = SahteKol()
    monkeypatch.setattr(server.oturum, "arm", sahte)
    monkeypatch.setattr(server.oturum, "ready", True)
    monkeypatch.setattr(server.oturum, "ayarlar", {
        "speed_pct": 10.0, "accel_pct": 10.0, "grip_speed": 50, "grip_force": 50,
    })
    return sahte


def _isi_bekle():
    """run_job ayri bir is parcaciginda kosuyor - bitmesini bekle."""
    for _ in range(50):
        j = server.oturum.job_dict()
        if j and not j["running"]:
            return j
        time.sleep(0.05)
    raise AssertionError("is bitmedi")


def test_iki_ayri_cerceve_iki_ayri_cagriya_gidiyor(kol):
    """Base ve arac cerceveleri KARISMAMALI.

    Ikisinin govdesi ayni (dx_mm/dy_mm/dz_mm), farki sadece endpoint. Yanlis
    esleme sessizce yanlis yone hareket demek olurdu: alet egikken "base +Z"
    beklerken aletin bakis yonune gitmek gibi.
    """
    server.tcp_baz(server.TcpFark(dz_mm=-15))
    assert _isi_bekle()["ok"] is True

    server.tcp_yerel(server.TcpFark(dz_mm=-15))
    assert _isi_bekle()["ok"] is True

    assert kol.cagrilar == [("base", 0.0, 0.0, -15.0), ("local", 0.0, 0.0, -15.0)]


def test_verilmeyen_eksenler_sifir(kol):
    """Tek eksen jog'unda digerleri 0 gitmeli - eksik alan 'degistirme'
    demek, 'bilinmiyor' degil."""
    server.tcp_baz(server.TcpFark(dx_mm=25))
    _isi_bekle()

    assert kol.cagrilar == [("base", 25.0, 0.0, 0.0)]


def test_hazir_degilken_reddediliyor(monkeypatch):
    """Robot hazir degilken hareket istegi 503 donmeli - sessizce yutulmamali."""
    monkeypatch.setattr(server.oturum, "ready", False)

    with pytest.raises(server.HTTPException) as hata:
        server.tcp_baz(server.TcpFark(dz_mm=10))

    assert hata.value.status_code == 503

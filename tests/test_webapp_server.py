"""Web arayuzu HTTP katmani testleri.

ROBOT, ROS2 veya SDK GEREKTIRMEZ: endpoint fonksiyonlari dogrudan cagriliyor
(HTTP istemcisi de gerekmiyor, httpx bagimliligi yok) ve kolun yerine cagri
kaydeden bir sahte nesne konuyor.

Burada test edilen sey robot degil, ISTEGIN DOGRU YERE GITTIGI.
"""

import asyncio
import sys
import threading
import time
import types

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
        self.stop_requested = threading.Event()

    def get_current_state(self):
        # Her hareket cagrisindan sonra farkli acilar: pozla on-poz ayirt edilsin.
        return {"joints_deg": [float(len(self.cagrilar))] * 6}

    def stop_trajectory(self):
        # SDK'ya (StopMotion) DOKUNMAMALI - sahte kolda sdk yok, dokunursa test duser.
        self.cagrilar.append(("stop",))
        self.stop_requested.set()

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
    monkeypatch.setattr(server.oturum, "ayarlar", {"speed_pct": 10.0, "accel_pct": 10.0})
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


def test_hareketi_durdur_moveit_yurutmesini_iptal_ediyor(kol):
    """"Hareketi durdur" MoveIt yurutmesini iptal etmeli; robota SDK
    StopMotion gondermemeli (servo oturumunu dusurur)."""
    sonuc = server.hareketi_durdur()

    assert sonuc == {"durduruldu": True}
    assert kol.cagrilar == [("stop",)]


def test_yeni_is_onceki_durdurma_istegini_temizliyor(kol):
    """Eski bir durdurma istegi sonraki hareketi baslamadan durdurmamali."""
    kol.stop_requested.set()

    server.tcp_baz(server.TcpFark(dz_mm=-5))
    _isi_bekle()

    assert not kol.stop_requested.is_set()
    assert kol.cagrilar == [("base", 0.0, 0.0, -5.0)]


def _sahte_movecobot(monkeypatch, moves=None, run_pick_place=None, replay_sequence=None):
    """movecobot'u import etmek ROS/SDK ister; endpoint'lerin fonksiyon icinden
    yaptigi import'a sahte modul verilir. Kayitlar bellekte tutulur."""
    depo = {"moves": dict(moves or {})}
    mod = types.SimpleNamespace(
        load_moves=lambda: dict(depo["moves"]),
        save_moves=lambda m: depo.update(moves=dict(m)),
        sync_bashrc_aliases=lambda m: None,
        valid_move_name=lambda ad: (True, "") if ad.isidentifier() else (False, "gecersiz"),
        run_pick_place=run_pick_place,
        replay_sequence=replay_sequence,
    )
    monkeypatch.setitem(sys.modules, "fairino_cobot.movecobot", mod)
    return depo


def test_pick_place_pozu_kaydediliyor_ve_kol_geri_cekiliyor(kol, monkeypatch):
    monkeypatch.setattr(server.oturum, "pp_taslak", {})

    server.pp_poz_kaydet(server.PPPoz(etiket="pick", approach_mm=80))
    assert _isi_bekle()["ok"] is True

    assert kol.cagrilar == [("local", 0, 0, -80.0)], "TCP kendi Z'sinde geri cekilmeli"
    t = server.pp_taslak_oku()
    assert t["approach_mm"] == 80
    assert t["pick"] == [0.0] * 6, "poz geri cekilmeden ONCE okunmali"
    assert t["pre_pick"] == [1.0] * 6, "on-poz geri cekildikten SONRA okunmali"


def test_pick_place_farkli_yaklasma_mesafesi_reddediliyor(kol, monkeypatch):
    monkeypatch.setattr(server.oturum, "pp_taslak",
                        {"approach_mm": 80.0, "pick": [0.0] * 6, "pre_pick": [1.0] * 6})

    with pytest.raises(server.HTTPException) as hata:
        server.pp_poz_kaydet(server.PPPoz(etiket="place", approach_mm=50))

    assert hata.value.status_code == 409
    assert kol.cagrilar == [], "reddedilen istekte robot hareket etmemeli"


def test_pick_place_kaydi_terminal_bicimiyle_yaziliyor(kol, monkeypatch):
    depo = _sahte_movecobot(monkeypatch)
    monkeypatch.setattr(server.oturum, "pp_taslak", {
        "approach_mm": 80.0, "pick": [1.0] * 6, "pre_pick": [2.0] * 6,
        "place": [3.0] * 6, "pre_place": [4.0] * 6,
    })

    server.pp_kaydet(server.PPKaydet(name="kutu", approach_mm=80, grip_speed=50,
                                     grip_force=30, object_d_mm=40, object_h_mm=60))

    k = depo["moves"]["kutu"]
    assert k["type"] == "pick_place"
    assert k["pick_joints"] == [1.0] * 6 and k["pre_pick_joints"] == [2.0] * 6
    assert k["place_joints"] == [3.0] * 6 and k["pre_place_joints"] == [4.0] * 6
    assert k["approach_mm"] == 80 and k["object_dh_mm"] == [40, 60]
    assert server.oturum.pp_taslak == {}, "kaydedince taslak bosalmali"


def test_pick_place_eksik_pozla_kaydedilmiyor(kol, monkeypatch):
    depo = _sahte_movecobot(monkeypatch)
    monkeypatch.setattr(server.oturum, "pp_taslak",
                        {"approach_mm": 80.0, "pick": [1.0] * 6, "pre_pick": [2.0] * 6})

    with pytest.raises(server.HTTPException) as hata:
        server.pp_kaydet(server.PPKaydet(name="kutu", approach_mm=80, grip_speed=50,
                                         grip_force=30, object_d_mm=40, object_h_mm=60))

    assert hata.value.status_code == 400
    assert depo["moves"] == {}


def test_pick_place_ayni_isim_onaysiz_uzerine_yazilmiyor(kol, monkeypatch):
    depo = _sahte_movecobot(monkeypatch, moves={"kutu": {"type": "pick_place"}})
    monkeypatch.setattr(server.oturum, "pp_taslak", {
        "approach_mm": 80.0, "pick": [1.0] * 6, "pre_pick": [2.0] * 6,
        "place": [3.0] * 6, "pre_place": [4.0] * 6,
    })

    with pytest.raises(server.HTTPException) as hata:
        server.pp_kaydet(server.PPKaydet(name="kutu", approach_mm=80, grip_speed=50,
                                         grip_force=30, object_d_mm=40, object_h_mm=60))

    assert hata.value.status_code == 409
    assert depo["moves"]["kutu"] == {"type": "pick_place"}


def test_web_pick_place_oynatma_terminal_onayi_istemiyor_basarisizligi_gosteriyor(kol, monkeypatch):
    """Web'de terminal yok: onay istenirse input() EOFError verip isi iptal
    ediyordu. run_pick_place False donunce is 'tamam' gorunmemeli."""
    cagri = {}

    def sahte_run(arm, gripper, cfg, sor=False, onay_iste=True):
        cagri.update(sor=sor, onay_iste=onay_iste)
        return False

    _sahte_movecobot(monkeypatch, moves={"pp": {"type": "pick_place"}}, run_pick_place=sahte_run)

    server.oynat("pp", server.OynatIstek())
    j = _isi_bekle()

    assert cagri == {"sor": False, "onay_iste": False}
    assert j["ok"] is False


def test_web_kayitli_hareket_oynatma_terminal_onayi_istemiyor(kol, monkeypatch):
    cagri = {}

    def sahte_replay(arm, gripper, steps, robot_ip, onay_iste=True):
        cagri.update(onay_iste=onay_iste, adim=len(steps))
        return True

    _sahte_movecobot(monkeypatch, moves={"seq": {"steps": [{}, {}]}}, replay_sequence=sahte_replay)

    server.oynat("seq", server.OynatIstek())
    j = _isi_bekle()

    assert cagri == {"onay_iste": False, "adim": 2}
    assert j["ok"] is True


def test_arayuz_dosyalari_tarayici_onbelleginden_eski_gelmiyor():
    """Imaj yenilenince tarayici eski app.js'i kullanmamali (15 Eylul: eski
    app.js yuzunden Ayarlar penceresi acilmiyor gorundu)."""
    async def sonraki(_istek):
        return server.Response()

    def cagir(yol):
        istek = types.SimpleNamespace(url=types.SimpleNamespace(path=yol))
        return asyncio.run(server.onbellek_kapat(istek, sonraki))

    assert cagir("/").headers["Cache-Control"] == "no-cache"
    assert cagir("/static/app.js").headers["Cache-Control"] == "no-cache"
    assert "Cache-Control" not in cagir("/api/state").headers


def test_hazir_degilken_reddediliyor(monkeypatch):
    """Robot hazir degilken hareket istegi 503 donmeli - sessizce yutulmamali."""
    monkeypatch.setattr(server.oturum, "ready", False)

    with pytest.raises(server.HTTPException) as hata:
        server.tcp_baz(server.TcpFark(dz_mm=10))

    assert hata.value.status_code == 503

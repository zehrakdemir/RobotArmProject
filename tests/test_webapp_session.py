"""Web arayuzu oturum mantigi testleri.

ROBOT, ROS2 veya SDK GEREKTIRMEZ. session modulu SDK'ya ihtiyac duyan
importlari (movecobot, gripper_controller) _baglanmayi_dene() icine
erteliyor, o yuzden temiz bir makinede de import edilebiliyor.
Burada test edilen sey robot degil, oturumun DAVRANISI.
"""

import threading
import time

import pytest

session = pytest.importorskip(
    "fairino_cobot.webapp.session",
    reason="webapp bagimliliklari (veya SDK) yok",
)


def test_hazir_olana_kadar_denemeye_devam_ediyor(monkeypatch):
    """Baglanti BIR KEZ denenip pes EDILMEMELI.

    Eskiden tek seferlikti: basarisiz olunca arayuz kalici olarak
    "baglanilamadi" ekraninda kaliyor ve container yeniden baslatilmadan
    duzelmiyordu. Oysa sebep genelde gecici - robot henuz acilmamis,
    WebApp'ten enable edilmemis, kablo yeni takilmis, move_group hala
    hazirlaniyor. Robot hazir olunca arayuz kendiliginden acilmali.
    """
    o = session.RobotSession()
    monkeypatch.setattr(session.time, "sleep", lambda _s: None)  # testi bekletme

    sayac = {"n": 0}

    def sahte_deneme():
        sayac["n"] += 1
        if sayac["n"] >= 3:
            o.ready = True
        else:
            o.startup_error = "henuz degil"

    o._baglanmayi_dene = sahte_deneme

    t = threading.Thread(target=o.start, daemon=True)
    t.start()
    t.join(timeout=5)

    assert not t.is_alive(), "start() hazir olduktan sonra donmeliydi"
    assert o.ready is True
    assert sayac["n"] == 3, "ucuncu denemede basarili olmaliydi"


def test_bekleme_suresi_artiyor_ama_sinirli(monkeypatch):
    """Sonsuz dongude her saniye denemek robotu ve loglari bogar; bekleme
    kademeli artmali ama bir tavanda durmali ki robot acildiginda arayuz
    dakikalarca gec kalmasin."""
    o = session.RobotSession()
    beklemeler = []
    monkeypatch.setattr(session.time, "sleep", lambda s: beklemeler.append(s))

    sayac = {"n": 0}

    def sahte_deneme():
        sayac["n"] += 1
        if sayac["n"] >= 8:
            o.ready = True

    o._baglanmayi_dene = sahte_deneme
    o.start()

    assert beklemeler, "hic beklenmemis"
    assert beklemeler[0] < beklemeler[-1], "bekleme artmali"
    assert max(beklemeler) <= 60, "bekleme tavani asilmis"


def test_mesgulken_ikinci_hareket_reddediliyor():
    """Iki sekme ayni anda 'git' derse robota iki hedef gitmemeli. Istek
    KUYRUGA ALINMAMALI - hemen reddedilmeli ki kullanici ne oldugunu
    bilsin."""
    o = session.RobotSession()
    birakma = threading.Event()

    o.run_job("test", "uzun is", lambda job: birakma.wait(timeout=5))
    time.sleep(0.1)

    with pytest.raises(session.Busy):
        o.run_job("test", "ikinci is", lambda job: None)

    birakma.set()


def test_is_bitince_kilit_birakiliyor():
    o = session.RobotSession()

    o.run_job("test", "kisa is", lambda job: {"success": True})
    for _ in range(50):
        j = o.job_dict()
        if j and not j["running"]:
            break
        time.sleep(0.05)

    assert o.job_dict()["ok"] is True
    o.run_job("test", "sonraki is", lambda job: None)  # Busy firlatmamali


def test_basarisiz_is_detayi_tasiniyor():
    """Hareket reddedilince (or. zemin carpismasi) sebebi arayuze
    ulasmali - kullanici neden olmadigini gormeli."""
    o = session.RobotSession()

    o.run_job("test", "carpisan hareket",
              lambda job: {"success": False, "reason": "floor_collision",
                           "details": "zemine 3mm kaliyor"})
    for _ in range(50):
        j = o.job_dict()
        if j and not j["running"]:
            break
        time.sleep(0.05)

    j = o.job_dict()
    assert j["ok"] is False
    assert "zemine" in j["detail"]

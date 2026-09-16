"""Yol onizlemesi kurallari. Robot, ROS2 ve SDK GEREKTIRMEZ."""

import math

from fairino_cobot.yol_onizleme import (
    ONIZLEME_BASLANGIC_TOLERANSI_DEG,
    baslangic_farki,
    baslangic_uyuyor_mu,
    en_cok_donen_eklem,
    secenek_ozeti,
)


def test_tolerans_kullanici_karari_2_derece():
    assert ONIZLEME_BASLANGIC_TOLERANSI_DEG == 2.0


def test_kol_oynamadiysa_plan_gecerli():
    plan = [10.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    simdi = [10.4, -90.0, 90.0, -91.9, -90.0, 0.0]

    assert baslangic_uyuyor_mu(plan, simdi)


def test_kol_toleranstan_fazla_oynadiysa_plan_eskimis():
    """Onizlemeden sonra kol jog ile ya da elle oynatildiysa eski plan
    yurutulmemeli: plan baska bir baslangic icin cizildi."""
    plan = [10.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    simdi = [10.0, -90.0, 90.0, -90.0, -90.0, 2.5]

    assert round(baslangic_farki(plan, simdi), 3) == 2.5
    assert not baslangic_uyuyor_mu(plan, simdi)


def test_bozuk_okuma_uyusuyor_sayilmiyor():
    plan = [0.0] * 6

    assert not baslangic_uyuyor_mu(plan, [0.0, 0.0, math.nan, 0.0, 0.0, 0.0])
    assert not baslangic_uyuyor_mu(plan, [0.0, 0.0, math.inf, 0.0, 0.0, 0.0])
    assert not baslangic_uyuyor_mu(plan, [0.0] * 5), "eksik eklem"


def test_en_cok_donen_eklem_yol_ortasindaki_savrulmayi_da_goruyor():
    baslangic = [0.0] * 6
    noktalar = [
        [0.0] * 6,
        [5.0, 0.0, 40.0, 0.0, 0.0, 0.0],   # j3 yolda 40 dereceye savruluyor
        [5.0, 0.0, 1.0, 0.0, 0.0, 0.0],    # bitiste yalnizca 5 derece fark
    ]

    assert en_cok_donen_eklem(baslangic, noktalar) == ("j3", 40.0)


def test_en_cok_donen_eklem_bos_ya_da_bozuk_plan():
    assert en_cok_donen_eklem([0.0] * 6, []) == (None, None)
    assert en_cok_donen_eklem([0.0] * 6, [[math.nan] * 6]) == (None, None)


def test_reddedilen_secenek_calistirilamaz_ama_yolu_gorunur():
    noktalar = [[0.0] * 6, [1.0] * 6]
    ozet = secenek_ozeti(2, "Arm configuration 1", "PTP", [0.0] * 6, noktalar, [0.0, 1.5],
                         {"success": False, "reason": "floor_collision", "details": "step 1"},
                         [1.0] * 6)

    assert ozet["ok"] is False
    assert ozet["reason"] == "floor_collision"
    assert ozet["points"] == noktalar and ozet["duration_s"] == 1.5
    assert ozet["index"] == 2 and ozet["label"] == "Arm configuration 1"


def test_gecen_secenek_calistirilabilir():
    ozet = secenek_ozeti(1, "Straight line", "LIN", [0.0] * 6, [[0.0] * 6, [0.0, 0, 3.0, 0, 0, 0]],
                         [0.0, 2.0], None, None)

    assert ozet["ok"] is True
    assert (ozet["max_joint"], ozet["max_deg"]) == ("j3", 3.0)


def test_plansiz_secenek_ok_degil():
    ozet = secenek_ozeti(1, "Arm configuration 1", None, [0.0] * 6, [], [], None, None)

    assert ozet["ok"] is False
    assert ozet["duration_s"] is None and ozet["max_deg"] is None

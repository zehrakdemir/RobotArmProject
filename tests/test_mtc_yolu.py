"""MTC cozümünü onizleme yoluna cevirme. Robot, ROS2 ve SDK GEREKTIRMEZ."""

import math

import pytest

from fairino_cobot.mtc_yolu import (
    EKLEM_ADLARI,
    MtcYoluHatasi,
    asamalari_birlestir,
    toplam_eklem_yolu_deg,
)


def _stage(ad, joint_names, noktalar_deg_ve_t):
    """Test kolayligi: acilari DERECE olarak yazip radyana cevirir."""
    return {
        "ad": ad,
        "joint_names": joint_names,
        "noktalar": [([math.radians(d) for d in degs], t) for degs, t in noktalar_deg_ve_t],
    }


def test_tek_gorev_tek_hareketli_stage():
    gorev = [_stage("hareket", list(EKLEM_ADLARI), [
        ([0, 0, 0, 0, 0, 0], 0.0),
        ([10, 0, 0, 0, 0, 0], 1.0),
    ])]

    noktalar, zamanlar, bilgi = asamalari_birlestir([gorev])

    assert noktalar == [[0, 0, 0, 0, 0, 0], [10, 0, 0, 0, 0, 0]]
    assert zamanlar == [0.0, 1.0]
    assert bilgi == [{"ad": "hareket", "tip": "kol", "ilk": 0, "son": 1, "t0": 0.0, "t1": 1.0}]


def test_hareketsiz_stage_sahne_olarak_isaretlenir():
    """FixedState/ModifyPlanningScene stage'i (0 nokta) yol noktasi eklemez,
    ama gorunur olsun diye sifir-uzunluklu bir asama kaydi birakir."""
    gorev = [
        _stage("nesneyi ilistir", [], []),
        _stage("hareket", list(EKLEM_ADLARI), [([0] * 6, 0.0), ([5, 0, 0, 0, 0, 0], 1.0)]),
    ]

    noktalar, zamanlar, bilgi = asamalari_birlestir([gorev])

    assert len(noktalar) == 2
    assert bilgi[0] == {"ad": "nesneyi ilistir", "tip": "sahne", "ilk": 0, "son": 0, "t0": 0.0, "t1": 0.0}
    assert bilgi[1]["tip"] == "kol" and bilgi[1]["ilk"] == 0 and bilgi[1]["son"] == 1


def test_eklem_sirasi_farkli_olsa_da_dogru_eslesiyor():
    """MTC'nin joint_names sirasi j1..j6 olmak zorunda degil."""
    stage = {
        "ad": "hareket",
        "joint_names": ["j3", "j1", "j2", "j4", "j5", "j6"],
        "noktalar": [([math.radians(30), math.radians(10), math.radians(20), 0, 0, 0], 0.0)],
    }

    noktalar, _zamanlar, _bilgi = asamalari_birlestir([[stage]])

    assert len(noktalar) == 1
    assert noktalar[0] == pytest.approx([10.0, 20.0, 30.0, 0.0, 0.0, 0.0])


def test_eksik_eklem_hata_firlatir():
    stage = {"ad": "bozuk", "joint_names": ["j1", "j2", "j3", "j4", "j5"],  # j6 eksik
             "noktalar": [([0, 0, 0, 0, 0], 0.0)]}

    with pytest.raises(MtcYoluHatasi):
        asamalari_birlestir([[stage]])


def test_iki_gorev_arasindaki_dikis_noktasi_tekrarlanmiyor():
    """Gorev 2'nin ilk noktasi Gorev 1'in son noktasinin AYNISI (MTC
    FixedState zincirlemesinin dogal sonucu) - cift sayilmamali."""
    g1 = [_stage("h1", list(EKLEM_ADLARI), [([0] * 6, 0.0), ([10, 0, 0, 0, 0, 0], 1.0)])]
    g2 = [_stage("h2", list(EKLEM_ADLARI), [([10, 0, 0, 0, 0, 0], 0.0), ([20, 0, 0, 0, 0, 0], 1.0)])]

    noktalar, zamanlar, _bilgi = asamalari_birlestir([g1, g2])

    assert noktalar == [[0] * 6, [10, 0, 0, 0, 0, 0], [20, 0, 0, 0, 0, 0]]
    assert zamanlar == [0.0, 1.0, 2.0]


def test_bosluk_gripper_araya_giriyor_ve_zamani_oteliyor():
    """bosluk_s>0 iken dikis noktasi ATLANMAZ: kol, gripper kapanirken/acilirken
    o pozda DURUYORMUS gibi gorunmesi icin ayni noktanin bosluk sonundaki
    kopyasi da eklenir (viewer bu araligi 'sabit dur' olarak animasyonlar)."""
    g1 = [_stage("h1", list(EKLEM_ADLARI), [([0] * 6, 0.0), ([10, 0, 0, 0, 0, 0], 1.0)])]
    g2 = [_stage("h2", list(EKLEM_ADLARI), [([10, 0, 0, 0, 0, 0], 0.0), ([20, 0, 0, 0, 0, 0], 1.0)])]

    noktalar, zamanlar, bilgi = asamalari_birlestir([g1, g2], bosluk_s=0.8)

    assert noktalar == [[0] * 6, [10, 0, 0, 0, 0, 0], [10, 0, 0, 0, 0, 0], [20, 0, 0, 0, 0, 0]]
    # h1 0->1.0, gripper bosluk 1.0->1.8 (kol sabit), h2'nin ikinci noktasi 1.8+1.0=2.8
    assert zamanlar == [0.0, 1.0, 1.8, 2.8]
    gripper_asamalari = [a for a in bilgi if a["tip"] == "gripper"]
    assert len(gripper_asamalari) == 1
    assert gripper_asamalari[0]["t0"] == 1.0 and gripper_asamalari[0]["t1"] == 1.8


def test_tekrarlanan_zaman_ileri_itilir():
    """Ayni stage icinde iki noktanin zamani esit cikarsa (MTC tuhafligi),
    controller'in reddetmemesi icin ileri itilir."""
    stage = _stage("h", list(EKLEM_ADLARI), [
        ([0] * 6, 0.0), ([0] * 6, 0.0), ([1, 0, 0, 0, 0, 0], 1.0),
    ])

    _noktalar, zamanlar, _bilgi = asamalari_birlestir([[stage]])

    assert zamanlar[0] == 0.0
    assert zamanlar[1] > zamanlar[0]
    assert zamanlar[2] > zamanlar[1]


def test_bos_gorev_listesi():
    noktalar, zamanlar, bilgi = asamalari_birlestir([])

    assert noktalar == [] and zamanlar == [] and bilgi == []


# -- toplam eklem yolu ("en kisa yol" olcusu) -------------------------------

def test_toplam_yol_ardisik_farklarin_toplami():
    noktalar = [[0, 0, 0, 0, 0, 0], [10, 0, 0, 0, 0, 0], [10, -5, 0, 0, 0, 0]]

    assert toplam_eklem_yolu_deg(noktalar) == 15.0


def test_toplam_yol_geri_donen_eklemi_de_sayiyor():
    """Gidip geri donen eklem 'sifir' sayilmamali - kol o yolu gercekten
    aliyor (bkz. fonksiyonun gerekcesi)."""
    gidip_gelen = [[0] * 6, [90, 0, 0, 0, 0, 0], [0] * 6]
    duz = [[0] * 6, [45, 0, 0, 0, 0, 0], [90, 0, 0, 0, 0, 0]]

    assert toplam_eklem_yolu_deg(gidip_gelen) == 180.0
    assert toplam_eklem_yolu_deg(duz) == 90.0


def test_toplam_yol_tek_nokta_ve_bos_yol_sifir():
    assert toplam_eklem_yolu_deg([]) == 0.0
    assert toplam_eklem_yolu_deg([[0] * 6]) == 0.0


def test_toplam_yol_gercek_olmayan_deger_sonsuz():
    """NaN/sonsuz iceren yol 'kisa' sayilmamali - siralamada sona dusmeli."""
    assert toplam_eklem_yolu_deg([[0] * 6, [float("nan"), 0, 0, 0, 0, 0]]) == math.inf
    assert toplam_eklem_yolu_deg([[0] * 6, [float("inf"), 0, 0, 0, 0, 0]]) == math.inf


def test_toplam_yol_uzunluk_tutmuyorsa_sonsuz():
    assert toplam_eklem_yolu_deg([[0] * 6, [0] * 5]) == math.inf

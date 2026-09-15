"""Hareket guvenlik kurallari.

Bu testler ROBOT, ROS2 ve SDK GEREKTIRMEZ - kurallar saf fonksiyon olarak
yazildigi icin (bkz. safety.py modul basligi) temiz bir makinede de kosuyor.
"""

import math

from fairino_cobot.safety import asiri_eklem_hareketi, bilek_tekili, tekil_ihlali


def test_canli_olay_yakalaniyor():
    """GERCEK OLAY: arayuzden TCP'ye 10mm oteleme verildi, plan j1'i
    37.08 -> 69.94 dereceye (32.9 derece) donduruyordu. Kol savruldu,
    kullanici acil stopa basmak zorunda kaldi.

    Zemin ve self-collision kontrolleri bunu gormuyordu - yay bos havadaydi.
    Bu kural tam olarak o plani reddetmek icin var.
    """
    baslangic = [37.079, -142.576, 53.355, -0.601, -86.266, -49.901]
    plan = [
        list(baslangic),
        [53.0, -144.0, 51.0, 4.0, -86.0, -57.0],
        [69.94, -146.35, 49.19, 8.06, -85.64, -65.38],
    ]

    ihlal = asiri_eklem_hareketi(baslangic, plan, 20.0)

    assert ihlal is not None, "canli olaydaki plan REDDEDILMELIYDI"
    assert ihlal["success"] is False
    assert ihlal["reason"] == "excessive_joint_motion"
    assert "j1" in ihlal["details"], "hangi eksenin sucu oldugu yazmali"
    assert "32.9" in ihlal["details"], "sapma miktari yazmali"


def test_makul_jog_gecirilyor():
    """Normal bir 10mm jog birkac derecelik eklem hareketi demek - bu
    engellenirse jog kullanilamaz hale gelir."""
    baslangic = [37.0, -142.0, 53.0, 0.0, -86.0, -50.0]
    plan = [
        list(baslangic),
        [37.8, -142.9, 53.6, 0.3, -86.2, -50.4],
        [38.4, -143.5, 54.1, 0.5, -86.3, -50.7],
    ]

    assert asiri_eklem_hareketi(baslangic, plan, 20.0) is None


def test_savrulma_ara_noktada_olsa_da_yakalaniyor():
    """Plan baslangica yakin bir yerde BITEBILIR ama yolda savrulabilir.
    Sadece bitis noktasina bakmak bunu kacirirdi."""
    baslangic = [0.0, -90.0, 0.0, 0.0, -90.0, 0.0]
    plan = [
        list(baslangic),
        [75.0, -90.0, 0.0, 0.0, -90.0, 0.0],   # yolda genis yay
        [1.0, -90.0, 0.0, 0.0, -90.0, 0.0],    # bitis baslangica yakin
    ]

    ihlal = asiri_eklem_hareketi(baslangic, plan, 20.0)

    assert ihlal is not None
    assert "adim 1/3" in ihlal["details"], "ihlalin nerede oldugu yazmali"


def test_sinirda_reddedilmiyor():
    """Esigin tam ustu ihlal, tam kendisi degil - keyfi degil, tutarli olsun."""
    baslangic = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    assert asiri_eklem_hareketi(baslangic, [[20.0, 0, 0, 0, 0, 0]], 20.0) is None
    assert asiri_eklem_hareketi(baslangic, [[20.1, 0, 0, 0, 0, 0]], 20.0) is not None


def test_negatif_yonde_savrulma_da_yakalaniyor():
    """Mutlak deger alinmali - ters yone savrulmak da savrulmadir."""
    baslangic = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    ihlal = asiri_eklem_hareketi(baslangic, [[0, 0, 0, 0, 0, -45.0]], 20.0)

    assert ihlal is not None
    assert "j6" in ihlal["details"]


# --- Tekil korumasi --------------------------------------------------------
# Sayilar FR3 modelinde olculen kosul sayilari (15 Eylul, robotsuz; bkz.
# movecobot.TEKIL_KOSUL_ESIGI yorumu).


def test_tekil_ev_pozu_ve_kayitli_pozlar_geciyor():
    """Ev pozu 12.6; kayitli gercek pozlarin en yuksegi 88.1 (sigara place).
    Bunlar engellenirse normal isler yapilamaz."""
    assert tekil_ihlali([12.6, 25.0, 88.1], 100.0, "the planned path") is None


def test_tekil_bilek_yakini_reddediliyor():
    """Ev pozunda j5=5 derece: 129.5. Planin ortasinda olsa da yakalanmali."""
    ihlal = tekil_ihlali([12.6, 129.5, 20.0], 100.0, "the planned path")

    assert ihlal is not None
    assert ihlal["success"] is False
    assert ihlal["reason"] == "singularity"
    assert "step 1/3" in ihlal["details"], "ihlalin nerede oldugu yazmali"
    assert "did not move" in ihlal["details"]


def test_tekil_tam_tekil_sonsuz():
    """j5=0: kosul sayisi sonsuz. Tek durusta adim numarasi yazilmaz."""
    ihlal = tekil_ihlali([math.inf], 100.0, "the current arm pose")

    assert ihlal is not None
    assert "infinite" in ihlal["details"]
    assert "step" not in ihlal["details"]


def test_tekil_nan_tekil_sayiliyor():
    """Hesap bozulursa guvenli taraf: reddet."""
    assert tekil_ihlali([12.6, math.nan], 100.0, "the planned path") is not None


def test_tekil_sonsuz_ve_eksi_sonsuz_reddediliyor():
    """Kosul sayisi eksi olamaz; eksi sonsuz bozuk hesap demek, reddet."""
    assert tekil_ihlali([12.6, -math.inf], 100.0, "the planned path") is not None
    assert tekil_ihlali([math.inf, 12.6], 100.0, "the planned path") is not None


def test_tekil_esikte_reddedilmiyor():
    assert tekil_ihlali([100.0], 100.0, "x") is None
    assert tekil_ihlali([100.1], 100.0, "x") is not None


def test_tekil_bos_liste():
    assert tekil_ihlali([], 100.0, "x") is None


# --- Bilek tekili (j5) -----------------------------------------------------


def test_bilek_canli_olay_yakalaniyor():
    """GERCEK DURUS (15 Eylul): +Y jog'undan sonra j5=1.69 derece, kosul
    sayisi 99.3 (esik 100 alti). Bilek kurali bunu reddetmeli."""
    durus = [-1.69, -66.54, 85.53, -198.99, 1.69, 90.0]

    ihlal = bilek_tekili([durus], 10.0, "the current arm pose")

    assert ihlal is not None
    assert ihlal["reason"] == "singularity"
    assert "1.7" in ihlal["details"], "j5'in tekile uzakligi yazmali"
    assert "did not move" in ihlal["details"]


def test_bilek_kayitli_pozlar_geciyor():
    """Ev pozu ve kayitli en yakin poz (winstob 3, j5=-15.073) engellenmemeli."""
    ev = [0, -90, 90, -90, -90, 0]
    winstob3 = [104.504, -31.45, -84.423, -65.685, -15.073, 136.188]

    assert bilek_tekili([ev, winstob3], 10.0, "the planned path") is None


def test_bilek_180_tarafi_da_yakalaniyor():
    assert bilek_tekili([[0, -90, 90, -90, 175.0, 0]], 10.0, "x") is not None
    assert bilek_tekili([[0, -90, 90, -90, -172.0, 0]], 10.0, "x") is not None


def test_bilek_payda_reddedilmiyor():
    assert bilek_tekili([[0, 0, 0, 0, 10.0, 0]], 10.0, "x") is None
    assert bilek_tekili([[0, 0, 0, 0, 9.9, 0]], 10.0, "x") is not None


def test_bilek_planin_ortasinda_yakalaniyor():
    """j5 +15'ten -15'e giden bir yol 0'dan gecer."""
    plan = [[0, -90, 90, -90, 15.0, 0], [0, -90, 90, -90, 0.5, 0], [0, -90, 90, -90, -15.0, 0]]

    ihlal = bilek_tekili(plan, 10.0, "the planned path")

    assert ihlal is not None
    assert "step 1/3" in ihlal["details"]


def test_bilek_nan_tekil_sayiliyor():
    assert bilek_tekili([[0, 0, 0, 0, math.nan, 0]], 10.0, "x") is not None


def test_bilek_sonsuz_planin_herhangi_bir_noktasinda_yakalaniyor():
    """OLCULEN HATA (15 Eylul): sonsuz j5 yalnizca ilk noktadaysa
    yakalaniyordu; ikinci noktada sessizce atlaniyordu."""
    ev = [0, -90, 90, -90, -90, 0]

    ihlal = bilek_tekili([ev, [0, -90, 90, -90, math.inf, 0]], 10.0, "the planned path")

    assert ihlal is not None
    assert "step 1/2" in ihlal["details"]
    assert bilek_tekili([ev, [0, -90, 90, -90, -math.inf, 0]], 10.0, "x") is not None


def test_bilek_bos_liste():
    assert bilek_tekili([], 10.0, "x") is None

"""Hareket guvenlik kurallari.

Bu testler ROBOT, ROS2 ve SDK GEREKTIRMEZ - kurallar saf fonksiyon olarak
yazildigi icin (bkz. safety.py modul basligi) temiz bir makinede de kosuyor.
"""

import math

from fairino_cobot.safety import (
    asiri_eklem_hareketi,
    bilek_tekili,
    kavranan_nesne_kutusu_merkezi,
    kutunun_en_alcak_z,
    tekil_ihlali,
)


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


# ---------------------------------------------------------------------------
# KAVRANAN NESNENIN CARPISMA KUTUSU
# ---------------------------------------------------------------------------
# Kural 21 Eylul'de degisti: nesneler artik ust yuzun birkac cm altindan
# kavraniyor (vision'in GRASP_HEIGHT_FRACTION=0.5'i, yani ortadan kavrama,
# kullanilmiyor). Kutu TCP'de ortali kalirsa model nesneyi oldugundan
# yukarida gosterir.


def test_ortadan_kavramada_kutu_tcp_de_kalir():
    """derinlik = yukseklik/2 verilirse kutu eski yerinde (TCP'de) durur -
    davranis degismemis olmali."""
    merkez = kavranan_nesne_kutusu_merkezi((0.01, 0.0, 0.2), 49.0, 24.5)
    assert merkez == (0.01, 0.0, 0.2)


def test_derinlik_verilmezse_eski_davranis():
    """Kayitli pick & place hareketlerinde derinlik bilinmiyor - o zaman
    kutu yine TCP'de ortalanir."""
    assert kavranan_nesne_kutusu_merkezi((0.0, 0.0, 0.2), 49.0) == (0.0, 0.0, 0.2)


def test_ustten_kavramada_kutu_asagi_kayar():
    """49mm nesne, ust yuzun 10mm altindan: nesne TCP'nin 10mm ustunden
    39mm altina uzanir, yani merkez TCP'nin 14.5mm ALTINDA (tool +Z nesneye
    dogru bakiyor)."""
    _, _, z = kavranan_nesne_kutusu_merkezi((0.0, 0.0, 0.2), 49.0, 10.0)
    assert abs(z - (0.2 + 0.0145)) < 1e-9


def test_derinlik_yukseklikle_sinirlanir():
    """Nesneden daha derin bir kavrama fiziksel olarak anlamsiz; kutu
    nesnenin tamamen ustune cikmasin diye kirpiliyor."""
    _, _, z = kavranan_nesne_kutusu_merkezi((0.0, 0.0, 0.2), 40.0, 999.0)
    assert abs(z - (0.2 - 0.02)) < 1e-9


def test_kutunun_en_alcak_z_birim_yonelimde():
    """Alet asagi bakmiyorken (birim kuaterniyon) kutu dogrudan asagi
    uzanir: merkez 0.2, yarim yukseklik 0.0245 -> 0.1755."""
    z = kutunun_en_alcak_z((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                           (0.0, 0.0, 0.2), (0.05, 0.03, 0.049))
    assert abs(z - (0.2 - 0.0245)) < 1e-9


def test_kutunun_en_alcak_z_tcp_asagi_bakarken():
    """Gercek kavrama durusu: alet X ekseni etrafinda 180 derece donuk
    (RX=180), yani tool +Z DUNYA -Z'sine bakiyor. Flans 0.5m'de olsun;
    kutu merkezi alette +0.2 -> dunyada 0.3, alt yuzu 0.3 - 0.0245."""
    z = kutunun_en_alcak_z((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0),
                           (0.0, 0.0, 0.2), (0.05, 0.03, 0.049))
    assert abs(z - (0.5 - 0.2 - 0.0245)) < 1e-9


def test_kutunun_en_alcak_z_yatik_alette_en_uzak_koseyi_alir():
    """45 derece yatik bir alette en alcak nokta kutunun bir KOSESI - kutu
    orta noktasina bakmak nesnenin zemine girdigini kacirir."""
    yarim_kok2 = math.sqrt(0.5)
    # Y ekseni etrafinda 90 derece: tool +Z dunya +X'e doner.
    z = kutunun_en_alcak_z((0.0, 0.0, 0.0), (0.0, yarim_kok2, 0.0, yarim_kok2),
                           (0.0, 0.0, 0.2), (0.05, 0.03, 0.049))
    # Merkez artik dunyada z=0; en alcak nokta kutunun yarim genisligi kadar
    # asagida (donmus eksende genislik dusey).
    assert abs(z - (-0.025)) < 1e-9

"""Hareket guvenlik kurallari.

Bu testler ROBOT, ROS2 ve SDK GEREKTIRMEZ - kurallar saf fonksiyon olarak
yazildigi icin (bkz. safety.py modul basligi) temiz bir makinede de kosuyor.
"""

from fairino_cobot.safety import asiri_eklem_hareketi


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

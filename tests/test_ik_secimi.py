"""IK aday siralamasi. Robot, ROS2 ve SDK GEREKTIRMEZ."""

from fairino_cobot.ik_secimi import ik_adaylarini_sirala


def test_en_yakin_aday_once_geliyor():
    baslangic = [87.2, -77.0, 28.5, -42.0, -85.2, -53.6]
    uzak = [87.2, -50.0, -28.5, 10.0, -85.2, -53.6]       # dirsek ters
    yakin = [87.5, -79.1, 33.0, -44.3, -85.2, -53.9]
    orta = [60.0, -77.0, 28.5, -42.0, -85.2, -53.6]

    sirali = ik_adaylarini_sirala(baslangic, [uzak, yakin, orta])

    assert [a for a, _ in sirali] == [yakin, orta, uzak]
    assert round(sirali[0][1], 1) == 4.5, "en buyuk eklem farki (j3) donmeli"


def test_ayni_cozum_bir_kez_tutuluyor():
    """config=-1 genelde config=k'lerden biriyle ayni cozumu verir."""
    baslangic = [0.0] * 6
    a = [10.0, 0, 0, 0, 0, 0]

    sirali = ik_adaylarini_sirala(baslangic, [a, list(a), [10.0000001, 0, 0, 0, 0, 0]])

    assert len(sirali) == 1


def test_esitlikte_toplam_donus_kucuk_olan_once():
    baslangic = [0.0] * 6
    tek_eklem = [20.0, 0, 0, 0, 0, 0]
    iki_eklem = [20.0, 20.0, 0, 0, 0, 0]

    sirali = ik_adaylarini_sirala(baslangic, [iki_eklem, tek_eklem])

    assert sirali[0][0] == tek_eklem


def test_aday_yoksa_bos_liste():
    assert ik_adaylarini_sirala([0.0] * 6, []) == []

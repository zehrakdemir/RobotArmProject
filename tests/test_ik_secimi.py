"""IK aday siralamasi. Robot, ROS2 ve SDK GEREKTIRMEZ."""

from fairino_cobot.ik_secimi import (
    ik_adaylarini_sirala,
    uygun_cifti_bul,
    uygun_ciftleri_bul,
)


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


def _j(x):
    return [float(x), 0.0, 0.0, 0.0, 0.0, 0.0]


def test_cift_ilk_uygun_pick_ve_place_seciliyor():
    pick, place, deneme = uygun_cifti_bul(
        [_j(0), _j(50)], [_j(10)], lambda p: True, lambda p, q: True)

    assert (pick, place) == (_j(0), _j(10))
    assert deneme == {"alma": 1, "zincir": 1}


def test_yaklasma_planlanamayan_pick_atlaniyor_ve_zincir_denenmiyor():
    zincir_cagrilari = []

    def zincir(p, q):
        zincir_cagrilari.append(p)
        return True

    pick, place, deneme = uygun_cifti_bul(
        [_j(0), _j(50)], [_j(10)], lambda p: p == _j(50), zincir)

    assert pick == _j(50)
    assert zincir_cagrilari == [_j(50)], "yaklasamayan pick icin zincir planlanmamali"
    assert deneme == {"alma": 2, "zincir": 1}


def test_place_adaylari_pick_a_yakinliga_gore_deneniyor():
    cagrilar = []

    def zincir(p, q):
        cagrilar.append(q)
        return False

    uygun_cifti_bul([_j(0)], [_j(100), _j(5)], lambda p: True, zincir)

    assert cagrilar == [_j(5), _j(100)]


def test_zincir_planlanamazsa_sonraki_place_deneniyor():
    pick, place, deneme = uygun_cifti_bul(
        [_j(0)], [_j(5), _j(100)], lambda p: True, lambda p, q: q == _j(100))

    assert (pick, place) == (_j(0), _j(100))
    assert deneme == {"alma": 1, "zincir": 2}


def test_zincir_hicbir_place_ile_planlanamazsa_sonraki_pick_deneniyor():
    pick, place, _ = uygun_cifti_bul(
        [_j(0), _j(50)], [_j(10)], lambda p: True, lambda p, q: p == _j(50))

    assert (pick, place) == (_j(50), _j(10))


def test_hic_uygun_cift_yoksa_none_donuyor():
    pick, place, deneme = uygun_cifti_bul(
        [_j(0), _j(50)], [_j(10)], lambda p: True, lambda p, q: False)

    assert (pick, place) == (None, None)
    assert deneme == {"alma": 2, "zincir": 2}


def test_aday_yoksa_hicbir_sey_planlanmiyor():
    cagrilar = []
    pick, place, deneme = uygun_cifti_bul(
        [], [_j(10)], lambda p: cagrilar.append(p) or True, lambda p, q: True)

    assert (pick, place) == (None, None)
    assert cagrilar == []
    assert deneme == {"alma": 0, "zincir": 0}


# -- birden fazla secenek (onizlemede sunulan yollar) -----------------------

def test_her_pick_icin_tek_cift_toplaniyor():
    """Secenekler farkli KOL DURUSLARI olsun diye bir pick'in ikinci place
    cozumu alinmaz; sonraki pick adayina gecilir."""
    ciftler, deneme = uygun_ciftleri_bul(
        [_j(0), _j(50), _j(80)], [_j(10), _j(20)],
        lambda p: True, lambda p, q: True, en_fazla=3)

    assert ciftler == [(_j(0), _j(10)), (_j(50), _j(20)), (_j(80), _j(20))]
    assert deneme == {"alma": 3, "zincir": 3}


def test_en_fazla_kadar_secenek_toplanip_duruluyor():
    ciftler, deneme = uygun_ciftleri_bul(
        [_j(0), _j(50), _j(80)], [_j(10)],
        lambda p: True, lambda p, q: True, en_fazla=2)

    assert ciftler == [(_j(0), _j(10)), (_j(50), _j(10))]
    assert deneme["alma"] == 2, "ucuncu pick adayi hic denenmemeli"


def test_planlanamayan_pick_secenek_uretmiyor():
    ciftler, _ = uygun_ciftleri_bul(
        [_j(0), _j(50)], [_j(10)],
        lambda p: p != _j(0), lambda p, q: True, en_fazla=3)

    assert ciftler == [(_j(50), _j(10))]


def test_zincir_butcesi_bitince_duruluyor():
    """Hicbir zincir planlanamayan sahnede onizleme dakikalarca surmesin."""
    cagrilar = []

    def zincir(p, q):
        cagrilar.append((p, q))
        return False

    ciftler, deneme = uygun_ciftleri_bul(
        [_j(0), _j(50)], [_j(10), _j(20), _j(30)],
        lambda p: True, zincir, en_fazla=2, en_fazla_zincir_denemesi=4)

    assert ciftler == []
    assert deneme["zincir"] == 4 and len(cagrilar) == 4


def test_tek_cift_sarmalayicisi_ilk_uygunu_donuyor():
    """uygun_cifti_bul (run_at_pose yolu) davranisi degismedi."""
    pick, place, deneme = uygun_cifti_bul(
        [_j(0), _j(50)], [_j(10)], lambda p: True, lambda p, q: p == _j(50))

    assert (pick, place) == (_j(50), _j(10))
    assert deneme == {"alma": 2, "zincir": 2}

"""IK aday cozumlerini secmenin SAF (bagimliliksiz) kismi.

NEDEN AYRI MODUL: movecobot.py import edilirken rclpy, moveit_py ve Fairino
SDK'si gerekiyor. Siralama mantigi robot olmadan test edilebilsin diye
burada (bkz. safety.py ile ayni gerekce).
"""

from __future__ import annotations

from collections.abc import Callable


def ik_adaylarini_sirala(
    baslangic_deg: list[float],
    adaylar_deg: list[list[float]],
) -> list[tuple[list[float], float]]:
    """Aday eklem cozumlerini baslangica yakinliga gore siralar.

    Yakinlik olcusu: adayda EN COK donen eklemin donusu (derece). Esitlikte
    toplam donus kucuk olan once gelir. Ayni cozum birden fazla kez geldiyse
    (ör. config=-1 ile config=k ayni durusu verdiyse) bir kez tutulur;
    "ayni" = tum eklemler 0.001 derece hassasiyetinde esit.

    Donus: [(aday, en_buyuk_eklem_farki_deg), ...] en yakindan uzaga.
    """
    gorulen = set()
    tekil = []
    for aday in adaylar_deg:
        anahtar = tuple(round(v, 3) for v in aday)
        if anahtar in gorulen:
            continue
        gorulen.add(anahtar)
        farklar = [abs(a - b) for a, b in zip(aday, baslangic_deg)]
        tekil.append((list(aday), max(farklar), sum(farklar)))

    tekil.sort(key=lambda x: (x[1], x[2]))
    return [(aday, en_buyuk) for aday, en_buyuk, _ in tekil]


def uygun_ciftleri_bul(
    pick_adaylari_deg: list[list[float]],
    place_adaylari_deg: list[list[float]],
    alma_planlanir_mi: Callable[[list[float]], bool],
    zincir_planlanir_mi: Callable[[list[float], list[float]], bool],
    en_fazla: int = 1,
    en_fazla_zincir_denemesi: int | None = None,
) -> tuple[list[tuple[list[float], list[float]]], dict[str, int]]:
    """Zincirin bastan sona planlanabildigi (pick, place) ciftlerini, EN COK
    `en_fazla` tane olacak sekilde toplar.

    NEDEN BIRDEN FAZLA (23 Eylul, kullanici istegi): eskiden ilk uygun cift
    bulununca durulup o calistiriliyordu; "ilk uygun" = "en kisa yol" DEGIL -
    adaylar baslangica yakinliga gore sirali ama zincirin toplam eklem yolu
    tasima ve birakma durusuna da bagli (21 Eylul: pick j1=163 ile place
    j1=-54 secilmisti, nesneyle ~217 derece taban donusu). Simdi birkac uygun
    cift planlanip arayuze SECENEK olarak veriliyor; maliyete gore siralamak
    cagiranin isi (bkz. mtc_yolu.toplam_eklem_yolu_deg).

    POLITIKA: her pick adayi icin O PICK'E EN YAKIN planlanabilen place
    alinir ve sonraki pick adayina gecilir. Boylece secenekler birbirinden
    farkli KOL DURUSLARI olur (ayni pick'in komsu place cozumleri degil) ve
    planlama maliyeti pick adayi basina bir zincirle sinirli kalir.

    en_fazla_zincir_denemesi: toplam zincir planlama butcesi (None = sinirsiz).
    Zincir planlamasi pahali (3 MTC gorevi); cok adayli sahnelerde onizlemenin
    dakikalarca surmesini engeller.

    Donus: ([(pick, place), ...], deneme). Liste maliyet sirasinda DEGIL,
    bulunma sirasindadir.
    """
    deneme = {"alma": 0, "zincir": 0}
    ciftler: list[tuple[list[float], list[float]]] = []
    for pick in pick_adaylari_deg:
        if len(ciftler) >= en_fazla:
            break
        deneme["alma"] += 1
        if not alma_planlanir_mi(pick):
            continue
        for place, _ in ik_adaylarini_sirala(pick, place_adaylari_deg):
            if (en_fazla_zincir_denemesi is not None
                    and deneme["zincir"] >= en_fazla_zincir_denemesi):
                return ciftler, deneme
            deneme["zincir"] += 1
            if zincir_planlanir_mi(pick, place):
                ciftler.append((pick, place))
                break
    return ciftler, deneme


def uygun_cifti_bul(
    pick_adaylari_deg: list[list[float]],
    place_adaylari_deg: list[list[float]],
    alma_planlanir_mi: Callable[[list[float]], bool],
    zincir_planlanir_mi: Callable[[list[float], list[float]], bool],
) -> tuple[list[float] | None, list[float] | None, dict[str, int]]:
    """Pick ve place on-pozu icin, TUM zincirin robot HAREKET ETMEDEN
    planlanabildigi ilk (pick, place) ciftini bulur.

    NEDEN: MTC tutorial'i gorevi tek parca kurup tamamini once planliyor
    (task.plan), sonra calistiriyor. Bizde pick ve place ayri gorevler ve
    arada gripper gercekten kapaniyor; place planlanamazsa nesne elde
    kaliyordu. Burada cift, calistirmadan ONCE secilir.

    pick_adaylari_deg: baslangica yakin sirada (bkz. ik_adaylarini_sirala).
    place_adaylari_deg: her pick icin O PICK'E yakinliga gore yeniden
    siralanir (tasima eklem uzayinda duz gidiyor, yakin duruslar az savrulur).
    alma_planlanir_mi(pick): yalniz yaklasma planlanir mi (ucuz eleme).
    zincir_planlanir_mi(pick, place): tum zincir planlanir mi.

    Donus: (pick, place, deneme). Uygun cift yoksa (None, None, deneme);
    deneme = {"alma": n, "zincir": m} yapilan plan sayilari.

    uygun_ciftleri_bul'un en_fazla=1 hali (bu yol - run_at_pose - secenek
    sunmaz, ilk uygun cifti calistirir).
    """
    ciftler, deneme = uygun_ciftleri_bul(
        pick_adaylari_deg, place_adaylari_deg,
        alma_planlanir_mi, zincir_planlanir_mi, en_fazla=1)
    if not ciftler:
        return None, None, deneme
    pick, place = ciftler[0]
    return pick, place, deneme

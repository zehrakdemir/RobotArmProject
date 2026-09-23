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
    """
    deneme = {"alma": 0, "zincir": 0}
    for pick in pick_adaylari_deg:
        deneme["alma"] += 1
        if not alma_planlanir_mi(pick):
            continue
        for place, _ in ik_adaylarini_sirala(pick, place_adaylari_deg):
            deneme["zincir"] += 1
            if zincir_planlanir_mi(pick, place):
                return pick, place, deneme
    return None, None, deneme

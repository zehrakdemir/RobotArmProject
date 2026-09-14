"""IK aday cozumlerini secmenin SAF (bagimliliksiz) kismi.

NEDEN AYRI MODUL: movecobot.py import edilirken rclpy, moveit_py ve Fairino
SDK'si gerekiyor. Siralama mantigi robot olmadan test edilebilsin diye
burada (bkz. safety.py ile ayni gerekce).
"""

from __future__ import annotations


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

"""Yol onizlemesinin SAF (bagimliliksiz) kismi.

Web arayuzunde "Preview before moving" acikken hareket hemen yurutulmez: once
secenekler (robotun IK duruslari, LIN/PTP/OMPL) planlanip gosterilir, kullanici
birini secince YALNIZCA o plan yurutulur.

NEDEN AYRI MODUL: movecobot.py import edilirken rclpy, moveit_py ve Fairino
SDK'si gerekiyor. Karar veren kurallar robot olmadan test edilebilsin diye
burada (bkz. ik_secimi.py, safety.py ile ayni gerekce).
"""

from __future__ import annotations

import math

EKLEM_ADLARI = ("j1", "j2", "j3", "j4", "j5", "j6")

# Onizlenen yol yurutulmeden once kolun, planin BASLANGIC acilarindan en cok bu
# kadar oynamis olmasina izin verilir (derece, her eklem ayri). Daha fazlaysa
# plan eskimis sayilir ve yurutulmez; yeniden planlanmasi istenir.
# Kullanici karari (15 Eylul). Ayni deger movecobot'ta hareket sonrasi varis
# dogrulamasinda ve surucunun servo moduna geri giris sicrama korumasinda
# kullaniliyor.
ONIZLEME_BASLANGIC_TOLERANSI_DEG = 2.0


def baslangic_farki(plan_baslangic_deg: list[float], simdiki_deg: list[float]) -> float:
    """Planin baslangici ile kolun su anki acilari arasinda EN COK farkli
    eklemin farki (derece).

    Eksik eklem ya da gercek sayi olmayan deger (NaN, sonsuz) varsa sonsuz
    doner: boyle bir okuma "uyusuyor" SAYILMAZ.
    """
    if len(plan_baslangic_deg) != 6 or len(simdiki_deg) != 6:
        return math.inf
    en_buyuk = 0.0
    for a, b in zip(plan_baslangic_deg, simdiki_deg):
        a, b = float(a), float(b)
        if not (math.isfinite(a) and math.isfinite(b)):
            return math.inf
        en_buyuk = max(en_buyuk, abs(a - b))
    return en_buyuk


def baslangic_uyuyor_mu(plan_baslangic_deg: list[float], simdiki_deg: list[float],
                        tolerans_deg: float = ONIZLEME_BASLANGIC_TOLERANSI_DEG) -> bool:
    return baslangic_farki(plan_baslangic_deg, simdiki_deg) <= tolerans_deg


def en_cok_donen_eklem(baslangic_deg: list[float],
                       noktalar_deg: list[list[float]]) -> tuple[str | None, float | None]:
    """Planin HER noktasini baslangicla karsilastirip yol boyunca en cok donen
    eklemi ve donusunu (derece) verir.

    Yalnizca bitis noktasina bakilmaz: bir eklem yolda savrulup geri donebilir
    (bkz. safety.asiri_eklem_hareketi). Nokta yoksa ya da gercek sayi olmayan
    deger varsa (None, None) - arayuze JSON'da gecersiz olan sonsuz gitmesin.
    """
    eklem, en_cok = None, None
    for nokta in noktalar_deg:
        for i, (a, b) in enumerate(zip(nokta, baslangic_deg)):
            fark = abs(float(a) - float(b))
            if not math.isfinite(fark):
                return None, None
            if en_cok is None or fark > en_cok:
                eklem, en_cok = EKLEM_ADLARI[i], fark
    return eklem, en_cok


def secenek_ozeti(sira: int, etiket: str, yontem: str | None, baslangic_deg: list[float],
                  noktalar_deg: list[list[float]], zamanlar_s: list[float],
                  problem: dict | None, hedef_deg: list[float] | None) -> dict:
    """Bir secenegin arayuze giden (JSON) ozeti.

    sira 1'den baslar; calistirma istegi bu numarayla gelir. etiket arayuzde
    gorunen ad ("Straight line", "Arm configuration 2", "Joint move").

    ok: plan var VE guvenlik kontrollerinden gecti. Reddedilen bir secenegin
    plani varsa noktalari yine gonderilir (neden reddedildigi gorulsun diye),
    ama calistirilamaz - bu karar sunucuda da ayrica verilir (server.py).
    """
    eklem, derece = en_cok_donen_eklem(baslangic_deg, noktalar_deg)
    return {
        "index": sira,
        "label": etiket,
        "method": yontem,
        "ok": problem is None and len(noktalar_deg) > 0,
        "reason": problem.get("reason") if problem else None,
        "details": problem.get("details") if problem else None,
        "max_joint": eklem,
        "max_deg": None if derece is None else round(derece, 1),
        "duration_s": round(zamanlar_s[-1], 2) if zamanlar_s else None,
        "goal_joints": None if hedef_deg is None else [round(float(v), 3) for v in hedef_deg],
        "points": [[round(float(v), 3) for v in nokta] for nokta in noktalar_deg],
        "times": [round(float(t), 3) for t in zamanlar_s],
    }

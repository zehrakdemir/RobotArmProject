"""Hareket guvenlik kurallarinin SAF (bagimliliksiz) kismi.

NEDEN AYRI MODUL: movecobot.py import edilirken rclpy, moveit_py ve Fairino
SDK'si gerekiyor - yani bu kurallar ancak tam bir robot ortaminda test
edilebiliyordu, pratikte de hic test edilmiyordu. Guvenlik kurallari, tam da
test edilmesi en gerekli olan yerdir. Buradaki fonksiyonlar sadece sayi
listeleri alip karar donduruyor, boylece temiz bir makinede de kosuyorlar.
"""

from __future__ import annotations


def asiri_eklem_hareketi(
    baslangic_deg: list[float],
    plan_acilari_deg: list[list[float]],
    sinir_deg: float,
) -> dict | None:
    """Kucuk bir goreceli hareket icin plan asiri eklem donusu iceriyor mu?

    CANLI OLAY (bu kuralin var olma sebebi): arayuzden TCP'yi arac ekseninde
    SADECE 10mm oteleme komutu verildi. Uretilen plan j1'i 37.08 -> 69.94
    dereceye, yani 32.9 derece dondurecekti; kol genis bir yay cizerek
    savruldu ve kullanici acil stopa basmak zorunda kaldi.

    Zemin ve self-collision kontrolleri bunu YAKALAYAMAZ: o yay bos havada,
    iki kontrol de ihlal gormuyor. Carpilacak sey MoveIt'in modelinde OLMAYAN
    gercek dunyadaki bir nesneydi. Yakalayan sey oranti olmali - birkac
    milimetrelik oteleme icin kolun onlarca derece donmesi, NEREYE gittiginden
    bagimsiz olarak basli basina anormaldir.

    PLANIN TAMAMI kontrol ediliyor, sadece bitis noktasi degil: savrulma ara
    noktalarda oluyor, bitis noktasi baslangica yakin olsa bile yol
    tehlikeli olabilir.

    Ihlal varsa sebebi anlatan bir sozluk, yoksa None doner.
    """
    en_kotu_eksen, en_kotu_sapma, en_kotu_adim = 0, 0.0, 0

    for adim, acilar in enumerate(plan_acilari_deg):
        for eksen in range(min(len(acilar), len(baslangic_deg))):
            sapma = abs(acilar[eksen] - baslangic_deg[eksen])
            if sapma > en_kotu_sapma:
                en_kotu_eksen, en_kotu_sapma, en_kotu_adim = eksen, sapma, adim

    if en_kotu_sapma <= sinir_deg:
        return None

    return {
        "success": False,
        "reason": "excessive_joint_motion",
        "details": (
            f"kucuk bir goreceli hareket icin j{en_kotu_eksen + 1} "
            f"{en_kotu_sapma:.1f} derece donecekti (sinir: {sinir_deg:.0f} derece, "
            f"plandaki adim {en_kotu_adim}/{len(plan_acilari_deg)}). "
            "Plan REDDEDILDI - robot hareket etmedi. Planlayici buyuk ihtimalle "
            "IK'nin baska bir daline atliyor; hedefe eklem acilariyla gitmeyi dene "
            "ya da once kolu biraz farkli bir duruma getir."
        ),
    }

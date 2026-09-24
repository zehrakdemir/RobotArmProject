"""Hareket guvenlik kurallarinin SAF (bagimliliksiz) kismi.

NEDEN AYRI MODUL: movecobot.py import edilirken rclpy, moveit_py ve Fairino
SDK'si gerekiyor - yani bu kurallar ancak tam bir robot ortaminda test
edilebiliyordu, pratikte de hic test edilmiyordu. Guvenlik kurallari, tam da
test edilmesi en gerekli olan yerdir. Buradaki fonksiyonlar sadece sayi
listeleri alip karar donduruyor, boylece temiz bir makinede de kosuyorlar.
"""

from __future__ import annotations

import math


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


def tekil_ihlali(kosul_sayilari: list[float], esik: float, ne: str) -> dict | None:
    """Plan (ya da tek bir durus) tekile cok yakin mi?

    kosul_sayilari: her nokta icin flanstaki Jacobian'in kosul sayisi
    (movecobot.jacobian_kosul_sayisi). ne: mesajda neyin kontrol edildigi.

    NEDEN: tekil yakininda TCP'ye gore bir hareket (TCP/Tool jog, TCP pozu,
    LIN) icin eklemlerin cok fazla ve cok hizli donmesi gerekir; bilek ya da
    taban savrulur. Zemin ve self-collision kontrolleri bunu gormez, yol bos
    havada. Eklem hareketleri bu kontrolden GECMEZ: tekilden etkilenmezler
    ve tekilden cikmanin yolu onlardir.

    NaN ya da eksi sonsuz tekil sayilir (hesap bozulduysa guvenli taraf;
    kosul sayisi eksi olamaz). Esigin tam kendisi ihlal degil, ustu ihlal.
    Ihlal varsa sebebi anlatan sozluk, yoksa None.
    """
    if not kosul_sayilari:
        return None
    degerler = [k if math.isfinite(k) else math.inf for k in kosul_sayilari]
    en_kotu_adim = max(range(len(degerler)), key=lambda i: degerler[i])
    en_kotu = degerler[en_kotu_adim]
    if en_kotu <= esik:
        return None

    deger = "infinite" if math.isinf(en_kotu) else f"{en_kotu:.0f}"
    konum = f" (step {en_kotu_adim}/{len(degerler)})" if len(degerler) > 1 else ""
    return {
        "success": False,
        "reason": "singularity",
        "details": (
            f"{ne} is too close to a singular pose: Jacobian condition number {deger} "
            f"> {esik:.0f}{konum}. Not planned - robot did not move. Move away with "
            "joint jog or an absolute joint target, then try again."
        ),
    }


def bilek_tekili(duruslar_deg: list[list[float]], pay_deg: float, ne: str) -> dict | None:
    """Bir durus (ya da planin bir noktasi) bilek tekiline cok yakin mi?

    Bilek tekili: j5 = 0 ya da 180 derece (Fairino kilavuzu 9.21). duruslar_deg:
    [j1..j6] derece listeleri. pay_deg: j5'in 0/180'e en az ne kadar uzak
    olmasi gerektigi.

    NEDEN KOSUL SAYISINA EK (canli, 15 Eylul): j5=0 durusunda base +Y 10 mm
    jog'u PTP'ye dustu; bilek j4 -109, j6 +90 derece dondu, uc 10 mm'lik
    dogrudan ~39 mm sapti (modelle hesap). Varilan durusta j5=1.69 derece ama
    kosul sayisi 99.3 idi - esik 100'un altinda, kosul sayisi tek basina
    bir sonraki jog'u durdurmayacakti. Kosul sayisi kolun diger acilarina gore
    degisiyor (ev pozunda j5=2 derece 324), bilek payi ise acidan dogrudan.

    Gercek bir sayi olmayan aci (NaN, sonsuz) tekil sayilir: "emin degilsem
    dur". Eskiden yalnizca NaN yakalaniyordu; sonsuz % 180 NaN verdigi ve NaN
    ile her karsilastirma yanlis oldugu icin sonsuz, planin ilk noktasi
    degilse sessizce atlaniyordu (guvenlik-inceleyici, 15 Eylul; olculdu).
    Payin tam kendisi ihlal degil, alti ihlal.
    """
    en_yakin, en_yakin_adim = None, 0
    for adim, q in enumerate(duruslar_deg):
        if not math.isfinite(q[4]):
            uzaklik = 0.0
        else:
            m = abs(q[4]) % 180.0
            uzaklik = min(m, 180.0 - m)
        if en_yakin is None or uzaklik < en_yakin:
            en_yakin, en_yakin_adim = uzaklik, adim
    if en_yakin is None or en_yakin >= pay_deg:
        return None

    konum = f" (step {en_yakin_adim}/{len(duruslar_deg)})" if len(duruslar_deg) > 1 else ""
    return {
        "success": False,
        "reason": "singularity",
        "details": (
            f"{ne} is too close to a wrist singularity: j5 is {en_yakin:.1f} degrees from "
            f"0/180 (minimum {pay_deg:.0f}){konum}. Not planned - robot did not move. Move j5 "
            "away with joint jog or an absolute joint target, then try again."
        ),
    }


def kavranan_nesne_kutusu_merkezi(
    tcp_offset_m: tuple[float, float, float],
    yukseklik_mm: float,
    kavrama_derinligi_mm: float | None = None,
) -> tuple[float, float, float]:
    """Kavranan nesnenin carpisma kutusunun, ALET (tip) cercevesindeki
    merkezi. Tool +Z nesneye dogru bakar (bkz. pickplace._move_relative_z:
    yaklasma +Z).

    NEDEN GEREKLI: SolidPrimitive.BOX verilen pozun ETRAFINDA SIMETRIK
    buyur. Kutu dogrudan TCP'ye konursa nesnenin yarisi TCP'nin yukarisinda
    (gripper govdesine dogru), yarisi asagisinda kabul edilir - yani kod
    "nesne tam ortasindan kavranmis" varsayar. Bu varsayim 21 Eylul'e kadar
    DOGRUYDU (vision'in GRASP_HEIGHT_FRACTION=0.5'i yuksekligin ortasindan
    kavriyordu). O gun kural degisti: artik UST YUZUN birkac cm altindan
    kavraniyor (webapp "Grasp depth below top"), yani nesnenin buyuk kismi
    kavrama noktasinin ALTINDA. Kutu buna gore kaydirilmazsa modeldeki
    nesne gercekte oldugundan (yukseklik/2 - derinlik) kadar YUKARIDA
    durur; hem zemin kontrolu nesnenin alt ucunu gormez hem de model
    gereksiz yere gripper govdesinin icine tasar.

    kavrama_derinligi_mm None ise eski (ortadan kavrama) davranis korunur -
    kayitli pick & place hareketlerinde derinlik bilinmiyor.
    """
    ox, oy, oz = tcp_offset_m
    h = yukseklik_mm / 1000.0
    if kavrama_derinligi_mm is None:
        derinlik = h / 2.0
    else:
        derinlik = min(max(kavrama_derinligi_mm / 1000.0, 0.0), h)
    return (ox, oy, oz + h / 2.0 - derinlik)


def kutunun_en_alcak_z(
    baglanti_konumu_m: tuple[float, float, float],
    baglanti_kuaterniyonu: tuple[float, float, float, float],
    merkez_m: tuple[float, float, float],
    boyut_m: tuple[float, float, float],
) -> float:
    """Bir baglantiya (link) SABIT bir kutunun DUNYA cercevesindeki en
    alcak noktasinin Z'si.

    check_trajectory_for_floor yalnizca link ORIJINLERININ Z'sine bakiyor;
    kavranan nesne bir link degil, gripper'in ucunda asili bir kutu -
    orijin kontrolu onu hic gormez. Burada kutunun 8 kosesinin en
    alcagi hesaplaniyor (eksene hizali sinir kutusunun alt yuzu).

    baglanti_kuaterniyonu: (x, y, z, w). Yalnizca donme matrisinin Z satiri
    gerekiyor - kutu, baglantiyla ayni yonelimde kabul ediliyor (pickplace
    kutuyu birim kuaterniyonla ekliyor).
    """
    qx, qy, qz, qw = baglanti_kuaterniyonu
    # Donme matrisinin Z (ucuncu) satiri.
    rz = (2.0 * (qx * qz - qw * qy),
          2.0 * (qy * qz + qw * qx),
          1.0 - 2.0 * (qx * qx + qy * qy))
    merkez_z = baglanti_konumu_m[2] + sum(rz[i] * merkez_m[i] for i in range(3))
    yayilim = sum(abs(rz[i]) * boyut_m[i] / 2.0 for i in range(3))
    return merkez_z - yayilim

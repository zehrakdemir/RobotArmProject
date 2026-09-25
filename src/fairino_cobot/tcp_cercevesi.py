"""Fairino TCP pozu <-> MoveIt pozu donusumleri. SAF modul (ROS/SDK yok),
robot olmadan test edilebilir (bkz. safety.py ile ayni gerekce).

NEDEN: MTC'nin ComputeIK asamasi hedefi MoveIt'in kendi IK'siyle cozer ve
"hangi cerceve hedefe gitsin" (ik_frame) sorusuna wrist3_link'e gore sabit bir
donusum ister. Fairino'nun TCP'si wrist3_link'ten hem OTELENMIS (tcp_offset_m)
hem de (tool tanimina gore) DONUK olabilir; yalnizca oteleme verilirse RX/RY/RZ
hedefi yanlis yonelime cozulur.

Fairino RX/RY/RZ = extrinsic ZYX: R = Rz(rz) * Ry(ry) * Rx(rx)
(GUNCEL_DURUM §2, kullanici olcumu 31 Agustos). Quaternion sirasi (x, y, z, w).
"""

from __future__ import annotations

import math

Quat = tuple[float, float, float, float]


def fairino_rpy_quat(rx_deg: float, ry_deg: float, rz_deg: float) -> Quat:
    """Fairino RX/RY/RZ (derece) -> quaternion (x, y, z, w)."""
    hr, hp, hy = (math.radians(a) / 2.0 for a in (rx_deg, ry_deg, rz_deg))
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_carp(a: Quat, b: Quat) -> Quat:
    """a * b (once b, sonra a donusu)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_ters(q: Quat) -> Quat:
    """Birim quaternion'un tersi (eslenigi)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def tcp_donusu(wrist3_q: Quat, tcp_rpy_deg: tuple[float, float, float]) -> Quat:
    """wrist3_link -> TCP sabit donusu: q_rel = q_wrist3^-1 * q_tcp.

    Ayni anda okunmus wrist3_link yonelimi (MoveIt FK) ve SDK'nin TCP
    RX/RY/RZ'si verilir. Tool tanimi degismedikce sonuc duruştan bagimsizdir.
    """
    return quat_carp(quat_ters(wrist3_q), fairino_rpy_quat(*tcp_rpy_deg))


def quat_dondur(q: Quat, v: tuple[float, float, float]) -> tuple[float, float, float]:
    """v vektorunu q ile dondurur (q * v * q^-1)."""
    x, y, z, _ = quat_carp(quat_carp(q, (v[0], v[1], v[2], 0.0)), quat_ters(q))
    return (x, y, z)


# ---------------------------------------------------------------------------
# Kavrama noktasi (parmak uclari) - pick & place
# ---------------------------------------------------------------------------
# STANDART YOL (25 Eylul, kullanici istegi "normalde nasil yapiyorlar"):
# robota "su noktayi tut" denir; o nokta aletin KAVRAMA noktasidir (parmak
# uclarinin ortasi). Fairino'da bu, aletin TCP'sini parmak ucuna kalibre
# etmek (WebApp arac koordinati, 4/6 nokta); MoveIt/MTC'de el linkine sabit
# bir ofsetle ik_frame vermek (MTC demo: setIKFrame(grasp_frame_transform,
# hand_frame)). Robotun TCP ayarina DOKUNULMAZ: el-goz kalibrasyonu bugunku
# TCP okumalariyla yapildi (GUNCEL_DURUM §12). Ofset yalnizca MTC'ye verilir.
#
# Bizim TCP parmak ucunda degil: wrist3'ten 199,4 mm (SDK+FK olcumu), parmak
# ucu ~19-20 mm daha asagida (17 Eylul masa olcumu 19, CAD 20,0). 19,
# 24 Eylul'den beri web panelinde kullanilan deger (kullanici karari) - 3
# canli kavrama bununla yapildi. Dogrulanmadi.
PARMAK_UCU_MM = 19.0

# Birakmada nesnenin alti yuzeyin bu kadar ustunde birakilir: MTC demo'sunun
# place_surface_offset'i (0.0001 m, config/panda_config.yaml). Tam 0'da
# nesne yuzeye degerdi. 24-25 Eylul'de 6,1 idi (+6 mm ilistirilen nesne
# padding'i); son inise nesne-zemin izni verilince (pickplace
# _birakmada_zemin_izni) gerek kalmadi.
BIRAKMA_PAYI_MM = 0.1


def kavrama_cercevesi(
    tcp_ofset_m: tuple[float, float, float],
    tcp_q: Quat,
    parmak_ucu_mm: float = PARMAK_UCU_MM,
) -> tuple[tuple[float, float, float], Quat]:
    """wrist3_link -> kavrama noktasi (parmak uclari) donusumu: TCP'nin
    kendi +Z'si boyunca parmak_ucu_mm ilerisi, yonelim TCP'ninkiyle ayni.
    Tool +Z nesneye dogru bakar (pickplace._move_relative_z: yaklasma +Z)."""
    ileri = quat_dondur(tcp_q, (0.0, 0.0, parmak_ucu_mm / 1000.0))
    return (tuple(o + d for o, d in zip(tcp_ofset_m, ileri)), tcp_q)


def birakma_noktasi(
    yuzey_xyz_mm: tuple[float, float, float],
    yukseklik_mm: float,
    kavrama_derinligi_mm: float,
    pay_mm: float = BIRAKMA_PAYI_MM,
) -> list[float]:
    """Birakmada kavrama noktasinin (parmak uclari) konumu (taban, mm).

    MTC demo'su gibi birakma yeri NESNENIN yeri olarak verilir: x, y =
    nesnenin konacagi nokta, z = uzerine konacagi YUZEYIN yuksekligi (masa
    / zemin icin 0). Parmak uclari nesnenin ust yuzunun kavrama_derinligi_mm
    altinda tutuyor; nesnenin alti yuzeyin pay_mm ustunde kalir."""
    x, y, z = yuzey_xyz_mm
    return [x, y, z + pay_mm + yukseklik_mm - kavrama_derinligi_mm]

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

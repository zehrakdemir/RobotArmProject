"""TCP cercevesi donusumleri. Robot, ROS2 ve SDK GEREKTIRMEZ."""

import math

import pytest

from fairino_cobot.tcp_cercevesi import fairino_rpy_quat, quat_carp, quat_ters, tcp_donusu


def _vektor_dondur(q, v):
    p = (v[0], v[1], v[2], 0.0)
    x, y, z, _ = quat_carp(quat_carp(q, p), quat_ters(q))
    return (x, y, z)


def _ayni_donus(a, b):
    """q ve -q ayni donus."""
    return abs(sum(x * y for x, y in zip(a, b))) == pytest.approx(1.0, abs=1e-9)


def test_asagi_bakan_tcp_z_ekseni_asagi():
    """RX=180, RY=0: TCP'nin Z ekseni tabanda -Z (GUNCEL_DURUM §2)."""
    z = _vektor_dondur(fairino_rpy_quat(180, 0, 0), (0, 0, 1))
    assert z == pytest.approx((0, 0, -1), abs=1e-9)


def test_rz_tool_x_eksenini_rz_yonune_ceviriyor():
    """RX=180, RY=0: tool X ekseni tabanda rz yonunde (§5 Gripper)."""
    for rz in (0, 35, -127.5, 90):
        x = _vektor_dondur(fairino_rpy_quat(180, 0, rz), (1, 0, 0))
        assert x == pytest.approx((math.cos(math.radians(rz)), math.sin(math.radians(rz)), 0),
                                  abs=1e-9)


def test_extrinsic_zyx_sirasi():
    """R = Rz * Ry * Rx: once X, sonra Y, en son Z etrafinda (sabit eksenler)."""
    q = fairino_rpy_quat(90, 90, 0)
    beklenen = quat_carp(fairino_rpy_quat(0, 90, 0), fairino_rpy_quat(90, 0, 0))
    assert _ayni_donus(q, beklenen)


def test_tcp_donusu_geri_uretir():
    """q_wrist3 * q_rel = q_tcp."""
    q_w3 = fairino_rpy_quat(12, -40, 77)
    tcp = (180.0, 0.0, -129.2)
    q_rel = tcp_donusu(q_w3, tcp)
    assert _ayni_donus(quat_carp(q_w3, q_rel), fairino_rpy_quat(*tcp))

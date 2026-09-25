"""TCP cercevesi donusumleri. Robot, ROS2 ve SDK GEREKTIRMEZ."""

import math

import pytest

from fairino_cobot.tcp_cercevesi import (
    BIRAKMA_PAYI_MM,
    PARMAK_UCU_MM,
    birakma_noktasi,
    fairino_rpy_quat,
    kavrama_cercevesi,
    quat_carp,
    quat_ters,
    tcp_donusu,
)


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


def test_kavrama_cercevesi_donuksuz_tcp():
    """Bizim olcum (24 Eylul): TCP wrist3'e gore donuksuz, (-0,13; 0,07;
    199,39) mm -> parmak ucu wrist3 +Z'sinde 19 mm ileride, yonelim ayni."""
    ofset, q = kavrama_cercevesi((-0.00013, 0.00007, 0.19939), (0.0, 0.0, 0.0, 1.0))
    assert ofset == pytest.approx((-0.00013, 0.00007, 0.19939 + PARMAK_UCU_MM / 1000), abs=1e-12)
    assert q == (0.0, 0.0, 0.0, 1.0)


def test_kavrama_cercevesi_donuk_tcp_kendi_z_ekseninde():
    """TCP wrist3'e gore donukse ofset TCP'nin KENDI Z'si boyunca eklenir."""
    q = fairino_rpy_quat(90, 0, 0)   # TCP Z'si wrist3'te -Y
    ofset, _ = kavrama_cercevesi((0.0, 0.0, 0.2), q, 20.0)
    assert ofset == pytest.approx((0.0, -0.02, 0.2), abs=1e-12)


def test_birakma_noktasi_zemine():
    """Zemine (z=0) 48 mm yuksek nesne, parmaklar ust yuzun 30 mm altinda:
    parmak ucu = 0 + pay + 48 - 30."""
    assert birakma_noktasi((-400.0, 200.0, 0.0), 48.0, 30.0) == pytest.approx(
        [-400.0, 200.0, BIRAKMA_PAYI_MM + 18.0])


def test_birakma_noktasi_kavramayla_ayni_yukseklik():
    """Ayni yuzeye birakilan nesne, kavrandigi yukseklige (pay kadar
    yukarida) doner: kavrama ucu = masa + h - d."""
    masa, h, d = 25.0, 50.0, 20.0
    kavrama_z = masa + h - d
    assert birakma_noktasi((0.0, 0.0, masa), h, d)[2] == pytest.approx(kavrama_z + BIRAKMA_PAYI_MM)

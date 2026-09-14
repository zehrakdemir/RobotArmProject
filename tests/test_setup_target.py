"""setup_target.py kaynak secimi testleri.

Robot, ROS2 veya SDK GEREKTIRMEZ - sahte klasor agaclari uzerinde calisir.

Korunan hata (3.9.3'te yasandi): surucu bir tag'den, fairino_msgs baska bir
tag'den gelince derleme kiriliyor:
  'fairino_msgs::msg::RobotNonrtState_' has no member named 'servoj_cmd_num'
"""

import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from fairino_cobot import setup_target


def sahte_surucu(klasor: Path) -> Path:
    """source_is_complete()'in aradigi dosyalari olan minimal surucu."""
    (klasor / "src").mkdir(parents=True)
    (klasor / "src" / "fairino_hardware_interface.cpp").write_text("// sahte\n", encoding="utf-8")
    (klasor / "libfairino" / "lib").mkdir(parents=True)
    (klasor / "libfairino" / "lib" / "libfairino.so.2").write_bytes(b"")
    return klasor


def sahte_tag_klonu(kok: Path, *surucu_adlari: str) -> Path:
    for ad in surucu_adlari:
        sahte_surucu(kok / ad)
    (kok / "fairino_msgs").mkdir(parents=True)
    (kok / "fairino_description").mkdir(parents=True)
    return kok


@pytest.fixture
def ortam(monkeypatch, tmp_path):
    ayna = tmp_path / "ayna"
    ev = tmp_path / "ev"
    kaynaklar = tmp_path / "kaynaklar"
    for d in (ayna, ev, kaynaklar):
        d.mkdir()
    (ayna / "fairino_msgs").mkdir()  # aynanin TEK fairino_msgs'i (3.9.9)
    monkeypatch.setattr(setup_target, "MIRROR", ayna)
    monkeypatch.setattr(setup_target, "HOME", ev)
    monkeypatch.setattr(setup_target, "LEGACY_SOURCES", ev / "fairino_hardware_sources")
    monkeypatch.setenv("FAIRINO_SOURCES_DIR", str(kaynaklar))
    return SimpleNamespace(ayna=ayna, ev=ev, kaynaklar=kaynaklar)


def test_aynadaki_surum_aynadan_seciliyor(ortam):
    """Bugunku canli robotun yolu (3.9.8, aynada var) DEGISMEMELI."""
    sahte_surucu(ortam.ayna / "fairino_hardware_v3_9_8")

    hw = setup_target.find_hardware_source("3", "9", "8")

    assert hw == ortam.ayna / "fairino_hardware_v3_9_8"
    assert setup_target.find_msgs_source(hw, "fairino_msgs") == ortam.ayna / "fairino_msgs"


def test_aynada_olmayan_surum_indirilmeden_bulunmuyor(ortam):
    assert setup_target.find_hardware_source("3", "9", "2") is None


def test_indirilen_tam_klon_bulunuyor_ve_msgs_AYNI_klondan(ortam):
    klon = sahte_tag_klonu(ortam.kaynaklar / "frcobot_ros2_v3_9_2", "fairino_hardware")

    hw = setup_target.find_hardware_source("3", "9", "2")

    assert hw == klon / "fairino_hardware"
    # Asil nokta: mesajlar aynanin 3.9.9'undan DEGIL, surucunun yanindan.
    assert setup_target.find_msgs_source(hw, "fairino_msgs") == klon / "fairino_msgs"


def test_tam_klon_aynadan_once_geliyor(ortam):
    sahte_surucu(ortam.ayna / "fairino_hardware_v3_9_8")
    klon = sahte_tag_klonu(ortam.kaynaklar / "frcobot_ros2_v3_9_8", "fairino_hardware_v3_9_8")

    assert setup_target.find_hardware_source("3", "9", "8") == klon / "fairino_hardware_v3_9_8"


def test_cok_surumlu_tag_klonunda_surum_klasoru_seciliyor(ortam):
    """3.9.6 ve sonrasi tag'lerde duz 'fairino_hardware' ile surum klasorleri
    yan yana; GitHub'da SHA'lari farkli. Aynadaki (canli calisan) kalip surum
    klasoru."""
    klon = sahte_tag_klonu(
        ortam.kaynaklar / "frcobot_ros2_v3_9_10",
        "fairino_hardware", "fairino_hardware_v3_9_9", "fairino_hardware_v3_9_10",
    )

    assert setup_target.find_hardware_source("3", "9", "10") == klon / "fairino_hardware_v3_9_10"


def test_imajdaki_eski_393_yolu_calismaya_devam_ediyor(ortam):
    """Dockerfile 3.9.3'u /root/frcobot_ros2_v393'e klonluyor."""
    klon = sahte_tag_klonu(ortam.ev / "frcobot_ros2_v393", "fairino_hardware")

    hw = setup_target.find_hardware_source("3", "9", "3")

    assert hw == klon / "fairino_hardware"
    assert setup_target.find_msgs_source(hw, "fairino_msgs") == klon / "fairino_msgs"


def test_eksik_klon_atlaniyor(ortam):
    klon = ortam.kaynaklar / "frcobot_ros2_v3_9_2"
    (klon / "fairino_hardware" / "src").mkdir(parents=True)  # libfairino yok

    assert setup_target.find_hardware_source("3", "9", "2") is None


def test_indirilen_surum_mevcut_surumler_listesinde(ortam):
    sahte_tag_klonu(ortam.kaynaklar / "frcobot_ros2_v3_9_2", "fairino_hardware")
    sahte_surucu(ortam.ayna / "fairino_hardware_v3_9_8")

    surumler = setup_target.list_available_versions()

    assert "3.9.2" in surumler
    assert "3.9.8" in surumler


@pytest.mark.skipif(shutil.which("patch") is None, reason="patch komutu yok")
def test_yama_tutmayan_kaynakla_yamasiz_devam_edilmiyor(tmp_path):
    """Eskiden yama tutmayinca yalnizca robot_ip eklenip devam ediliyordu:
    UDP servo kanali ve hiz sinirlayici OLMAYAN bir surucu. Artik durmali."""
    frcobot = tmp_path / "frcobot_ros2"
    cpp = frcobot / "fairino_hardware" / "src" / "fairino_hardware_interface.cpp"
    cpp.parent.mkdir(parents=True)
    cpp.write_text("// yamanin baglamina uymayan dosya\nint main() { return 0; }\n", encoding="utf-8")

    with pytest.raises(SystemExit):
        setup_target.apply_hardware_patch(frcobot, cpp)

    assert "robot_ip" not in cpp.read_text(encoding="utf-8")

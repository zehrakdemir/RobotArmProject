"""fairino-fetch-source testleri.

Robot, ROS2, SDK ve INTERNET gerektirmez: tag secimi saf fonksiyonlarla,
indirme ise test icinde kurulan yerel bir git deposuyla deneniyor.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

from fairino_cobot import fetch_source, setup_target
from fairino_cobot.fetch_source import FetchError

# Tag adlari GitHub'daki gercek adlar (14 Eylul 2026); SHA'lar sahte.
LS_REMOTE = (
    "1111111111111111111111111111111111111111\trefs/tags/V3.0.0_RobotV3.9.2\n"
    "2222222222222222222222222222222222222222\trefs/tags/V3.0.0_RobotV3.9.2^{}\n"
    "3333333333333333333333333333333333333333\trefs/tags/v3.0.0_RobotV3.9.8\n"
    "4444444444444444444444444444444444444444\trefs/tags/v3.0.0_RobotV3.9.8_v1.1\n"
    "5555555555555555555555555555555555555555\trefs/tags/V3.0.0_RobotV3.9.9\n"
)

git_gerekli = pytest.mark.skipif(shutil.which("git") is None, reason="git yok")


# ---------------------------------------------------------------------------
# Tag secimi
# ---------------------------------------------------------------------------

def test_tagler_ayristiriliyor_peel_satirlari_atiliyor():
    assert fetch_source.tagleri_ayristir(LS_REMOTE) == [
        "V3.0.0_RobotV3.9.2",
        "v3.0.0_RobotV3.9.8",
        "v3.0.0_RobotV3.9.8_v1.1",
        "V3.0.0_RobotV3.9.9",
    ]


def test_kucuk_v_ile_yazilmis_tag_bulunuyor_ekli_olan_listeleniyor():
    tagler = fetch_source.tagleri_ayristir(LS_REMOTE)
    assert fetch_source.tag_sec(tagler, "3", "9", "8") == (
        "v3.0.0_RobotV3.9.8", ["v3.0.0_RobotV3.9.8_v1.1"])


def test_yalnizca_ekli_tag_varsa_sessizce_secilmiyor():
    assert fetch_source.tag_sec(["V3.0.0_RobotV3.9.9_v1.1"], "3", "9", "9") == (
        None, ["V3.0.0_RobotV3.9.9_v1.1"])


def test_benzer_surum_karistirilmiyor():
    """3.9.1 istenince 3.9.10 secilmemeli."""
    assert fetch_source.tag_sec(["V3.0.0_RobotV3.9.10"], "3", "9", "1") == (None, [])


@pytest.mark.parametrize("metin, beklenen", [
    ("3.9.2", ("3", "9", "2")),
    ("V3.9.8", ("3", "9", "8")),
    (" 3.10.0 ", ("3", "10", "0")),
])
def test_surum_ayristiriliyor(metin, beklenen):
    assert fetch_source.surum_ayristir(metin) == beklenen


@pytest.mark.parametrize("metin", ["3.9", "3.9.2.1", "abc", ""])
def test_gecersiz_surum_hata_veriyor(metin):
    with pytest.raises(FetchError):
        fetch_source.surum_ayristir(metin)


# ---------------------------------------------------------------------------
# Indirme (yerel sahte depo ile)
# ---------------------------------------------------------------------------

def _git(*args, cwd):
    subprocess.run(
        ["git", "-c", "user.name=test", "-c", "user.email=test@example.com", *args],
        cwd=cwd, check=True, capture_output=True,
    )


def sahte_frcobot_deposu(kok: Path, tag: str, surucu_adlari=("fairino_hardware",),
                         libfairino=True) -> str:
    """frcobot_ros2 duzenini taklit eden yerel bir depo; URL'sini doner."""
    depo = kok / "depo"
    for ad in surucu_adlari:
        (depo / ad / "src").mkdir(parents=True)
        (depo / ad / "src" / "fairino_hardware_interface.cpp").write_text("// sahte\n", encoding="utf-8")
        if libfairino:
            (depo / ad / "libfairino" / "lib").mkdir(parents=True)
            (depo / ad / "libfairino" / "lib" / "libfairino.so.2").write_text("sahte\n", encoding="utf-8")
    for paket in ("fairino_msgs", "fairino_description", "fairino3_v6_moveit2_config"):
        (depo / paket).mkdir(parents=True)
        (depo / paket / "package.xml").write_text("<package/>\n", encoding="utf-8")
    _git("init", "-q", cwd=depo)
    _git("add", ".", cwd=depo)
    _git("commit", "-q", "-m", "sahte", cwd=depo)
    _git("tag", tag, cwd=depo)
    return depo.as_uri()


@pytest.fixture
def kaynaklar(monkeypatch, tmp_path):
    hedef = tmp_path / "kaynaklar"
    monkeypatch.setenv("FAIRINO_SOURCES_DIR", str(hedef))
    monkeypatch.setattr(setup_target, "MIRROR", tmp_path / "ayna_yok")
    monkeypatch.setattr(setup_target, "HOME", tmp_path / "ev")
    monkeypatch.setattr(setup_target, "LEGACY_SOURCES", tmp_path / "ev" / "fairino_hardware_sources")
    return hedef


@git_gerekli
def test_indirilen_tag_setup_target_tarafindan_bulunuyor(kaynaklar, tmp_path):
    url = sahte_frcobot_deposu(tmp_path, "V3.0.0_RobotV3.9.2")

    hedef, tag = fetch_source.indir("3", "9", "2", repo_url=url)

    assert tag == "V3.0.0_RobotV3.9.2"
    assert hedef == kaynaklar / "frcobot_ros2_v3_9_2"
    assert not (hedef / ".git").exists()
    assert not (hedef / "fairino3_v6_moveit2_config").exists(), "gereksiz klasor indirilmemeli"
    hw = setup_target.find_hardware_source("3", "9", "2")
    assert hw == hedef / "fairino_hardware"
    assert setup_target.find_msgs_source(hw, "fairino_msgs") == hedef / "fairino_msgs"


@git_gerekli
def test_cok_surumlu_tagde_yalnizca_istenen_surum_klasoru_iniyor(kaynaklar, tmp_path):
    url = sahte_frcobot_deposu(
        tmp_path, "V3.0.0_RobotV3.9.10",
        ("fairino_hardware", "fairino_hardware_v3_9_9", "fairino_hardware_v3_9_10"),
    )

    hedef, _ = fetch_source.indir("3", "9", "10", repo_url=url)

    assert (hedef / "fairino_hardware_v3_9_10").is_dir()
    assert not (hedef / "fairino_hardware_v3_9_9").exists()
    assert setup_target.find_hardware_source("3", "9", "10") == hedef / "fairino_hardware_v3_9_10"


@git_gerekli
def test_tag_yoksa_hata_veriyor_klasor_olusmuyor(kaynaklar, tmp_path):
    url = sahte_frcobot_deposu(tmp_path, "V3.0.0_RobotV3.9.9")

    with pytest.raises(FetchError, match="3.9.2 icin tag yok"):
        fetch_source.indir("3", "9", "2", repo_url=url)

    assert not (kaynaklar / "frcobot_ros2_v3_9_2").exists()


@git_gerekli
def test_eksik_tag_yarim_klasor_birakmiyor(kaynaklar, tmp_path):
    """Dogrulanmamis indirme setup_target'a 'kaynak var' diye gorunmemeli."""
    url = sahte_frcobot_deposu(tmp_path, "V3.0.0_RobotV3.9.2", libfairino=False)

    with pytest.raises(FetchError, match="kullanilabilir fairino_hardware yok"):
        fetch_source.indir("3", "9", "2", repo_url=url)

    assert list(kaynaklar.iterdir()) == []


def test_var_olan_klasor_force_olmadan_ezilmiyor(kaynaklar):
    (kaynaklar / "frcobot_ros2_v3_9_2").mkdir(parents=True)

    with pytest.raises(FetchError, match="--force"):
        fetch_source.indir("3", "9", "2", repo_url="kullanilmamali")

"""paths.py testleri.

Bu testler ROBOT, ROS2 veya Fairino SDK GEREKTIRMEZ - CI'da temiz bir
makinede kosuyorlar. paths.py'nin bunu mumkun kilmasi kasitli: modul
hicbir donanim kutuphanesini import etmiyor.
"""

import importlib
import os

import pytest

from fairino_cobot import paths


@pytest.fixture
def temiz_ortam(monkeypatch, tmp_path):
    """FAIRINO_* degiskenlerini temizler, state dizinini tmp'ye alir."""
    for k in list(os.environ):
        if k.startswith("FAIRINO_"):
            monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv("FAIRINO_STATE_DIR", str(tmp_path / "state"))
    return tmp_path


# ---------------------------------------------------------------------------
# .env yukleme
# ---------------------------------------------------------------------------

def test_dotenv_yorum_ve_tirnaklari_ayikliyor(monkeypatch, tmp_path):
    env = tmp_path / ".env"
    env.write_text(
        "# yorum satiri\n"
        "\n"
        'FAIRINO_TEST_A="tirnakli"\n'
        "FAIRINO_TEST_B=tirnaksiz\n"
        "bozuk-satir-esittir-yok\n",
        encoding="utf-8",
    )
    monkeypatch.delenv("FAIRINO_TEST_A", raising=False)
    monkeypatch.delenv("FAIRINO_TEST_B", raising=False)

    paths.load_dotenv(env)

    assert os.environ["FAIRINO_TEST_A"] == "tirnakli"
    assert os.environ["FAIRINO_TEST_B"] == "tirnaksiz"


def test_dotenv_mevcut_degiskeni_EZMEZ(monkeypatch, tmp_path):
    """Kabuktan verilen deger .env'den once gelir - yoksa kullanici
    'export FAIRINO_ROBOT_IP=...' ile gecici bir deneme yapamazdi."""
    env = tmp_path / ".env"
    env.write_text("FAIRINO_TEST_C=dosyadan\n", encoding="utf-8")
    monkeypatch.setenv("FAIRINO_TEST_C", "kabuktan")

    paths.load_dotenv(env)

    assert os.environ["FAIRINO_TEST_C"] == "kabuktan"


def test_dotenv_olmayan_dosyada_patlamiyor(tmp_path):
    paths.load_dotenv(tmp_path / "boyle-bir-dosya-yok")


# ---------------------------------------------------------------------------
# Ortam degiskeni onceligi
# ---------------------------------------------------------------------------

def test_ros2_ws_ortam_degiskenini_kullaniyor(monkeypatch, tmp_path):
    monkeypatch.setenv("FAIRINO_ROS2_WS", str(tmp_path / "baska_ws"))
    assert paths.ros2_ws() == tmp_path / "baska_ws"


def test_ros2_ws_varsayilani(temiz_ortam):
    assert paths.ros2_ws() == paths.HOME / "ros2_ws"


def test_tilde_genisletiliyor(monkeypatch):
    monkeypatch.setenv("FAIRINO_MIRROR_DIR", "~/ayna")
    assert paths.mirror_dir() == paths.HOME / "ayna"


def test_bos_deger_varsayilana_dusuyor(monkeypatch):
    """Bos birakilmis bir .env satiri (FAIRINO_ROS2_WS=) yolu bozmamali."""
    monkeypatch.setenv("FAIRINO_ROS2_WS", "   ")
    assert paths.ros2_ws() == paths.HOME / "ros2_ws"


# ---------------------------------------------------------------------------
# Robot IP
# ---------------------------------------------------------------------------

def test_robot_ip_verilmeyince_None(temiz_ortam):
    assert paths.robot_ip_override() is None


def test_robot_ip_verilince_donuyor(monkeypatch):
    monkeypatch.setenv("FAIRINO_ROBOT_IP", "10.0.0.5")
    assert paths.robot_ip_override() == "10.0.0.5"


# ---------------------------------------------------------------------------
# Calisma zamani durumu
# ---------------------------------------------------------------------------

def test_state_dir_olusturuluyor(temiz_ortam, tmp_path):
    d = paths.state_dir()
    assert d.is_dir()
    assert d == tmp_path / "state"


def test_durum_dosyalari_state_dizininde(temiz_ortam, tmp_path):
    for f in (paths.moves_file(), paths.state_cache_file(), paths.draft_file()):
        assert f.parent == tmp_path / "state"


def test_state_dir_xdg_degiskenine_saygili(monkeypatch, tmp_path):
    monkeypatch.delenv("FAIRINO_STATE_DIR", raising=False)
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "xdg"))
    assert paths.state_dir() == tmp_path / "xdg" / "fairino-cobot"


# ---------------------------------------------------------------------------
# SDK cozumleme
# ---------------------------------------------------------------------------

def test_sdk_ortam_degiskeniyle_bulunuyor(monkeypatch, tmp_path):
    sdk = tmp_path / "sdk" / "linux"
    (sdk / "fairino").mkdir(parents=True)
    monkeypatch.setenv("FAIRINO_SDK_DIR", str(sdk))
    assert paths.sdk_dir() == sdk


def test_sdk_bulunamayinca_nereye_bakildigini_soyluyor(monkeypatch, tmp_path):
    """Sessizce yanlis yola dusmek yerine anlamli bir hata - bu davranis
    bilerek boyle, hata mesaji cozumu de icermeli."""
    monkeypatch.setenv("FAIRINO_SDK_DIR", str(tmp_path / "yok"))
    monkeypatch.setattr(paths, "LEGACY_CLIENT_DIR", tmp_path / "yok2")
    monkeypatch.setattr(paths, "HOME", tmp_path / "bos_ev")
    monkeypatch.setattr(paths, "REPO_ROOT", tmp_path / "bos_repo")
    monkeypatch.setattr(paths, "_scan_for_sdk", lambda: None)

    with pytest.raises(paths.PathError) as e:
        paths.sdk_dir()

    mesaj = str(e.value)
    assert "fairino-python-sdk" in mesaj
    assert "FAIRINO_SDK_DIR" in mesaj


# ---------------------------------------------------------------------------
# detect_robot.py
# ---------------------------------------------------------------------------

def test_detect_robot_paket_icinde_bulunuyor():
    p = paths.detect_robot_script()
    assert p.name == "detect_robot.py"
    assert p.exists()


# ---------------------------------------------------------------------------
# Paketin kendisi
# ---------------------------------------------------------------------------

def test_paket_donanim_kutuphanesi_olmadan_import_ediliyor():
    """fairino_cobot/__init__.py alt modulleri import ETMEMELI - yoksa
    ROS2 kurulu olmayan bir makinede (CI dahil) paket hic acilmazdi."""
    mod = importlib.import_module("fairino_cobot")
    assert hasattr(mod, "__version__")

"""tools/apply_ros2_patches.py testleri.

Yama araci vendor agacinda MODELE OZGU isimler uretiyor
(fairino3_v6_group, fairino10_controller...). Yanlis isim uretmesi sessiz
bir hata olurdu: dosya yazilir, YAML gecerli kalir, ama MoveIt o bolumu
hic okumaz. Bu yuzden isim cikarma mantigi burada test ediliyor.

Gercek robot veya ROS2 gerektirmez - sahte paket agaclari uzerinde calisir.
"""

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def arac():
    """tools/ bir paket degil, dosyadan yukleniyor."""
    yol = REPO_ROOT / "tools" / "apply_ros2_patches.py"
    spec = importlib.util.spec_from_file_location("apply_ros2_patches", yol)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["apply_ros2_patches"] = mod
    spec.loader.exec_module(mod)
    return mod


def sahte_paket(kok: Path, model: str) -> Path:
    """Gercek frcobot_ros2 duzenini taklit eden minimal bir paket."""
    pkg = kok / f"fairino{model}_v6_moveit2_config"
    (pkg / "config").mkdir(parents=True)
    (pkg / "config" / f"fairino{model}_v6.srdf").write_text(
        f'<?xml version="1.0"?>\n'
        f'<robot name="fairino{model}_v6">\n'
        f'    <group name="fairino{model}_v6_group">\n'
        f'        <chain base_link="base_link" tip_link="wrist3_link"/>\n'
        f'    </group>\n'
        f'</robot>\n',
        encoding="utf-8",
    )
    (pkg / "config" / "ros2_controllers.yaml").write_text(
        "controller_manager:\n"
        "  ros__parameters:\n"
        "    update_rate: 100\n"
        "\n"
        f"    fairino{model}_controller:\n"
        "      type: joint_trajectory_controller/JointTrajectoryController\n"
        "\n"
        "    joint_state_broadcaster:\n"
        "      type: joint_state_broadcaster/JointStateBroadcaster\n"
        "\n"
        f"fairino{model}_controller:\n"
        "  ros__parameters:\n"
        "    joints:\n"
        "      - j1\n"
        "      - j2\n"
        "      - j3\n"
        "      - j4\n"
        "      - j5\n"
        "      - j6\n"
        "    command_interfaces:\n"
        "      - position\n"
        "    state_interfaces:\n"
        "      - position\n",
        encoding="utf-8",
    )
    (pkg / "config" / "ompl_planning.yaml").write_text(
        "planning_plugin: ompl_interface/OMPLPlanner\n", encoding="utf-8"
    )
    return pkg


# ---------------------------------------------------------------------------
# Isim cikarma
# ---------------------------------------------------------------------------

def test_grup_adi_srdften_okunuyor(arac, tmp_path):
    pkg = sahte_paket(tmp_path, "10")
    assert arac.srdf_group_name(pkg) == "fairino10_v6_group"


def test_controller_adi_top_level_bloktan_okunuyor(arac, tmp_path):
    """controller_manager ALTINDA da ayni isim geciyor; girintili olan
    degil, top-level blok secilmeli."""
    pkg = sahte_paket(tmp_path, "20")
    text = (pkg / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8")
    ad, eklemler = arac.controller_block(text)
    assert ad == "fairino20_controller"
    assert eklemler == ["j1", "j2", "j3", "j4", "j5", "j6"]


# ---------------------------------------------------------------------------
# Uygulama
# ---------------------------------------------------------------------------

def test_ompl_modele_ozgu_grup_ekliyor(arac, tmp_path, monkeypatch):
    monkeypatch.setattr(arac, "PATCH_DIR", REPO_ROOT / "ros2" / "patches")
    pkg = sahte_paket(tmp_path, "5")

    sonuc = arac.patch_ompl(pkg, dry=False)

    icerik = (pkg / "config" / "ompl_planning.yaml").read_text(encoding="utf-8")
    assert "fairino5_v6_group:" in icerik
    assert "fairino3_v6_group:" not in icerik, "baska modelin grup adi sizmamali"
    assert "longest_valid_segment_fraction: 0.005" in icerik
    assert "fairino5_v6_group" in sonuc


def test_controllers_constraints_ekliyor(arac, tmp_path):
    pkg = sahte_paket(tmp_path, "16")

    arac.patch_controllers(pkg, dry=False)

    icerik = (pkg / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8")
    assert "constraints:" in icerik
    assert "goal_time: 2.0" in icerik
    for j in ("j1", "j6"):
        assert f"{j}: {{ trajectory: 0.0, goal: 0.05 }}" in icerik


def test_uretilen_yaml_gecerli(arac, tmp_path, monkeypatch):
    """Metin ekleyerek yaziyoruz - sonucun hala gecerli YAML oldugundan
    ve degerlerin dogru yere oturdugundan emin ol."""
    yaml = pytest.importorskip("yaml")
    monkeypatch.setattr(arac, "PATCH_DIR", REPO_ROOT / "ros2" / "patches")
    pkg = sahte_paket(tmp_path, "30")

    arac.patch_ompl(pkg, dry=False)
    arac.patch_controllers(pkg, dry=False)

    ompl = yaml.safe_load((pkg / "config" / "ompl_planning.yaml").read_text(encoding="utf-8"))
    assert ompl["fairino30_v6_group"]["longest_valid_segment_fraction"] == 0.005

    ctl = yaml.safe_load((pkg / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8"))
    kis = ctl["fairino30_controller"]["ros__parameters"]["constraints"]
    assert kis["goal_time"] == 2.0
    assert kis["stopped_velocity_tolerance"] == 0.0
    assert kis["j3"]["goal"] == 0.05
    # Hareket sirasindaki takip hatasi kontrolu KAPALI olmali - acik olsa
    # ServoJ gecikmesi yuzunden saglikli hareketler ortasinda abort olurdu.
    assert kis["j3"]["trajectory"] == 0.0


def test_idempotent(arac, tmp_path, monkeypatch):
    """Arac tekrar tekrar calistirilabilmeli - ikinci kez calisinca
    constraints i IKI KEZ eklememeli."""
    monkeypatch.setattr(arac, "PATCH_DIR", REPO_ROOT / "ros2" / "patches")
    pkg = sahte_paket(tmp_path, "3")

    arac.patch_ompl(pkg, dry=False)
    arac.patch_controllers(pkg, dry=False)
    ilk_ompl = (pkg / "config" / "ompl_planning.yaml").read_text(encoding="utf-8")
    ilk_ctl = (pkg / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8")

    assert "zaten" in arac.patch_ompl(pkg, dry=False)
    assert "zaten" in arac.patch_controllers(pkg, dry=False)

    assert (pkg / "config" / "ompl_planning.yaml").read_text(encoding="utf-8") == ilk_ompl
    assert (pkg / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8") == ilk_ctl


def test_dry_run_hicbir_sey_yazmiyor(arac, tmp_path, monkeypatch):
    monkeypatch.setattr(arac, "PATCH_DIR", REPO_ROOT / "ros2" / "patches")
    pkg = sahte_paket(tmp_path, "3mt")
    once = (pkg / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8")

    arac.patch_ompl(pkg, dry=True)
    arac.patch_controllers(pkg, dry=True)

    assert (pkg / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8") == once


def test_controller_son_blok_degilse_atliyor(arac, tmp_path):
    """Blok dosya sonuna ekleniyor; controller son top-level blok degilse
    yanlis yere yazmaktansa ATLAMALI."""
    pkg = sahte_paket(tmp_path, "5")
    yol = pkg / "config" / "ros2_controllers.yaml"
    yol.write_text(yol.read_text(encoding="utf-8") + "\nbaska_blok:\n  x: 1\n", encoding="utf-8")

    sonuc = arac.patch_controllers(pkg, dry=False)

    assert "ATLANDI" in sonuc
    assert "constraints:" not in yol.read_text(encoding="utf-8")


def test_ompl_dosya_yoksa_OLUSTURULUYOR(arac, tmp_path, monkeypatch):
    """Vendor'da ompl_planning.yaml GENELDE YOK.

    frcobot_ros2'nin model paketleri bu dosya olmadan geliyor ve MoveIt o
    durumda kendi varsayilanini kullaniyor - iceriginde sadece
    AddTimeOptimalParameterization var, yani jerk hic sinirlanmiyor ve robot
    yanlis-pozitif "Shaft collision fault" verebiliyor.

    "Dosya yoksa atla" demek, duzeltmenin EN COK gerektigi durumda sessizce
    uygulanmamasi olurdu. Once tam boyle bir hata vardi: temiz bir vendor
    klonunda (tag V3.0.0_RobotV3.9.9) dosya olmadigi icin yama atlaniyordu.
    """
    monkeypatch.setattr(arac, "PATCH_DIR", REPO_ROOT / "ros2" / "patches")
    pkg = sahte_paket(tmp_path, "5")
    (pkg / "config" / "ompl_planning.yaml").unlink()

    sonuc = arac.patch_ompl(pkg, dry=False)

    icerik = (pkg / "config" / "ompl_planning.yaml").read_text(encoding="utf-8")
    assert "OLUSTURULDU" in sonuc
    assert "fairino5_v6_group:" in icerik
    # Jerk duzeltmesi de gelmis olmali - dosyanin asil varlik sebebi bu.
    assert "AddRuckigTrajectorySmoothing" in icerik
    assert "AddTimeOptimalParameterization" in icerik


def test_ompl_dry_run_da_dosya_olusturmuyor(arac, tmp_path, monkeypatch):
    monkeypatch.setattr(arac, "PATCH_DIR", REPO_ROOT / "ros2" / "patches")
    pkg = sahte_paket(tmp_path, "5")
    (pkg / "config" / "ompl_planning.yaml").unlink()

    arac.patch_ompl(pkg, dry=True)

    assert not (pkg / "config" / "ompl_planning.yaml").exists()


# ---------------------------------------------------------------------------
# Eklem ivme limiti
#
# CANLI OLAY: FR3 config'inde ivme limiti KAPALI geliyordu, bu yuzden Pilz PTP
# ve LIN hic planlayamiyor ("acceleration limit not set for group") ve HER
# hareket sessizce OMPL/RRTConnect'e duesuyordu. 10mm'lik bir TCP otelemesi
# icin uretilen plan j1'i 32.9 derece donduruyordu; kol savruldu, kullanici
# acil stopa basmak zorunda kaldi.
# ---------------------------------------------------------------------------

VENDOR_JOINT_LIMITS = """default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

joint_limits:
  j1:
    has_velocity_limits: true
    max_velocity: 3.1499999999999999
    has_acceleration_limits: false
    max_acceleration: 0
  j2:
    has_velocity_limits: true
    max_velocity: 3.2000000000000002
    has_acceleration_limits: false
    max_acceleration: 0
"""


def _limit_dosyasi(pkg: Path, icerik: str = VENDOR_JOINT_LIMITS) -> Path:
    yol = pkg / "config" / "joint_limits.yaml"
    yol.write_text(icerik, encoding="utf-8")
    return yol


def test_ivme_limiti_aciliyor(arac, tmp_path):
    pkg = sahte_paket(tmp_path, "3")
    yol = _limit_dosyasi(pkg)

    sonuc = arac.patch_joint_limits(pkg, dry=False)

    metin = yol.read_text(encoding="utf-8")
    assert "has_acceleration_limits: false" not in metin
    assert metin.count("has_acceleration_limits: true") == 2
    assert "2 eklem" in sonuc


def test_ivme_degeri_vendor_fr5_ile_ayni(arac, tmp_path):
    """Deger uydurma DEGIL - Fairino'nun kendi FR5 config'indeki 0.7.

    O dosya FR3'unkiyle hiz limitleri dahil birebir ayni; tek farki ivmenin
    acik ve 0.7 olmasi. Ayni uretici, ayni hiz profili, kendi verdigi deger.
    """
    yaml = pytest.importorskip("yaml")
    pkg = sahte_paket(tmp_path, "3")
    yol = _limit_dosyasi(pkg)

    arac.patch_joint_limits(pkg, dry=False)

    d = yaml.safe_load(yol.read_text(encoding="utf-8"))
    for eklem in ("j1", "j2"):
        assert d["joint_limits"][eklem]["has_acceleration_limits"] is True
        assert d["joint_limits"][eklem]["max_acceleration"] == 0.7
    # Hiz limitleri DEGISMEMELI - sadece ivme aciliyor.
    assert d["joint_limits"]["j1"]["max_velocity"] == pytest.approx(3.15)


def test_mevcut_ivme_degeri_ezilmiyor(arac, tmp_path):
    """Ivme zaten tanimliysa dokunma - kullanicinin kendi ayari olabilir."""
    pkg = sahte_paket(tmp_path, "3")
    ozel = VENDOR_JOINT_LIMITS.replace(
        "has_acceleration_limits: false", "has_acceleration_limits: true"
    ).replace("max_acceleration: 0\n", "max_acceleration: 1.5\n")
    yol = _limit_dosyasi(pkg, ozel)

    sonuc = arac.patch_joint_limits(pkg, dry=False)

    assert "zaten var" in sonuc
    assert "max_acceleration: 1.5" in yol.read_text(encoding="utf-8")


def test_ivme_limiti_dry_run_yazmiyor(arac, tmp_path):
    pkg = sahte_paket(tmp_path, "3")
    yol = _limit_dosyasi(pkg)
    onceki = yol.read_text(encoding="utf-8")

    arac.patch_joint_limits(pkg, dry=True)

    assert yol.read_text(encoding="utf-8") == onceki


def test_ivme_limiti_dosya_yoksa_atliyor(arac, tmp_path):
    """joint_limits.yaml olmayan bir pakette sessizce atlanmali, patlamamali."""
    pkg = sahte_paket(tmp_path, "3")

    assert "atlandi" in arac.patch_joint_limits(pkg, dry=False)

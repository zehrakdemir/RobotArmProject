#!/usr/bin/env python3
"""frcobot_ros2 (vendor) agacina bizim override'larimizi uygular.

NEDEN BIR ARAC: frcobot_ros2 destekledigi HER Fairino modeli icin ayri bir
`fairino<N>_v6_moveit2_config` paketi iceriyor ve detect_robot.py hangi robot
takiliysa onu seciyor. Yani duzeltmelerin TEK bir modele degil, olasi HER
modele uygulanmis olmasi gerekiyor - yoksa baska bir robot takildiginda
duzeltmesiz varsayilan config'e geri dusulur.

Eski apply_patch.sh bunu tek bir dosyayi her pakete KOPYALAYARAK yapiyordu.
Bu artik yetmiyor, cunku eklenen ayarlarin bir kismi MODELE OZGU isimler
iceriyor:

    ompl_planning.yaml     -> "fairino3_v6_group:" (grup adi modele gore)
    ros2_controllers.yaml  -> "fairino3_controller:" (controller adi modele gore)

Bu yuzden bu arac isimleri her paketin KENDI SRDF/yaml dosyasindan okuyup
uyguluyor. Idempotent: tekrar tekrar calistirilabilir.

Kullanim:
    python3 tools/apply_ros2_patches.py                # ~/ros2_ws/src/frcobot_ros2
    python3 tools/apply_ros2_patches.py --frcobot /yol/frcobot_ros2
    python3 tools/apply_ros2_patches.py --dry-run
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PATCH_DIR = REPO_ROOT / "ros2" / "patches"

# ---------------------------------------------------------------------------
# Ayarlanan degerler ve GEREKCELERI
# ---------------------------------------------------------------------------

# OMPL'in yol uzerinde iki carpisma kontrol noktasi arasinda biraktigi mesafe,
# durum uzayinin azami genisliginin orani olarak. OMPL'in kendi varsayilani
# 0.01 (%1) ve bu 6 eksenli bir kolda COK kaba - iki kontrol noktasi
# arasindan ince bir yakin-gecis "sizabiliyor".
#
# Bu paketlerde daha once HIC grup bolumu yoktu, dolayisiyla
# model_based_planning_context.cpp:278'e gore config bos kalip useConfig()
# hemen donuyordu, yani varsayilan gecerliydi.
#
# 0.001 daha siki olurdu ama planlama suresi ~10 kat artiyor.
LONGEST_VALID_SEGMENT_FRACTION = 0.005

# JointTrajectoryController hedef toleranslari (radyan / saniye).
#
# constraints bolumu tanimlanmayinca JTC HICBIR tolerans kontrolu yapmiyor:
# yorunge bitince goal'u KOSULSUZ "SUCCEEDED" isaretliyor - robot hedefe hic
# ulasmamis olsa bile.
#
# GOAL_TOLERANCE_RAD (~2.9 derece) movecobot'un SDK uzerinden yaptigi 2
# derecelik kontrolden KASITLI OLARAK biraz gevsek: gercek bir sapmada once
# bizim kendi kontrolumuz konussun (o daha aciklayici, ayrica robotun fault
# kodunu basiyor); bu tolerans onun kacirdigi durumlar icin ikinci ag.
GOAL_TOLERANCE_RAD = 0.05

# Yorunge bittikten sonra hedefe oturmak icin taninan sure. Fairino'da ServoJ
# komutu ile gercek eksen pozisyonu arasinda dogal bir gecikme var; yorunge
# biter bitmez kol hala yetisiyor olabiliyor. Varsayilan 0.0 ("aninda tut")
# burada surekli yanlis abort uretirdi.
GOAL_TIME_S = 2.0


def jtc_constraints_block(joints: list[str]) -> str:
    """ros2_controllers.yaml'a eklenecek constraints bolumu."""
    lines = [
        "",
        "    # -------------------------------------------------------------------",
        "    # HEDEF TOLERANSLARI  (tools/apply_ros2_patches.py tarafindan eklendi)",
        "    # -------------------------------------------------------------------",
        "    # Bu bolum tanimlanmayinca JointTrajectoryController hicbir tolerans",
        "    # kontrolu yapmaz: yorunge bitince goal'u KOSULSUZ SUCCEEDED",
        "    # isaretler - robot hedefe hic ulasmamis olsa bile.",
        "    #",
        "    # trajectory: 0.0 -> hareket SIRASINDAKI takip hatasi kontrolu KAPALI.",
        "    #   Acilmadi cunku ServoJ komutu ile gercek pozisyon arasindaki",
        "    #   gecikme normal calismada da var; buraya esik koymak saglikli",
        "    #   hareketleri ortasinda kesip abort ettirirdi.",
        "    #",
        "    # stopped_velocity_tolerance: 0.0 -> kapali. Bu kurulumda velocity",
        "    #   STATE arayuzu yok (state_interfaces sadece position); JTC hizi",
        "    #   pozisyon farkindan tahmin etmek zorunda kalir ve bu tahmin",
        "    #   guvenilir degil.",
        "    constraints:",
        "      stopped_velocity_tolerance: 0.0",
        f"      goal_time: {GOAL_TIME_S}",
    ]
    for j in joints:
        lines.append(f"      {j}: {{ trajectory: 0.0, goal: {GOAL_TOLERANCE_RAD} }}")
    return "\n".join(lines) + "\n"


def ompl_group_block(group_name: str) -> str:
    """ompl_planning.yaml sonuna eklenecek modele ozgu grup bolumu."""
    return f"""
# ---------------------------------------------------------------------------
# PLANLAMA-ZAMANI CARPISMA KONTROLU SIKLIGI
# (tools/apply_ros2_patches.py tarafindan eklendi - grup adi bu paketin
#  kendi SRDF'inden okundu)
# ---------------------------------------------------------------------------
# Bu dosyalarda daha once HIC grup bolumu yoktu. ompl_interface kaynagina gore
# (model_based_planning_context.cpp:278) config BOSSA useConfig() hemen donuyor
# ve OMPL'in kendi varsayilani gecerli oluyor: iki carpisma kontrol noktasi
# arasi mesafe = durum uzayinin azami genisliginin %1'i. 6 eksenli bir kolda bu
# COK kaba - aradan ince bir yakin-gecis "sizabiliyor". Plan SONRASI her
# waypoint'i tek tek olcen kendi kontrolumuzu yazma sebebimiz buydu.
#
# Kendi kontrolumuzun YERINE degil ONUNDE ikinci bir ag: %1 -> %0.5.
#
# NEDEN SADECE BU TEK SATIR: ompl_interface.cpp:199-215'e gore bir grup icin
# "planner_configs" VEYA "default_planner_config" verilmezse MoveIt varsayilan
# yapilandirmayi kendisi olusturup type degerini "geometric::RRTConnect" yapar -
# yani fiilen calisan planlayici bu. Buraya planner_configs listesi eklemek
# planlayici secimini degistirme riski tasirdi. Oysa
# longest_valid_segment_fraction KNOWN_GROUP_PARAMS icinde
# (ompl_interface.cpp:164) ayrica okunur ve varsayilan RRTConnect'e aynen
# uygulanir - secim degismez.
{group_name}:
  longest_valid_segment_fraction: {LONGEST_VALID_SEGMENT_FRACTION}
"""


# ---------------------------------------------------------------------------
# Yardimcilar
# ---------------------------------------------------------------------------

def srdf_group_name(pkg: Path) -> str | None:
    """Paketin SRDF'indeki ilk <group name="..."> degeri."""
    for srdf in sorted(pkg.glob("config/*.srdf")):
        m = re.search(r'<group\s+name="([^"]+)"', srdf.read_text(encoding="utf-8"))
        if m:
            return m.group(1)
    return None


def controller_block(text: str) -> tuple[str, list[str]] | None:
    """ros2_controllers.yaml icindeki JTC blogunun adi ve eklem listesi.

    controller_manager altindaki ayni isimli girdiyle karistirmamak icin
    SADECE girintisiz (top-level) anahtarlara bakiyoruz.
    """
    names = re.findall(r"^([A-Za-z0-9_]+_controller):\s*$", text, re.MULTILINE)
    if not names:
        return None
    name = names[-1]
    # Bu blogun icindeki joints listesi
    start = text.index(f"\n{name}:")
    body = text[start:]
    m = re.search(r"^\s+joints:\s*\n((?:\s+-\s+\S+\s*\n)+)", body, re.MULTILINE)
    if not m:
        return None
    joints = re.findall(r"-\s+(\S+)", m.group(1))
    return name, joints


def son_top_level_anahtar(text: str) -> str | None:
    keys = re.findall(r"^([A-Za-z0-9_]+):\s*$", text, re.MULTILINE)
    return keys[-1] if keys else None


# ---------------------------------------------------------------------------
# Uygulama adimlari
# ---------------------------------------------------------------------------

def patch_ompl(pkg: Path, dry: bool) -> str:
    """ompl_planning.yaml'i yazar - YOKSA OLUSTURUR.

    DIKKAT, BU DOSYA VENDOR'DA GENELDE YOK: frcobot_ros2'nin model
    paketleri kendi ompl_planning.yaml'lari olmadan geliyor ve MoveIt
    o durumda kendi varsayilanini kullaniyor - iceriginde SADECE
    AddTimeOptimalParameterization var, yani jerk (ani ivme degisimi)
    hic sinirlanmiyor. Canli testte kucuk eklem hareketlerinde bile
    robotun kendi carpisma algilamasi yanlis-pozitif "Shaft collision
    fault" veriyordu.

    Yani "dosya yoksa atla" demek, duzeltmenin en cok gerektigi durumda
    SESSIZCE uygulanmamasi olurdu. Once tam boyle bir hata vardi: temiz
    bir vendor klonunda (tag V3.0.0_RobotV3.9.3) dosya olmadigi icin yama
    atlaniyordu ve bunu ancak robot fault verince fark ederdik.
    """
    config_dir = pkg / "config"
    if not config_dir.is_dir():
        return "atlandi (config/ klasoru yok)"
    hedef = config_dir / "ompl_planning.yaml"
    grup = srdf_group_name(pkg)
    if not grup:
        return "ATLANDI (SRDF'te grup adi bulunamadi)"
    taban = (PATCH_DIR / "ompl_planning.yaml").read_text(encoding="utf-8")
    yeni = taban.rstrip("\n") + "\n" + ompl_group_block(grup)
    vardi = hedef.exists()
    if vardi and hedef.read_text(encoding="utf-8") == yeni:
        return f"zaten guncel ({grup})"
    if not dry:
        hedef.write_text(yeni, encoding="utf-8")
    fiil = "guncellendi" if vardi else "OLUSTURULDU (vendor'da yoktu)"
    return f"{fiil} ({grup}, longest_valid_segment_fraction={LONGEST_VALID_SEGMENT_FRACTION})"


def patch_controllers(pkg: Path, dry: bool) -> str:
    hedef = pkg / "config" / "ros2_controllers.yaml"
    if not hedef.exists():
        return "atlandi (ros2_controllers.yaml yok)"
    text = hedef.read_text(encoding="utf-8")
    if "constraints:" in text:
        return "zaten var (constraints)"
    blok = controller_block(text)
    if not blok:
        return "ATLANDI (controller blogu / joints listesi okunamadi)"
    name, joints = blok
    # Blok dosyanin sonuna ekleniyor; bu ancak controller blogu EN SON
    # top-level blok ise dogru olur. Degilse elle bakilmali - sessizce
    # yanlis yere yazmaktansa atlamak iyidir.
    if son_top_level_anahtar(text) != name:
        return f"ATLANDI ({name} son top-level blok degil - elle ekle)"
    if not dry:
        hedef.write_text(text.rstrip("\n") + "\n" + jtc_constraints_block(joints), encoding="utf-8")
    return f"yazildi ({name}, {len(joints)} eklem, goal={GOAL_TOLERANCE_RAD} rad)"


def patch_hardware(frcobot: Path, dry: bool) -> str:
    """fairino_hardware_interface.cpp yamasi (UDP servo kanali, olculen cmdT,
    hiz sinirlayici, bosta servo modundan cikis)."""
    yama = PATCH_DIR / "fairino_hardware_interface.patch"
    if not yama.exists():
        return "atlandi (yama dosyasi yok)"
    hedef = frcobot / "fairino_hardware" / "src" / "fairino_hardware_interface.cpp"
    if not hedef.exists():
        return "atlandi (fairino_hardware yok)"
    if "_servo_com_type" in hedef.read_text(encoding="utf-8"):
        return "zaten uygulanmis"
    cmd = ["patch", "-p1", "--forward", "--batch"]
    if dry:
        cmd.append("--dry-run")
    r = subprocess.run(cmd + ["-i", str(yama)], cwd=frcobot, capture_output=True, text=True)
    if r.returncode != 0:
        return f"BASARISIZ (rc={r.returncode}): {(r.stdout + r.stderr).strip()[:200]}"
    return "uygulandi" if not dry else "uygulanabilir (dry-run)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frcobot", type=Path, default=Path.home() / "ros2_ws" / "src" / "frcobot_ros2",
                    help="frcobot_ros2 kaynak klasoru")
    ap.add_argument("--dry-run", action="store_true", help="hicbir sey yazma, ne yapilacagini goster")
    args = ap.parse_args()

    frcobot: Path = args.frcobot.expanduser()
    if not frcobot.is_dir():
        print(f"HATA: {frcobot} bulunamadi.", file=sys.stderr)
        return 1
    if not shutil.which("patch"):
        print("HATA: 'patch' komutu yok (apt install patch).", file=sys.stderr)
        return 1

    print(f"frcobot_ros2: {frcobot}")
    if args.dry_run:
        print("(dry-run - hicbir dosya degistirilmiyor)")
    print()

    print(f"fairino_hardware_interface.cpp : {patch_hardware(frcobot, args.dry_run)}")
    print()

    pkgler = sorted(frcobot.glob("fairino*_v6_moveit2_config"))
    if not pkgler:
        print(f"UYARI: {frcobot} altinda hicbir fairino*_v6_moveit2_config bulunamadi.", file=sys.stderr)
        return 1

    for pkg in pkgler:
        print(f"{pkg.name}")
        print(f"  ompl_planning.yaml    : {patch_ompl(pkg, args.dry_run)}")
        print(f"  ros2_controllers.yaml : {patch_controllers(pkg, args.dry_run)}")

    print()
    print(f"{len(pkgler)} paket islendi. Simdi yeniden derle:")
    print(f"  cd {frcobot.parent.parent} && colcon build")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

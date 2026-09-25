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
import math
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


# ---------------------------------------------------------------------------
# EKLEM IVME LIMITI
# ---------------------------------------------------------------------------
# CANLI OLAY: arayuzden TCP'ye 10mm oteleme verildi; uretilen plan j1'i 32.9
# derece donduruyordu, kol savruldu, acil stopa basildi.
#
# SEBEP: FR3 (ve diger birkac model) config'inde ivme limiti KAPALI geliyor:
#   has_acceleration_limits: false
#   max_acceleration: 0
# Pilz PTP ve LIN ivme limiti olmadan HIC planlayamiyor:
#   "Exception caught: 'acceleration limit not set for group ...'"
# Sonucta her hareket sessizce OMPL/RRTConnect'e duesuyor. RRTConnect
# rastgele bir planlayici ve POZ hedefi verilince IK'nin BASKA BIR DALINI
# secebiliyor - TCP 10mm gitsin diye kol komple yer degistiriyor. Yani
# "once PTP dene, monoton gider; olmazsa OMPL'e dus" tasariminin PTP ayagi
# bu yiginda hic calismamis.
#
# DEGER NEREDEN: uydurma DEGIL - Fairino'nun KENDI FR5 config'inden
# (fairino5_v6_moveit2_config/config/joint_limits.yaml). O dosya FR3'unkiyle
# hiz limitleri dahil birebir ayni, tek farki ivmenin acik ve 0.7 olmasi.
# Yani ayni uretici, ayni hiz profili, kendi verdigi deger.
#
# ZATEN MUHAFAZAKAR: dosyadaki default_acceleration_scaling_factor 0.1, ustune
# movecobot da kendi accel_pct'ini uyguluyor. Limit koymak robotu
# HIZLANDIRMAZ - sadece planlayicinin uyacagi bir tavan tanimlar.
JOINT_MAX_ACCEL_RAD_S2 = 0.7


def patch_joint_limits(pkg: Path, dry: bool) -> str:
    hedef = pkg / "config" / "joint_limits.yaml"
    if not hedef.exists():
        return "atlandi (joint_limits.yaml yok)"
    text = hedef.read_text(encoding="utf-8")
    if "has_acceleration_limits: true" in text:
        return "zaten var (ivme limiti acik)"

    # Satir bazli degistirme - basit string replace "max_acceleration: 0.7"
    # ifadesini de vururdu ("max_acceleration: 0" onun on eki).
    yeni, sayac = [], 0
    for satir in text.splitlines():
        kirpik = satir.strip()
        girinti = satir[: len(satir) - len(satir.lstrip())]
        if kirpik == "has_acceleration_limits: false":
            yeni.append(f"{girinti}has_acceleration_limits: true")
            sayac += 1
        elif kirpik == "max_acceleration: 0":
            yeni.append(f"{girinti}max_acceleration: {JOINT_MAX_ACCEL_RAD_S2}")
        else:
            yeni.append(satir)

    if sayac == 0:
        return "ATLANDI (beklenen 'has_acceleration_limits: false' satiri yok - elle bak)"
    if not dry:
        hedef.write_text("\n".join(yeni) + "\n", encoding="utf-8")
    return f"yazildi ({sayac} eklem, max_acceleration={JOINT_MAX_ACCEL_RAD_S2} rad/s^2)"


# ---------------------------------------------------------------------------
# J3 POZISYON LIMITI (yalnizca FR3)
# ---------------------------------------------------------------------------
# CANLI OLAY (4 Eylul): robot "Shaft out of soft limit" hatasi verdi. FR3'un
# kontrolcusundeki J3 yazilim limiti +-150 derece; Fairino'nun URDF'i
# (fairino_description/urdf/fairino3_v6.urdf) ise +-2.8274 rad = +-162 derece
# diyor. Yani MoveIt 150-162 arasini gecerli sanip planlayabiliyor, robot o
# bolgeye girince fault verip duruyor.
#
# 4 Eylul'de URDF elle 150'ye cekilmisti ama KAYBOLDU: setup_target
# fairino_description'i kaynaktan yeniden kopyaliyor. 13 Eylul olcumu: native
# kaynak, ayna ve Docker imajindaki URDF'lerin hepsi yine +-162.
#
# NEDEN URDF DEGIL joint_limits.yaml: MoveIt robot modelini yuklerken
# robot_description_planning.joint_limits.<eklem>.max_position/min_position
# degerlerini okuyup URDF sinirinin yerine koyuyor (robot_model_loader.cpp
# :133-159 -> setVariableBounds). OMPL, IK ve Pilz ayni modeli kullaniyor.
# Config paketi hem bu aracla hem setup_target'ta zaten yamalaniyor, yeniden
# kopyalanirken sessizce kaybolmaz.
#
# DEGER NEREDEN: robotun kendi soft limiti (4 Eylul, robotun hata mesaji).
# Yalnizca FR3 icin olculdu; diger modellere UYGULANMIYOR.
J3_SOFT_LIMIT_DEG = 150.0
J3_LIMIT_PAKETLERI = ("fairino3_v6_moveit2_config",)


def patch_j3_position_limit(pkg: Path, dry: bool) -> str:
    if pkg.name not in J3_LIMIT_PAKETLERI:
        return "atlandi (bu model icin olculmus J3 limiti yok)"
    hedef = pkg / "config" / "joint_limits.yaml"
    if not hedef.exists():
        return "ATLANDI (joint_limits.yaml yok - J3 limiti UYGULANMADI)"
    satirlar = hedef.read_text(encoding="utf-8").splitlines()

    def girinti(s: str) -> int:
        return len(s) - len(s.lstrip())

    bas = next((i for i, s in enumerate(satirlar) if s.strip() == "j3:"), None)
    if bas is None:
        return "ATLANDI (j3 blogu bulunamadi - J3 limiti UYGULANMADI, elle bak)"
    son = bas + 1
    while son < len(satirlar) and satirlar[son].strip() and girinti(satirlar[son]) > girinti(satirlar[bas]):
        son += 1
    blok = satirlar[bas + 1:son]

    limit_rad = math.radians(J3_SOFT_LIMIT_DEG)
    mevcut = {}
    for s in blok:
        anahtar, _, deger = s.strip().partition(":")
        if anahtar in ("min_position", "max_position"):
            mevcut[anahtar] = float(deger)
    if mevcut:
        # Daha dar bir limit daha guvenlidir, dokunma. Daha genisi ise
        # sessizce birakilmamali.
        if (mevcut.get("max_position", math.inf) <= limit_rad
                and mevcut.get("min_position", -math.inf) >= -limit_rad):
            return f"zaten var (j3 +-{J3_SOFT_LIMIT_DEG:.0f} derece veya daha dar)"
        return "ATLANDI (j3 icin DAHA GENIS bir pozisyon limiti yazili - J3 limiti UYGULANMADI, elle bak)"

    cocuk = " " * (girinti(blok[0]) if blok else girinti(satirlar[bas]) + 2)
    # Tam float degeri yaziliyor: yuvarlamak limiti 150'nin bir tik USTUNE
    # tasiyabilirdi.
    eklenen = [
        f"{cocuk}# Robotun kendi J3 soft limiti +-{J3_SOFT_LIMIT_DEG:.0f} derece (tools/apply_ros2_patches.py)",
        f"{cocuk}has_position_limits: true",
        f"{cocuk}min_position: {-limit_rad!r}",
        f"{cocuk}max_position: {limit_rad!r}",
    ]
    if not dry:
        hedef.write_text("\n".join(satirlar[:son] + eklenen + satirlar[son:]) + "\n", encoding="utf-8")
    return f"yazildi (j3 +-{J3_SOFT_LIMIT_DEG:.0f} derece = +-{limit_rad:.4f} rad)"


# ---------------------------------------------------------------------------
# JODELL EPG40-050 GRIPPER MODELI (yalnizca FR3)
# ---------------------------------------------------------------------------
# NEDEN (24 Eylul, kullanici istegi): MTC'nin GeneratePlacePose gibi asamalari
# SRDF'te bir end_effector (gripper grubu) ister; gripper ROS'ta hic yoktu.
# Burada gripper SABIT linkler olarak eklenir (gripper yine SDK'dan surulur,
# eklem/controller yok) ve 3D gorunumde gercek sekliyle gorunur.
#
# CARPISMA GEOMETRISI EKLENMEZ, yalnizca <visual>: planlamadaki gripper
# guvenlik kutusu (movecobot.py, 95x60x130 mm, attached body) oldugu gibi
# kalir. CAD sekli kutudan ince; carpismaya koymak guvenlik payini azaltirdi.
#
# MESH KAYNAGI: Fairino destek sitesi EPG40-050.zip (fairino.support/download,
# 7 Temmuz 2025) icindeki STEP'ler, gmsh ile STL'ye cevrildi (mm).
# Govde STL cercevesi: montaj yuzu y=-72, ceneler y=+15 (tirnaklar 2 mm),
# kapanma ekseni x, x=0/z=0 eksenine simetrik. Parmak STL: boy y 0..35,
# genislik x -11..0, kalinlik z (uc 5 mm), taban blogu y 0..8.
#
# YERLESIM (olcumler ve kaynaklari):
#   - Flans wrist3_link'te z = 99,43 mm (acilista olculen flange_offset_m).
#   - Fairino TCP cercevesi = wrist3_link yonelimi, oteleme z = 199,4 mm (FK +
#     SDK TCP ile olculdu, 24 Eylul). jodell_tcp linki orada.
#   - Kapanma ekseni wrist3 X'inden -46,37 derece: rz = kapanma yonu - 46,37
#     (Zehra README YAW_CORRECTION_DEG, kullanici 21 Eylul gozle dogruladi) ve
#     TCP = wrist3 yonelimi oldugundan.
#   - Montaj adaptoru dosyada YOK; kalinligi 0 alindi. Parmaklar cene
#     tirnagina oturur (taban y_mesh=13): parmak ucu flanstan 120 mm, TCP'nin
#     ~20 mm altinda. Masadan olculen deger 19 mm (masa z=0 varsayimiyla) -
#     tutarli, dogrulanmadi.
#   - PARMAKLAR (kullanici karari 24 Eylul, standart paralel gripper modeli):
#     sag parmak prismatic eklem (jodell_finger_joint), sol parmak ona mimic.
#     Aralik parmak basina 0..20 mm = uretici stroku 40 mm / 2 (Jodell
#     EPG40-050 verisi). 0 = tam kapali: parmak ic yuzleri arasi 8 mm
#     (kullanici olcumu 16 Eylul). Deger gripper'in OKUDUGU konumdan gelir
#     (web 3D gorunumu: (100 - position) / 100 * 20 mm). Bu gripper'in ROS
#     surucusu yok, /joint_states'e yayin yapilmiyor -> SRDF'te passive_joint:
#     MoveIt bu eklem icin durum beklemez. Konum okumasi daha once guvenilmez
#     cikti (GUNCEL_DURUM 9-6); kullanici "simdilik boyle, sonra kontrol" dedi.
#     Surulmedigi icin <limit velocity="0">.
GRIPPER_PAKETLERI = ("fairino3_v6_moveit2_config",)
GRIPPER_MESH_DIR = "jodell_epg40"
GRIPPER_ISARET = "JODELL EPG40-050 v2 (tools/apply_ros2_patches.py)"
PARMAK_STROK_M = 0.020      # parmak basina (40 mm / 2)
PARMAK_KAPALI_YARIM_M = 0.004   # kapaliyken ic yuz eksenden 4 mm (8 mm bosluk)
PARMAK_KUVVET_N = 50.0      # uretici: parmak basina en cok 50 N
FLANS_Z_M = 0.0994
TCP_Z_M = 0.1994


def gripper_urdf_blogu(paket: str) -> str:
    kok = f"package://{paket}/config/{GRIPPER_MESH_DIR}"
    return f"""
    <!-- {GRIPPER_ISARET}: yalnizca gorsel, carpisma yok (gerekce yama aracinda) -->
    <link name="jodell_epg40_base">
      <visual>
        <origin xyz="0 0.072 0" rpy="0 0 0"/>
        <geometry><mesh filename="{kok}/epg40_050_govde.stl" scale="0.001 0.001 0.001"/></geometry>
        <material name="jodell_gri"><color rgba="0.62 0.66 0.72 1"/></material>
      </visual>
    </link>
    <joint name="jodell_epg40_mount" type="fixed">
      <parent link="wrist3_link"/>
      <child link="jodell_epg40_base"/>
      <origin xyz="0 0 {FLANS_Z_M}" rpy="1.570796 0 -0.809309"/>
    </joint>
    <link name="jodell_finger_right">
      <visual>
        <geometry><mesh filename="{kok}/epg40_parmak.stl" scale="0.001 0.001 0.001"/></geometry>
        <material name="jodell_koyu"><color rgba="0.25 0.27 0.30 1"/></material>
      </visual>
    </link>
    <!-- Parmak STL'inde ic yuz z=4 mm; eklem 0'da ic yuz eksenden PARMAK_KAPALI_YARIM_M -->
    <joint name="jodell_finger_joint" type="prismatic">
      <parent link="jodell_epg40_base"/>
      <child link="jodell_finger_right"/>
      <origin xyz="{PARMAK_KAPALI_YARIM_M - 0.004:.4f} 0.085 -0.0055" rpy="0 1.570796 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="0" upper="{PARMAK_STROK_M}" effort="{PARMAK_KUVVET_N}" velocity="0"/>
    </joint>
    <link name="jodell_finger_left">
      <visual>
        <geometry><mesh filename="{kok}/epg40_parmak.stl" scale="0.001 0.001 0.001"/></geometry>
        <material name="jodell_koyu"><color rgba="0.25 0.27 0.30 1"/></material>
      </visual>
    </link>
    <joint name="jodell_finger_left_joint" type="prismatic">
      <parent link="jodell_epg40_base"/>
      <child link="jodell_finger_left"/>
      <origin xyz="{-(PARMAK_KAPALI_YARIM_M - 0.004):.4f} 0.085 0.0055" rpy="0 -1.570796 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="0" upper="{PARMAK_STROK_M}" effort="{PARMAK_KUVVET_N}" velocity="0"/>
      <mimic joint="jodell_finger_joint" multiplier="1" offset="0"/>
    </joint>
    <link name="jodell_tcp"/>
    <joint name="jodell_tcp_joint" type="fixed">
      <parent link="wrist3_link"/>
      <child link="jodell_tcp"/>
      <origin xyz="0 0 {TCP_Z_M}" rpy="0 0 0"/>
    </joint>
"""


def gripper_srdf_blogu(kol_grubu: str) -> str:
    return f"""    <!-- {GRIPPER_ISARET}: MTC end-effector tanimi -->
    <group name="jodell_hand">
        <link name="jodell_epg40_base"/>
        <link name="jodell_finger_left"/>
        <link name="jodell_finger_right"/>
        <link name="jodell_tcp"/>
    </group>
    <end_effector name="jodell" parent_link="wrist3_link" group="jodell_hand" parent_group="{kol_grubu}"/>
    <passive_joint name="jodell_finger_joint"/>
"""


def patch_gripper_model(pkg: Path, dry: bool) -> str:
    if pkg.name not in GRIPPER_PAKETLERI:
        return "atlandi (bu modelde Jodell EPG40 takili degil)"
    xacrolar = sorted(pkg.glob("config/*_robot.urdf.xacro"))
    srdfler = sorted(pkg.glob("config/*.srdf"))
    kaynak = PATCH_DIR / GRIPPER_MESH_DIR
    if len(xacrolar) != 1 or len(srdfler) != 1:
        return "ATLANDI (urdf.xacro / srdf tek degil - elle bak)"
    if not kaynak.is_dir():
        return f"ATLANDI ({kaynak} yok)"
    grup = srdf_group_name(pkg)
    if not grup:
        return "ATLANDI (SRDF'te grup adi yok)"

    xacro, srdf = xacrolar[0], srdfler[0]
    xt, st = xacro.read_text(encoding="utf-8"), srdf.read_text(encoding="utf-8")
    if GRIPPER_ISARET in xt and GRIPPER_ISARET in st:
        return "zaten var"
    if "JODELL EPG40-050" in xt or "JODELL EPG40-050" in st:
        return "ATLANDI (eski surum gripper blogu var - paketi aynadan yeniden kopyala)"
    if xt.count("</robot>") != 1 or st.count("</robot>") != 1:
        return "ATLANDI (</robot> etiketi tek degil - elle bak)"
    if not dry:
        hedef = pkg / "config" / GRIPPER_MESH_DIR
        hedef.mkdir(exist_ok=True)
        for stl in sorted(kaynak.glob("*.stl")):
            shutil.copy2(stl, hedef / stl.name)
        if GRIPPER_ISARET not in xt:
            xacro.write_text(xt.replace("</robot>", gripper_urdf_blogu(pkg.name) + "</robot>"),
                             encoding="utf-8")
        if GRIPPER_ISARET not in st:
            srdf.write_text(st.replace("</robot>", gripper_srdf_blogu(grup) + "</robot>"),
                            encoding="utf-8")
    return f"yazildi (gorsel linkler + jodell_hand/end_effector, parent_group={grup})"


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
        print(f"  joint_limits.yaml     : {patch_joint_limits(pkg, args.dry_run)}")
        print(f"  j3 pozisyon limiti    : {patch_j3_position_limit(pkg, args.dry_run)}")
        print(f"  jodell gripper modeli : {patch_gripper_model(pkg, args.dry_run)}")

    print()
    print(f"{len(pkgler)} paket islendi. Simdi yeniden derle:")
    print(f"  cd {frcobot.parent.parent} && colcon build")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

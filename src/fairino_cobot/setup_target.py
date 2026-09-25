#!/usr/bin/env python3
"""
detect_robot.py'nin bulduğu HERHANGİ bir model (FR3, FR5, FR10, FR16, FR20,
FR30...) ve HERHANGİ bir sürüm (3.8.x - 3.9.9 arası, resmi mirror'da neler
varsa) için doğru kaynağı otomatik bulur, robot_ip parametresini yamalar,
workspace'e kurar ve derler. Elle "fr3"/"fr5" ya da "v393"/"v398" gibi
sabit isimler YOK — her şey argüman olarak gelen model+sürüme göre.

Kaynak öncelik sırası:
  1) TAM TAG KLONU (fairino_msgs yanında, aynı sürümden):
     FAIRINO_SOURCES_DIR/frcobot_ros2_v{MAJ}_{MIN}_{MIN2}  (fairino-fetch-source indirir)
     ~/frcobot_ros2_v{MAJ}{MIN}{MIN2}                       (eski ad; imajdaki 3.9.3 klonu)
  2) ~/fairino_official_mirror/fairino_hardware_v{MAJ}_{MIN}_{MIN2}
  3) ~/fairino_hardware_sources/v{MAJ}{MIN}{MIN2} (izole/kardeşsiz kopyalar)

Aynada olmayan bir sürüm için: fairino-fetch-source <sürüm>

Kullanım:
  python3 setup_fairino_target.py <model_num> <maj> <min> <min2> <robot_ip>

Bu script auto_fairino_launch.sh tarafından otomatik çağrılır.
"""
import hashlib
import re
import shutil
import subprocess
import sys
from pathlib import Path

from fairino_cobot import paths

HOME = Path.home()
# Bu agaclarin yeri .env ile degistirilebilir (FAIRINO_ROS2_WS,
# FAIRINO_MIRROR_DIR) - eskiden ev dizinine sabitlenmisti.
ROS2_WS = paths.ros2_ws()
MIRROR = paths.mirror_dir()
LEGACY_SOURCES = HOME / "fairino_hardware_sources"
STATE_FILE = paths.last_target_file()
HASH_DIR = HOME / ".fairino_build_hashes"


def source_hash(path: Path) -> str:
    h = hashlib.sha256()
    for f in sorted(path.rglob("*")):
        if f.is_file():
            h.update(str(f.relative_to(path)).encode())
            h.update(f.read_bytes())
    return h.hexdigest()


def needs_rebuild(pkg_name: str, src: Path, extra: str = "") -> bool:
    """src'nin icerigi (ve verilirse extra - orn. robot_ip) son basarili
    build'den beri degismemisse (ve install/ hala mevcutsa) True donmez -
    boylece degismeyen bir paket her fairino_launch'ta gereksiz yere silinip
    yeniden derlenmez."""
    install_dir = ROS2_WS / "install" / pkg_name
    hash_file = HASH_DIR / f"{pkg_name}.sha256"
    if not install_dir.exists() or not hash_file.exists():
        return True
    current = source_hash(src)
    if extra:
        current = hashlib.sha256((current + extra).encode()).hexdigest()
    return hash_file.read_text().strip() != current


def mark_built(pkg_name: str, src: Path, extra: str = ""):
    HASH_DIR.mkdir(exist_ok=True)
    digest = source_hash(src)
    if extra:
        digest = hashlib.sha256((digest + extra).encode()).hexdigest()
    (HASH_DIR / f"{pkg_name}.sha256").write_text(digest)


def die(msg):
    print(f"HATA: {msg}", file=sys.stderr)
    sys.exit(1)


REQUIRED_HW_FILES = [
    "src/fairino_hardware_interface.cpp",
]


def source_is_complete(candidate: Path) -> bool:
    for rel in REQUIRED_HW_FILES:
        if not (candidate / rel).exists():
            return False
    lib_dir = candidate / "libfairino" / "lib"
    if not lib_dir.exists() or not any(lib_dir.glob("libfairino.so*")):
        return False
    return True


def tam_klon_yerleri(maj, minr, min2):
    """Bu surumun TAM frcobot_ros2 tag klonunun olabilecegi klasorler.

    Tam klonda fairino_hardware, fairino_msgs ve fairino_description AYNI
    tag'den yan yana durur. Aynaya ya da izole bir klasore konan surucunun
    yaninda ise dogru fairino_msgs yok; find_msgs_source aynanin kokundekine
    (3.9.9) duser ve derleme kirilabilir. 3.9.3'te olan buydu:
      'fairino_msgs::msg::RobotNonrtState_' has no member named 'servoj_cmd_num'
    Eskiden bu yol YALNIZCA 3.9.3 icin koda yaziliydi; artik her surum icin.
    """
    return [
        # fairino-fetch-source buraya indirir.
        paths.sources_dir() / f"frcobot_ros2_v{maj}_{minr}_{min2}",
        # Eski adlandirma - imajdaki 3.9.3 klonu bu adla duruyor.
        HOME / f"frcobot_ros2_v{maj}{minr}{min2}",
    ]


def tam_klon_surucusu(klon: Path, maj, minr, min2):
    """Tam tag klonunun icinde bu surume ait fairino_hardware klasoru.

    frcobot_ros2 tag duzenleri (GitHub'da olculdu, 14 Eylul 2026):
      - 3.9.4 ve oncesi: tek 'fairino_hardware' (o tag'in kendi surumu)
      - 3.9.6 ve sonrasi: 'fairino_hardware' + 'fairino_hardware_v3_9_x'
        surum klasorleri + TEK fairino_msgs
    Surum klasoru varsa o secilir: aynada da ayni kalip kullaniliyor ve canli
    robotta calisti (fairino_hardware_v3_9_8 + ayni tag'in fairino_msgs'i).
    """
    for ad in (f"fairino_hardware_v{maj}_{minr}_{min2}", "fairino_hardware"):
        if (klon / ad).is_dir():
            return klon / ad
    return None


def find_hardware_source(maj, minr, min2):
    candidates = []
    # 1) Tam tag klonlari ONCE: yanlarindaki fairino_msgs kesin ayni surumden.
    for klon in tam_klon_yerleri(maj, minr, min2):
        surucu = tam_klon_surucusu(klon, maj, minr, min2)
        if surucu is not None:
            candidates.append(surucu)
    # 2) Ayna. 3) LEGACY_SOURCES: sadece fairino_hardware'in izole kopyasi,
    # kardessiz - secilirse fairino_msgs aynaya duser (main() uyarir).
    candidates.append(MIRROR / f"fairino_hardware_v{maj}_{minr}_{min2}")
    candidates.append(LEGACY_SOURCES / f"v{maj}{minr}{min2}")

    for c in candidates:
        if not c.exists():
            continue
        if source_is_complete(c):
            return c
        print(f"  UYARI: {c} eksik/bozuk görünüyor (kritik dosyalar yok), atlanıyor.")
    return None


def list_available_versions():
    versions = []
    kaynaklar = paths.sources_dir()
    if kaynaklar.exists():
        for d in kaynaklar.glob("frcobot_ros2_v*"):
            m = re.fullmatch(r"frcobot_ros2_v(\d+)_(\d+)_(\d+)", d.name)
            if not m:
                continue
            surucu = tam_klon_surucusu(d, *m.groups())
            if surucu is not None and source_is_complete(surucu):
                versions.append(".".join(m.groups()))
    if MIRROR.exists():
        for d in MIRROR.glob("fairino_hardware_v*"):
            if source_is_complete(d):
                versions.append(d.name.replace("fairino_hardware_v", "").replace("_", "."))
    if LEGACY_SOURCES.exists():
        for d in LEGACY_SOURCES.iterdir():
            if d.is_dir() and source_is_complete(d):
                versions.append(d.name)
    return sorted(set(versions))


def find_config_source(model_num):
    return MIRROR / f"fairino{model_num}_v6_moveit2_config"


def apply_hardware_patch(frcobot_dir: Path, cpp_path: Path):
    """Surucunun TAM yamasini uygular (ros2/patches/*.patch).

    NEDEN BOYLE: burada eskiden patch_on_init() vardi ve SADECE robot_ip
    blogunu ekliyordu. Temiz bir makinede sira "fairino_launch ->
    setup_target -> derle" oldugu icin sonuc su oluyordu: UDP servo kanali,
    olculen cmdT, hiz sinirlayici ve bosta servo modundan cikis HIC
    uygulanmamis bir surucu. Robot sicramali calisir, "axis N command speed
    exceeds the limit" faultu verebilirdi. Ustelik sonradan tam yamayi
    uygulamak da MUMKUN OLMUYORDU: robot_ip blogu eklendigi icin patch'in
    baglami kayiyor ve yama tutmuyordu.

    Artik tek kaynak var. Yama dosyasi bulunamazsa (kurulum eksik) eski
    davranisa (yalniz robot_ip) duesuluyor ve yuksek sesle uyariliyor. Yama
    dosyasi VAR ama kaynaga TUTMUYORSA durulur - bkz. fonksiyonun sonu.
    """
    try:
        yama = paths.patches_dir() / "fairino_hardware_interface.patch"
    except paths.PathError as e:
        yama = None
        print(f"  UYARI: yama klasoru bulunamadi ({e.args[0].splitlines()[0]})")

    if yama is None or not yama.exists():
        print("  UYARI: TAM surucu yamasi uygulanamiyor - sadece robot_ip blogu")
        print("         eklenecek. UDP servo kanali, olculen cmdT, hiz sinirlayici")
        print("         ve bosta servo cikisi DEVREDE OLMAYACAK; robot sicramali")
        print("         calisabilir. Bkz. ros2/README.md")
        patch_on_init(cpp_path)
        return

    if "_servo_com_type" in cpp_path.read_text():
        print("  Surucu yamasi zaten uygulanmis.")
        return

    r = subprocess.run(
        ["patch", "-p1", "--forward", "--batch", "-i", str(yama)],
        cwd=str(frcobot_dir), capture_output=True, text=True,
    )
    if r.returncode == 0:
        print("  Surucu yamasi uygulandi (UDP servo kanali, olculen cmdT,")
        print("  hiz sinirlayici, bosta servo cikisi, robot_ip parametresi).")
        return

    # YAMA DOSYASI VAR AMA BU KAYNAGA TUTMUYOR: DUR.
    # Eskiden burada sadece robot_ip blogu eklenip devam ediliyordu. Sonuc,
    # UDP servo kanali, olculen cmdT, hiz sinirlayici ve bosta servo cikisi
    # OLMAYAN bir surucuydu. Aynada olmayan surumler artik fairino-fetch-source
    # ile indirilebildigi icin yamanin tutmadigi bir kaynakla karsilasmak
    # olagan hale geldi - yamasiz surucuyle robot surulmez.
    die(
        f"surucu yamasi bu kaynaga uygulanamadi (rc={r.returncode}):\n"
        f"  {(r.stdout + r.stderr).strip()[:300]}\n"
        "Yamasiz surucuyle DEVAM EDILMIYOR: UDP servo kanali, olculen cmdT, hiz "
        "sinirlayici ve bosta servo cikisi olmadan robot sicramali calisir, "
        "'axis N command speed exceeds the limit' faultu verebilir.\n"
        "Bu surum icin ros2/patches/fairino_hardware_interface.patch uyarlanmali "
        "(gerekceler: ros2/README.md)."
    )


def patch_on_init(cpp_path: Path):
    text = cpp_path.read_text()
    if 'hardware_parameters.count("robot_ip")' in text:
        return

    m = re.search(r"FairinoHardwareInterface::on_init\s*\([^)]*\)\s*\{", text)
    if not m:
        m = re.search(r"\bon_init\s*\([^)]*\)\s*\{", text)
    if not m:
        print(f"  UYARI: on_init() bulunamadı, yama atlanıyor: {cpp_path}")
        return

    start = m.end() - 1
    depth = 0
    end = None
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                end = i
                break
    if end is None:
        print(f"  UYARI: on_init() gövde sonu bulunamadı: {cpp_path}")
        return

    body = text[start:end]
    success_matches = list(re.finditer(
        r"[ \t]*return\s+hardware_interface::CallbackReturn::SUCCESS\s*;", body))
    if not success_matches:
        print(f"  UYARI: on_init() içinde SUCCESS return bulunamadı: {cpp_path}")
        return

    last = success_matches[-1]
    line_start = body.rfind("\n", 0, last.start()) + 1
    indent_match = re.match(r"[ \t]*", body[line_start:])
    indent = indent_match.group(0) if indent_match else "    "

    patch_code = (
        f'{indent}if (info_.hardware_parameters.count("robot_ip")) {{\n'
        f'{indent}    _controller_ip = info_.hardware_parameters.at("robot_ip");\n'
        f'{indent}    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),\n'
        f'{indent}                "robot_ip parametresi kullanılıyor: %s", _controller_ip.c_str());\n'
        f'{indent}}}\n'
    )

    insert_pos = start + line_start
    new_text = text[:insert_pos] + patch_code + text[insert_pos:]
    cpp_path.write_text(new_text)
    print(f"  on_init() yamalandı: {cpp_path}")


def patch_xacro(xacro_path: Path, robot_ip: str):
    text = xacro_path.read_text()

    new_text, n = re.subn(
        r"<plugin>[^<]*</plugin>",
        "<plugin>fairino_hardware/FairinoHardwareInterface</plugin>",
        text, count=1,
    )
    if n:
        text = new_text

    if 'name="robot_ip"' in text:
        text = re.sub(
            r'<param name="robot_ip">[^<]*</param>',
            f'<param name="robot_ip">{robot_ip}</param>',
            text,
        )
    else:
        def add_param(match):
            return match.group(0) + f'\n                <param name="robot_ip">{robot_ip}</param>'
        new_text, n = re.subn(
            r"<plugin>fairino_hardware/FairinoHardwareInterface</plugin>",
            add_param, text, count=1,
        )
        if n:
            text = new_text
        else:
            new_text, n = re.subn(r"<hardware>",
                                   f'<hardware>\n                <param name="robot_ip">{robot_ip}</param>',
                                   text, count=1)
            if n:
                text = new_text

    xacro_path.write_text(text)
    print(f"  xacro güncellendi (robot_ip={robot_ip}): {xacro_path}")


def run(cmd, cwd):
    print(f"  $ {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        die(f"komut başarısız oldu: {' '.join(cmd)}")


def rename_package_to(pkg_dir: Path, new_name: str):
    pkg_xml = pkg_dir / "package.xml"
    if pkg_xml.exists():
        text = pkg_xml.read_text()
        old_text = text
        text = re.sub(r"<name>[^<]*</name>", f"<name>{new_name}</name>", text, count=1)
        if text != old_text:
            pkg_xml.write_text(text)
            print(f"  package.xml adı '{new_name}' yapıldı: {pkg_xml}")
        else:
            print(f"  UYARI: package.xml'de <name> etiketi bulunamadı/değiştirilemedi: {pkg_xml}")
    else:
        print(f"  UYARI: {pkg_xml} yok, isim değiştirilemedi.")

    cmake_file = pkg_dir / "CMakeLists.txt"
    if cmake_file.exists():
        text = cmake_file.read_text()
        old_text = text
        text = re.sub(r"^project\(\s*\S+\s*\)", f"project({new_name})", text, count=1, flags=re.MULTILINE)
        if text != old_text:
            cmake_file.write_text(text)
            print(f"  CMakeLists.txt project() adı '{new_name}' yapıldı: {cmake_file}")
        else:
            print(f"  UYARI: CMakeLists.txt'de project() bulunamadı/değiştirilemedi: {cmake_file}")


def clear_build_cache(pkg_name: str):
    for sub in ("build", "install", "log"):
        p = ROS2_WS / sub / pkg_name
        if p.exists():
            shutil.rmtree(p)
            print(f"  Eski önbellek temizlendi: {p}")


def safe_copytree(src: Path, dst: Path):
    tmp_dst = dst.with_name(dst.name + ".tmp_copy")
    if tmp_dst.exists():
        shutil.rmtree(tmp_dst)
    shutil.copytree(src, tmp_dst)
    if dst.exists():
        shutil.rmtree(dst)
    tmp_dst.rename(dst)


def apply_config_patches(config_target: Path):
    """Aynadan YENI kopyalanan moveit config paketine yamalarimizi uygular.

    NEDEN VAR: bu adim eskiden HIC yapilmiyordu. Imajda FR3 paketi hazir
    geldigi ve zaten yamali oldugu icin fark edilmiyordu - ama BASKA BIR
    MODEL (FR5, FR10...) takan biri paketi aynadan kopyaliyor ve YAMASIZ
    kullaniyordu:
      - ompl_planning.yaml yok        -> jerk sinirlanmiyor, robot yanlis
                                         pozitif "Shaft collision fault"
                                         verebiliyor
      - JTC constraints yok           -> hedefe varilmasa da hareket
                                         BASARILI sayiliyor
      - eklem ivme limiti yok         -> Pilz PTP/LIN HIC planlayamiyor,
                                         her hareket OMPL/RRTConnect'e
                                         duesuyor
      - FR3'te J3 limiti +-162 derece -> MoveIt robotun +-150 derecelik soft
                                         limitinin disini planlayabiliyor
    Ivme maddesi canli bir olaya yol acti: 10mm'lik bir TCP otelemesi icin
    planlayici IK'nin baska bir dalina atladi, kol 32.9 derece savruldu ve
    acil stopa basildi.

    Yama araci tools/ altinda ve pakete dahil degil; dosyadan yukleniyor.
    Sadece CONFIG yamalari cagriliyor - donanim yamasi zaten
    apply_hardware_patch ile ayrica uygulaniyor ve 'patch' komutunu
    gerektiriyor, buradaki yamalar ise saf Python.
    """
    import importlib.util

    betik = paths.patcher_script()
    spec = importlib.util.spec_from_file_location("apply_ros2_patches", betik)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    # patch_ompl sablonu PATCH_DIR'den okuyor - tek kaynak paths.patches_dir().
    mod.PATCH_DIR = paths.patches_dir()

    print(f"  ompl_planning.yaml    : {mod.patch_ompl(config_target, False)}")
    print(f"  ros2_controllers.yaml : {mod.patch_controllers(config_target, False)}")
    print(f"  joint_limits.yaml     : {mod.patch_joint_limits(config_target, False)}")
    print(f"  j3 pozisyon limiti    : {mod.patch_j3_position_limit(config_target, False)}")
    print(f"  jodell gripper modeli : {mod.patch_gripper_model(config_target, False)}")


def sync_shared_package(pkg_name: str, src: Path):
    if not src.exists():
        print(f"  UYARI: {src} yok, {pkg_name} senkronize edilemedi.")
        return False
    if not needs_rebuild(pkg_name, src):
        print(f"  {pkg_name} degismemis, senkron/build atlaniyor.")
        return False
    target = ROS2_WS / "src" / "frcobot_ros2" / pkg_name
    clear_build_cache(pkg_name)
    safe_copytree(src, target)
    print(f"  {pkg_name} senkronize edildi: {src} -> {target}")
    return True


def find_msgs_source(hw_src: Path, pkg_name: str):
    sibling = hw_src.parent / pkg_name
    if sibling.exists():
        return sibling
    return MIRROR / pkg_name


def main():
    if len(sys.argv) != 6:
        die("Kullanım: setup_fairino_target.py <model_num> <maj> <min> <min2> <robot_ip>")

    model_num, maj, minr, min2, robot_ip = sys.argv[1:6]

    if not MIRROR.exists():
        die(f"{MIRROR} bulunamadı.\n"
            f"Bu klasör frcobot_ros2'nin V3.0.0_RobotV3.9.9 tag'idir — içinde\n"
            f"fairino_hardware'in v3.8.2.11'den v3.9.9'a kadar tüm sürümleri var.\n"
            f"İndirmek için:\n"
            f"  git clone --depth 1 --branch V3.0.0_RobotV3.9.9 \\\n"
            f"      https://github.com/FAIR-INNOVATION/frcobot_ros2.git {MIRROR}\n"
            f"Başka bir yerdeyse .env içinde FAIRINO_MIRROR_DIR ile bildir.")

    print(f"=== FR{model_num}, sürüm {maj}.{minr}.{min2} için kaynak aranıyor ===")
    hw_src = find_hardware_source(maj, minr, min2)
    if not hw_src:
        avail = list_available_versions()
        die(
            f"{maj}.{minr}.{min2} için fairino_hardware kaynağı bulunamadı.\n"
            f"Mevcut sürümler: {avail if avail else '(hiçbiri)'}\n"
            f"Çözüm: bu sürümün kaynağını KENDİ tag'iyle indir (robota bağlanmaz):\n"
            f"  fairino-fetch-source {maj}.{minr}.{min2}\n"
            f"  (Docker: docker compose exec cobot fairino-fetch-source {maj}.{minr}.{min2})\n"
            f"İndirileceği yer: {tam_klon_yerleri(maj, minr, min2)[0]}\n"
            f"Sonra fairino_launch'i yeniden başlat (Docker: docker compose restart cobot)."
        )
    print(f"Kaynak bulundu: {hw_src}")

    print("")
    print("=== fairino_msgs ve fairino_description senkronize ediliyor (hardware ile aynı kaynaktan) ===")
    msgs_src = find_msgs_source(hw_src, "fairino_msgs")
    desc_src = find_msgs_source(hw_src, "fairino_description")
    print(f"  fairino_msgs kaynağı: {msgs_src}")
    print(f"  fairino_description kaynağı: {desc_src}")
    if msgs_src.parent != hw_src.parent:
        print("  UYARI: fairino_msgs sürücüyle AYNI tag'den gelmiyor (sürücünün yanında yok).")
        print("         Derleme kırılabilir; doğrusu bu sürümü tam tag olarak indirmek:")
        print(f"         fairino-fetch-source {maj}.{minr}.{min2}")
    synced_msgs = sync_shared_package("fairino_msgs", msgs_src)
    synced_desc = sync_shared_package("fairino_description", desc_src)

    if synced_msgs or synced_desc:
        print("")
        print("=== colcon build --packages-select fairino_msgs fairino_description ===")
        pkgs = []
        if synced_msgs:
            pkgs.append("fairino_msgs")
        if synced_desc:
            pkgs.append("fairino_description")
        run(["colcon", "build", "--packages-select"] + pkgs, cwd=str(ROS2_WS))
        if synced_msgs:
            mark_built("fairino_msgs", msgs_src)
        if synced_desc:
            mark_built("fairino_description", desc_src)

    if needs_rebuild("fairino_hardware", hw_src):
        print(f"=== fairino_hardware kuruluyor: {hw_src} ===")
        hw_target = ROS2_WS / "src" / "frcobot_ros2" / "fairino_hardware"
        clear_build_cache("fairino_hardware")
        safe_copytree(hw_src, hw_target)
        rename_package_to(hw_target, "fairino_hardware")

        cpp_file = hw_target / "src" / "fairino_hardware_interface.cpp"
        if cpp_file.exists():
            apply_hardware_patch(hw_target.parent, cpp_file)
        else:
            print(f"  UYARI: {cpp_file} yok, yama atlanıyor (belki eski kaynak zaten yamalı).")

        print("")
        print("=== colcon build --packages-select fairino_hardware ===")
        run(["colcon", "build", "--packages-select", "fairino_hardware"], cwd=str(ROS2_WS))
        mark_built("fairino_hardware", hw_src)
    else:
        print(f"=== fairino_hardware degismemis ({hw_src}), rebuild atlaniyor ===")

    config_pkg_name = f"fairino{model_num}_v6_moveit2_config"
    config_target = ROS2_WS / "src" / "frcobot_ros2" / config_pkg_name
    config_src = find_config_source(model_num)

    if not config_target.exists():
        print("")
        print(f"=== {config_pkg_name} workspace'te yok, mirror'dan kuruluyor ===")
        if not config_src.exists():
            die(f"{config_pkg_name} mirror'da da yok: {config_src}\n"
                f"FR{model_num} için resmi config paketi bende mevcut değil, bana söyle.")
        safe_copytree(config_src, config_target)

        # AYNADAN GELEN PAKET YAMASIZDIR - hemen yamalanmali.
        # Bu atlanirsa robot yamasiz bir config ile surulur; ayrintili
        # gerekce apply_config_patches docstring'inde. Yamalar
        # uygulanamiyorsa DEVAM ETME: sessizce yamasiz surmek, bu projede
        # bir kez acil stopa basilmasina yol acti.
        print("")
        print(f"=== {config_pkg_name} yamalari uygulaniyor (aynadan yeni geldi) ===")
        try:
            apply_config_patches(config_target)
        except Exception as e:
            die(f"{config_pkg_name} yamalanamadi: {type(e).__name__}: {e}\n"
                "Yamasiz config ile devam edilmiyor - eklem ivme limiti olmadan "
                "Pilz PTP/LIN hic planlayamiyor ve her hareket rastgele planlayiciya "
                "duesuyor (kol savrulabilir).")

    if config_src.exists() and not needs_rebuild(config_pkg_name, config_src, extra=robot_ip):
        print(f"=== {config_pkg_name} degismemis (kaynak+robot_ip ayni), rebuild atlaniyor ===")
    else:
        xacro_candidates = list(config_target.glob("config/*ros2_control.xacro"))
        if xacro_candidates:
            patch_xacro(xacro_candidates[0], robot_ip)
        else:
            print(f"  UYARI: {config_target} içinde ros2_control.xacro bulunamadı.")

        print("")
        print(f"=== colcon build --packages-select {config_pkg_name} ===")
        run(["colcon", "build", "--packages-select", config_pkg_name], cwd=str(ROS2_WS))
        if config_src.exists():
            mark_built(config_pkg_name, config_src, extra=robot_ip)

    STATE_FILE.write_text(f"MOVEIT_PACKAGE={config_pkg_name}\nROBOT_IP={robot_ip}\n")

    print("")
    print(f"Bitti. Paket: {config_pkg_name}")


if __name__ == "__main__":
    main()

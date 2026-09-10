"""Butun yol cozumlemesi tek yerde.

NEDEN BU DOSYA VAR: modullerin her biri kendi basina
`os.path.dirname(__file__) + "fairino-python-sdk/linux"` ve
`os.path.expanduser("~/detect_robot.py")` gibi varsayimlar yapiyordu. Bu
varsayimlar dosyalarin nerede durduguna bagliydi - dizin duzeni degisince
hepsi sessizce kirildi. Ayrica repoyu klonlayan baska birinde SDK baska
yerde oldugu icin hicbiri calismiyordu ("bende calisiyor ama onda
calismiyor").

Sira her zaman ayni: once ortam degiskeni (.env ile verilebilir), sonra
bilinen konumlar, sonra son care olarak arama. Bulunamazsa NE ARANDIGINI
soyleyen bir hata - sessizce yanlis yola dusmek yok.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

HOME = Path.home()

# Paketin kendi koku: .../src/fairino_cobot
PACKAGE_DIR = Path(__file__).resolve().parent
# Repo koku: .../src/fairino_cobot -> .../src -> repo. Pip ile kurulunca
# site-packages altinda olabilir; o durumda bu yol repo olmaz, sadece
# ".env burada mi" diye bakildigi icin zararsiz.
REPO_ROOT = PACKAGE_DIR.parent.parent

# Kurulum oncesi (paketlenmeden once) dosyalarin durdugu eski yer. Calisan
# sistemi bir anda kirmamak icin her aramada son basamak olarak duruyor.
LEGACY_CLIENT_DIR = HOME / "fairino" / "client"


class PathError(RuntimeError):
    """Bir yol cozulemedi. Mesaji nereye bakildigini tek tek yazar."""


# ---------------------------------------------------------------------------
# .env
# ---------------------------------------------------------------------------

def load_dotenv(path: Path | None = None) -> None:
    """Repo kokundeki .env dosyasini os.environ'a yukler.

    Bagimlilik eklememek icin elle: "KEY=value" satirlari, # ile yorum,
    istege bagli tirnak. ZATEN TANIMLI bir degiskeni EZMEZ - kabuktan
    verilen deger .env'den once gelir.
    """
    env_file = path or (REPO_ROOT / ".env")
    try:
        text = env_file.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


load_dotenv()


def _env_path(name: str) -> Path | None:
    raw = os.environ.get(name, "").strip()
    return Path(raw).expanduser() if raw else None


# ---------------------------------------------------------------------------
# Fairino Python SDK
# ---------------------------------------------------------------------------

def _sdk_candidates() -> list[Path]:
    """SDK'nin aranacagi yerler, oncelik sirasiyla."""
    out: list[Path] = []
    env = _env_path("FAIRINO_SDK_DIR")
    if env:
        out.append(env)
    out.append(REPO_ROOT / "vendor" / "fairino-python-sdk" / "linux")
    out.append(LEGACY_CLIENT_DIR / "fairino-python-sdk" / "linux")
    out.append(HOME / "fairino-python-sdk" / "linux")
    return out


def _scan_for_sdk() -> Path | None:
    """Son care: ev dizininde ara. Yavas (bu yuzden en sonda), ama SDK
    ~500MB oldugu icin insanlar onu her yere kuruyor."""
    try:
        out = subprocess.run(
            ["find", str(HOME), "-type", "d", "-path", "*fairino-python-sdk/linux"],
            capture_output=True, text=True, timeout=30,
        ).stdout
    except Exception:
        return None
    for line in out.splitlines():
        line = line.strip()
        # build/install altindaki kopyalar derleme ciktisi - onlari alma.
        if line and "build" not in line and "install" not in line:
            return Path(line)
    return None


def sdk_dir() -> Path:
    for cand in _sdk_candidates():
        if (cand / "fairino").is_dir():
            return cand
    found = _scan_for_sdk()
    if found:
        return found
    bakilan = "\n  ".join(str(p) for p in _sdk_candidates())
    raise PathError(
        "Fairino Python SDK bulunamadi (fairino-python-sdk/linux).\n"
        f"Bakilan yerler:\n  {bakilan}\n"
        "Cozum: SDK'yi klonla\n"
        "  git clone https://github.com/FAIR-INNOVATION/fairino-python-sdk.git\n"
        "ve yerini .env icinde FAIRINO_SDK_DIR ile bildir."
    )


def ensure_sdk_on_path() -> Path:
    """SDK'yi sys.path'e ekler ve yolunu doner. Bircok kez cagrilabilir."""
    d = sdk_dir()
    s = str(d)
    if s not in sys.path:
        sys.path.insert(0, s)
    return d


# ---------------------------------------------------------------------------
# Calisma zamani durumu (kayitli hareketler, durum onbellegi, taslak)
# ---------------------------------------------------------------------------

def state_dir() -> Path:
    """Calisma zamaninda uretilen dosyalarin yeri.

    Bunlar repoya ait DEGIL: makineye ozgu, her calistirmada degisiyor.
    XDG state dizini kullaniliyor; FAIRINO_STATE_DIR ile degistirilebilir.
    """
    env = _env_path("FAIRINO_STATE_DIR")
    if env:
        env.mkdir(parents=True, exist_ok=True)
        return env
    xdg = os.environ.get("XDG_STATE_HOME", "").strip()
    base = Path(xdg).expanduser() if xdg else HOME / ".local" / "state"
    d = base / "fairino-cobot"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _with_legacy_migration(name: str, legacy_name: str | None = None) -> Path:
    """Yeni konumdaki dosyayi doner; yoksa ve eski konumda varsa BIR KEZ
    kopyalar. Kullanicinin ogrettigi hareketler dizin duzeni degisti diye
    kaybolmasin diye."""
    target = state_dir() / name
    if target.exists():
        return target
    legacy = LEGACY_CLIENT_DIR / (legacy_name or name)
    if legacy.exists():
        try:
            shutil.copy2(legacy, target)
            print(f"[fairino_cobot] {legacy} -> {target} tasindi (eski konumdan bir kerelik).")
        except OSError:
            # Kopyalanamadiysa eski dosyayi dogrudan kullan - veri kaybindansa iyidir.
            return legacy
    return target


def moves_file() -> Path:
    """movecobot'un kayitli hareketleri."""
    return _with_legacy_migration("movecobot_moves.json")


def state_cache_file() -> Path:
    """movecobot'un en son okudugu robot durumu - konumcobot bunu robota hic
    baglanmadan okur (port 20004 tek-tuketicili)."""
    return _with_legacy_migration("movecobot_state_cache.json", ".movecobot_state_cache.json")


def draft_file() -> Path:
    """Yarim kalmis hareket kaydi taslagi."""
    return _with_legacy_migration("movecobot_record_draft.json", ".movecobot_record_draft.json")


# ---------------------------------------------------------------------------
# ROS2 / vendor agaclari
# ---------------------------------------------------------------------------

def ros2_ws() -> Path:
    return _env_path("FAIRINO_ROS2_WS") or (HOME / "ros2_ws")


def mirror_dir() -> Path:
    """Resmi Fairino kaynak surumlerinin yerel aynasi.

    Bu "ayna" ozel bir sey DEGIL: frcobot_ros2'nin V3.0.0_RobotV3.9.9 tag'i,
    icinde fairino_hardware_v3_8_2_11'den v3_9_9'a kadar butun surumleri
    tasiyor. Yani herkes indirebilir:

        git clone --depth 1 --branch V3.0.0_RobotV3.9.9 \\
            https://github.com/FAIR-INNOVATION/frcobot_ros2.git

    setup_target.py robotun bildirdigi surume uyan klasoru buradan secip
    workspace'e kuruyor.
    """
    return _env_path("FAIRINO_MIRROR_DIR") or (HOME / "fairino_official_mirror")


def moveit_ws() -> Path:
    return _env_path("FAIRINO_MOVEIT_WS") or (HOME / "ws_moveit2")


def patches_dir() -> Path:
    """Vendor surucusune uygulanan yamalarimizin klasoru.

    TEK KAYNAK OLMASI KRITIK: hem tools/apply_ros2_patches.py hem
    setup_target.py buradan okuyor. Eskiden setup_target'in KENDI kucuk
    yamasi vardi (yalnizca robot_ip blogunu ekliyordu) ve temiz bir makinede
    once o calisiyordu. Sonucu: UDP servo kanali, olculen cmdT, hiz
    sinirlayici ve bosta servo cikisi HIC uygulanmiyordu - robot sicramali
    calisir, "axis N command speed exceeds the limit" faultu verebilirdi.
    Ustelik sonradan tam yamayi uygulamak da MUMKUN OLMUYORDU: robot_ip
    blogu eklendigi icin patch'in baglami kayiyor ve yama tutmuyordu.
    """
    env = _env_path("FAIRINO_PATCHES_DIR")
    if env and env.is_dir():
        return env
    aday = REPO_ROOT / "ros2" / "patches"
    if aday.is_dir():
        return aday
    raise PathError(
        "ros2/patches klasoru bulunamadi.\n"
        f"Bakilan: {aday}\n"
        "Paket kaynak agacindan ('pip install -e .') kurulmus olmali, ya da "
        "FAIRINO_PATCHES_DIR ile yeri bildirilmeli."
    )


def last_target_file() -> Path:
    """fairino_launch'in en son algiladigi robotu yazdigi dosya."""
    return _env_path("FAIRINO_LAST_TARGET") or (HOME / ".fairino_last_target")


# ---------------------------------------------------------------------------
# Robot algilama
# ---------------------------------------------------------------------------

def robot_ip_override() -> str | None:
    """FAIRINO_ROBOT_IP verilmisse ag taramasi hic yapilmaz.

    Robotun IP'si makineye/aga ozgu bir deger - kodun icine gomulmez,
    .env ile verilir (.env.example'a bak). Verilmezse detect_robot.py
    agi tarayip kendisi bulur.
    """
    ip = os.environ.get("FAIRINO_ROBOT_IP", "").strip()
    return ip or None


def detect_robot_script() -> Path:
    """detect_robot.py'nin yolu.

    Artik paketin kendi icinde - eskiden ~/detect_robot.py'de sabitti ve
    dosya baska yere konunca sessizce kirilyordu. Eski konum yine de son
    basamak olarak deneniyor.
    """
    packaged = PACKAGE_DIR / "detect_robot.py"
    if packaged.exists():
        return packaged
    legacy = HOME / "detect_robot.py"
    if legacy.exists():
        return legacy
    raise PathError(
        f"detect_robot.py bulunamadi. Bakilan: {packaged}, {legacy}"
    )

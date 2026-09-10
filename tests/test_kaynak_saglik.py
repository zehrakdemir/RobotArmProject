"""Kaynak agacinin temel saglik kontrolleri.

Robot, ROS2 veya SDK GEREKTIRMEZ. Amac, CI'nin bu kurulumdan bagimsiz
olarak yakalayabilecegi hatalari yakalamasi: sozdizimi hatalari, geride
kalmis eski import lar, kaybolan giris noktalari, repoya sizmis gizli
dosyalar.
"""

import ast
import compileall
import io
import re
from contextlib import redirect_stdout
from pathlib import Path

import pytest

try:  # tomllib yalnizca Python 3.11+; hedefimiz 3.10 (ROS2 Humble).
    import tomllib
except ModuleNotFoundError:
    tomllib = None

REPO_ROOT = Path(__file__).resolve().parent.parent
PKG_DIR = REPO_ROOT / "src" / "fairino_cobot"


def python_dosyalari(kok: Path) -> list[Path]:
    return sorted(p for p in kok.rglob("*.py") if "__pycache__" not in p.parts)


def konsol_komutlari() -> dict[str, str]:
    """pyproject.toml icindeki [project.scripts] tablosu.

    tomllib Python 3.11+ ile geliyor ama hedefimiz 3.10 (ROS2 Humble'in
    Python'u). Yoksa bu basit tablo elle ayristiriliyor - sirf bu test
    icin bir bagimlilik (tomli) eklemeye degmez.
    """
    metin = (REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    if tomllib is not None:
        return tomllib.loads(metin).get("project", {}).get("scripts", {})

    out: dict[str, str] = {}
    icinde = False
    for satir in metin.splitlines():
        s = satir.strip()
        if s.startswith("["):
            icinde = s == "[project.scripts]"
            continue
        if icinde and "=" in s and not s.startswith("#"):
            k, _, v = s.partition("=")
            out[k.strip()] = v.strip().strip('"').strip("'")
    return out


# ---------------------------------------------------------------------------
# Sozdizimi
# ---------------------------------------------------------------------------

def test_tum_kaynaklar_derleniyor():
    for kok in (PKG_DIR, REPO_ROOT / "examples", REPO_ROOT / "tools"):
        buf = io.StringIO()
        with redirect_stdout(buf):
            ok = compileall.compile_dir(str(kok), quiet=2, force=True)
        assert ok, f"{kok} altinda derlenmeyen dosya var:\n{buf.getvalue()}"


# ---------------------------------------------------------------------------
# Import hijyeni
# ---------------------------------------------------------------------------

DUZ_IMPORTLAR = ("gripper_controller", "pickplace", "movecobot", "konumcobot")


def test_paket_ici_import_lar_duz_dosya_adi_kullanmiyor():
    """Moduller eskiden birbirini duz dosya adiyla import ediyordu
    (`from gripper_controller import ...`) ve bu ancak sys.path e kendi
    klasorlerini eklediklerinde calisiyordu. Paketlestikten sonra geride
    kalmis boyle bir import sessizce ImportError verirdi."""
    hatalar = []
    for p in python_dosyalari(PKG_DIR):
        agac = ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
        for d in ast.walk(agac):
            if isinstance(d, ast.ImportFrom) and d.level == 0 and d.module in DUZ_IMPORTLAR:
                hatalar.append(f"{p.relative_to(REPO_ROOT)}:{d.lineno} -> from {d.module} import ...")
    assert not hatalar, "duz dosya adiyla import:\n  " + "\n  ".join(hatalar)


def test_sys_path_ile_sdk_eklenmiyor():
    """SDK yolu artik SADECE paths.ensure_sdk_on_path() uzerinden
    eklenmeli - her modul kendi tahminini yaparsa dizin duzeni degisince
    yine sessizce kirilir."""
    hatalar = []
    for p in python_dosyalari(PKG_DIR):
        if p.name == "paths.py":
            continue
        for i, satir in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if "fairino-python-sdk" in satir and satir.strip().startswith("sys.path"):
                hatalar.append(f"{p.relative_to(REPO_ROOT)}:{i}")
    assert not hatalar, "elle SDK yolu ekleyen dosyalar: " + ", ".join(hatalar)


def test_mutlak_ev_dizini_yolu_yok():
    """/home/<biri>/... seklinde sabit bir yol baska makinede calismaz."""
    hatalar = []
    desen = re.compile(r"/home/[a-z][a-z0-9_-]*/")
    for p in python_dosyalari(PKG_DIR) + python_dosyalari(REPO_ROOT / "tools"):
        for i, satir in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if desen.search(satir) and not satir.lstrip().startswith("#"):
                hatalar.append(f"{p.relative_to(REPO_ROOT)}:{i}: {satir.strip()[:70]}")
    assert not hatalar, "gomulu ev dizini yolu:\n  " + "\n  ".join(hatalar)


# ---------------------------------------------------------------------------
# Paket tanimi
# ---------------------------------------------------------------------------

def test_konsol_komutlarinin_hedefleri_var():
    """pyproject'teki her giris noktasinin modulu ve main() fonksiyonu
    gercekten var mi? Moduller ROS2 gerektirdigi icin IMPORT ETMEDEN,
    kaynagi ayristirarak bakiliyor."""
    scripts = konsol_komutlari()
    assert scripts, "hic konsol komutu tanimlanmamis"

    for komut, hedef in scripts.items():
        modul, _, fonksiyon = hedef.partition(":")
        yol = REPO_ROOT / "src" / Path(*modul.split(".")).with_suffix(".py")
        assert yol.exists(), f"{komut}: {yol.relative_to(REPO_ROOT)} yok"

        agac = ast.parse(yol.read_text(encoding="utf-8"), filename=str(yol))
        adlar = {
            d.name for d in agac.body
            if isinstance(d, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        assert fonksiyon in adlar, f"{komut}: {modul} icinde {fonksiyon}() yok"


# ---------------------------------------------------------------------------
# Hijyen
# ---------------------------------------------------------------------------

def test_env_dosyasi_repoda_degil():
    """.env gizli degerler icerir ve ASLA commit edilmemeli."""
    assert not (REPO_ROOT / ".env").exists() or ".env" in (
        REPO_ROOT / ".gitignore"
    ).read_text(encoding="utf-8")


def test_env_example_var():
    ornek = REPO_ROOT / ".env.example"
    assert ornek.exists(), ".env.example olmadan kimse hangi degiskenlerin oldugunu bilemez"
    metin = ornek.read_text(encoding="utf-8")
    for anahtar in ("FAIRINO_ROBOT_IP", "FAIRINO_SDK_DIR", "FAIRINO_ROS2_WS"):
        assert anahtar in metin


@pytest.mark.parametrize("ad", ["README.md", "pyproject.toml", "requirements.txt", "Dockerfile"])
def test_temel_dosyalar_var(ad):
    assert (REPO_ROOT / ad).exists(), f"{ad} yok"

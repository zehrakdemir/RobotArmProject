#!/usr/bin/env python3
"""Robotun yazilim surumune ait frcobot_ros2 kaynagini KENDI tag'iyle indirir.

NEDEN: Aynada (frcobot_ros2 V3.0.0_RobotV3.9.9) bir suru fairino_hardware
surumu var ama fairino_msgs TEK - 3.9.9'unki. Aynada olmayan bir surumun
surucusu 3.9.9 mesajlariyla derlenince kirilabiliyor (3.9.3'te oldu):
  'fairino_msgs::msg::RobotNonrtState_' has no member named 'servoj_cmd_num'
Bu arac surumun kendi tag'inden uc paketi (fairino_hardware, fairino_msgs,
fairino_description) YAN YANA indirir; setup_target bu klasoru kendisi bulur.

ROBOTA BAGLANMAZ - yalnizca GitHub'dan dosya indirir.

Kullanim:
    fairino-fetch-source 3.9.2                                # tag'i bulur, indirir
    fairino-fetch-source 3.9.8 --tag v3.0.0_RobotV3.9.8_v1.1  # tag'i elle ver
    fairino-fetch-source --list                               # GitHub'daki tag'ler

Docker'da:
    docker compose exec cobot fairino-fetch-source 3.9.2
    docker compose restart cobot
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from fairino_cobot import paths, setup_target

REPO_URL = "https://github.com/FAIR-INNOVATION/frcobot_ros2.git"

# Olculen tag bicimleri (git ls-remote, 14 Eylul 2026):
#   V3.0.0_RobotV3.9.2        (buyuk V)
#   v3.0.0_RobotV3.9.8        (kucuk v - Fairino tutarli degil)
#   V3.0.0_RobotV3.9.9_v1.1   (ayni robot surumunun ek surumu)
_TAG_RE = re.compile(r"^[vV]\d+\.\d+\.\d+_RobotV(\d+)\.(\d+)\.(\d+)(_\S+)?$")


class FetchError(RuntimeError):
    """Indirme yapilamadi. Mesaj sebebini ve cozumu soyler."""


def surum_ayristir(metin: str) -> tuple[str, str, str]:
    m = re.fullmatch(r"\s*[vV]?(\d+)\.(\d+)\.(\d+)\s*", metin)
    if not m:
        raise FetchError(f"surum '{metin}' anlasilmadi - beklenen bicim: 3.9.2")
    return m.group(1), m.group(2), m.group(3)


def tagleri_ayristir(ls_remote_ciktisi: str) -> list[str]:
    """'git ls-remote --tags' ciktisindan tag adlari (^{} tekrarlari haric)."""
    tagler = []
    for satir in ls_remote_ciktisi.splitlines():
        parca = satir.split()
        if len(parca) != 2 or not parca[1].startswith("refs/tags/"):
            continue
        ad = parca[1][len("refs/tags/"):]
        if not ad.endswith("^{}"):
            tagler.append(ad)
    return tagler


def tag_sec(tagler: list[str], maj: str, minr: str, min2: str) -> tuple[str | None, list[str]]:
    """(secilen tag, ayni surume ait ekli tag'ler).

    Ekisiz tag secilir (ornek: v3.0.0_RobotV3.9.8). Ekli olanlar (_v1.1)
    SESSIZCE SECILMEZ: aralarindaki farki Fairino belgelemiyor. Listelenir,
    istenirse --tag ile acikca verilir.
    """
    ekisiz, ekli = [], []
    for t in tagler:
        m = _TAG_RE.match(t)
        if not m or m.groups()[:3] != (maj, minr, min2):
            continue
        (ekli if m.group(4) else ekisiz).append(t)
    return (sorted(ekisiz)[0] if ekisiz else None), sorted(ekli)


def _git(*args: str) -> str:
    try:
        r = subprocess.run(["git", *args], capture_output=True, text=True)
    except FileNotFoundError as e:
        raise FetchError("'git' komutu yok (apt install git)") from e
    if r.returncode != 0:
        raise FetchError(f"git {' '.join(args)} basarisiz:\n  {(r.stderr or r.stdout).strip()[:500]}")
    return r.stdout


def uzak_tagler(repo_url: str = REPO_URL) -> list[str]:
    return tagleri_ayristir(_git("ls-remote", "--tags", repo_url))


def klonu_dogrula(klon: Path, maj: str, minr: str, min2: str) -> Path:
    """Klon setup_target'in kullanabilecegi durumda mi? Surucu klasorunu doner."""
    surucu = setup_target.tam_klon_surucusu(klon, maj, minr, min2)
    if surucu is None or not setup_target.source_is_complete(surucu):
        raise FetchError(
            "indirilen tag'de kullanilabilir fairino_hardware yok. Beklenen: "
            "src/fairino_hardware_interface.cpp ve libfairino/lib/libfairino.so*")
    eksik = [p for p in ("fairino_msgs", "fairino_description") if not (klon / p).is_dir()]
    if eksik:
        raise FetchError(f"indirilen tag'de eksik paket: {', '.join(eksik)}")
    return surucu


def yama_tutuyor_mu(surucu: Path) -> bool | None:
    """Surucu yamamiz bu kaynaga uygulanabilir mi? (--dry-run, kaynaga dokunmaz)

    None: denenemedi ('patch' komutu ya da yama dosyasi yok).
    """
    if not shutil.which("patch"):
        return None
    try:
        yama = paths.patches_dir() / "fairino_hardware_interface.patch"
    except paths.PathError:
        return None
    if not yama.exists():
        return None
    with tempfile.TemporaryDirectory() as gecici:
        # Yamadaki yollar 'fairino_hardware/...'; surum klasorunun adi farkli olabilir.
        shutil.copytree(surucu, Path(gecici) / "fairino_hardware")
        r = subprocess.run(
            ["patch", "-p1", "--dry-run", "--forward", "--batch", "-i", str(yama)],
            cwd=gecici, capture_output=True, text=True,
        )
    return r.returncode == 0


def indir(maj: str, minr: str, min2: str, tag: str | None = None,
          repo_url: str = REPO_URL, zorla: bool = False) -> tuple[Path, str]:
    """Surumun tag'ini kaynak klasorune indirir. (hedef klasor, kullanilan tag) doner."""
    hedef = setup_target.tam_klon_yerleri(maj, minr, min2)[0]
    if hedef.exists() and not zorla:
        raise FetchError(f"{hedef} zaten var. Yeniden indirmek icin --force.")

    if tag is None:
        tagler = uzak_tagler(repo_url)
        tag, ekli = tag_sec(tagler, maj, minr, min2)
        if tag is None:
            if ekli:
                raise FetchError(
                    f"{maj}.{minr}.{min2} icin ekisiz tag yok; ekli olanlar: {', '.join(ekli)}\n"
                    "Hangisinin istendigini --tag ile acikca ver.")
            var_olanlar = sorted(t for t in tagler if _TAG_RE.match(t))
            raise FetchError(
                f"{repo_url} icinde {maj}.{minr}.{min2} icin tag yok.\n"
                f"Var olanlar: {', '.join(var_olanlar) or '(hic)'}")
        if ekli:
            print(f"Not: ayni surumun ekli tag'leri de var: {', '.join(ekli)} "
                  f"(secilen: {tag}; digeri icin --tag)")

    hedef.parent.mkdir(parents=True, exist_ok=True)
    # Nokta ile baslayan gecici klasor: setup_target'in aramasina takilmaz.
    gecici = Path(tempfile.mkdtemp(prefix=f".{hedef.name}.", dir=hedef.parent))
    try:
        klon = gecici / hedef.name
        print(f"Indiriliyor: {repo_url} @ {tag}")
        # Dockerfile'daki 3.9.3 klonuyla ayni yontem: yalnizca gereken klasorler iner.
        _git("clone", "--depth", "1", "--branch", tag, "--filter=blob:none", "--sparse",
             repo_url, str(klon))
        _git("-C", str(klon), "sparse-checkout", "set",
             "fairino_hardware", f"fairino_hardware_v{maj}_{minr}_{min2}",
             "fairino_msgs", "fairino_description")
        klonu_dogrula(klon, maj, minr, min2)
        shutil.rmtree(klon / ".git")
        (klon / "KAYNAK_TAG.txt").write_text(f"{repo_url}\n{tag}\n", encoding="utf-8")
        # Klasor ANCAK dogrulandiktan sonra yerine konuyor: yarim bir indirme
        # setup_target'a "kaynak var" diye gorunmesin.
        if hedef.exists():
            shutil.rmtree(hedef)
        klon.rename(hedef)
    finally:
        shutil.rmtree(gecici, ignore_errors=True)
    return hedef, tag


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="fairino-fetch-source",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("surum", nargs="?", help="robotun yazilim surumu, ornek: 3.9.2")
    ap.add_argument("--tag", help="tag'i elle ver, ornek: v3.0.0_RobotV3.9.8_v1.1")
    ap.add_argument("--force", action="store_true", help="klasor varsa silip yeniden indir")
    ap.add_argument("--list", action="store_true", help="GitHub'daki surum tag'lerini listele")
    args = ap.parse_args(argv)

    try:
        if args.list:
            for t in sorted(uzak_tagler()):
                if _TAG_RE.match(t):
                    print(t)
            return 0
        if not args.surum:
            ap.error("surum gerekli (ornek: fairino-fetch-source 3.9.2) ya da --list")
        maj, minr, min2 = surum_ayristir(args.surum)
        hedef, tag = indir(maj, minr, min2, tag=args.tag, zorla=args.force)
    except FetchError as e:
        print(f"HATA: {e}", file=sys.stderr)
        return 1

    surucu = setup_target.tam_klon_surucusu(hedef, maj, minr, min2)
    print(f"\nIndirildi: {hedef}  (tag {tag}, surucu klasoru: {surucu.name})")
    tutuyor = yama_tutuyor_mu(surucu)
    if tutuyor is True:
        print("Surucu yamamiz bu kaynaga TUTUYOR (dry-run). Derlenip derlenmedigi ilk acilista gorulur.")
    elif tutuyor is False:
        print("UYARI: surucu yamamiz bu kaynaga TUTMUYOR (dry-run). setup_target yamasiz surucuyle")
        print("       devam etmez; once ros2/patches/fairino_hardware_interface.patch uyarlanmali.")
    else:
        print("Not: yama uyumu denenemedi ('patch' komutu ya da yama dosyasi yok).")
    print("\nSimdi fairino_launch'i yeniden baslat (Docker: docker compose restart cobot).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

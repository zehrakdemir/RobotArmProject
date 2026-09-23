"""Vision (kamera + nesne tespit) API'sine SAF (bagimliliksiz) HTTP client.

NEDEN AYRI MODUL: yalnizca stdlib (urllib) kullanir - movecobot.py'nin
gerektirdigi rclpy/moveit_py/Fairino SDK olmadan da import edilebilir/test
edilebilir (bkz. yol_onizleme.py, ik_secimi.py ile ayni gerekce).

VISION_API_URL: valiz-olcum-robot repo'sundaki tools/detect_api.py servisi
(yeni, Zehra'nin dosyalarina dokunmaz - bkz. o dosyanin docstring'i).
Varsayilan container-to-container adres: ikisi de ayni Docker Desktop
motorunda calisiyor, host.docker.internal ile birbirine ulasiyor.

Canli kamera goruntusu (MJPEG) BU MODULDEN GECMEZ: tarayici cam_web.py'ye
(host, port 8600) DOGRUDAN baglanir, hicbir container arada degil.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request

VISION_API_URL = os.environ.get("VISION_API_URL", "http://host.docker.internal:8502")
VISION_TIMEOUT_S = float(os.environ.get("VISION_TIMEOUT_S", "120"))


class VizyonHatasi(RuntimeError):
    """Vision API'sine ulasilamadi ya da tespit basarisiz oldu."""


def _istek(yol: str, method: str = "GET", govde: dict | None = None,
          zaman_asimi: float = 10.0, taban_url: str | None = None) -> dict:
    url = (taban_url or VISION_API_URL).rstrip("/") + yol
    data = json.dumps(govde).encode() if govde is not None else None
    basliklar = {"Content-Type": "application/json"} if data is not None else {}
    req = urllib.request.Request(url, data=data, method=method, headers=basliklar)
    try:
        with urllib.request.urlopen(req, timeout=zaman_asimi) as r:
            return json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        try:
            hata_govde = json.loads(e.read().decode())
        except Exception:
            hata_govde = {}
        raise VizyonHatasi(hata_govde.get("err") or f"HTTP {e.code}") from e
    except urllib.error.URLError as e:
        raise VizyonHatasi(f"vision API'sine ulasilamadi ({url}): {e.reason}") from e
    except (TimeoutError, OSError) as e:
        raise VizyonHatasi(f"vision API zaman asimi ({url}): {e}") from e


def bilgi(taban_url: str | None = None) -> dict:
    """/health + /slots birlesimi + tarayicinin kamera akisina (cam_web.py,
    backend'den DEGIL) baglanacagi adres ipucu. Erisilemezse VizyonHatasi."""
    saglik = _istek("/health", taban_url=taban_url)
    slotlar = _istek("/slots", taban_url=taban_url)
    return {**saglik, "slots": slotlar.get("slots", []),
            "cam_url": os.environ.get("VISION_CAM_URL") or None}


def algila(taban_url: str | None = None) -> dict:
    """POST /detect. Stereo+tespit suresi uzun olabilir - genis zaman asimi
    (VISION_TIMEOUT_S). Basarisizsa VizyonHatasi (arayuzde okunabilir mesaj)."""
    sonuc = _istek("/detect", method="POST", govde={}, zaman_asimi=VISION_TIMEOUT_S,
                   taban_url=taban_url)
    if not sonuc.get("ok"):
        raise VizyonHatasi(sonuc.get("err") or "tespit basarisiz")
    return sonuc

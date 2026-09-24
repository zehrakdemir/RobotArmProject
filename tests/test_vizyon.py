"""Vision API HTTP client. Robot, ROS2, SDK ve gercek ag GEREKTIRMEZ -
urllib.request.urlopen sahtelenir."""

import io
import json
import urllib.error

import pytest

from fairino_cobot import vizyon


class _SahteYanit:
    def __init__(self, payload):
        self._data = json.dumps(payload).encode()

    def read(self):
        return self._data

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def test_bilgi_health_ve_slotlari_birlestirir(monkeypatch):
    cagrilar = []

    def sahte_urlopen(req, timeout=None):
        cagrilar.append(req.full_url)
        if req.full_url.endswith("/health"):
            return _SahteYanit({"ok": True, "handeye": True, "mode": "nesne"})
        return _SahteYanit({"slots": [[1, 2, 3, 4, 5, 6]]})

    monkeypatch.setattr(vizyon.urllib.request, "urlopen", sahte_urlopen)

    b = vizyon.bilgi(taban_url="http://x:1")

    assert b["handeye"] is True and b["mode"] == "nesne"
    assert b["slots"] == [[1, 2, 3, 4, 5, 6]]
    assert cagrilar == ["http://x:1/health", "http://x:1/slots"]


def test_algila_basarili_sonucu_donuyor(monkeypatch):
    monkeypatch.setattr(vizyon.urllib.request, "urlopen",
                        lambda req, timeout=None: _SahteYanit({"ok": True, "objects": [1, 2]}))

    sonuc = vizyon.algila(taban_url="http://x:1")

    assert sonuc["ok"] is True and sonuc["objects"] == [1, 2]


def test_algila_basarisiz_govdedeki_mesaji_firlatir(monkeypatch):
    monkeypatch.setattr(vizyon.urllib.request, "urlopen",
                        lambda req, timeout=None: _SahteYanit({"ok": False, "err": "no camera"}))

    with pytest.raises(vizyon.VizyonHatasi, match="no camera"):
        vizyon.algila(taban_url="http://x:1")


def test_http_hatasi_govdesindeki_err_okunur(monkeypatch):
    def sahte_urlopen(req, timeout=None):
        raise urllib.error.HTTPError(
            req.full_url, 409, "conflict", {},
            io.BytesIO(json.dumps({"err": "another detection is already running"}).encode()))

    monkeypatch.setattr(vizyon.urllib.request, "urlopen", sahte_urlopen)

    with pytest.raises(vizyon.VizyonHatasi, match="another detection"):
        vizyon.algila(taban_url="http://x:1")


def test_baglanti_hatasi_okunabilir_mesaj_verir(monkeypatch):
    def sahte_urlopen(req, timeout=None):
        raise urllib.error.URLError("connection refused")

    monkeypatch.setattr(vizyon.urllib.request, "urlopen", sahte_urlopen)

    with pytest.raises(vizyon.VizyonHatasi, match="vision API"):
        vizyon.bilgi(taban_url="http://x:1")


def test_zaman_asimi_okunabilir_mesaj_verir(monkeypatch):
    def sahte_urlopen(req, timeout=None):
        raise TimeoutError("timed out")

    monkeypatch.setattr(vizyon.urllib.request, "urlopen", sahte_urlopen)

    with pytest.raises(vizyon.VizyonHatasi, match="timed out"):
        vizyon.algila(taban_url="http://x:1")

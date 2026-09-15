"""Container acilirkenki gecici "Starting..." sayfasi (scripts/starting_page.py).

ROBOT, ROS2 veya SDK GEREKTIRMEZ: gercek bir HTTP sunucusu bos bir portta
acilip standart kutuphaneyle istek atiliyor.
"""

import importlib.util
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

YOL = Path(__file__).resolve().parents[1] / "scripts" / "starting_page.py"
_spec = importlib.util.spec_from_file_location("starting_page", YOL)
starting_page = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(starting_page)


@pytest.fixture
def sunucu(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text("\n".join(f"satir {i}" for i in range(20)), encoding="utf-8")
    s = starting_page.http.server.ThreadingHTTPServer(
        ("127.0.0.1", 0), starting_page.isleyici_sinifi(log))
    threading.Thread(target=s.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{s.server_address[1]}"
    s.shutdown()
    s.server_close()


def _al(url):
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()


def test_her_yol_bekleme_sayfasini_donuyor(sunucu):
    kod, govde = _al(sunucu + "/")
    assert kod == 200 and "Starting" in govde


def test_api_state_basarili_donmuyor(sunucu):
    """200 donerse sayfa 'asil arayuz geldi' sanip sonsuza kadar yenilenirdi."""
    kod, _ = _al(sunucu + "/api/state")
    assert kod == 503


def test_launch_logunun_son_satirlari_gorunuyor(sunucu):
    kod, govde = _al(sunucu + "/startup-log")
    assert kod == 200
    assert govde.splitlines() == [f"satir {i}" for i in range(8, 20)]


def test_log_dosyasi_yoksa_bos(tmp_path):
    assert starting_page.son_satirlar(tmp_path / "yok.log") == ""

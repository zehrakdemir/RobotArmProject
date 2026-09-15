"""Container acilirken 8088'de gecici "Starting..." sayfasi.

NEDEN: start_all.sh web arayuzunu move_group ayaga kalkana kadar (en fazla
3 dakika) bekletiyor. O surede port bos oldugu icin tarayici "baglanilamadi"
diyordu ve kullanici sayfayi ne zaman yenileyecegini bilmiyordu (15 Eylul).

Bu sunucu container acilir acilmaz baslar; asil arayuz baslamadan hemen once
start_all.sh tarafindan kapatilir. Sayfa /api/state'i yoklar: asil arayuz
cevap verince kendini yeniler. Aradaki bir iki saniyelik bosluk sorun degil,
yoklama JS icinde devam ediyor (tarayicinin hata sayfasina dusmuyor).

Robota BAGLANMAZ, ROS'a dokunmaz; yalnizca standart kutuphane.
"""

from __future__ import annotations

import argparse
import http.server
from pathlib import Path

SON_SATIR = 12

SAYFA = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>movecobot - starting</title>
<style>
  body { margin: 0; min-height: 100vh; display: flex; flex-direction: column;
         align-items: center; justify-content: center; gap: 16px; padding: 24px;
         box-sizing: border-box; background: #0f1115; color: #e6e9ef;
         font: 14px/1.5 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; text-align: center; }
  .spin { width: 30px; height: 30px; border-radius: 50%; border: 3px solid #2a2f3a;
          border-top-color: #4c9aff; animation: spin 1s linear infinite; }
  @keyframes spin { to { transform: rotate(360deg); } }
  h1 { margin: 0; font-size: 15px; font-weight: 600; line-height: 1.2; }
  p { margin: 0; color: #8b93a3; font-size: 13px; }
  pre { max-width: 680px; width: 100%; margin: 0; white-space: pre-wrap; text-align: left;
        color: #8b93a3; font: 12px/1.55 ui-monospace, Consolas, monospace; }
</style>
</head>
<body>
  <div class="spin"></div>
  <h1>Starting robot software…</h1>
  <p>The driver and MoveIt are starting. This page opens the control panel automatically.</p>
  <pre id="log"></pre>
<script>
async function poll() {
  try {
    const r = await fetch("/api/state", { cache: "no-store" });
    if (r.ok) { location.reload(); return; }
  } catch (_) { /* gecis sirasinda port bir an bos olabilir */ }
  try {
    const t = await fetch("/startup-log", { cache: "no-store" });
    if (t.ok) document.getElementById("log").textContent = await t.text();
  } catch (_) {}
  setTimeout(poll, 2000);
}
poll();
</script>
</body>
</html>
"""


def son_satirlar(log_yolu: Path, adet: int = SON_SATIR) -> str:
    """Launch logunun son satirlari. Dosya yoksa ya da okunamazsa bos."""
    try:
        satirlar = log_yolu.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return ""
    return "\n".join(satirlar[-adet:])


def isleyici_sinifi(log_yolu: Path) -> type[http.server.BaseHTTPRequestHandler]:
    class Isleyici(http.server.BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - http.server bu adi cagiriyor
            if self.path.startswith("/api/"):
                # Asil arayuz degiliz: /api/state'e 200 donersek sayfa sonsuz yenilenir.
                self._gonder(503, "application/json", b'{"starting": true}')
            elif self.path == "/startup-log":
                self._gonder(200, "text/plain; charset=utf-8", son_satirlar(log_yolu).encode())
            else:
                self._gonder(200, "text/html; charset=utf-8", SAYFA.encode())

        def _gonder(self, kod: int, tur: str, govde: bytes) -> None:
            self.send_response(kod)
            self.send_header("Content-Type", tur)
            self.send_header("Content-Length", str(len(govde)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(govde)

        def log_message(self, *args) -> None:
            pass  # container logunu her yoklamada doldurmasin

    return Isleyici


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="0.0.0.0")  # noqa: S104 - container'dan disari acilmali
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--log", type=Path, default=Path("/tmp/fairino_launch.log"))  # noqa: S108
    args = ap.parse_args()
    sunucu = http.server.ThreadingHTTPServer((args.host, args.port), isleyici_sinifi(args.log))
    sunucu.serve_forever()


if __name__ == "__main__":
    main()

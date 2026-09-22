#!/usr/bin/env python3
"""
CANLI KAMERA ONIZLEME - iki USB kamerayi tarayicida yan yana goster + cift kaydet.

HOST tarafinda calisir (WSL/konteyner DEGIL - orada USB kamera yok).
Sadece stdlib + opencv; ek paket yok.

    python tools/cam_web.py --left 2 --right 3 --port 8600
    # tarayici:  http://localhost:8600

Sayfadaki "CIFT KAYDET" -> snaps/left_<ts>.png + snaps/right_<ts>.png
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

STATE = {"L": None, "R": None, "run": True, "idx": {"L": 0, "R": 1}, "snapdir": "snaps",
         "rotate": {"L": None, "R": None}}
LOCK = {"L": threading.Lock(), "R": threading.Lock()}

_ROTATE_CODES = {
    "cw": cv2.ROTATE_90_CLOCKWISE,
    "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE,
    "180": cv2.ROTATE_180,
}


def _capture(key: str, index: int, w: int, h: int):
    backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else 0
    cap = cv2.VideoCapture(index, backend)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if not cap.isOpened():
        print(f"[{key}] /dev index {index} ACILAMADI")
        return
    print(f"[{key}] index {index} acildi")
    while STATE["run"]:
        ok, f = cap.read()
        if ok and f is not None:
            # her karede OKUNUYOR (sabit degil) - sayfadaki butonla sunucuyu
            # yeniden baslatmadan CANLI degistirilebilsin diye.
            rot = _ROTATE_CODES.get(STATE["rotate"][key])
            if rot is not None:
                f = cv2.rotate(f, rot)
            with LOCK[key]:
                STATE[key] = f
        time.sleep(0.02)
    cap.release()


PAGE = """<!doctype html><meta charset=utf-8><title>Kamera onizleme</title>
<style>body{{background:#111;color:#eee;font:14px system-ui;margin:0;padding:12px}}
.row{{display:flex;gap:12px;flex-wrap:wrap}}
.cam{{flex:1;min-width:360px}} img{{width:100%;border:1px solid #333;background:#000}}
h3{{margin:6px 0}} button{{font:16px system-ui;padding:10px 18px;margin-top:10px;cursor:pointer}}
.rot{{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}}
.rot button{{font:13px system-ui;padding:6px 10px;margin:0}}
.rot button.on{{background:#4c8dff;color:#fff}}
#msg{{margin-top:8px;color:#8f8}}</style>
<h2>Canli kamera onizleme</h2>
<div class=row>
 <div class=cam><h3>SOL (index {L})</h3><img src="/stream/L">
  <div class=rot id=rotL></div></div>
 <div class=cam><h3>SAG (index {R})</h3><img src="/stream/R">
  <div class=rot id=rotR></div></div>
</div>
<button onclick="snap()">CIFT KAYDET</button> <span id=msg></span>
<script>
const MODES = [["none","Yok"],["cw","90° sag"],["ccw","90° sol"],["180","180°"]];
function buildRot(cam, current){{
 const el = document.getElementById('rot'+cam);
 el.innerHTML = '';
 for (const [mode,label] of MODES) {{
   const b = document.createElement('button');
   b.textContent = label;
   if (mode === current) b.classList.add('on');
   b.onclick = () => setRotate(cam, mode, b);
   el.appendChild(b);
 }}
}}
async function setRotate(cam, mode, btn){{
 await fetch('/rotate?cam='+cam+'&mode='+mode, {{method:'POST'}});
 for (const c of btn.parentElement.children) c.classList.remove('on');
 btn.classList.add('on');
}}
buildRot('L', '{rotL}'); buildRot('R', '{rotR}');
async function snap(){{
 document.getElementById('msg').textContent='...';
 const r=await fetch('/snap',{{method:'POST'}}); const j=await r.json();
 document.getElementById('msg').textContent = j.ok ? ('kaydedildi: '+j.left+' , '+j.right) : ('HATA: '+j.err);
}}
</script>
"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):  # sessiz
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = PAGE.format(L=STATE["idx"]["L"], R=STATE["idx"]["R"],
                               rotL=STATE["rotate"]["L"] or "none",
                               rotR=STATE["rotate"]["R"] or "none").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/stream/"):
            key = self.path.rsplit("/", 1)[-1].upper()
            if key not in ("L", "R"):
                self.send_error(404); return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while STATE["run"]:
                    with LOCK[key]:
                        f = STATE[key]
                    if f is None:
                        time.sleep(0.1); continue
                    ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ok:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode()
                                     + b"\r\n\r\n" + jpg.tobytes() + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path.startswith("/rotate"):
            import json
            from urllib.parse import urlsplit, parse_qs
            q = parse_qs(urlsplit(self.path).query)
            cam = (q.get("cam", [""])[0]).upper()
            mode = q.get("mode", ["none"])[0]
            if cam not in ("L", "R") or mode not in ("none", "cw", "ccw", "180"):
                self.send_error(400); return
            STATE["rotate"][cam] = None if mode == "none" else mode
            out = json.dumps({"ok": True, "cam": cam, "mode": mode}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
            return
        if self.path != "/snap":
            self.send_error(404); return
        import json
        import os
        try:
            os.makedirs(STATE["snapdir"], exist_ok=True)
            # birkac deneme - anlik bozuk kare / USB dropout'a karsi
            fL = fR = None
            for _ in range(20):
                with LOCK["L"]:
                    a = STATE["L"]
                with LOCK["R"]:
                    b = STATE["R"]
                ok = lambda x: x is not None and getattr(x, "size", 0) > 1000
                if ok(a) and ok(b):
                    fL, fR = a.copy(), b.copy()
                    break
                time.sleep(0.1)
            if fL is None or fR is None:
                raise RuntimeError("gecerli kare alinamadi (kamera dropout?)")

            ts = time.strftime("%Y%m%d_%H%M%S")
            pL = os.path.join(STATE["snapdir"], f"left_{ts}.png")
            pR = os.path.join(STATE["snapdir"], f"right_{ts}.png")
            wL = cv2.imwrite(pL, fL)
            wR = cv2.imwrite(pR, fR)
            if not (wL and wR and os.path.exists(pL) and os.path.exists(pR)):
                raise RuntimeError(f"yazma basarisiz (L={wL} R={wR})")
            out = json.dumps({"ok": True, "left": pL, "right": pR,
                              "L_shape": list(fL.shape), "R_shape": list(fR.shape)}).encode()
        except Exception as exc:  # noqa: BLE001
            out = json.dumps({"ok": False, "err": str(exc)}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(out)))
        self.end_headers()
        self.wfile.write(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", type=int, default=0)
    ap.add_argument("--right", type=int, default=1)
    ap.add_argument("--port", type=int, default=8600)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--snapdir", default="snaps")
    ap.add_argument("--rotate", choices=["none", "cw", "ccw", "180"], default="cw",
                     help="her iki kamerayi da bu yonde dondur (varsayilan: saat yonunde 90)")
    args = ap.parse_args()

    STATE["idx"] = {"L": args.left, "R": args.right}
    STATE["snapdir"] = args.snapdir
    initial_rot = None if args.rotate == "none" else args.rotate
    STATE["rotate"] = {"L": initial_rot, "R": initial_rot}
    threading.Thread(target=_capture, args=("L", args.left, args.width, args.height),
                     daemon=True).start()
    threading.Thread(target=_capture, args=("R", args.right, args.width, args.height),
                     daemon=True).start()
    time.sleep(1.0)

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), H)
    print(f"-> http://localhost:{args.port}   (Ctrl+C ile durdur)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    STATE["run"] = False
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

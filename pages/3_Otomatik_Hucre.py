"""
OTOMATIK COK-NESNELI HUCRE - gercek zamanli izleme + BASLAT/DURDUR.

Masadaki tum kutulari algilar, HACIMCE en buyugu secer, FR3 ile kaldirip
bir sonraki slota koyar, tekrar tarar (kalan en buyuk), nesne bitene kadar.
"""
from __future__ import annotations

import threading
import time

import streamlit as st

from src import config

st.set_page_config(page_title="Otomatik Hucre", layout="wide")
st.title("🔁 Otomatik Cok-Nesneli Hucre")
st.caption("Masadaki kutulari algila → **en buyuk** → FR3 kaldir/yerlestir → tekrar → "
           "kucuge dogru. Kameralar robot masasina bakiyor.")

ss = st.session_state
ss.setdefault("auto_cell", None)
ss.setdefault("auto_thread", None)
ss.setdefault("auto_stopper", None)     # threading.Event

with st.sidebar:
    st.header("Ayarlar")
    dry = st.checkbox("Kuru calisma (robota gonderme)", value=True,
                      help="Kapali: FR3'e GERCEK hareket. Once kuru dene.")
    st.write(f"**ROBOT_IP** = {config.ROBOT_IP}  ·  ENABLED={config.ROBOT_ENABLED}")
    st.write(f"**Slot sayisi** = {len(config.PLACE_SLOTS)}")
    with st.expander("Yerlestirme slotlari"):
        for i, s in enumerate(config.PLACE_SLOTS):
            st.text(f"{i+1}: {s}")
    st.caption("Slotlari .env PLACE_SLOTS ile degistir: \"x,y,z,rx,ry,rz; ...\"")

    from src.calib import calibration_status
    cs = calibration_status()
    st.write(f"{'✅' if cs['stereo']['var'] else '❌'} stereo_calib   ·   "
             f"{'✅ hand-eye var' if cs['base_T_cam']['var'] else '⚠️ hand-eye YOK'}")
    if not cs["base_T_cam"]["var"]:
        st.warning("base_T_cam yok → kol oynar ama nesnenin tam yerine gitmez. "
                   "Robot baglaninca `tools/handeye_solve.py`.")

col1, col2 = st.columns(2)
if col1.button("▶️ BASLAT", type="primary", use_container_width=True,
               disabled=bool(ss.auto_thread and ss.auto_thread.is_alive())):
    from src.auto import AutoCell
    stopper = threading.Event()
    ss.auto_stopper = stopper
    cell = AutoCell(dry_run=dry)
    ss.auto_cell = cell
    t = threading.Thread(target=cell.run,
                         kwargs=dict(stop=stopper.is_set, max_cycles=50),
                         daemon=True)
    t.start()
    ss.auto_thread = t
    time.sleep(0.5)
    st.rerun()

if col2.button("⏹️ DURDUR", use_container_width=True):
    if ss.auto_stopper:
        ss.auto_stopper.set()
    st.rerun()

cell = ss.auto_cell
if cell is None:
    st.info("BASLAT'a bas. Once **Kuru calisma** ile dene, sonra kapatip gercek robot.")
    st.stop()

s = cell.state
alive = bool(ss.auto_thread and ss.auto_thread.is_alive())
c1, c2, c3 = st.columns(3)
c1.metric("Durum", "ÇALIŞIYOR" if alive else ("BİTTİ" if s.done else "durdu"))
c2.metric("Yerlestirilen", f"{s.placed} / {len(config.PLACE_SLOTS)}")
c3.metric("Son taramada nesne", len(s.objects))
if s.error:
    st.error(f"Hata: {s.error}")

# --- canli goruntu: solda ham akis, sagda tespit cerceveleri ---
vc1, vc2 = st.columns(2)
with vc1:
    st.caption("Canlı kamera (sol) — ham")
    st.markdown(
        f'<img src="http://localhost:8600/stream/L" style="width:100%;border:1px solid #444">',
        unsafe_allow_html=True)
with vc2:
    st.caption("Son tarama — tespit çerçeveleri (yeşil = sıradaki)")
    if s.overlay_png:
        st.image(s.overlay_png, use_container_width=True)
    else:
        st.info("Henüz tarama yok. BAŞLAT.")

if s.objects:
    st.subheader("Son taramadaki nesneler (buyukten kucuge)")
    st.dataframe(s.objects, use_container_width=True, hide_index=True)

if s.plan:
    st.subheader(f"KURU PLAN — {len(s.plan)} nesne, buyukten kucuge yerlestirme sirasi")
    st.caption("Kuru modda tek tarama yapilir ve masadaki TUM nesneler icin sira "
               "cikarilir. Gercek modda robot her nesneyi kaldirinca sahne degisir, "
               "dongu kalan en buyugu secer.")
    st.table([{"sira": i + 1, "boy": p["nesne"]["boy_cm"], "en": p["nesne"]["en_cm"],
               "derinlik": p["nesne"]["derinlik_cm"], "hacim_L": p["nesne"]["hacim_L"],
               "slot": p["slot"][:3], "adim": len(p["program"]["adimlar"])}
              for i, p in enumerate(s.plan)])

st.subheader("Olay gunlugu")
st.code("\n".join(s.events[-40:]) or "(hebuz yok)", language="text")

if alive:
    time.sleep(2.0)
    st.rerun()

"""
HIZLI STEREO arayuzu - 2 goruntu yukle, disparite + derinlik + 3B bulut'u
DOGRUDAN gor. Kalibrasyon YOK, valiz tespiti YOK.

Kameralari nisanlarken / hizalarken kullan. Metrik olcum ve robot icin
Hucre Modu + kalibrasyon gerekir.
"""
from __future__ import annotations

import json
import logging

import numpy as np
import streamlit as st

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
st.set_page_config(page_title="Hizli Stereo", layout="wide")


def _rgb(img):
    return img[:, :, ::-1] if img is not None and img.ndim == 3 else img


def _read(up):
    import cv2
    return cv2.imdecode(np.frombuffer(up.getvalue(), np.uint8), cv2.IMREAD_COLOR)


def _latest_snap_pair(folder="snaps"):
    import glob
    import os

    import cv2
    lefts = sorted(glob.glob(os.path.join(folder, "left_*.png")))
    if not lefts:
        raise RuntimeError(f"'{folder}/' bos. http://localhost:8600 → ÇİFT KAYDET.")
    lp = lefts[-1]
    rp = lp.replace("left_", "right_")
    L, R = cv2.imread(lp), cv2.imread(rp)
    if L is None or R is None:
        raise RuntimeError("Cift okunamadi.")
    return L, R, os.path.basename(lp)


def _png(img_bgr):
    import cv2
    ok, b = cv2.imencode(".png", img_bgr)
    return b.tobytes() if ok else b""


st.title("⚡ Hizli Stereo — disparite / derinlik / 3B bulut")
st.caption("2 goruntu yukle → poz kestir → rektifiye → SGBM+WLS → 3B. "
           "Kalibrasyon ve valiz tespiti YOK; kamera hizalama/onizleme icin. "
           "Metrik cm + robot icin: **Hucre Modu** + kalibrasyon.")

with st.sidebar:
    st.header("Girdi")
    src_mode = st.radio("Kaynak", ["Son ÇİFT KAYDET (snaps/)", "Dosya yukle"])
    up_l = up_r = None
    if src_mode == "Dosya yukle":
        up_l = st.file_uploader("Sol goruntu", type=["png", "jpg", "jpeg", "bmp"])
        up_r = st.file_uploader("Sag goruntu", type=["png", "jpg", "jpeg", "bmp"])
    else:
        st.caption("http://localhost:8600 → ÇİFT KAYDET → burada Çalıştır.")

    import os
    from src import config
    has_calib = os.path.exists(config.STEREO_CALIB_PATH)
    use_rig = st.checkbox("Kalibreli mod (stereo_calib.yaml)", value=has_calib,
                          disabled=not has_calib,
                          help="Kalibrasyon dosyasi varsa: METRIK mm, duz zemin, "
                               "odak/poz tahmini YOK. Yoksa gri.")
    if not has_calib:
        st.caption("stereo_calib.yaml yok -> kalibrasyonsuz mod. "
                   "`tools.stereo_calibrate` calistirin.")

    focal = st.number_input("Odak (piksel)  — sadece kalibrasyonsuz modda", 0.0, 8000.0,
                            1150.0, 10.0, disabled=use_rig,
                            help="Logitech C310 @1280x720 icin ~1150.")

    st.markdown("**Nesne kutusu** (kadrajda nerede)")
    st.caption("Sahnede baska sey varsa (robot, kutu, dagiklik) nesneyi kutuya al. "
               "Tum kadraj = 0–100.")
    bx = st.slider("Yatay (X) %", 0, 100, (15, 85))
    by = st.slider("Dikey (Y) %", 0, 100, (10, 90))
    run = st.button("⚡ Calistir", type="primary", use_container_width=True)

roi_box = (bx[0] / 100, bx[1] / 100, by[0] / 100, by[1] / 100)

if run:
    from src.quickstereo import run_quick_stereo

    try:
        if src_mode == "Dosya yukle":
            if not (up_l and up_r):
                st.error("Iki goruntu de gerekli."); st.stop()
            imgL, imgR = _read(up_l), _read(up_r)
        else:
            imgL, imgR, nm = _latest_snap_pair()
            st.caption(f"islenen cift: {nm}")
    except Exception as exc:  # noqa: BLE001
        st.error(str(exc)); st.stop()

    with st.spinner("rektifikasyon → disparite → 3B ..."):
        try:
            res = run_quick_stereo(imgL, imgR, focal_px=focal,
                                   use_rig=use_rig, roi_box=roi_box)
        except Exception as exc:  # noqa: BLE001
            st.exception(exc)
            st.stop()
    st.session_state["qs"] = res
    st.session_state["qs_box"] = roi_box

res = st.session_state.get("qs")
if res is None:
    st.info("Soldan iki goruntu yukleyip **Calistir**'a bas.")
    st.stop()

(st.success if res.ok else st.warning)(("✅ " if res.ok else "⚠️ ") + res.message)
if res.rect_left is None:
    st.stop()

tab_d, tab_z, tab_c, tab_r, tab_j = st.tabs(
    ["Disparite", "Derinlik", "3B nokta bulutu", "Rektifiye cift", "Sayilar"])

with tab_d:
    st.image(_rgb(res.disparity_color), use_container_width=True,
             caption=f"Disparite — gecerli %{res.stats.get('gecerli_disparite_%')}")
    st.download_button("Disparite PNG", _png(res.disparity_color), "disparity.png", "image/png")

with tab_z:
    st.image(_rgb(res.depth_color), use_container_width=True,
             caption="Derinlik (yakin = sicak, baseline birimi — olcek yok)")
    d = res.depth
    fin = np.isfinite(d)
    if fin.any():
        st.caption(f"goreli derinlik: {np.nanmin(d[fin]):.2f} .. {np.nanmax(d[fin]):.2f} "
                   f"(baseline birimi) | medyan {np.nanmedian(d[fin]):.2f}")
    st.download_button("Derinlik PNG", _png(res.depth_color), "depth.png", "image/png")

with tab_c:
    metric = str(res.stats.get("mod", "")).startswith("KALIBRELI")
    c1, c2, c3 = st.columns([1.3, 1.2, 1.2])
    only_obj = c1.checkbox("Sadece nesne", value=True,
                           help="Zemin duzlemini kaldir + gurultu temizle + "
                                "en buyuk kumeyi al -> geriye SADECE nesne kalir.")
    adv = c2.checkbox("Zemini kaldir (RANSAC)", value=metric,
                      help="Yerde duran duz nesne (kitap, kutu) icin ŞART. "
                           "Kalibreli modda otomatik acik. Kalibrasyonsuzda zemin "
                           "egri cikabilir, kapatmak gerekebilir.")
    nearest = c3.slider("En yakin % (ikincil)", 40, 100, 92, 2,
                        help="Zemin gittikten sonra kalanin en yakin %N'i. "
                             "100 = kapali. Nesnenin arka sacilmalarini kirpar.")

    pc = res.pointcloud(isolate=only_obj, nearest_pct=float(nearest),
                        drop_ground=adv, denoise=True)
    info = getattr(res, "last_isolate_info", {}) or {}
    if pc is None or len(pc) == 0:
        st.warning("Bulut bos. 'Sadece nesne'yi kapat ya da 'En yakin %'yi dusur.")
    else:
        # --- OLCULER (sadece kalibreli modda anlamli) ---
        if metric and len(pc) >= 80:
            try:
                from src.grasp import fit_obb
                o = fit_obb(np.asarray(pc.xyz, float), up_axis="z")
                ex = sorted([o.width_mm, o.thickness_mm, o.height_mm], reverse=True)
                st.markdown("**📏 Nesne olculeri** (kalibreli, mm→cm)")
                m1, m2, m3 = st.columns(3)
                m1.metric("En (uzun)", f"{ex[0]/10:.1f} cm")
                m2.metric("Boy (orta)", f"{ex[1]/10:.1f} cm")
                m3.metric("Derinlik (kisa)", f"{ex[2]/10:.1f} cm")
                st.caption(f"hacim ~{np.prod(ex)/1e6:.1f} L · kamera cercevesinde "
                           "(hand-eye yok → eksen yönü yaklaşık). Kesin en/boy/derinlik "
                           "için **Hucre Modu** + hand-eye.")
            except Exception as exc:  # noqa: BLE001
                st.caption(f"OBB hesaplanamadi: {exc}")
        elif not metric:
            st.info("Ölçü (cm) için sol menüden **Kalibreli mod**'u aç.")

        unit = "mm" if metric else "baseline birimi"
        tail = f"  |  ayiklama: {info}" if info else ""
        st.caption(f"{len(pc)} nokta ({unit}){tail}")
        try:
            from src.pointcloud import plotly_figure
            st.plotly_chart(plotly_figure(pc, title="3B nokta bulutu", unit=unit),
                            use_container_width=True)
        except Exception as exc:  # noqa: BLE001
            st.warning(f"Plotly cizilemedi: {exc}")
        st.download_button("Nokta bulutu (.ply)", pc.to_ply_bytes(),
                           "nesne_bulutu.ply", "application/octet-stream")

with tab_r:
    import cv2 as _cv
    box = st.session_state.get("qs_box", roi_box)
    ov = res.rect_left.copy()
    hh, ww = ov.shape[:2]
    p1 = (int(ww * box[0]), int(hh * box[2]))
    p2 = (int(ww * box[1]), int(hh * box[3]))
    _cv.rectangle(ov, p1, p2, (0, 255, 0), 3)
    st.image(_rgb(ov), caption="Rektifiye sol + NESNE KUTUSU (yesil). Kutu nesneyi "
             "sarmıyorsa soldan X/Y %'yi ayarla ve tekrar Calistir.",
             use_container_width=True)
    c1, c2 = st.columns(2)
    c1.image(_rgb(res.rect_left), caption="Rektifiye sol", use_container_width=True)
    c2.image(_rgb(res.rect_right), caption="Rektifiye sag", use_container_width=True)
    st.caption("Iyi rektifikasyonda ayni nokta iki karede AYNI yatay satirda olur "
               f"(dikey hata {res.stats.get('rektifiye_dikey_hata_px')} px, ideal <1).")

with tab_j:
    st.json(res.stats)
    st.download_button("Sayilar JSON", json.dumps(res.stats, ensure_ascii=False, indent=2),
                       "hizli_stereo.json", "application/json")

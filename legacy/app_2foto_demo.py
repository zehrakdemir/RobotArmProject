"""
ESKI 2-FOTOGRAF DEMOSU - fiziksel 2-kamera hucresi kurulmadan onceki ilk demo.
Nav'dan kaldirildi (2026-09-11): kalibrasyon kullanmiyor, mutlak olcek icin
elle cm girmen gerekiyor. Gercek senaryo artik Otomatik Hucre / Hucre Modu.
Kod calisir durumda kalsin diye burada duruyor - istersen app.py'deki
st.navigation listesine tekrar ekleyip geri getirebilirsin:

    st.Page("legacy/app_2foto_demo.py", title="2 Foto Demo", icon="🧳")

Calistirma (tek basina, nav disinda):
    streamlit run legacy/app_2foto_demo.py
"""
from __future__ import annotations

import json
import logging

import numpy as np
import streamlit as st

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
st.set_page_config(page_title="Valiz Stereo Olcum + FR3", layout="wide", page_icon="🧳")

from src.ui import inject_theme, page_header  # noqa: E402

inject_theme()


# ---------------------------------------------------------------- yardimcilar
def _read_image(uploaded) -> tuple[np.ndarray, str]:
    import cv2

    data = np.frombuffer(uploaded.getvalue(), np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"Goruntu okunamadi: {uploaded.name}")
    return img, uploaded.name


def _rgb(img: np.ndarray) -> np.ndarray:
    return img[:, :, ::-1] if img.ndim == 3 else img


def _png_bytes(img_bgr: np.ndarray) -> bytes:
    import cv2

    ok, buf = cv2.imencode(".png", img_bgr)
    return buf.tobytes() if ok else b""


# ---------------------------------------------------------------- baslik
page_header(
    "🧳", "Valiz Stereo Ölçüm + Fairino FR3 (2 Foto Demo)",
    "İki açıdan valiz fotoğrafı → YOLO26-seg tespiti → bağıl poz → SGBM+WLS derinlik "
    "haritası → 3B nokta bulutu → medyan tabanlı en/boy/derinlik + yamukluk → "
    "duruş sınıfı → FR3 düzeltme komutu.",
)

with st.sidebar:
    st.header("Girdiler")
    up_left = st.file_uploader("Sol / 1. fotograf", type=["jpg", "jpeg", "png", "bmp"])
    up_right = st.file_uploader("Sag / 2. fotograf", type=["jpg", "jpeg", "png", "bmp"])

    st.subheader("Olcek kalibrasyonu")
    known_dim = st.selectbox(
        "Bilinen gercek olcu ekseni", ["width", "height", "depth"],
        format_func={"width": "En (genislik)", "height": "Boy (yukseklik)",
                     "depth": "Derinlik"}.get,
    )
    known_val = st.number_input("Bu eksenin GERCEK degeri (cm)", 5.0, 200.0, 40.0, 0.5)

    st.subheader("Kamera (opsiyonel)")
    focal_35 = st.number_input("35mm-ekiv odak (mm) - 0=EXIF/otomatik", 0.0, 300.0, 0.0, 1.0)
    focal_px_in = st.number_input("Dogrudan piksel odak - 0=kapali", 0.0, 20000.0, 0.0, 10.0)
    swap = st.checkbox("Sol/sag goruntuleri degistir", value=False)

    st.subheader("Robot")
    robot_enabled = st.checkbox("FR3'e GERCEKTEN gonder", value=False,
                                help="Kapali iken komut sadece plan olarak gosterilir (dry-run).")
    robot_ip = st.text_input("FR3 IP", "192.168.58.2")

    run_btn = st.button("🚀 Calistir", type="primary", use_container_width=True)


# ---------------------------------------------------------------- calistir
if run_btn:
    if not up_left or not up_right:
        st.error("Iki fotograf da gerekli.")
        st.stop()

    from src import config
    from src.pipeline import run_pipeline

    try:
        img_l, name_l = _read_image(up_left)
        img_r, name_r = _read_image(up_right)
    except Exception as exc:  # noqa: BLE001
        st.error(f"Goruntu hatasi: {exc}")
        st.stop()

    run_cfg = config.RunConfig(
        known_width_cm=known_val, known_dimension=known_dim,
        focal_length_35mm=focal_35, manual_focal_px=focal_px_in,
        swap_images=swap, robot_enabled=robot_enabled, robot_ip=robot_ip,
    )

    with st.spinner("Isleniyor: tespit -> poz -> disparite -> 3B -> olcum ..."):
        try:
            result = run_pipeline(img_l, img_r, run_cfg, name_l, name_r)
        except Exception as exc:  # noqa: BLE001
            st.exception(exc)
            st.stop()

    st.session_state.update(
        result=result, run_cfg=run_cfg,
        img_l=img_l if not swap else img_r,
        img_r=img_r if not swap else img_l,
    )


result = st.session_state.get("result")
if result is None:
    st.info("Soldan iki fotograf yukleyip **Calistir**'a basin.")
    st.stop()

img_l = st.session_state["img_l"]
img_r = st.session_state["img_r"]
run_cfg = st.session_state["run_cfg"]


# ---------------------------------------------------------------- sonuc
if not result.is_suitcase:
    st.error(f"❌ {result.message}")
    st.stop()

# --- ölçüm üretilemedi: teşhis ekranı ---
if not getattr(result, "ok", True) or result.measurement is None:
    st.error(f"❌ {result.message}")
    for w in result.warnings:
        st.warning("⚠️ " + w)

    st.subheader("Teşhis")
    if result.diagnostics:
        st.json(result.diagnostics)

    from src.visualize import overlay_detection

    c1, c2 = st.columns(2)
    if result.detection_left:
        dl = result.detection_left
        c1.image(_rgb(overlay_detection(img_l, dl.mask, dl.score, dl.weights_used)),
                 caption="Sol tespit", use_container_width=True)
    if result.detection_right:
        dr = result.detection_right
        c2.image(_rgb(overlay_detection(img_r, dr.mask, dr.score, dr.weights_used)),
                 caption="Sag tespit", use_container_width=True)
    if result.rectified is not None:
        c1.image(_rgb(result.rectified.left), caption="Rektifiye sol", use_container_width=True)
        c2.image(_rgb(result.rectified.right), caption="Rektifiye sag", use_container_width=True)
    if result.disparity_color is not None:
        st.image(_rgb(result.disparity_color),
                 caption=f"Disparite ({result.diagnostics.get('disparite_parametre','')}) "
                         f"— ROI geçerli %{result.diagnostics.get('disparite_gecerli_ROI_%','?')}",
                 use_container_width=True)

    st.markdown(
        "**Ne yapmalı?**\n"
        "- İki çekim arasındaki **açı farkını azalt** (~10–15°). Bu iki foto arası fark çok büyük.\n"
        "- İki çekim arasında **valizi kesinlikle oynatma** — sadece kamera hareket etsin.\n"
        "- Kadrajda valizin etrafında **daha çok halı/zemin dokusu** kalsın.\n"
        "- Yine olmazsa `.env` içine ekleyip container'ı yeniden başlat:\n"
        "  ```\n  WLS_CONFIDENCE_THRESHOLD=45\n  SGBM_NUM_DISPARITIES=320\n  SGBM_UNIQUENESS_RATIO=5\n  ```"
    )
    st.stop()

st.success(f"✅ {result.message}")
for w in result.warnings:
    st.warning("⚠️ " + w)

tabs = st.tabs([
    "1) Tespit", "2) Olcum", "3) Derinlik haritasi",
    "4) 3B nokta bulutu", "5) Durus", "6) FR3 komut", "7) JSON",
])

# --- 1) Tespit ---
with tabs[0]:
    from src.visualize import overlay_detection

    c1, c2 = st.columns(2)
    dl, dr = result.detection_left, result.detection_right
    c1.image(_rgb(overlay_detection(img_l, dl.mask, dl.score, dl.weights_used)),
             caption=f"Sol - suitcase {dl.score:.2f}", use_container_width=True)
    c2.image(_rgb(overlay_detection(img_r, dr.mask, dr.score, dr.weights_used)),
             caption=f"Sag - suitcase {dr.score:.2f}", use_container_width=True)
    st.caption(
        f"Agirlik: {dl.weights_used} | intrinsic: {result.intrinsics.source} "
        f"(focal_px={result.intrinsics.focal_px:.0f}) | "
        f"bakis acisi farki: {result.pose.baseline_angle_deg:.1f}° | "
        f"eslesme/inlier: {result.pose.num_matches}/{result.pose.num_inliers} "
        f"({result.pose.method})"
    )

# --- 2) Olcum ---
with tabs[1]:
    from src.visualize import overlay_measurement

    m = result.measurement
    c1, c2, c3 = st.columns(3)
    c1.metric("En (genislik)", f"{m.width_cm:.1f} cm")
    c2.metric("Boy (yukseklik)", f"{m.height_cm:.1f} cm")
    c3.metric("Derinlik (uzunluk)", f"{m.depth_cm:.1f} cm")
    c1.metric("Yamukluk (2B)", f"{m.rect.tilt_deg:.1f}°",
              delta="YAMUK" if m.rect.is_skewed else "duz",
              delta_color="inverse" if m.rect.is_skewed else "normal")
    c2.metric("Egim (3B)", f"{m.lean_deg:.1f}°")
    c3.metric("Olcek", f"{m.scale:.4f} cm/birim")

    st.image(_rgb(overlay_measurement(result.rectified.left, m)),
             caption="Rektifiye sol goruntu: yonelimli kutu + 4 kose (x,y) + TEPE referansi",
             use_container_width=True)

    st.write("**4 kose piksel koordinatlari:**")
    st.dataframe({
        "kose": ["TL", "TR", "BR", "BL"],
        "x": [int(p[0]) for p in m.rect.corners_px],
        "y": [int(p[1]) for p in m.rect.corners_px],
    }, use_container_width=True)
    st.write(f"**Valiz tepesi (kavrama referansi) x,y:** {m.top_ref.pixel} "
             f"— {'3B koordinat var' if m.top_ref.valid else '3B koordinat yok'}")
    if m.method_notes:
        st.info("  \n".join(m.method_notes))
    from src import config as _cfg
    st.caption(
        f"EN medyani {result.raw_measurement.width_samples} satirdan, "
        f"BOY medyani {result.raw_measurement.height_samples} sutundan hesaplandi. "
        f"Derinlik: en yakin/uzak %{_cfg.DEPTH_EDGE_PERCENTILE:.0f} medyan farki."
    )

# --- 3) Derinlik haritasi ---
with tabs[2]:
    c1, c2 = st.columns(2)
    if result.disparity_color is not None:
        c1.image(_rgb(result.disparity_color),
                 caption=f"Disparite (gecerli %{result.disparity.valid_ratio*100:.1f})",
                 use_container_width=True)
    if result.depth_color is not None:
        c2.image(_rgb(result.depth_color),
                 caption="Derinlik haritasi (yakin = sicak, cm)", use_container_width=True)
    d = result.depth.depth
    finite = np.isfinite(d)
    if finite.any():
        st.caption(f"Derinlik araligi {np.nanmin(d[finite]):.1f} – {np.nanmax(d[finite]):.1f} cm "
                   f"| medyan {np.nanmedian(d[finite]):.1f} cm")
        st.download_button("Derinlik haritasi PNG indir", _png_bytes(result.depth_color),
                           "derinlik_haritasi.png", "image/png")

# --- 4) 3B nokta bulutu ---
with tabs[3]:
    pc = result.pointcloud
    if pc is None or len(pc) == 0:
        st.warning("Nokta bulutu bos (disparite kalitesi dusuk).")
    else:
        st.caption(f"{len(pc)} nokta (valiz maskesi icinde)")
        fig = result.pointcloud_figure()
        if fig is not None:
            st.plotly_chart(fig, use_container_width=True)
        st.download_button("Nokta bulutu (.ply) indir", pc.to_ply_bytes(),
                           "valiz_nokta_bulutu.ply", "application/octet-stream")

# --- 5) Durus ---
with tabs[4]:
    p = result.posture
    icon = {"standing_on_wheels": "🧳✅", "tipped_over": "🧳↩️", "uncertain": "🧳❓"}[p.label]
    st.subheader(f"{icon} {p.label_tr}")
    c1, c2 = st.columns(2)
    c1.metric("Robot mudahalesi", "GEREKLI" if p.needs_robot else "gerekmiyor")
    c2.metric("Guven", f"{p.confidence:.0%}")
    st.write("**Gerekce:**")
    for r in p.reasons:
        st.write("- " + r)

# --- 6) FR3 komut ---
with tabs[5]:
    cmd = result.robot_command
    st.subheader(f"action = `{cmd.action}` — {cmd.reason}")
    st.json(cmd.to_dict())
    for n in cmd.notes:
        st.write("• " + n)

    if cmd.action == "none":
        st.info("Hareket gerekmiyor - valiz zaten normal durusta.")
    elif run_cfg.robot_enabled:
        st.warning("`FR3'e GERCEKTEN gonder` acik. Buton FR3'e GERCEK komut yollar.")
        if st.button("⚠️ FR3'e gonder", type="primary"):
            from src.robot import FairinoFR3

            fr3 = FairinoFR3(ip=run_cfg.robot_ip, enabled=True)
            fr3.connect()
            st.json(fr3.execute(cmd))
    else:
        st.info("Dry-run: gercek gonderim icin kenar cubugundan "
                "`FR3'e GERCEKTEN gonder` isaretleyin.")
        if st.button("Kuru calisma adimlarini goster"):
            from src.robot import FairinoFR3

            st.json(FairinoFR3(enabled=False).execute(cmd))

# --- 7) JSON ---
with tabs[6]:
    summ = result.summary()
    st.json(summ)
    st.download_button("Ozet JSON indir",
                       json.dumps(summ, ensure_ascii=False, indent=2),
                       "valiz_ozet.json", "application/json")

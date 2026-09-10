"""
HUCRE MODU arayuzu - sabit 2 kamera (kalibreli) + FR3 + Jodell gripper.

app.py (2-fotograf demo) ayni kaliyor; bu Streamlit'in cok-sayfa
ozelligiyle otomatik eklenen ikinci sayfa.
"""
from __future__ import annotations

import json
import logging

import numpy as np
import streamlit as st

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
st.set_page_config(page_title="Hucre Modu - Valiz + FR3", layout="wide")


def _rgb(img):
    return img[:, :, ::-1] if img is not None and img.ndim == 3 else img


def _read_upload(up):
    import cv2
    data = np.frombuffer(up.getvalue(), np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def _latest_snap_pair(folder="snaps"):
    """cam_web'in 'CIFT KAYDET' ile yazdigi en son left_*/right_* ciftini oku."""
    import glob
    import os

    import cv2
    lefts = sorted(glob.glob(os.path.join(folder, "left_*.png")))
    if not lefts:
        raise RuntimeError(
            f"'{folder}/' icinde cift yok. Once http://localhost:8600 -> CIFT KAYDET.")
    lp = lefts[-1]
    rp = lp.replace("left_", "right_")
    if not os.path.exists(rp):
        raise RuntimeError(f"Sag es yok: {os.path.basename(rp)}")
    L, R = cv2.imread(lp), cv2.imread(rp)
    if L is None or R is None:
        raise RuntimeError("Cift okunamadi (bozuk PNG?).")
    return L, R, os.path.basename(lp)


# ---------------------------------------------------------------- baslik
st.title("🤖 Hucre Modu — Valiz olcum + FR3 kavrama")
st.caption("2 sabit kamera (kalibreli stereo) → VALIZ MI? → 3B bulut (robot taban) "
           "→ OBB + durus → DIK: kavrama plani / YATIK: dik konuma getirme → FR3 programi.")

from src import config
from src.calib import calibration_status

# ---------------------------------------------------------------- kenar cubugu
with st.sidebar:
    st.header("Kalibrasyon durumu")
    stt = calibration_status()
    s, b = stt["stereo"], stt["base_T_cam"]
    st.write(f"{'✅' if s['var'] else '❌'} **stereo_calib.yaml**"
             + (f" — RMS {s.get('rms_px', -1):.3f}px, {s.get('boyut')}" if s['var'] else ""))
    st.write(f"{'✅' if b['var'] else '❌'} **base_T_cam.yaml**"
             + (f" — artik {b.get('residual_mm', -1):.1f}mm" if b['var'] else ""))
    if not s["var"]:
        st.error("Stereo kalibrasyon yok → hucre modu calismaz.\n\n"
                 "`docker compose run --rm valiz-olcum python -m tools.stereo_calibrate ...`")
    elif not b["var"]:
        st.warning("base_T_cam yok → olcum/bulut calisir ama **FR3 pozlari gecersiz** "
                   "(birim matris). `tools/handeye_solve.py` calistirin.")

    st.header("Girdi")
    src_mode = st.radio("Goruntu kaynagi",
                        ["Son ÇİFT KAYDET (snaps/)", "Dosya yukle"])
    up_l = up_r = None
    if src_mode == "Dosya yukle":
        up_l = st.file_uploader("Sol kamera", type=["png", "jpg", "jpeg", "bmp"])
        up_r = st.file_uploader("Sag kamera", type=["png", "jpg", "jpeg", "bmp"])
    else:
        st.caption("http://localhost:8600 → pozu al → **ÇİFT KAYDET** → burada "
                   "**Çalıştır**. En son kaydedilen çift işlenir.")

    test_obj = st.checkbox("🧪 Diğer nesneler (test: sırt çantası / çanta)",
                           value=bool(config.ACCEPT_CLASS_NAMES),
                           help="AÇIK: YOLO 'backpack/handbag' de valiz gibi işlenir. "
                                "KAPALI: sadece 'suitcase'. Ölçüm/kavrama mantığı değişmez.")
    extra = {"backpack", "handbag", "suitcase"} if test_obj else set()

    # --- Boş masayı öğren (YOLO olmadan otomatik nesne ayırma) ---
    import os as _os
    _tref = _os.path.join(config.CALIBRATION_DIR, "table_ref.npz")
    _has_tref = _os.path.exists(_tref)
    st.caption(("✅ Boş masa referansı var — YOLO bulamazsa nesne **otomatik** ayrılır."
                if _has_tref else
                "⚠️ Boş masa referansı yok. Masayı BOŞALT, çift kaydet, aşağıdaki butona bas."))
    if st.button("📚 Boş masayı öğren (son çift)", use_container_width=True):
        from src.cell import learn_table
        try:
            imgL, imgR, nm = _latest_snap_pair()
            with st.spinner("Masa düzlemi çıkarılıyor..."):
                info = learn_table(imgL, imgR)
            st.success(f"Öğrenildi ({nm}): düzlem inlier %{info['duzlem_inlier_%']}, "
                       f"bölge %{info['bolge_%']}")
        except Exception as exc:  # noqa: BLE001
            st.error(f"Olmadı: {exc}")

    with st.expander("Elle ROI (nadir — otomatik çalışmazsa)"):
        roi_str = st.text_input("x,y,g,y (sol görüntü pikseli)", "",
                                help="Görüntü 1280×720. İçindeki masa düzlemi otomatik atılır.")
    roi_xywh = None
    if roi_str.strip():
        try:
            roi_xywh = tuple(int(float(v)) for v in roi_str.replace(" ", "").split(","))
            assert len(roi_xywh) == 4
        except Exception:  # noqa: BLE001
            st.error("ROI formatı: x,y,genişlik,yükseklik  (örn. 300,160,470,340)")
            roi_xywh = None

    st.header("FR3 (SimMachine)")
    st.caption(f"ROBOT_IP={config.ROBOT_IP} · ENABLED={config.ROBOT_ENABLED}")
    if st.button("🔌 FR3 baglan / durum", use_container_width=True):
        from src.robot import FairinoFR3
        with st.spinner("FR3'e baglaniliyor..."):
            st.session_state["fr3_status"] = FairinoFR3(enabled=True).status()
    fs = st.session_state.get("fr3_status")
    if fs:
        if fs.get("bagli"):
            st.success(f"✅ Bagli — {fs['ip']}")
            if fs.get("eklemler_derece"):
                st.write("Eklemler (°): " + ", ".join(f"{v:.1f}" for v in fs["eklemler_derece"]))
            if fs.get("tcp_mm_derece"):
                t = fs["tcp_mm_derece"]
                st.write(f"TCP: x{t[0]:.0f} y{t[1]:.0f} z{t[2]:.0f}  rx{t[3]:.0f} ry{t[4]:.0f} rz{t[5]:.0f}")
        else:
            st.error(f"❌ Baglanamadi — {fs['ip']}:20003  (SimMachine ayakta mi? ag `fairino-net` bagli mi?)")
        if fs.get("uyari"):
            st.caption("⚠️ " + fs["uyari"])

    st.header("Durus")
    st.caption("Normalde **otomatik**. Elle secenekler sadece stereo bulutu bozuk "
               "oldugunda (kalibrasyon eski / az isik) acil cikis - once yeniden "
               "kalibre et.")
    posture_mode = st.radio(
        "Durus karari",
        ["otomatik", "elle: DİK (kavra)", "elle: YATIK (dik konuma getir)"],
        index=0)
    p_override = {"otomatik": None,
                  "elle: DİK (kavra)": "standing_on_wheels",
                  "elle: YATIK (dik konuma getir)": "tipped_over"}[posture_mode]
    g_z = g_th = None
    if p_override == "standing_on_wheels":
        st.caption("Bos birak -> OBB'den olculur. Sadece OBB dikey ekseni bozuksa doldur.")
        c1, c2 = st.columns(2)
        _gz = c1.number_input("Kavrama Z (mm, 0=oto)", 0.0, 600.0, 220.0, 10.0,
                              help="0 -> OBB yuksekliginden. SimMachine'de tool asagi "
                                   "bakarken bu XY'de ~200mm altina IK cozulmuyor - "
                                   "hata alirsan artir.")
        _gt = c2.number_input("Kalinlik (mm, 0=oto)", 0.0, 480.0, 0.0, 10.0,
                              help="0 -> OBB kalinligindan. Gripper acilmasi = bu + bosluk.")
        g_z = _gz if _gz > 0 else None
        g_th = _gt if _gt > 0 else None

    st.header("Hedef / robot")
    _sim = str(config.ROBOT_IP).startswith("192.168.58.")
    _place_def = "-350,-250,300,180,0,0" if _sim else ",".join(str(v) for v in config.PLACE_POSE)
    place_str = st.text_input("PLACE_POSE  x,y,z,rx,ry,rz (mm,deg)", _place_def,
                              help="SimMachine icin FR3'un rahat erisebildigi bir poz. "
                                   "IK hatasi alirsan Z'yi 250-350 arasi dene.")
    robot_go = st.checkbox("FR3'e GERCEKTEN gonder", value=False)
    run = st.button("🚀 Calistir", type="primary", use_container_width=True,
                    disabled=not s["var"])


# ---------------------------------------------------------------- calistir
if run:
    from src.cell import run_cell

    try:
        if src_mode == "Dosya yukle":
            if not (up_l and up_r):
                st.error("Iki goruntu de gerekli."); st.stop()
            imgL, imgR = _read_upload(up_l), _read_upload(up_r)
        else:
            imgL, imgR, nm = _latest_snap_pair()
            st.caption(f"islenen cift: {nm}")
        place = [float(x) for x in place_str.split(",")]
    except Exception as exc:  # noqa: BLE001
        st.error(str(exc)); st.stop()

    with st.spinner("Tespit → rektifikasyon → disparite → 3B → OBB → plan ..."):
        try:
            res = run_cell(imgL, imgR, place_pose=place, accept_extra=extra,
                           posture_override=p_override,
                           grasp_z_mm=g_z, grasp_thickness_mm=g_th,
                           roi_xywh=roi_xywh)
        except FileNotFoundError as exc:
            st.error(f"Kalibrasyon dosyasi eksik: {exc}"); st.stop()
        except Exception as exc:  # noqa: BLE001
            st.exception(exc); st.stop()

    st.session_state["cell_res"] = res
    st.session_state["cell_imgs"] = (imgL, imgR)


res = st.session_state.get("cell_res")
if res is None:
    st.info("Soldan goruntu verip **Calistir**'a basin.")
    st.stop()

imgL, imgR = st.session_state["cell_imgs"]

if not res.is_suitcase:
    st.error(f"❌ {res.message}")
    st.stop()

(st.success if res.ok else st.warning)(("✅ " if res.ok else "⚠️ ") + res.message)
for w in res.warnings:
    st.warning("⚠️ " + w)

tabs = st.tabs(["Tespit", "Olcu + Durus", "Disparite + 3B bulut",
                "Kavrama / Tip-up plani", "FR3 program", "JSON"])

with tabs[0]:
    from src.visualize import overlay_detection
    c1, c2 = st.columns(2)
    if res.detection_left:
        d = res.detection_left
        c1.image(_rgb(overlay_detection(imgL, d.mask, d.score, d.weights_used)),
                 caption=f"Sol — suitcase {d.score:.2f}", use_container_width=True)
    if res.detection_right:
        d = res.detection_right
        c2.image(_rgb(overlay_detection(imgR, d.mask, d.score, d.weights_used)),
                 caption=f"Sag — suitcase {d.score:.2f}", use_container_width=True)
    st.json(res.diagnostics)

with tabs[1]:
    if res.obb is not None:
        o = res.obb
        dims = sorted([o.width_mm, o.height_mm, o.thickness_mm], reverse=True)
        st.subheader("📏 Valiz olculeri")
        c1, c2, c3 = st.columns(3)
        c1.metric("Boy  (en uzun)", f"{dims[0]/10:.1f} cm")
        c2.metric("En  (ortanca)", f"{dims[1]/10:.1f} cm")
        c3.metric("Derinlik  (en kisa)", f"{dims[2]/10:.1f} cm")
        st.caption(
            f"boy / en / derinlik = **{dims[0]/10:.1f} / {dims[1]/10:.1f} / "
            f"{dims[2]/10:.1f} cm**  ·  hacim ~{np.prod(dims)/1e6:.1f} L  ·  "
            f"yaw {o.yaw_deg:.0f}°  ·  cerceve: {res.diagnostics.get('cerceve','?')}"
        )
        c1.metric("Yaw (uzun eksen)", f"{o.yaw_deg:.1f}°")
        c2.metric("3B egim", f"{res.diagnostics.get('egim_derece','?')}°")
        c3.metric("Merkez (mm)", ", ".join(f"{v:.0f}" for v in o.center))
        st.caption("Kaynak: kalibreli stereo (stereo_calib.yaml) -> METRIK. "
                   "'derinlik' = kavrama ekseni (2 duz kabuk arasi).")
    else:
        st.warning("OBB uretilemedi (bulut yetersiz).")
    if res.posture:
        p = res.posture
        st.subheader({"standing_on_wheels": "🧳 DIK", "tipped_over": "🧳↩️ YATIK",
                      "uncertain": "🧳❓ BELIRSIZ"}[p.label] + f" — {p.label_tr}")
        st.write(f"Robot mudahalesi: **{'GEREKLI' if p.needs_robot else 'gerekmiyor'}** "
                 f"(guven {p.confidence:.0%})")
        for r in p.reasons:
            st.write("- " + r)

with tabs[2]:
    if res.disparity_color is not None:
        st.image(_rgb(res.disparity_color), caption="Disparite", use_container_width=True)
    pc = res.pointcloud_base
    if pc is not None and len(pc):
        frame = res.diagnostics.get("cerceve", "?")
        st.caption(f"{len(pc)} nokta — {frame} — birim: **mm**")
        try:
            from src.pointcloud import plotly_figure
            st.plotly_chart(plotly_figure(pc, title=f"3B bulut ({frame})", unit="mm"),
                            use_container_width=True)
        except Exception as exc:  # noqa: BLE001
            st.warning(f"Plotly gosterilemedi: {exc}")
        st.download_button("Bulut (.ply) indir", pc.to_ply_bytes(),
                           "valiz_bulut_taban.ply", "application/octet-stream")

with tabs[3]:
    if res.grasp is not None:
        gp = res.grasp
        (st.success if gp.feasible else st.error)(gp.reason)
        st.json(gp.as_dict())
    if res.tipup is not None:
        tu = res.tipup
        (st.success if tu.feasible else st.error)(tu.reason)
        st.json(tu.as_dict())
    if res.grasp is None and res.tipup is None:
        st.info("Plan uretilmedi (durus belirsiz).")

with tabs[4]:
    prog = res.program
    if prog is None:
        st.info("Program yok.")
    else:
        st.subheader(f"`{prog.action}` — uygulanabilir: {prog.feasible}")
        st.table([{"#": i + 1, "tip": k, "aciklama": n}
                  for i, (k, a, n) in enumerate(prog.steps)])
        st.code(prog.to_json(), language="json")
        if not prog.feasible:
            st.warning(prog.reason)
        elif robot_go:
            if not b["var"]:
                st.error("⚠️ **base_T_cam yok** → bu programdaki pozlar KAMERA cercevesinde, "
                         "robot taban cercevesinde DEGIL. SimMachine'e gonderirsen kol "
                         "yanlis yere gider (simulasyon oldugu icin zararsiz ama anlamsiz). "
                         "Anlamli hareket icin once `tools/base_frame_from_floor.py`.")
            st.warning("`FR3'e GERCEKTEN gonder` acik — buton SimMachine'e **gercek hareket** yollar "
                       "(hata temizle + Mode(0) + RobotEnable(1) + adimlar). "
                       "Gripper SimMachine'de YOK, o adimlar atlanir (GRIPPER_ENABLED=false).")
            if st.button("⚠️ FR3'e gonder", type="primary"):
                from src.robot import FairinoFR3
                fr3 = FairinoFR3(enabled=True)
                with st.spinner("FR3'e gonderiliyor... (kol hareket ederken bekleyin)"):
                    fr3.connect()
                    out = fr3.run_program(prog)
                st.session_state["fr3_run_out"] = out
            out = st.session_state.get("fr3_run_out")
            if out:
                moved = out.get("kol_oynadi")
                if out.get("dispatched") and moved:
                    st.success("✅ Program SimMachine'e gonderildi ve **kol hareket etti**. "
                               "3B goruntu guncellenmezse `http://127.0.0.1/` sekmesini YENILEYIN.")
                    if out.get("denetleyici_kurtarma"):
                        st.caption(f"⚠️ {out['denetleyici_kurtarma']} adimda tekillik oldu, "
                                   "otomatik kurtarildi. Poz kalitesi icin Kavrama Z / "
                                   "PLACE_POSE'u ayarla ya da tabani yeniden hizala.")
                elif out.get("dispatched"):
                    st.warning("Program gonderildi ama eklem acilari degismedi "
                               "(plan zaten baslangic pozunda olabilir ya da adim atlandi).")
                else:
                    err = str(out.get("error") or out.get("steps"))
                    st.error(f"Gonderilemedi: {err}")
                    if "ters kinematik" in err or "112" in err:
                        st.info("↳ Bu poz robotun erisemedigi yerde. Kenar cubugundan "
                                "**Kavrama Z**'yi artir (orn. 180 → 250) ve tekrar Calistir. "
                                "Kalici cozum: taban cercevesini FR3 tabanina hizala "
                                "(`tools/base_frame_from_floor.py --yaw-deg / --base-z-mm`).")
                    if moved:
                        st.caption("Not: hata oncesi bazi adimlar calisti - kol kismen hareket etti.")
                eb, ee = out.get("eklem_baslangic"), out.get("eklem_bitis")
                if eb and ee:
                    c1, c2 = st.columns(2)
                    c1.caption("eklem baslangic (°)"); c1.code(", ".join(f"{v:.1f}" for v in eb))
                    c2.caption("eklem bitis (°)");     c2.code(", ".join(f"{v:.1f}" for v in ee))
                with st.expander("run_program ciktisi (JSON)"):
                    st.json(out)
        else:
            st.info("Dry-run. Gercek gonderim icin kenar cubugundan **FR3'e GERCEKTEN gonder**'i isaretle.")
            if st.button("Kuru calisma (dry-run)"):
                from src.robot import FairinoFR3
                st.json(FairinoFR3(enabled=False).run_program(prog))

with tabs[5]:
    summ = res.summary()
    st.json(summ)
    st.download_button("Ozet JSON", json.dumps(summ, ensure_ascii=False, indent=2),
                       "hucre_ozet.json", "application/json")

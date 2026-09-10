"""
Proje genel ayarlari - STEREO (iki fotograf) surumu.

Onceki tek-kamera + monokuler derinlik surumunden farki: kamera
mesafesi / baseline SABIT DEGIL ve BILINMIYOR. Sistem, ayni valizin iki
farkli acidan cekilmis fotografi arasindaki poz farkini (rotasyon + yon,
ama BUYUKLUK DEGIL) kendisi kestirir; mutlak olcek, kullanicinin girdigi
tek bir gercek olcuyle (valizin gercek genisligi) kalibre edilir.

Bu dosya, kullanicinin "diger projemde calisiyordu" dedigi SGBM + WLS
tabanli yogun (dense) stereo mimarisinin parametrelerini korur.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field


def _env_str(key: str, default: str) -> str:
    return os.getenv(key, default)


def _env_float(key: str, default: float) -> float:
    try:
        return float(os.getenv(key, str(default)))
    except (TypeError, ValueError):
        return default


def _env_int(key: str, default: int) -> int:
    try:
        return int(os.getenv(key, str(default)))
    except (TypeError, ValueError):
        return default


def _env_bool(key: str, default: bool) -> bool:
    raw = os.getenv(key)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on", "evet"}


# ---------------------------------------------------------------------------
# YOLO26-seg tespit
# ---------------------------------------------------------------------------
# Kullanici "YOLO26seg kullanalim" dedi. Ultralytics API'si model adindan
# bagimsiz oldugu icin (yolov8 / yolo11 / yolo26 ayni arayuz), agirlik
# dosyasi env ile degistirilebilir. Elinizde YOLO26-seg agirligi yoksa
# gecici olarak "yolov8s-seg.pt" ile test edin - o da COCO'nun 'suitcase'
# (28) sinifini tanir.
SEG_WEIGHTS: str = _env_str("SEG_WEIGHTS", "yolo26s-seg.pt")
SEG_WEIGHTS_FALLBACK: str = _env_str("SEG_WEIGHTS_FALLBACK", "yolov8s-seg.pt")
COCO_SUITCASE_CLASS_ID: int = _env_int("COCO_SUITCASE_CLASS_ID", 28)
YOLO_CONF_THRESHOLD: float = _env_float("YOLO_CONF_THRESHOLD", 0.25)
YOLO_IOU_THRESHOLD: float = _env_float("YOLO_IOU_THRESHOLD", 0.45)
YOLO_DEVICE: str = _env_str("YOLO_DEVICE", "")  # "" -> otomatik, "cpu", "0"
# Test amacli ek kabul edilen sinif adlari (virgulle). Hazir stereo cift
# bulmak zor oldugu icin, orn. "backpack" ekleyip Middlebury "Backpack"
# rektifiye ciftiyle tum pipeline'i deneyebilirsiniz.
ACCEPT_CLASS_NAMES: set[str] = {
    s.strip().lower() for s in _env_str("ACCEPT_CLASS_NAMES", "").split(",") if s.strip()
}

# ---------------------------------------------------------------------------
# Geometri / olcum
# ---------------------------------------------------------------------------
# Yamukluk (tilt) toleransi - bu acinin uzerinde valiz "yamuk" sayilir.
TILT_THRESHOLD_DEG: float = _env_float("TILT_THRESHOLD_DEG", 5.0)

# Uzunluk (derinlik ekseni) hesabinda kullanilan persentil araligi: en
# yakin %P ve en uzak %P'nin medyani karsilastirilir.
DEPTH_EDGE_PERCENTILE: float = _env_float("DEPTH_EDGE_PERCENTILE", 10.0)

# En/boy olcumunde robust persentil araligi (medyan tabanli). Maske
# kenarindaki gurultuyu dislamak icin uc %2.5 kirpilir.
DIM_LOW_PERCENTILE: float = _env_float("DIM_LOW_PERCENTILE", 2.5)
DIM_HIGH_PERCENTILE: float = _env_float("DIM_HIGH_PERCENTILE", 97.5)

# Aykiri derinlik degerlerini kirpma esigi (medyandan kac std sapma).
DEPTH_OUTLIER_STD_THRESHOLD: float = _env_float("DEPTH_OUTLIER_STD_THRESHOLD", 2.0)

# ---------------------------------------------------------------------------
# Ozellik eslestirme (relative pose icin)
# ---------------------------------------------------------------------------
FEATURE_MATCH_RATIO: float = _env_float("FEATURE_MATCH_RATIO", 0.75)  # Lowe ratio
MIN_MATCH_COUNT: int = _env_int("MIN_MATCH_COUNT", 15)
FEATURE_MASK_DILATE_PX: int = _env_int("FEATURE_MASK_DILATE_PX", 25)
MAX_FEATURES: int = _env_int("MAX_FEATURES", 8000)
RANSAC_PROB: float = _env_float("RANSAC_PROB", 0.999)
RANSAC_THRESHOLD_PX: float = _env_float("RANSAC_THRESHOLD_PX", 1.0)

# ---------------------------------------------------------------------------
# Kamera intrinsic tahmini
# ---------------------------------------------------------------------------
# EXIF'te FocalLengthIn35mmFilm bulunamazsa:  focal_px = width_px * ratio
# (~50mm 35mm-ekivalani tipik bir telefon lensi icin kaba tahmin. Kesin
# olmasi SART DEGIL - nihai metrik olcek zaten bilinen genislikle ayrica
# kalibre ediliyor.)
DEFAULT_FOCAL_RATIO: float = _env_float("DEFAULT_FOCAL_RATIO", 1.4)
SENSOR_35MM_WIDTH: float = 36.0  # mm

# ---------------------------------------------------------------------------
# SGBM (Semi-Global Block Matching)
# ---------------------------------------------------------------------------
SGBM_MIN_DISPARITY: int = _env_int("SGBM_MIN_DISPARITY", 0)
SGBM_NUM_DISPARITIES: int = _env_int("SGBM_NUM_DISPARITIES", 16 * 12)  # 16'nin kati
SGBM_BLOCK_SIZE: int = _env_int("SGBM_BLOCK_SIZE", 5)
SGBM_UNIQUENESS_RATIO: int = _env_int("SGBM_UNIQUENESS_RATIO", 12)
SGBM_SPECKLE_WINDOW: int = _env_int("SGBM_SPECKLE_WINDOW", 100)
SGBM_SPECKLE_RANGE: int = _env_int("SGBM_SPECKLE_RANGE", 2)
SGBM_DISP12_MAX_DIFF: int = _env_int("SGBM_DISP12_MAX_DIFF", 2)
SGBM_PRE_FILTER_CAP: int = _env_int("SGBM_PRE_FILTER_CAP", 31)

# ---------------------------------------------------------------------------
# WLS (Weighted Least Squares) disparite post-filtre
# ---------------------------------------------------------------------------
WLS_LAMBDA: float = _env_float("WLS_LAMBDA", 8000.0)
WLS_SIGMA: float = _env_float("WLS_SIGMA", 1.5)
# Guven haritasi (0-255) esigi: altindaki pikseller "uydurma" sayilip
# NaN yapilir. Gurultu cok ise yukseltin (150), cok piksel eleniyorsa
# dusurun (90).
WLS_CONFIDENCE_THRESHOLD: float = _env_float("WLS_CONFIDENCE_THRESHOLD", 120.0)

# ---------------------------------------------------------------------------
# Nokta bulutu
# ---------------------------------------------------------------------------
POINTCLOUD_MAX_POINTS: int = _env_int("POINTCLOUD_MAX_POINTS", 60000)  # gorsellestirme icin ornekleme
POINTCLOUD_VOXEL_CM: float = _env_float("POINTCLOUD_VOXEL_CM", 0.0)    # >0 ise voxel downsample

# ---------------------------------------------------------------------------
# Duruş sinifi (dik / yatik)
# ---------------------------------------------------------------------------
# Valiz DIK (tekerlekleri uzerinde, normal durus) sayilmasi icin
# yukseklik/genislik orani bu esigin uzerinde olmali.
UPRIGHT_MIN_ASPECT: float = _env_float("UPRIGHT_MIN_ASPECT", 1.15)
# Valiz YATIK sayilmasi icin genislik/yukseklik orani bu esigin uzerinde.
TIPPED_MIN_ASPECT: float = _env_float("TIPPED_MIN_ASPECT", 1.15)
# Uzun eksen ile goruntu dikeyi arasindaki aci bu deger altinda ise
# "dikey" kabul edilir (derece).
UPRIGHT_MAX_AXIS_DEG: float = _env_float("UPRIGHT_MAX_AXIS_DEG", 25.0)

# ---------------------------------------------------------------------------
# Fairino FR3 robot
# ---------------------------------------------------------------------------
ROBOT_ENABLED: bool = _env_bool("ROBOT_ENABLED", False)  # False -> sadece log
ROBOT_IP: str = _env_str("ROBOT_IP", "192.168.58.2")
ROBOT_TCP_SPEED: float = _env_float("ROBOT_TCP_SPEED", 30.0)     # %
ROBOT_APPROACH_CM: float = _env_float("ROBOT_APPROACH_CM", 10.0)  # kavrama oncesi yaklasma
ROBOT_TOOL_NUM: int = _env_int("ROBOT_TOOL_NUM", 0)   # SDK MoveL/MoveJ zorunlu arg
ROBOT_USER_NUM: int = _env_int("ROBOT_USER_NUM", 0)
# SimMachine'de dogrusal MoveL siklikla tekillik/joint-config hatasi veriyor.
# false (varsayilan) -> tum adimlar PTP (IK + MoveJ). Gercek hucrede dogrusal
# yaklasma istiyorsan true yap.
ROBOT_LINEAR_MOVES: bool = _env_bool("ROBOT_LINEAR_MOVES", False)
# Her run_program basinda gidilecek guvenli HOME eklem pozu (derece, 6 eksen).
# GERCEK robotta kendi hucrende carpismasiz bir poz gir.
ROBOT_HOME_JOINTS: list[float] = [
    float(v) for v in _env_str("ROBOT_HOME_JOINTS", "0,-90,90,-90,-90,0").split(",")
]
# Bir ara nokta tekillige dusunce: True -> denetleyiciyi kurtar + adimi 1 kez tekrar
# dene (SimMachine icin pratik). GERCEK robotta ilk testlerde False yap -> tekillikte
# dur ve bildir, insan karar versin.
ROBOT_SINGULARITY_RETRY: bool = _env_bool("ROBOT_SINGULARITY_RETRY", True)
# YOLO nesneyi bulamazsa: sabit kamera + duz masa varsayimiyla masa duzlemi
# ustundeki en buyuk kumeyi nesne kabul et (valiz olmayan test nesneleri icin).
CELL_PLANE_SEG: bool = _env_bool("CELL_PLANE_SEG", True)
# El-goz (hand-eye) kalibrasyon dosyasi: kamera -> robot taban donusumu.
# Bkz. src/pipeline.py::image_to_robot_frame
HANDEYE_PATH: str = _env_str(
    "HANDEYE_PATH",
    os.path.join(os.path.dirname(__file__), "calibration", "handeye.json"),
)

CALIBRATION_DIR: str = os.path.join(os.path.dirname(__file__), "calibration")

# ---------------------------------------------------------------------------
# HUCRE MODU: sabit 2 kamera (kalibreli stereo) + FR3 + Jodell gripper
# ---------------------------------------------------------------------------
# tools/stereo_calibrate.py ciktisi (K1,d1,K2,d2,R,T,image_size).
STEREO_CALIB_PATH: str = _env_str(
    "STEREO_CALIB_PATH", os.path.join(CALIBRATION_DIR, "stereo_calib.yaml"))
# tools/handeye_solve.py ciktisi: 4x4  base_T_cam  (kamera -> robot taban).
BASE_T_CAM_PATH: str = _env_str(
    "BASE_T_CAM_PATH", os.path.join(CALIBRATION_DIR, "base_T_cam.yaml"))

# USB kameralar (Ubuntu /dev/videoN). Docker'da --device ile gecir.
CAM_LEFT_INDEX: int = _env_int("CAM_LEFT_INDEX", 0)
CAM_RIGHT_INDEX: int = _env_int("CAM_RIGHT_INDEX", 2)
CAM_WIDTH: int = _env_int("CAM_WIDTH", 1280)
CAM_HEIGHT: int = _env_int("CAM_HEIGHT", 720)

# Yer duzlemi: robot taban cercevesinde zemin Z'si (mm) ve "yukari" ekseni.
GROUND_Z_MM: float = _env_float("GROUND_Z_MM", 0.0)
# Bilinen boyutlu bir nesneyle otomatik olcek kalibrasyonu (tools/calib_scale.py
# yazar). Kamera geometrisi kalibrasyondan sonra kaydiysa disparite->derinlik
# olcegi kayar; bu carpan onu duzeltir. 1.0 = duzeltme yok.
MEASURE_SCALE: float = _env_float("MEASURE_SCALE", 1.0)
UP_AXIS: str = _env_str("UP_AXIS", "z")  # robot tabaninda yukari yon

# Cevrimici stereo oz-kalibrasyon (src/selfcalib.py). detect_objects, merkez
# ROI'de gecerli disparite orani bu esigin altina duserse (kameralar kaymis
# demektir) dokulu sahneden R,T'yi bir kez yeniden kestirir.
SELFCAL_MIN_VALID_FRAC: float = _env_float("SELFCAL_MIN_VALID_FRAC", 0.18)

# Ayak izleri bu mesafeden yakin 2B kumeler tek nesnede birlestirilir. Yumusak
# cisimlerde (sirt cantasi koyu deri cep/dikis stereo bosluklari) parcalanmayi
# onler; gorunur bosluklu bitisik kutular ayri kalir.
CLUSTER_MERGE_GAP_MM: float = _env_float("CLUSTER_MERGE_GAP_MM", 35.0)

# --- Jodell gripper (Fairino SDK: MoveGripper, poz 0..100) ---
# SimMachine simulatorunde gercek gripper YOKTUR; MoveGripper cagrisi hareket
# denetleyicisini hataya (err 8-1) sokuyor. Gercek gripper takilinca true yapin.
GRIPPER_ENABLED: bool = _env_bool("GRIPPER_ENABLED", False)
GRIPPER_INDEX: int = _env_int("GRIPPER_INDEX", 1)
GRIPPER_OPEN_VALUE: int = _env_int("GRIPPER_OPEN_VALUE", 100)   # tam acik komut degeri
GRIPPER_CLOSE_VALUE: int = _env_int("GRIPPER_CLOSE_VALUE", 0)   # tam kapali komut degeri
GRIPPER_MAX_OPENING_MM: float = _env_float("GRIPPER_MAX_OPENING_MM", 80.0)  # OLC ve GIR
GRIPPER_MIN_OPENING_MM: float = _env_float("GRIPPER_MIN_OPENING_MM", 0.0)
GRIPPER_SPEED: int = _env_int("GRIPPER_SPEED", 50)
GRIPPER_FORCE: int = _env_int("GRIPPER_FORCE", 50)
GRIPPER_FINGER_LEN_MM: float = _env_float("GRIPPER_FINGER_LEN_MM", 40.0)

# --- Kavrama ---
# Valizin 2 DUZ KABUGU (buyuk on/arka yuz) arasindan, KALINLIK ekseni
# boyunca kavranir. Kavrama noktasi bu iki yuzun orta duzlemi, tabandan
# bu oranda yukarida (0.5 = orta).
GRASP_HEIGHT_FRACTION: float = _env_float("GRASP_HEIGHT_FRACTION", 0.5)
GRASP_APPROACH_MM: float = _env_float("GRASP_APPROACH_MM", 120.0)   # on-yaklasma mesafesi
GRASP_LIFT_MM: float = _env_float("GRASP_LIFT_MM", 150.0)
GRASP_CLEARANCE_MM: float = _env_float("GRASP_CLEARANCE_MM", 8.0)   # yuz basina parmak boslugu
# "Duz yuz" testi: kutu yuzeyine uzakliklarin std'si bu esigin altindaysa
# o yuz cifti "duz/pruzsuz" (kulp/fermuar cikintisi yok) sayilir (mm).
FLAT_FACE_STD_MM: float = _env_float("FLAT_FACE_STD_MM", 6.0)

# --- Hedef yerlestirme pozu (robot taban, mm & derece; FR3 [x,y,z,rx,ry,rz]) ---
PLACE_POSE: list[float] = [
    float(v) for v in _env_str("PLACE_POSE", "500,0,150,180,0,0").split(",")
]
# Istenen final yon: valizin uzun ekseni robot X'ine gore bu aci (derece).
TARGET_YAW_DEG: float = _env_float("TARGET_YAW_DEG", 0.0)


def _parse_slots(s: str) -> list[list[float]]:
    out = []
    for part in s.split(";"):
        part = part.strip()
        if not part:
            continue
        v = [float(x) for x in part.split(",")]
        if len(v) == 6:
            out.append(v)
    return out


# --- COK NESNELI otomatik hucre: sirayla yerlestirme slotlari ---
# "x,y,z,rx,ry,rz; x,y,z,rx,ry,rz; ..."  Bos -> PLACE_POSE'tan bir sira uret.
PLACE_SLOTS: list[list[float]] = _parse_slots(_env_str("PLACE_SLOTS", "")) or [
    [PLACE_POSE[0], PLACE_POSE[1] + dy, PLACE_POSE[2], PLACE_POSE[3],
     PLACE_POSE[4], PLACE_POSE[5]]
    for dy in (-180.0, -60.0, 60.0, 180.0, 300.0, 420.0)
]
# Otomatik dongude her tur arasi bekleme ve azami nesne
AUTO_LOOP_PAUSE_S: float = _env_float("AUTO_LOOP_PAUSE_S", 2.0)
AUTO_MAX_OBJECTS: int = _env_int("AUTO_MAX_OBJECTS", 8)
# Sabit sahneden kac kare birlestirilsin (1 = kapali). Dokulu yuzeyde 3 dene.
AUTO_MERGE_FRAMES: int = _env_int("AUTO_MERGE_FRAMES", 1)
# Konteynerden kameralara erisim: host'ta calisan cam_web (tools/cam_web.py)
CAM_WEB_URL: str = _env_str("CAM_WEB_URL", "http://host.docker.internal:8600")
SNAP_DIR: str = _env_str("SNAP_DIR", "snaps")

# --- Yan yatik -> dik konuma getirme (align.py) ---
TIPUP_PUSH_HEIGHT_FRAC: float = _env_float("TIPUP_PUSH_HEIGHT_FRAC", 0.8)  # ust kenara yakin it
TIPUP_PUSH_DEPTH_MM: float = _env_float("TIPUP_PUSH_DEPTH_MM", 120.0)


@dataclass
class RunConfig:
    """Tek bir calistirmanin parametreleri (arayuzden gelir)."""

    known_width_cm: float = 40.0        # valizin GERCEK genisligi (kullanici girer)
    known_dimension: str = "width"      # width | height | depth  (hangi olcu biliniyor)
    focal_length_35mm: float = 0.0      # >0 ise EXIF yerine bu kullanilir
    manual_focal_px: float = 0.0        # >0 ise dogrudan piksel odak
    swap_images: bool = False           # sol/sag ters ise
    robot_enabled: bool = field(default_factory=lambda: ROBOT_ENABLED)
    robot_ip: str = field(default_factory=lambda: ROBOT_IP)

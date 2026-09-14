#!/usr/bin/env python3
"""movecobot - Fairino kolu + Jodell gripper icin birlesik interaktif kontrol
ve hareket ezberleme/tekrar oynatma araci.

movepy (moveit_py_interactive.py), movegripper (gripper_interactive.py) ve
fairino_controller.py / gripper_controller.py'deki dogrulanmis mantigi (robot
otomatik algilama, TCP offset telafisi, zemin collision kontrolu, gripper
retry/reset mekanizmasi) tek bir oturumda birlestirir; ustune de "ogret ve
tekrar oynat" (teach & playback) katmani ekler: robotu/gripper'i istedigin
poza getirip 'adim olarak ekle' diyerek bir hareket dizisi olusturabilir, bu
diziye bir isim verip kaydedebilirsin. Bu isim otomatik olarak ~/.bashrc'ye
bir alias olarak yazilir; daha sonra terminale sadece o ismi yazarak
(ör. 'pick1') ayni hareketi tek komutla, etkilesime girmeden tekrar
oynatabilirsin.

ON KOSUL: Once fairino_launch calistirilmis olmali (ROS2 move_group + rviz
ayakta olmali) - movecobot da tipki movepy gibi calisan bir MoveIt sunucusuna
baglanir, kendisi baslatmaz.

Kullanim:
    movecobot                  -> interaktif menu (jog + kayit + oynatma)
    movecobot --replay <isim>  -> kayitli bir hareketi dogrudan, etkilesimsiz
                                   oynatir (kayitli hareketler icin otomatik
                                   olusturulan aliaslar bunu cagirir)
    movecobot --list           -> kayitli hareketleri listeler
    movecobot --clear-fault    -> WebApp'te "Clear" hicbir sey yapmiyormus
                                   gibi gorunuyorsa (fairino_launch calisirken
                                   ros2_control'un 100Hz komut dongusu hatayi
                                   aninda geri getiriyor olabilir) - bu, o
                                   donguyu GECICI durdurup Clear'a basmani
                                   bekler, sonra geri acar. fairino_launch'i
                                   durdurmaz/yeniden baslatmaz.
"""
import json
import math
import re
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from fairino_cobot import paths
from fairino_cobot.ik_secimi import ik_adaylarini_sirala
from fairino_cobot.safety import asiri_eklem_hareketi

# SDK'nin yeri makineye gore degisiyor - tek yerden cozuluyor (paths.py).
paths.ensure_sdk_on_path()

from fairino import Robot

from fairino_cobot.gripper_controller import GripperController, GripperError

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
TIP_LINK = "wrist3_link"
ALL_LINKS = ["shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"]
FLOOR_Z_M = 0.0

# Jodell EPG40-050 gripper'in collision kutusu icin boyutlar. Bu gripper
# fairino_description'daki URDF'de HIC modellenmemis (wrist3_link son link,
# bkz. fairino_description/urdf/fairino{N}_v6.urdf) - yani MoveIt'in kendi
# self-collision kontrolu bu geometriler eklenmeden gripper'in var oldugundan
# habersizdir. Taban 40x24mm DEGERI KULLANICININ KENDI OLCTUGU deger (ilk
# yorumda yanlislikla "urun sayfasindan govde-only" denmisti, DUZELTILDI) -
# ceneler ACIKKEN alinan olcuye zaten bir pay eklenerek belirlenmisti (ham
# acik-cene olcusu DEGIL, ustune pay eklenmis hali). Kilitlenme riski
# _add_gripper_collision'daki baslangic-durumu
# kontrolu ile karsilaniyor (colliding ise kullaniciya sorup gerekirse
# korumayi o oturum icin kapatiyor). Uzunluk icin ise gercek olcum olan
# tcp_offset_m kullanilir (sabit bir deger yerine - boylece kalibrasyon
# neyi olcerse kutu da onu kapsar).
# ÇARPMA YAŞANDI: planli (jog/replay) bir hareket sirasinda gripper kola
# gercekten carpti - yani koruma AKTIFTI ama kutu gercek gripperi yeterince
# kapsamiyordu. Kullanici talebiyle SADECE GRIPPER icin (MIN_CLEARANCE_M'e
# DOKUNULMADI) HER YONDE +1cm pay eklendi: XY (genislik/derinlik) her
# yandan +10mm (toplam +20mm), uzunluk ucundan +10mm (GRIPPER_LENGTH_
# MARGIN_M ile).
GRIPPER_WIDTH_M = 0.095
GRIPPER_DEPTH_M = 0.06
# Flanstan itibaren korunan TOPLAM mesafe 120mm + kullanicinin istedigi
# +1cm ekstra pay (bkz. yukaridaki not - canli carpma sonrasi eklendi).
GRIPPER_LENGTH_MARGIN_M = 0.01
# Gripper uzunlugu icin guvenlik tabani. TCP offset (GetTCPOffset(), canli
# testte 100mm) collision boyutu icin GUVENILIR bir kaynak DEGIL - TCP bir
# kinematik/kontrol referans noktasi (cogu paralel gripper'da cenelerin
# SIMETRI MERKEZINE ayarlanir), gripper'in fiziksel govdesinin/parmaklarinin
# tam nereye kadar uzandigini garanti etmez (bkz. Viam frame-system ve
# EncyCAM TCP dokumantasyonu - "TCP mesafesinin aracin tam fiziksel uzunluguna
# esit oldugunu varsaymamalisin"). Resmi urun sayfasi rakami (40x24x87mm)
# govde-only, parmaklar dahil degil. Bu belirsizlik nedeniyle taban 120mm'ye
# cekildi (kullanici karari); canli olcumle bu tabandan BUYUK OLANI + asagidaki
# pay kullanilir - WebApp'teki tool offset ayari gercek gripperdan kisa
# girilse bile kutu bu tabanin altina dusmez.
GRIPPER_LENGTH_FLOOR_M = 0.120

# Self-collision kontrolu icin minimum guvenlik mesafesi. Canli testte
# "az kalsin carpiyordu" diye durdurulan bir olayda, is_state_colliding()
# (salt 'deger mi degmez mi') False donmustu ama olculen gercek mesafe
# sadece 9.54mm cikti - yani yazilim teknik olarak "carpismiyor" diyordu
# ama pratikte hicbir guvenlik payi yoktu, hareket devam etseydi carpardi.
# Bu yuzden artik salt boolean degil, MESAFE bazli kontrol kullaniliyor
# (bkz. check_trajectory_for_self_collision) - herhangi bir noktada iki
# govde arasi mesafe bu esigin altina duserse (gercekten degmese bile)
# hareket reddedilir.
#
# DEGER SECIMI: ilk denemede 30mm kullanildi ama bu, TAMAMEN GUVENLI, OMPL'in
# 15 farkli denemede tutarli sekilde ürettigi bir hareketi bile reddetti -
# olculen sabit ~16.7mm mesafe, gercek bir riskten degil, gripper kutusunun
# KENDI genis payindan (75x40mm - gercek gripper govdesi 40x24mm) kaynaklanan
# YAPISAL bir yakinlikti (kutu zaten kendi icinde fazladan pay tasidigi icin
# raporlanan mesafe, gercek fiziksel gripper icin olceceginden daha kucuk
# gorunuyor - payi iki kez saymamak gerekiyor). Esik, iki bilinen referans
# noktasi arasina (gercek tehlike=9.54mm / guvenli-ama-yapisal-yakin=16.7mm)
# cekildi.
MIN_CLEARANCE_M = 0.012

# ---------------------------------------------------------------------------
# GORECELI HAREKETTE ASIRI EKLEM SALINIMI SINIRI
# ---------------------------------------------------------------------------
# CANLI OLAY (bu sinirin var olma sebebi): arayuzden TCP'yi arac ekseninde
# SADECE 10mm oteleme komutu verildi. Uretilen plan j1'i 37.08 -> 69.94
# dereceye, yani 32.9 derece dondurecekti; kol genis bir yay cizerek
# savruldu ve kullanici acil stopa basmak zorunda kaldi.
#
# SEBEP: Pilz PTP ve LIN, joint_limits.yaml'da ivme limiti tanimli olmadigi
# icin ("acceleration limit not set for group") HIC planlayamiyor; her
# hareket sessizce OMPL/RRTConnect'e duesuyor. RRTConnect rastgele bir
# planlayici ve POZ hedefi verildiginde IK'nin BASKA BIR DALINI (kol
# ters-dirsek konumu) secebiliyor - TCP 10mm gitsin diye kol komple yer
# degistiriyor.
#
# NEDEN MEVCUT KONTROLLER YAKALAMADI: zemin ve self-collision kontrolleri
# o yayin kendisinde bir ihlal gormuyor - yay bos havada. Carpilacak sey
# MoveIt'in modelinde OLMAYAN gercek dunyadaki bir nesneydi. Yani o iki
# kontrol bu sinif hatayi yakalayamaz, yakalayacak sey ORANTI kontrolu:
# birkac milimetrelik bir oteleme icin kolun onlarca derece donmesi
# BASLI BASINA anormaldir, nereye gittiginden bagimsiz olarak.
#
# Bu esik asilirsa hareket ICRA EDILMEDEN reddedilir ve sebebi arayuzde
# gorunur. Mutlak hedeflerde buyuk eklem hareketi mesru olabilir.
#
# KALDIRILDI (kullanici karari, 14 Eylul): jog'lar artik bu siniri
# UYGULAMIYOR (bkz. _ik_adaylariyla_git). Mekanizma _plan_and_execute'ta
# max_joint_excursion_deg parametresiyle duruyor; su an hicbir cagri vermiyor.
MAX_JOG_JOINT_EXCURSION_DEG = 20.0

# ---------------------------------------------------------------------------
# LIN (DUZ CIZGI) DENEMESINDE KADEMELI YAVASLATMA
# ---------------------------------------------------------------------------
# OLCUM: eklem ivme limiti ayarlandiktan (0.7 rad/s^2) sonra LIN hala
# planlanamiyordu, ama artik FARKLI bir sebeple:
#   "Joint acceleration limit of j3 violated. Set the acceleration scaling
#    factor lower! Actual joint acceleration is -1.07017, while the limit is 0.7"
# Yani limit YOK degil; Pilz'in urettigi duz-cizgi profili onu asiyor.
#
# SEBEP: pilz_cartesian_limits.yaml'daki degerler (max_trans_vel 1.0,
# max_trans_acc 2.25, max_trans_dec -5.0) MoveIt'in GENEL ornek degerleri -
# Fairino'ya ozgu degil, Setup Assistant'in koydugu sablon. Kisa bir
# harekette profil tamamen hizlanma/yavaslama fazindan ibaret oluyor ve
# gereken eklem ivmesi limiti asiyor.
#
# NEDEN O YAML'I DEGISTIRMIYORUZ: oraya yazacagimiz her sayi uydurma olurdu
# ve TUM LIN hareketlerinin tavan hizini kalici olarak duesururdu. Pilz
# zaten istegin hiz/ivme olcegini kartezyen limitlere uyguluyor - yani
# DAHA YAVAS ISTEMEK ayni ise yariyor, ustelik sadece gerektiginde.
#
# Bu yuzden LIN once istenen hizda deneniyor; ivme ihlaliyle kalirsa
# kademeli olarak yavaslatilip tekrar deneniyor. Hicbiri tutmazsa PTP'ye
# duesuluyor (guvenli ve monoton, sadece TCP yolu hafif kavisli).
# Basarisiz bir plan denemesi ucuz - olcumde 4 ms.
LIN_YAVASLATMA_KADEMELERI = (1.0, 0.5, 0.25)

# MIN_CLEARANCE_M artik SADECE plandan SONRA kontrol edilen bir esik degil -
# MoveIt'in link padding mekanizmasiyla PLANLAMAYA da veriliyor (bkz.
# CobotArm.__init__). Her govde bu kadar "sismis" sayildigi icin iki govde
# arasindaki gercek bosluk 2*LINK_PADDING_M olur; hedeflenen 12mm'yi
# tutturmak icin padding bunun YARISI.
#
# DIKKAT: padding devredeyken carpisma kontrollerinin dondurdugu mesafe
# SISMIS govdeler arasindaki mesafedir. Yani "gercek bosluk" =
# olculen_mesafe + 2*LINK_PADDING_M ve "sismis govdeler degiyor" demek
# zaten "gercek bosluk < MIN_CLEARANCE_M" demektir - bu yuzden kontrollerde
# ayrica mesafe esigi UYGULANMAZ, yoksa pay iki kez sayilir.
LINK_PADDING_M = MIN_CLEARANCE_M / 2.0

# Gripper cenelerinin TAM ACIK aciklig (strok). Kavrama sirasinda okunan
# pozisyon yuzdesinden (0=tam acik, 100=tam kapali) nesnenin cene arasi
# GENISLIGINI turetmek icin kullanilir:
#     genislik = (100 - pozisyon) / 100 * GRIPPER_STROKE_M
# Kamera olmadigi icin nesnenin varligini ve genisligini BASINCTAN
# (gripper'in kuvvet geri bildirimiyle nerede durdugundan) cikariyoruz -
# ureticinin "grip feedback and object detection" olarak belirttigi ozellik.
#
# DEGER KAYNAGI (uretici/satici teknik verileri): Jodell EPG40-050 ->
# strok 40 mm, kuvvet 2-50 N/parmak, tasima 0.8 kg, agirlik 0.36 kg.
# Model adindaki "EPG40-050" = 40 mm strok + 50 N kuvvet demek.
GRIPPER_STROKE_M = 0.040

# Kavranan nesnenin planlama sahnesindeki kimligi (bkz. attach_grasped_object).
GRASPED_OBJECT_ID = "grasped_object"

MOVES_FILE = paths.moves_file()
# Hareket kaydi SIRASINDA (record_new_move/drag_teach_add_steps) eklenen her
# adimdan sonra buraya yazilir. Manuel modda robot bir fault verirse (ör.
# "hiz limiti asildi") kullanicinin launch'i durdurup WebApp'ten temizleyip
# yeniden baslatmasi gerekebiliyor - bu movecobot surecini de oldurup o ana
# kadar eklenen TUM adimlari kaybettiriyordu. Simdi her adim eklendiginde
# taslak buraya kaydediliyor, movecobot yeniden acildiginda "yarim kalan
# kaydi devam ettir" diye sorup kaldigi yerden devam ettiriyor.
DRAFT_FILE = paths.draft_file()
# konumcobot (ayri, hafif bir "durum goster" araci) ile movecobot AYNI ANDA
# calisirken robota IKINCI bir SDK baglantisi ACMAMASI icin - Fairino'nun
# gercek zamanli durum soketi (port 20004) tek-tuketicili, iki ayri surecten
# baglanmaya calismak ikisini de kilitleyebiliyor (canli testte yasandi).
# Bunun yerine movecobot, en son okudugu durumu buraya yazar; konumcobot da
# (movecobot calisiyorsa) robota hic baglanmadan bunu okur.
STATE_CACHE_FILE = paths.state_cache_file()
BASHRC = Path.home() / ".bashrc"
ALIAS_MARK_START = "# >>> movecobot kayitli hareketler (otomatik - elle duzenleme) >>>"
ALIAS_MARK_END = "# <<< movecobot kayitli hareketler <<<"
RESERVED_NAMES = {
    "movepy", "movegripper", "movecobot", "fairino_launch", "fairino_params",
    "gohome_py", "launch_moveit_py", "runpose", "launch_moveit", "run_sim",
    "stop_sim", "fixweb", "visiontest",
}
NAME_RE = re.compile(r"^[a-zA-Z][a-zA-Z0-9_]*$")
BACK_WORDS = ("g", "geri", "b", "back")  # ask_float/ask_int icin - "0" burada YOK, gecerli bir sayi olabilir

# MoveIt2 Setup Assistant'in urettigi ros2_control donanim bileseninin adi.
# Tum FRx modellerinde ayni ("FakeSystem" - degistirilmemis sablon adi, bkz.
# her modelin *_moveit2_config/config/*_robot.urdf.xacro dosyasi).
DRAG_TEACH_HW_COMPONENT = "FakeSystem"


class GoBack(Exception):
    """ask_*(allow_back=True) icinde kullanici 'geri' yazinca firlatilir;
    o an icinde bulunulan alt-akis (jog_arm, adim ekleme, kaydi bitirme...)
    bunu yakalayip robotu hic hareket ettirmeden bir onceki menuye doner."""
    pass


# ---------------------------------------------------------------------------
# Robot algilama (movepy / fairino_controller.py ile ayni mekanizma)
# ---------------------------------------------------------------------------

def detect_robot():
    try:
        script_path = str(paths.detect_robot_script())
    except paths.PathError as e:
        print(f"HATA: {e}")
        sys.exit(1)

    print("Robot algilaniyor (ag taramasi)...")
    try:
        result = subprocess.run(
            [sys.executable, script_path],
            stdout=subprocess.PIPE, stderr=None, text=True,
            timeout=120,
        )
    except subprocess.TimeoutExpired:
        print("HATA: detect_robot.py 120 saniyede bitmedi (zaman asimi).")
        sys.exit(1)

    if result.returncode != 0 or not result.stdout.strip():
        print(f"HATA: Robot algilanamadi. returncode={result.returncode}, stdout={result.stdout!r}")
        sys.exit(1)

    info = {}
    for line in result.stdout.splitlines():
        m = re.match(r"^([A-Z_][A-Z0-9_]*)=(.*)$", line)
        if m:
            info[m.group(1)] = m.group(2)

    required = ["ROBOT_IP", "ROBOT_MODEL_NUM"]
    missing = [k for k in required if k not in info]
    if missing:
        print(f"HATA: detect_robot.py ciktisinda eksik alan(lar): {missing}")
        sys.exit(1)
    return info


# ---------------------------------------------------------------------------
# Pozisyon matematigi (movepy / fairino_controller.py ile ayni, dogrulanmis)
# ---------------------------------------------------------------------------

def euler_deg_to_quat(rx, ry, rz):
    r, p, y = math.radians(rx), math.radians(ry), math.radians(rz)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def quat_to_euler_deg(qx, qy, qz, qw):
    """euler_deg_to_quat'in TAM TERSI - ayni (roll=RX, pitch=RY, yaw=RZ,
    Rz*Ry*Rx sirali) taniminin standart tersi (bkz. Wikipedia 'Conversion
    between quaternions and Euler angles'). Joint modunda hareket etmeden
    ONCE 'bu eklem acilariyla hangi yonelime varilacak' onizlemesi icin
    (bkz. CobotArm.predict_tcp_pose)."""
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # gimbal lock (RY = +-90 derece)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def compute_tcp_offset(robot_model, group_name, current_joints_deg, current_tcp_pose):
    """wrist3_link -> SDK'nin TCP noktasi arasindaki farki, wrist3_link'in
    KENDI (yerel) eksenlerinde tam bir 3B vektor (ox,oy,oz) olarak doner.

    ONCEKI SURUM sadece yerel Z eksenine izdusum aliyordu (tek sayi) - yani
    "gripper wrist3'ten TAM olarak kendi Z ekseni boyunca, hic yanal (X/Y)
    kacıklik ve rotasyon olmadan cikiyor" varsayimi yapiyordu. Montaj gercekten
    boyleyse sonuc degismez (X/Y bilesenleri ~0 cikar), ama WebApp'te
    tanimlanan Tool koordinat sisteminde en kucuk bir yanal kaçıklik/asimetri
    varsa o bilesen sessizce atiliyor, ve bu atilan kisim TAM OLARAK yonelim
    degistikce (RX/RY/RZ farkli oldukca) global cerceve icinde degisen bir
    yone kayiyordu - buyuk (100mm) hareketlerde gozle gorulur sapmaya yol
    acabilecek buyuklukte bir hata kaynagi. Tam vektor almak bunu ortadan
    kaldirir."""
    from moveit.core.robot_state import RobotState
    state_now = RobotState(robot_model)
    state_now.set_joint_group_positions(group_name, [math.radians(v) for v in current_joints_deg])
    state_now.update()
    wrist3_pose = state_now.get_pose(TIP_LINK)

    wx, wy, wz = wrist3_pose.position.x, wrist3_pose.position.y, wrist3_pose.position.z
    qx, qy, qz, qw = (wrist3_pose.orientation.x, wrist3_pose.orientation.y,
                       wrist3_pose.orientation.z, wrist3_pose.orientation.w)
    # wrist3_link'in yerel X/Y/Z eksenlerinin GLOBAL (base_link) karsiligi -
    # quaternion'dan turetilen rotasyon matrisinin sutunlari.
    local_x_in_global = [
        1 - 2 * (qy * qy + qz * qz),
        2 * (qx * qy + qz * qw),
        2 * (qx * qz - qy * qw),
    ]
    local_y_in_global = [
        2 * (qx * qy - qz * qw),
        1 - 2 * (qx * qx + qz * qz),
        2 * (qy * qz + qx * qw),
    ]
    local_z_in_global = [
        2 * (qx * qz + qy * qw),
        2 * (qy * qz - qx * qw),
        1 - 2 * (qx * qx + qy * qy),
    ]

    tx_m, ty_m, tz_m = current_tcp_pose[0] / 1000.0, current_tcp_pose[1] / 1000.0, current_tcp_pose[2] / 1000.0
    diff = [tx_m - wx, ty_m - wy, tz_m - wz]
    ox = sum(diff[i] * local_x_in_global[i] for i in range(3))
    oy = sum(diff[i] * local_y_in_global[i] for i in range(3))
    oz = sum(diff[i] * local_z_in_global[i] for i in range(3))
    return (ox, oy, oz)


def write_moveit_cpp_yaml(path, max_vel, max_accel):
    content = f"""planning_scene_monitor_options:
  name: "planning_scene_monitor"
  robot_description: "robot_description"
  joint_state_topic: "/joint_states"
  attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
  publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
  monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
  wait_for_initial_state_timeout: 10.0

planning_pipelines:
  pipeline_names: ["ompl", "pilz_industrial_motion_planner"]

plan_request_params:
  planning_attempts: 1
  planning_pipeline: ompl
  max_velocity_scaling_factor: {max_vel}
  max_acceleration_scaling_factor: {max_accel}
"""
    with open(path, "w") as f:
        f.write(content)


def check_trajectory_for_floor(trajectory, links, floor_z_m):
    violations = []
    for i in range(len(trajectory)):
        state = trajectory[i]
        for link in links:
            pose = state.get_pose(link)
            if pose.position.z < floor_z_m:
                violations.append((i, link, pose.position.z))
    return violations


def check_trajectory_for_self_collision(scene, trajectory, group_name):
    """Planlanan yolun HER waypoint'inde kol linkleri + gripper kutusu
    arasindaki minimum mesafeyi olcer, MIN_CLEARANCE_M'in altina duserse
    (gercekten degmese BILE) reddeder.

    NEDEN SALT is_state_colliding() YETMEDI: O sadece True/False ('degiyor
    mu') doner - canli testte tam olarak bu yuzden yeterli olmadigi
    kanitlandi: is_state_colliding() False donmustu (yani "carpismiyor")
    ama CollisionRequest(distance=True) ile olculen GERCEK mesafe sadece
    9.54mm cikti - hareket devam etseydi gercekten carpacakti (kullanici
    acil stopla durdurmustu). Yani "degmiyor" ile "guvenli" AYNI SEY DEGIL.

    NEDEN AYRICA GEREKLI (OMPL'in kendi kontrolu yetmiyor): OMPL'in
    planlama-zamani gecerlilik kontrolu, yol boyunca SINIRLI SAYIDA (bu
    paket icin ozellestirilmemis kutuphane varsayimi) noktayi kontrol eder;
    iki kontrol noktasi arasinda ince bir yakin-gecis "sizabilir".
    time_optimal_trajectory_generation SONRASI trajectory cok daha yogun
    orneklendigi icin, burada HER noktayi tekrar (ve mesafeyle) kontrol
    etmek ikinci, cok daha sik VE cok daha sıkı bir guvenlik agi saglar.
    HIZ ONEMLI DEGIL - guvenlik oncelik, hicbir nokta atlanmiyor."""
    from moveit.core.collision_detection import CollisionRequest, CollisionResult
    violations = []
    for i in range(len(trajectory)):
        state = trajectory[i]
        req = CollisionRequest()
        req.distance = True
        req.joint_model_group_name = group_name
        res = CollisionResult()
        # check_self_collision (check_collision DEGIL) - dunya nesneleriyle
        # (ör. zemin) mesafeyi KARISTIRMAMAK icin. Zemin ayrica ve zaten
        # check_trajectory_for_floor ile kontrol ediliyor; ikisini tek bir
        # mesafe esiginde birlestirmek yanlis pozitiflere yol acar (canli
        # testte tam bunu yasadik - bkz. MIN_CLEARANCE_M yorumu).
        scene.check_self_collision(req, res, state)
        # PADDING DEVREDE: govdeler zaten LINK_PADDING_M kadar sismis
        # sayiliyor, dolayisiyla "sismis govdeler degiyor" = "gercek bosluk
        # MIN_CLEARANCE_M'in altinda". Buraya AYRICA mesafe esigi koymak payi
        # IKI KEZ saymak olurdu (ör. gercekte 16.7mm acik olan guvenli bir poz,
        # sismis olcumde 4.7mm gorunur ve yanlislikla reddedilirdi).
        # Raporlanan mesafe okunabilir olsun diye GERCEK bosluga cevriliyor.
        if res.collision:
            violations.append((i, res.distance + 2 * LINK_PADDING_M))
    return violations


# ---------------------------------------------------------------------------
# ONEMLI - DURUM OKUMA HANGI KANALDAN YAPILMALI
# ---------------------------------------------------------------------------
# Bu dosyada durum okumalari SDK SARMALAYICISI uzerinden yapilir:
#     sdk.GetActualJointPosDegree()[1]      (DOGRU)
# ASLA ham XML-RPC uc noktasi uzerinden DEGIL:
#     sdk.robot.GetActualJointPosDegree(0)  (YANLIS - "sdk.robot" ham proxy)
#
# Sebep: SDK sarmalayicisi (fairino/Robot.py) bu fonksiyonlarda XML-RPC'ye HIC
# gitmiyor - dogrudan arka plandaki gercek-zamanli durum paketinden okuyor
# (robot_state_pkg, port 20004); XML-RPC yolu SDK kaynaginda yorum satirinda.
# Ham "sdk.robot.*" cagrisi ise gercek bir XML-RPC istegi (port 20003) yapar ve
# ROBOT SERVO MODUNDAYKEN (ros2_control ServoMoveStart ile akis gonderirken)
# BU ISTEKLER YANITLANMIYOR - cagri sonsuza kadar asili kaliyor (xmlrpc
# istemcisinin varsayilan zaman asimi da yok). Canli testte movecobot tam
# burada, "ilk durum okunuyor" adiminda kilitlendi; fairino_launch calisirken
# her seferinde tekrarladi.
#
# Sarmalayicinin donus bicimi farkli: (0, [6 deger]) - yani [1] ile listeye
# eriliyor, ham proxy'deki gibi [1:7] dilimlemesi DEGIL.
# ---------------------------------------------------------------------------

def wait_for_live_joint_reading(sdk, sdk_lock, max_wait_s=1.5, poll_interval_s=0.1):
    """Taze acilan (veya yeniden acilan) bir SDK baglantisinda, arka plandaki
    realtime-durum thread'i (port 20004) ilk gercek paketi alana kadar
    GetActualJointPosDegree() eski/varsayilan bir deger (6 eksenin HEPSI tam
    olarak 0.0) dondurebiliyor - bu fonksiyon hep err=0 doner, SDK bu farki
    kendisi ayirt etmiyor. Onceden bu yaris durumu sabit bir time.sleep(1.0)
    ile "azaltiliyordu" ama bu sadece bir tahmindi - baglanti bu sureden
    daha uzun surerse (ör. ayni anda ros2_control_node de kendi baglanti/
    reset/enable/ServoMoveStart akisiyla robotu mesgul ediyorsa) sessizce
    sifir degerler dondurulup baslangic durumu olarak kullanilabiliyordu
    (canli testte tam bu yasandi: kol durumu TAMAMEN 0.000 gosterdi).

    6 eksenin HEPSININ ayni anda tam 0.000 olmasi gercek bir FR3 pozu icin
    pratikte imkansiz (TCP de dunya orijininde olamaz) - bu yuzden bu durumu
    "henuz gercek veri gelmedi" isareti olarak guvenilir sekilde kullanip,
    gercek (sifir olmayan) veri gelene ya da zaman asimina ugrayana kadar
    kisa araliklarla tekrar tekrar okuyoruz."""
    deadline = time.time() + max_wait_s
    last = None
    while True:
        with sdk_lock:
            last = sdk.GetActualJointPosDegree()[1]
        if any(abs(v) > 1e-9 for v in last):
            return last
        if time.time() >= deadline:
            print(f"UYARI: {max_wait_s:.0f}s icinde robottan GERCEK eklem verisi "
                  "alinamadi (hep 0.000 donuyor) - baglanti kurulmus olabilir ama "
                  "realtime durum akisi hala baslamamis olabilir. Gosterilen "
                  "degerlere GUVENME, WebApp'ten kontrol et ya da movecobot'u "
                  "yeniden baslat.")
            return last
        time.sleep(poll_interval_s)


# ---------------------------------------------------------------------------
# CobotArm: movepy + fairino_controller.py'nin birlesimi, ama modele/IP'ye
# gore OTOMATIK ayarlanan (hardcoded fairino5/127.0.0.1 degil) versiyonu.
# ---------------------------------------------------------------------------

class CobotArm:
    def __init__(self, robot_ip, model_num, speed_pct, accel_pct, node_name="movecobot"):
        import rclpy
        from geometry_msgs.msg import Pose
        from moveit.planning import MoveItPy
        from moveit_configs_utils import MoveItConfigsBuilder
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from shape_msgs.msg import SolidPrimitive

        self.robot_ip = robot_ip
        self.model_num = model_num
        self.group_name = f"fairino{model_num}_v6_group"
        self.robot_desc_name = f"fairino{model_num}_v6_robot"
        self.package_name = f"fairino{model_num}_v6_moveit2_config"
        self.speed_pct = speed_pct
        self.accel_pct = accel_pct

        # movecobot artik gripper GUI'yi (grippergui) kendi arka plan
        # thread'inde, AYNI self.sdk baglantisiyla otomatik acabiliyor - GUI'nin
        # canli slider hareketi + 1sn'de bir durum sorgulamasi, konsoldan
        # yapilan islemlerle (jog/replay/record) AYNI ANDA tetiklenebilir.
        # xmlrpc.client.ServerProxy thread-safe degil, bu yuzden self.sdk'ya
        # dokunan HER YER bu kilidi kullanmali (RLock - ic ice cagrilar var,
        # ör. GripperController.activate() -> is_active()).
        self.sdk_lock = threading.RLock()
        # execute() calisirken (ros2_control_node, AYRI bir process/baglanti
        # uzerinden, ServoJ akisini gonderirken) movecobot'un KENDI SDK
        # baglantisindan (ör. grippergui'nin arka plan durum sorgulama
        # thread'i - bkz. interactive_main) ES ZAMANLI baska bir istek
        # robota gitmesin diye - canli testte, GERCEK bir hareket
        # calistirilirken ServoJ'nin SUREKLI err=14 ile basarisiz oldugu
        # gozlemlendi (baslangicta/bosta degil, SADECE aktif calistirma
        # sirasinda) - robotun gercek-zamanli komut kanalinin, AYNI ANDA
        # IKI FARKLI baglantidan (ros2_control'un kendi + movecobot'un
        # kendi) gelen trafigi kaldiramadigi teorisiyle tutarli. Bu
        # Event, _plan_and_execute() icinde execute() etrafinda set/clear
        # edilir; grippergui'nin poll_status()'u bunu kontrol edip aktif
        # calistirma sirasinda kendi SDK sorgusunu ATLAR.
        self.executing = threading.Event()
        # ASAGIDAKI zaman-damgali print'ler SADECE tanisal amacli - __init__
        # icinde MoveItPy() gibi dogasi geregi yavas (planlama pipeline'lari,
        # robot modeli, kinematik eklentileri yukleyen) adimlar var ve
        # ONCEDEN hicbiri ekrana bir sey basmiyordu - kullanici bu sessizlik
        # yuzunden "movecobot cok bekletiyor/kitleniyor" diye dusunmustu,
        # oysa NEREDE zaman gectigi hicbir zaman gorunur degildi. Bu
        # printler robotu/planlamayi DEGISTIRMIYOR, sadece hangi asamanin ne
        # kadar surdugunu gosteriyor.
        _t0 = time.time()
        def _tlog(msg):
            print(f"  [+{time.time()-_t0:5.1f}s] {msg}")
        _tlog("Robota SDK baglantisi kuruluyor...")
        self.sdk = Robot.RPC(robot_ip)
        _tlog("SDK baglantisi kuruldu, ilk durum okunuyor...")
        # Yeni acilan bir SDK baglantisinda, arka plandaki realtime-durum
        # thread'i (port 20004) henuz ilk gercek paketi almadan
        # GetActualJointPosDegree() ESKI/varsayilan bir deger dondurebiliyor
        # (bu fonksiyon hep err=0 doner, SDK bu farki kendisi ayirt etmiyor) -
        # ayni risk enter_drag_teach()'te de var, orada da ayni sekilde
        # (sabit bekleme ile) ele aliniyor. Burada bu deger hem TCP offset
        # hesabinda hem de ILK hareketin _set_start_state_from_sdk() uzerinden
        # planlamanin BASLANGIC durumu olarak kullanilacagi icin, bayat bir
        # okuma robotun GERCEK pozisyonundan FARKLI bir "su an buradayim"
        # varsayimiyla planlama yapilmasina - execute() sirasinda robotun
        # GERCEK pozisyona aniden sicramasina (ve bunun tetikledigi "shaft
        # collision fault"a) yol acabilir. Canli testte connect+ilk hareket
        # sirasinda tam bu yasandi.
        current_joints = wait_for_live_joint_reading(self.sdk, self.sdk_lock)
        with self.sdk_lock:
            current_pose = self.sdk.GetActualTCPPose()[1]
            current_flange_pose = self.sdk.GetActualToolFlangePose(0)[1]
        _tlog("Robot durumu okundu, ROS2/MoveIt baslatiliyor...")

        max_vel = max(0.01, min(1.0, speed_pct / 100.0))
        max_accel = max(0.01, min(1.0, accel_pct / 100.0))
        yaml_path = f"/tmp/fairino{model_num}_movecobot.yaml"
        write_moveit_cpp_yaml(yaml_path, max_vel, max_accel)

        rclpy.init(args=None)
        self._node = Node(node_name)
        self._executor = MultiThreadedExecutor(2)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        moveit_config = (
            MoveItConfigsBuilder(self.robot_desc_name, package_name=self.package_name)
            .moveit_cpp(file_path=yaml_path)
            .to_moveit_configs()
        )
        # ------------------------------------------------------------------
        # GUVENLIK PAYINI PLANLAYICIYA OGRETMEK (link padding)
        # ------------------------------------------------------------------
        # Onceden pay SADECE plan URETILDIKTEN SONRA kontrol ediliyordu:
        # OMPL yolu buluyordu, biz "bu yol 12mm esiginin altina duyuyor"
        # deyip REDDEDIYORDUK ve hareket tamamen iptal oluyordu. Planlayicinin
        # payin varligindan HABERI YOKTU, dolayisiyla alternatif bir yol da
        # aramiyordu (canli olcum: ayni hedef icin plan() 5 kez cagrildiginda
        # 5'i de BIREBIR AYNI yolu verdi - OMPL burada deterministik, yani
        # "tekrar dene" yaklasimi ise yaramiyor; planning_attempts=1/5/20 de
        # ayni sonucu verdi).
        #
        # Dogru cozum MoveIt'in kendi mekanizmasi: link padding. MoveIt her
        # linki verilen kadar "sismis" kabul ederek carpisma kontrolu yapar
        # (planning_scene_monitor.cpp -> robot_description_planning.
        # default_robot_padding). Boylece pay GEOMETRIYE gomulur ve OMPL
        # zaten payi koruyan bir yol arar - carpiyorsa kendisi baska yola
        # gecer.
        #
        # Iki govde de sistigi icin aralarindaki GERCEK bosluk 2*padding olur;
        # bu yuzden padding = MIN_CLEARANCE_M / 2 (12mm hedef -> 6mm padding).
        # Gripper AYRI bir "attached object" oldugu icin onun payi ayri
        # parametreyle veriliyor (default_attached_padding), aksi halde
        # gripper paysiz kalirdi.
        _cfg = moveit_config.to_dict()
        _rdp = _cfg.setdefault("robot_description_planning", {})
        _rdp["default_robot_padding"] = LINK_PADDING_M
        _rdp["default_attached_padding"] = LINK_PADDING_M
        _tlog("MoveItPy baslatiliyor (planlama pipeline'lari + robot modeli yukleniyor, "
              "bu genelde en uzun suren asama)...")
        self._moveit = MoveItPy(node_name=f"{node_name}_core", config_dict=_cfg)
        _tlog("MoveItPy hazir.")
        self._planning_component = self._moveit.get_planning_component(self.group_name)
        self._planning_component.set_start_state_to_current_state()

        # robot_model/tcp_offset_m collision nesnelerinden ONCE hesaplanmali -
        # gripper collision kutusunun boyutu/konumu olculen offset'e bagli.
        self.robot_model = self._moveit.get_robot_model()
        self.tcp_offset_m = compute_tcp_offset(self.robot_model, self.group_name, current_joints, current_pose)
        # wrist3_link'in KENDI govdesi (mesh'i) zaten flansa kadar (bu robotta
        # ~100mm, modele gore degisir) collision olarak modellenmis - bkz.
        # fairino_description/meshes/.../wrist3_link.STL, Z araligi 0-100mm.
        # tcp_offset_m ise wrist3_link'in ORIJININDEN (eklem ekseninden)
        # itibaren olculuyor - yani bu govde payini icinde barindiriyor.
        # Gripper kutusunu flanstan degil orijinden baslatmak, zaten collision
        # kapsamindaki bu govdeyi bir de gripper kutusuna dahil edip toplam
        # uzunlugu gereksiz yere ikiye katliyordu (canli testte tam bu oldu:
        # olculen 199.7mm, oysa robotun kendi GetTCPOffset()'i sadece 100mm
        # diyordu). flange_offset_m bu govde payini ayirmak icin.
        self.flange_offset_m = compute_tcp_offset(self.robot_model, self.group_name, current_joints, current_flange_pose)

        self._add_floor_collision(Pose, CollisionObject, SolidPrimitive)
        self._add_gripper_collision(Pose, CollisionObject, AttachedCollisionObject, SolidPrimitive)
        _tlog("CobotArm hazir.")

        self._rclpy = rclpy
        self._shutdown_called = False

    def _add_floor_collision(self, Pose, CollisionObject, SolidPrimitive):
        try:
            psm = self._moveit.get_planning_scene_monitor()
            collision_object = CollisionObject()
            collision_object.header.frame_id = "base_link"
            collision_object.id = "floor"
            box_pose = Pose()
            box_pose.position.z = -0.05
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [2.0, 2.0, 0.1]
            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD
            with psm.read_write() as scene:
                scene.apply_collision_object(collision_object)
                # Zemin kutusunun ust yuzeyi z=0'da, robotun TABANI da orada
                # oturuyor - yani temas GERCEKTE normal (robot masaya cıvatalı).
                # Link padding devreye girdikten sonra (bkz. LINK_PADDING_M)
                # bu temas "carpisma" sayilir oldu ve MTC gorevleri
                # "base_link colliding with floor" ile basarisiz oldu.
                # Taban ile zemin arasindaki carpisma bu yuzden ACIKCA
                # serbest birakiliyor - zemin kontrolu KOLUN asagi inmesi
                # icin var, taban icin degil. SADECE bu tek cift serbest;
                # gripper kutusu ve tasinan nesne dahil digerlerinin zemin
                # kontrolu aynen devam ediyor.
                scene.allowed_collision_matrix.set_entry("floor", "base_link", True)
                scene.current_state.update()
        except Exception as e:
            print(f"UYARI: Zemin collision nesnesi eklenemedi ({type(e).__name__}: {e})")

    def _add_gripper_collision(self, Pose, CollisionObject, AttachedCollisionObject, SolidPrimitive):
        """Jodell EPG40-050 gripper URDF'de HIC modellenmemis - robot modeli
        tam wrist3_link'te bitiyor (bkz. fairino_description/urdf/
        fairino{N}_v6.urdf, j6/wrist3_link son link/joint). Yani MoveIt'in
        KENDI self-collision kontrolu - plan() icinde HER ZAMAN varsayilan
        olarak aktif, ayrica kod baska hicbir yerde devre disi birakmiyor -
        gripper'in fiziksel hacminden tamamen habersizdi: wrist3_link govdeye
        yeterince uzak planlansa bile, ucundaki ~90mm'lik gripper govdeye/
        diger uzuvlara carpabiliyordu (gercekte de oldu). Burada wrist3_link'e
        SABIT (attached) bir collision kutusu eklemek, ayni collision-aware
        planlamayi (self-collision + zemin, ikisi de zaten vardi) artik
        gripper'in gercek hacmini de kapsayacak sekilde genisletir - jog,
        replay, kayitli hareket... hepsi AYNI _plan_and_execute'tan gectigi
        icin TEK yerden tum hareket yollari korunur. Istisna: manuel/
        drag-teach modu (planlama kullanmiyor, fiziksel olarak elle yapiliyor,
        yazilimsal engelleme mumkun degil - kullanici dikkatli olmali)."""
        try:
            ox, oy, oz = self.tcp_offset_m
            fx, fy, fz = self.flange_offset_m
            # SADECE flanstan OTEYE (gercek gripper'a ait) kisim - wrist3_link
            # meshinin zaten kapsadigi flansa-kadarki govde payi (fx,fy,fz)
            # HARIC tutuluyor, bkz. yukaridaki flange_offset_m yorumu.
            gx, gy, gz = ox - fx, oy - fy, oz - fz
            # Canli olcum (GetTCPOffset uzerinden) ile guvenlik tabanindan
            # (GRIPPER_LENGTH_FLOOR_M) BUYUK olani kullan - WebApp'teki tool
            # offset ayari gercek gripperdan kisa girilmis olsa bile kutu bu
            # tabanin altina dusmesin.
            length = max(abs(gz), GRIPPER_LENGTH_FLOOR_M) + GRIPPER_LENGTH_MARGIN_M
            sign = 1.0 if gz >= 0 else -1.0
            psm = self._moveit.get_planning_scene_monitor()
            attached = AttachedCollisionObject()
            attached.link_name = TIP_LINK
            attached.object.header.frame_id = TIP_LINK
            attached.object.id = "jodell_gripper"
            box_pose = Pose()
            box_pose.position.x = fx + gx / 2.0
            box_pose.position.y = fy + gy / 2.0
            box_pose.position.z = fz + sign * length / 2.0
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [GRIPPER_WIDTH_M, GRIPPER_DEPTH_M, length]
            attached.object.primitives.append(box)
            attached.object.primitive_poses.append(box_pose)
            attached.object.operation = CollisionObject.ADD
            attached.touch_links = [TIP_LINK]
            from moveit.core.collision_detection import CollisionRequest, CollisionResult
            with psm.read_write() as scene:
                scene.process_attached_collision_object(attached)
                scene.current_state.update()
                # DIKKAT: sadece "degiyor mu" degil, GERCEK mesafeyi de
                # olcuyoruz - canli testte is_state_colliding() False
                # donduren ama gercekte sadece 9.54mm mesafesi olan (ve
                # devam etseydi carpacak) bir durum yakalanmisti. Eger
                # robotun SU ANKI pozisyonunda mesafe MIN_CLEARANCE_M'in
                # altindaysa (gercek carpisma DAHIL), MoveIt "start state"i
                # gecersiz sayar ve BUNDAN SONRA hicbir plan() (jog/replay
                # dahil, TUMU) basarili olamaz - robotu bu araçla RETREAT
                # bile edemezsin. Bunu sessizce yasamak yerine burada
                # erkenden tespit edip kullaniciya haber veriyoruz.
                req = CollisionRequest()
                req.distance = True
                req.joint_model_group_name = self.group_name
                res = CollisionResult()
                scene.check_collision(req, res, scene.current_state)
                # PADDING DEVREDE (bkz. LINK_PADDING_M): govdeler zaten sismis
                # sayildigi icin "degiyor" = "gercek bosluk esigin altinda".
                # Ayrica mesafe esigi uygulanmiyor - pay iki kez sayilmasin.
                too_close = res.collision
                true_gap_m = res.distance + 2 * LINK_PADDING_M
            print(f"  Gripper collision kutusu eklendi: {GRIPPER_WIDTH_M*1000:.0f}x{GRIPPER_DEPTH_M*1000:.0f}x"
                  f"{length*1000:.0f}mm (flanstan itibaren, {TIP_LINK}'e sabit; flans payi "
                  f"X={fx*1000:.1f} Y={fy*1000:.1f} Z={fz*1000:.1f}mm, canli olculen gripper="
                  f"{abs(gz)*1000:.1f}mm, guvenlik tabani={GRIPPER_LENGTH_FLOOR_M*1000:.0f}mm, "
                  f"buyuk olan + {GRIPPER_LENGTH_MARGIN_M*1000:.0f}mm pay kullanildi)")
            print(f"  Su anki pozisyonda en yakin iki govde arasi mesafe: {true_gap_m*1000:.1f}mm "
                  f"(guvenlik esigi: {MIN_CLEARANCE_M*1000:.0f}mm, planlayiciya "
                  f"{LINK_PADDING_M*1000:.0f}mm link padding olarak verildi)")

            if too_close:
                print(f"\nUYARI: Robotun SU ANKI pozisyonunda en yakin mesafe ({true_gap_m*1000:.1f}mm) "
                      f"guvenlik esiginin ({MIN_CLEARANCE_M*1000:.0f}mm) altinda "
                      f"{'(GERCEKTEN CAKISIYOR)' if res.collision else '(henuz degmiyor ama cok yakin)'}. "
                      "Iki ihtimal var:")
                print("  1) Gercekten cok yakin/riskli bir poz - dikkatli ol.")
                print("  2) Kutunun tahmini boyutlari (gercek CAD yok) bu poz icin fazla "
                      "muhafazakar - yanlis pozitif.")
                print("Bu haliyle plan() HER hareket icin 'Invalid start state' verip "
                      "BASARISIZ olur - jog/replay hicbiri calismaz.")
                answer = input(
                    "Bu oturum icin gripper collision korumasini KAPATIP devam etmek ister misin? "
                    "(e = kapat ve devam et, h = acik birak): "
                ).strip().lower()
                if answer == "e":
                    try:
                        remove_msg = AttachedCollisionObject()
                        remove_msg.link_name = TIP_LINK
                        remove_msg.object.id = "jodell_gripper"
                        remove_msg.object.operation = CollisionObject.REMOVE
                        with psm.read_write() as scene:
                            scene.process_attached_collision_object(remove_msg)
                            scene.current_state.update()
                        print("Gripper collision korumasi bu oturum icin KAPATILDI. "
                              "Cok dikkatli ol, hareketler artik gripper-govde carpmasina karsi "
                              "KONTROLSUZ.")
                    except Exception as e2:
                        print(f"UYARI: kutu kaldirilamadi ({type(e2).__name__}: {e2}).")
                else:
                    print("Koruma acik birakildi. plan() 'Invalid start state' ile basarisiz "
                          "olmaya devam edecek - robotu WebApp'ten manuel jog ile daha acik bir "
                          "poza al, sonra movecobot'u yeniden baslat.")
        except Exception as e:
            print(f"UYARI: Gripper collision nesnesi eklenemedi ({type(e).__name__}: {e}) - "
                  f"self-collision kontrolu gripper'in hacmini KAPSAMAYACAK, cok dikkatli ol!")

    # ------------------------------------------------------------------
    # KAVRANAN NESNEYI MODELE ILISTIRME (MoveIt attach / detach)
    # ------------------------------------------------------------------
    # MoveIt'in pick-and-place semasinda kavrama aninda nesne robotun
    # modeline ILISTIRILIR: o andan itibaren kolla birlikte hareket eden bir
    # govde olarak carpisma kontrolune girer ve gripper ile arasindaki temas
    # (bilerek dokunuyorlar) "izinli" sayilir. Birakinca AYRILIR.
    #
    # Bu olmadan MoveIt tasidigimiz nesneden habersizdir - orayi BOS sanar ve
    # nesneyi kola/masaya surtecek bir yol planlayabilir; bizim guvenlik
    # kontrollerimiz de ayni modele baktigi icin bunu yakalayamaz.
    #
    # Kutu, TCP'de (cenelerin arasinda) merkezlenir; gripper'in kendi kutusu
    # ve wrist3 ile temasi touch_links ile izinli kilinir.
    def attach_grasped_object(self, width_m, depth_m, height_m):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        from shape_msgs.msg import SolidPrimitive
        try:
            ox, oy, oz = self.tcp_offset_m
            psm = self._moveit.get_planning_scene_monitor()
            attached = AttachedCollisionObject()
            attached.link_name = TIP_LINK
            attached.object.header.frame_id = TIP_LINK
            attached.object.id = GRASPED_OBJECT_ID
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = ox, oy, oz
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [width_m, depth_m, height_m]
            attached.object.primitives.append(box)
            attached.object.primitive_poses.append(pose)
            attached.object.operation = CollisionObject.ADD
            # Gripper govdesi ve bilek ile temas normaldir - carpisma sayilmasin.
            attached.touch_links = [TIP_LINK, "jodell_gripper"]
            with psm.read_write() as scene:
                scene.process_attached_collision_object(attached)
                scene.current_state.update()
            print(f"  Nesne modele iliştirildi: {width_m*1000:.0f}x{depth_m*1000:.0f}x"
                  f"{height_m*1000:.0f}mm (planlama artik onu da hesaba katiyor).")
            self._object_attached = True
        except Exception as e:
            print(f"  UYARI: nesne modele iliştirilemedi ({type(e).__name__}: {e}) - "
                  f"planlama tasidigin nesneden HABERSIZ olacak, dikkatli ol.")

    def detach_object(self):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        if not getattr(self, "_object_attached", False):
            return
        try:
            psm = self._moveit.get_planning_scene_monitor()
            remove = AttachedCollisionObject()
            remove.link_name = TIP_LINK
            remove.object.id = GRASPED_OBJECT_ID
            remove.object.operation = CollisionObject.REMOVE
            with psm.read_write() as scene:
                scene.process_attached_collision_object(remove)
                scene.current_state.update()
            print("  Nesne modelden ayrildi (birakildi).")
        except Exception as e:
            print(f"  UYARI: nesne modelden ayrilamadi ({type(e).__name__}: {e}).")
        finally:
            self._object_attached = False

    def get_current_state(self):
        # DIKKAT: burada "tum eksenler 0.0 ise tekrar oku" gibi bir kontrol
        # ONCEDEN vardi ama GERI ALINDI - bu fonksiyon jog/replay/print_state
        # tarafindan COK SIK (her adimdan sonra) cagriliyor, ve robotun
        # GERCEK dinlenme/referans pozu da tum eksenlerde tam 0.0 olabiliyor
        # (canli testte oldu) - o durumda bu kontrol her cagrida yanlislikla
        # tetiklenip saniyelerce bekletiyordu ("movecobot cok bekletiyor").
        # Taze-baglanti placeholder riski sadece BAGLANTI KURULDUGU ANDA
        # (bkz. wait_for_live_joint_reading cagrilari: __init__,
        # enter_drag_teach, exit_drag_teach) ele aliniyor, burada degil.
        with self.sdk_lock:
            joints = self.sdk.GetActualJointPosDegree()[1]
            pose = self.sdk.GetActualTCPPose()[1]
        return {
            "joints_deg": [round(v, 3) for v in joints],
            "tcp_pose": {
                "x": round(pose[0], 3), "y": round(pose[1], 3), "z": round(pose[2], 3),
                "rx": round(pose[3], 3), "ry": round(pose[4], 3), "rz": round(pose[5], 3),
            },
        }

    def _plan_and_execute(self, confirm, speed_pct=None, accel_pct=None, prefer_lin=False,
                          max_joint_excursion_deg=None):
        """speed_pct/accel_pct verilirse (adim-bazli kayitli hiz/ivme), o TEK
        plan() cagrisi icin oturumun varsayilanini (moveit_cpp.yaml'a init'te
        yazilan) PlanRequestParameters ile gecici olarak ezer - CobotArm'in
        genel self.speed_pct/self.accel_pct'i DEGISMEZ, sadece bu hareket
        farkli hizda planlanir. Ikisi de None ise (jog / eski kayitlar,
        geriye donuk uyumluluk) eskisi gibi parametresiz plan() cagrilir,
        davranis DEGISMEZ."""
        plan_params = None
        if speed_pct is not None or accel_pct is not None:
            from moveit.planning import PlanRequestParameters
            plan_params = PlanRequestParameters(self._moveit)
            eff_speed = speed_pct if speed_pct is not None else self.speed_pct
            eff_accel = accel_pct if accel_pct is not None else self.accel_pct
            plan_params.max_velocity_scaling_factor = max(0.01, min(1.0, eff_speed / 100.0))
            plan_params.max_acceleration_scaling_factor = max(0.01, min(1.0, eff_accel / 100.0))
        # ------------------------------------------------------------------
        # NEDEN "KALIRSA YENIDEN PLANLA" DENMIYOR
        # ------------------------------------------------------------------
        # Bir ara buraya "plan guvenlik kontrolunden kalirsa 5 kez yeniden
        # planla" dongusu konmustu. OLCULDU VE KALDIRILDI: bu kurulumda OMPL
        # DETERMINISTIK davraniyor - ayni baslangic/hedef icin plan() 5 kez
        # cagrildiginda 5'i de BIREBIR AYNI yolu (ayni nokta sayisi, ayni ara
        # eklem acilari) dondurdu. PlanRequestParameters.planning_attempts
        # 1/5/20 denendi, sonuc yine ayni. Farkli planner_id vermek de ise
        # yaramiyor cunku bu paketin ompl_planning.yaml'inda planner_configs
        # bolumu yok; verilen her isim icin MoveIt "Cannot find planning
        # configuration ... will use defaults" deyip ayni varsayilana donuyor.
        # Yani yeniden planlamak sadece ayni sonucu tekrar uretip zaman
        # kaybettiriyordu.
        #
        # Bunun yerine pay PLANLAYICIYA ogretildi (link padding - bkz.
        # __init__ icindeki LINK_PADDING_M blogu): govdeler sismis kabul
        # edildigi icin OMPL zaten payi koruyan bir yol ariyor, "carpiyorsa
        # baska yola gec" isi planlayicinin kendi icinde oluyor. Asagidaki
        # kontroller artik son guvenlik agi olarak duruyor.
        # ------------------------------------------------------------------
        # ONCE EN KOLAY YOL: Pilz PTP (dogrudan eklem-uzayi hareketi)
        # ------------------------------------------------------------------
        # RRTConnect ORNEKLEME TABANLI bir planlayici: hedefe giden ilk
        # buldugu yolu doner, o yolun "kisa" veya "dogal" olmasi gibi bir
        # garantisi YOKTUR. Canli olcum (bugun): baslangic J3=90 derece,
        # hedef J3=80 derece iken uretilen planin orta noktasinda J3 =
        # -5.7 derece cikti - yani 10 derecelik bir is icin ~95 derecelik
        # gereksiz bir savrulma. Bu tur dolanmalar hem gereksiz uzun hem de
        # govdeye yaklastigi icin guvenlik kontrollerine takilmanin baslica
        # sebebi.
        #
        # Pilz'in PTP'si ise TANIMI GEREGI dogrudan yol: her eklem baslangic
        # acisindan hedef acisina monoton gider, dolanma yok, deterministik.
        # Sanayi robotlarindaki standart "point-to-point" hareketi budur.
        #
        # Bu yuzden sira: ONCE PTP dene, guvenlik kontrollerinden gecerse onu
        # kullan; gecmezse (ör. arada gercekten bir engel var, duz gitmek
        # carpismaya yol aciyor) OMPL/RRTConnect'e dus - o zaman dolanma
        # MESRU cunku duz yol kapali.
        def _try_plan(params):
            try:
                return self._planning_component.plan(parameters=params)
            except Exception as e:
                print(f"  (planlama denemesi hata verdi: {type(e).__name__}: {e})")
                return None

        def _check(candidate):
            """Guvenlik kontrolleri. Temizse None, degilse hata sozlugu doner."""
            v = check_trajectory_for_floor(candidate.trajectory, ALL_LINKS, FLOOR_Z_M)
            if v:
                w = min(v, key=lambda x: x[2])
                return {"success": False, "reason": "floor_collision",
                        "details": f"adim {w[0]}, link '{w[1]}', Z={w[2]*1000:.1f}mm"}
            psm_local = self._moveit.get_planning_scene_monitor()
            with psm_local.read_only() as sc:
                steps = check_trajectory_for_self_collision(sc, candidate.trajectory, self.group_name)
            if steps:
                ws, wd = min(steps, key=lambda x: x[1])
                return {"success": False, "reason": "self_collision",
                        "details": f"plandaki {len(steps)} noktada minimum guvenlik mesafesinin "
                                   f"({MIN_CLEARANCE_M*1000:.0f}mm) altina dusuluyor - en kotusu adim "
                                   f"{ws}/{len(candidate.trajectory)}, mesafe={wd*1000:.1f}mm"}
            return None

        from moveit.planning import PlanRequestParameters
        _eff_speed = speed_pct if speed_pct is not None else self.speed_pct
        _eff_accel = accel_pct if accel_pct is not None else self.accel_pct

        def _pilz(planner_id, yavaslatma=1.0):
            """yavaslatma < 1.0: LIN'in ivme ihlaliyle kalmasi halinde ayni
            hareketi daha yavas istemek icin (bkz. LIN_YAVASLATMA_KADEMELERI).
            Pilz bu olcekleri kartezyen limitlere de uyguluyor, dolayisiyla
            gereken eklem ivmesi de duesuyor."""
            prm = PlanRequestParameters(self._moveit)
            prm.planning_pipeline = "pilz_industrial_motion_planner"
            prm.planner_id = planner_id
            prm.max_velocity_scaling_factor = max(
                0.01, min(1.0, _eff_speed / 100.0 * yavaslatma))
            prm.max_acceleration_scaling_factor = max(
                0.01, min(1.0, _eff_accel / 100.0 * yavaslatma))
            return prm

        # prefer_lin=True: TCP'ye gore GORECELI hareketler (yaklasma/geri
        # cekilme) icin. Pick-and-place'in standart semasinda kavramaya
        # gidis ve nesneyle geri cekilis DUZ CIZGI olmalidir
        # (MoveIt: pre_grasp_approach / post_grasp_retreat) - gripper
        # nesneye yanlamasina dalmasin, kavradiktan sonra da nesneyi
        # yanlara savurarak degil dosdogru kaldirsin diye. Pilz LIN tam
        # olarak bunu garanti eder; PTP eklem-uzayinda duzdur ama TCP
        # kavisli gidebilir.
        _sira = []
        if prefer_lin:
            # Once istenen hizda; ivme ihlaliyle kalirsa kademeli yavaslatip
            # tekrar dene (bkz. LIN_YAVASLATMA_KADEMELERI).
            for _kat in LIN_YAVASLATMA_KADEMELERI:
                _etiket = ("Duz cizgi (LIN)" if _kat == 1.0
                           else f"Duz cizgi (LIN, %{int(_kat * 100)} hizda)")
                _sira.append(("LIN", _etiket, _kat))
        _sira.append(("PTP", "Dogrudan yol (PTP)", 1.0))

        plan_result = None
        for _pid, _ad, _kat in _sira:
            _cand = _try_plan(_pilz(_pid, _kat))
            if not _cand:
                print(f"  {_ad} planlanamadi - siradaki yonteme geciliyor.")
                continue
            _problem = _check(_cand)
            if _problem is None:
                plan_result = _cand
                print(f"  {_ad} kullanildi.")
                break
            print(f"  {_ad} guvenli degil ({_problem['reason']}) - siradaki yonteme geciliyor.")
        if plan_result is None:
            print("  OMPL'e geciliyor (engelden dolasarak).")

        if plan_result is None:
            plan_result = self._planning_component.plan(parameters=plan_params)
        if not plan_result:
            return {"success": False, "reason": "planning_failed"}

        violations = check_trajectory_for_floor(plan_result.trajectory, ALL_LINKS, FLOOR_Z_M)
        if violations:
            worst = min(violations, key=lambda v: v[2])
            return {"success": False, "reason": "floor_collision",
                    "details": f"adim {worst[0]}, link '{worst[1]}', Z={worst[2]*1000:.1f}mm"}

        # OMPL'in kendi planlama-zamani kontrolu KABA bir ornekleme
        # cozunurlugunde calisiyor (bkz. check_trajectory_for_self_collision
        # docstring'i) - buyuk/coklu-eklem hareketlerde bu, gercek bir
        # neredeyse-carpismayi kacirabiliyor (canli testte oldu). Bu yuzden
        # planlanan yolun TAMAMINI, cok daha yogun orneklenmis haliyle
        # (time-parameterization sonrasi) burada TEKRAR kontrol ediyoruz.
        psm = self._moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            self_collision_steps = check_trajectory_for_self_collision(
                scene, plan_result.trajectory, self.group_name)
        if self_collision_steps:
            worst_step, worst_dist = min(self_collision_steps, key=lambda v: v[1])
            return {"success": False, "reason": "self_collision",
                    "details": f"plandaki {len(self_collision_steps)} noktada minimum guvenlik "
                               f"mesafesinin ({MIN_CLEARANCE_M*1000:.0f}mm) altina dusuluyor - en "
                               f"kotusu adim {worst_step}/{len(plan_result.trajectory)}, mesafe="
                               f"{worst_dist*1000:.1f}mm"}

        # ORANTI KONTROLU - bkz. MAX_JOG_JOINT_EXCURSION_DEG ustundeki olay
        # kaydi. Zemin/self-collision kontrolleri bos havadaki genis bir yayi
        # ihlal saymaz; bu kontrol tam olarak onu yakalar. Plandaki HER nokta
        # baslangic acilariyla karsilastiriliyor - sadece bitis noktasi degil,
        # cunku yayin kendisi ara noktalarda savruluyor.
        if max_joint_excursion_deg is not None:
            with self.sdk_lock:
                baslangic = self.sdk.GetActualJointPosDegree()[1]
            plan_acilari = [
                [math.degrees(v) for v in
                 plan_result.trajectory[adim].get_joint_group_positions(self.group_name)]
                for adim in range(len(plan_result.trajectory))
            ]
            ihlal = asiri_eklem_hareketi(baslangic, plan_acilari, max_joint_excursion_deg)
            if ihlal is not None:
                return ihlal

        if confirm:
            answer = input("Bu hedefe uygulansin mi? (e/h): ").strip().lower()
            if answer != "e":
                return {"success": False, "reason": "user_cancelled"}

        # Planlanan yolun SON adimindaki eklem acilari - execute() sonrasi
        # robotun GERCEKTEN oraya ulasip ulasmadigini SDK'dan (ROS2/joint_states
        # UZERINDEN DEGIL, dogrudan robot kontrolcusune RPC ile) dogrulamak icin.
        # Bu ONEMLI: ros2_control/move_group cokup joint_states yayinlamayi
        # tamamen kesse bile execute() sessizce "basarili" gibi donebiliyor -
        # SDK okumasi bu durumdan BAGIMSIZ oldugu icin gercegi yakaliyor.
        last_state = plan_result.trajectory[len(plan_result.trajectory) - 1]
        target_joints_deg = [math.degrees(v) for v in last_state.get_joint_group_positions(self.group_name)]

        # bkz. __init__ icindeki self.executing yorumu - grippergui'nin arka
        # plan durum sorgulama thread'i bu pencere sirasinda kendi SDK
        # cagrisini atlasin diye.
        self.executing.set()
        try:
            self._moveit.execute(self.group_name, plan_result.trajectory)
        finally:
            self.executing.clear()

        # Hareket bittikten SONRAKI dogrulama okumasi.
        #
        # DIKKAT - BAYAT OKUMA TUZAGI: get_current_state() artik SDK'nin
        # ONBELLEKLI gercek-zamanli durum paketinden okuyor (ham XML-RPC
        # sorgusu yerine - bkz. wait_for_live_joint_reading ustundeki not).
        # Bu paket arka plan thread'i tarafindan guncelleniyor ve hareket
        # biter bitmez HENUZ guncellenmemis olabiliyor: canli testte
        # execute() dondukten hemen sonra okunan deger robotun BIR ONCEKI
        # pozisyonuydu, dolayisiyla "hedeften 137.5 derece uzakta" diye
        # SAHTE bir hata uretildi ve replay'in kalan adimlari iptal edildi -
        # oysa robot hedefe tam ulasmisti (/joint_states ile dogrulandi).
        #
        # Cozum: tek okumaya guvenme, deger oturana kadar kisa sure yokla.
        # GERCEK bir basarisizlikta deger zaten oturmaz ve hata dogru
        # sekilde raporlanir - guvenlik kontrolu zayiflamiyor, sadece
        # bayat veriye kanmiyor.
        VERIFY_TIMEOUT_S = 2.0
        VERIFY_POLL_S = 0.1
        _verify_deadline = time.time() + VERIFY_TIMEOUT_S
        while True:
            final_state = self.get_current_state()
            max_err_deg = max(abs(a - b) for a, b in zip(final_state["joints_deg"], target_joints_deg))
            if max_err_deg <= 2.0 or time.time() >= _verify_deadline:
                break
            time.sleep(VERIFY_POLL_S)
        if max_err_deg > 2.0:
            # ONEMLI: robot execute() SIRASINDA fault verip "Stopped" durumuna
            # duserse, bu SDK cagrisiyla dogrudan tespit edilebiliyor
            # (robot_state_pkg.main_code/sub_code - ayni realtime thread'in
            # doldurdugu alan, GetActualJointPosDegree ile ayni kaynaktan).
            # Bunu kontrol ETMEDEN once bu fonksiyon "hedeften X derece
            # uzakta kaldi" gibi belirsiz bir mesaj basiyordu - canli testte
            # bu mesajdaki X GERCEK bir sapma degil, WebApp'te GORUNEN fault
            # yuzunden final_state okumasi placeholder/sifir donmus olmasindan
            # kaynaklaniyordu (ör. J6 hedefi 135.9 iken final_state[J6]=0.0
            # okunmus, fark tam 135.9 cikmisti - tesadufen hedef degerin
            # kendisiyle ayni). Simdi robot GERCEKTEN faulted ise bunu ACIKCA
            # soyluyoruz - kullaniciyi WebApp'teki hatayi tahmin etmeye
            # birakmiyoruz.
            # TANI: canli testte robot hedefe GERCEKTEN ulastigi halde bu
            # kontrol "137.5 derece uzakta" dedi (ROS2 /joint_states ile
            # dogrulandi: tum eklemler hedefle birebir ayniydi). Yani
            # karsilastirmanin kendisinde bir uyusmazlik var. Suphe: SDK
            # okumasi j1..j6 sirasindayken, planlanan yolun son noktasi
            # PLANLAMA GRUBUNUN sirasinda geliyor olabilir - bu robotta
            # /joint_states'in isim sirasi j1,j2,j4,j5,j3,j6 (j1..j6 DEGIL).
            # Bir dahaki sefere sebebi tahmin etmemek icin iki vektoru de
            # ve grubun eklem sirasini basiyoruz.
            try:
                _names = list(self.robot_model.get_joint_model_group(
                    self.group_name).joint_model_names)
            except Exception:
                _names = ["(alinamadi)"]
            print(f"  TANI grup eklem sirasi : {_names}")
            print(f"  TANI hedef (plandan)   : {[round(v,2) for v in target_joints_deg]}")
            print(f"  TANI okunan (SDK)      : {[round(v,2) for v in final_state['joints_deg']]}")

            fault_note = ""
            try:
                with self.sdk_lock:
                    mc = self.sdk.robot_state_pkg.main_code
                    sc = self.sdk.robot_state_pkg.sub_code
                if mc != 0 or sc != 0:
                    fault_note = (f" ROBOT FAULT TESPIT EDILDI (main_code={mc}, sub_code={sc}) - "
                                  f"WebApp'teki hata mesajini kontrol et, 'Clear' ile temizleyip "
                                  f"tekrar dene.")
            except Exception:
                pass
            return {
                "success": False,
                "reason": "execution_not_confirmed",
                "details": (f"hedeften {max_err_deg:.1f} derece uzakta kaldi - robot hareket etmemis "
                            f"olabilir (ROS2/ros2_control tarafinda sorun var mi kontrol et: "
                            f"'ros2 node list' icinde controller_manager/joint_state_broadcaster "
                            f"gorunuyor mu?)." + fault_note),
                "final_state": final_state,
            }
        return {"success": True, "final_state": final_state}

    def _set_start_state_from_sdk(self, known_joints_deg=None):
        """set_start_state_to_current_state() YERINE kullanilir. O metod,
        MoveIt'in ROS2/joint_states uzerinden besledigi current_state_
        monitor'una guvenir. Bu projede (execution_not_confirmed, joint-
        speed fault'lari, RRTConnect'in tekrarlanamayan garip yollari...)
        ROS2/joint_states'in robotun GERCEK durumundan BAYAT/farkli
        olabildigini defalarca gorduk - kod zaten hareket SONRASI dogrulamayi
        bu yuzden ROS2 degil DOGRUDAN SDK okumasina dayandiriyor (bkz.
        get_current_state()). Ayni guvensizlik bir hareketin BASLANGIC
        durumu icin de gecerli: MoveIt yanlis/bayat bir baslangic pozundan
        planlama yaparsa, "carpismasiz" diye onayladigi plan GERCEK robotun
        GERCEK pozisyonu icin hic dogrulanmamis olabilir - canli testte
        beklenmedik bir carpisma-yakini olay ve garip bir bitis pozu
        (J4=+69 derece, hicbir hesapta olmayan bir deger) ile karsilastik,
        bu tam olarak boyle bir uyumsuzlukla aciklanabilir.

        known_joints_deg verilirse, SDK'ya YENIDEN SORMADAN dogrudan bu
        degeri baslangic durumu olarak kullanir - BIR ONCEKI hareketin
        _plan_and_execute() icindeki post-execution dogrulamasindan (yine
        DOGRUDAN SDK okumasi, ROS2 degil) gelen, HENUZ TAZE bir deger
        oldugunda guvenli: ayni bilgiyi ekstra bir ag turu ile tekrar
        sormaktan kacinir (bkz. replay_sequence - art arda adimlar arasinda
        "her adimda bekleme" sikayetine yol acan gereksiz turlardan biri
        buydu). Sadece bir replay dizisinin ILK adiminda (ya da known_
        joints_deg verilmedigi her durumda) taze SDK okumasi yapilir."""
        from moveit.core.robot_state import RobotState
        if known_joints_deg is None:
            with self.sdk_lock:
                current_joints = self.sdk.GetActualJointPosDegree()[1]
        else:
            current_joints = known_joints_deg
        start_state = RobotState(self.robot_model)
        start_state.set_joint_group_positions(self.group_name, [math.radians(v) for v in current_joints])
        start_state.update()
        self._planning_component.set_start_state(robot_state=start_state)

    def predict_tcp_pose(self, joint_degrees):
        """Robotu HIC HAREKET ETTIRMEDEN, verilen eklem acilarinin (FK ile)
        hangi TCP pozuna (X,Y,Z,RX,RY,RZ - SDK'nin get_current_state()'i ile
        AYNI birim/cerceve) karsilik geldigini hesaplar. Joint modunda
        hareket etmeden once 'nereye gidecegim' onizlemesi icin - bkz.
        jog_arm. compute_tcp_offset'in TERSI yonde ayni donusumu kullanir:
        orada (joints, tcp_pose) -> offset cikariliyordu, burada (joints,
        SABIT offset) -> tcp_pose hesaplaniyor."""
        from moveit.core.robot_state import RobotState
        state = RobotState(self.robot_model)
        state.set_joint_group_positions(self.group_name, [math.radians(v) for v in joint_degrees])
        state.update()
        wrist3_pose = state.get_pose(TIP_LINK)
        wx, wy, wz = wrist3_pose.position.x, wrist3_pose.position.y, wrist3_pose.position.z
        qx, qy, qz, qw = (wrist3_pose.orientation.x, wrist3_pose.orientation.y,
                           wrist3_pose.orientation.z, wrist3_pose.orientation.w)
        local_x_in_global = [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qz * qw), 2 * (qx * qz - qy * qw)]
        local_y_in_global = [2 * (qx * qy - qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + qx * qw)]
        local_z_in_global = [2 * (qx * qz + qy * qw), 2 * (qy * qz - qx * qw), 1 - 2 * (qx * qx + qy * qy)]
        ox, oy, oz = self.tcp_offset_m
        offset_global = [
            ox * local_x_in_global[i] + oy * local_y_in_global[i] + oz * local_z_in_global[i]
            for i in range(3)
        ]
        rx, ry, rz = quat_to_euler_deg(qx, qy, qz, qw)
        return {
            "x": (wx + offset_global[0]) * 1000.0,
            "y": (wy + offset_global[1]) * 1000.0,
            "z": (wz + offset_global[2]) * 1000.0,
            "rx": rx, "ry": ry, "rz": rz,
        }

    def move_to_joints(self, joint_degrees, confirm=False, speed_pct=None, accel_pct=None,
                       known_start_joints_deg=None, prefer_lin=False):
        from moveit.core.robot_state import RobotState
        self._set_start_state_from_sdk(known_joints_deg=known_start_joints_deg)
        target_state = RobotState(self.robot_model)
        target_state.set_joint_group_positions(self.group_name, [math.radians(v) for v in joint_degrees])
        self._planning_component.set_goal_state(robot_state=target_state)
        return self._plan_and_execute(confirm, speed_pct=speed_pct, accel_pct=accel_pct,
                                      prefer_lin=prefer_lin)

    def move_relative_joints(self, joint_deltas_deg, confirm=False, speed_pct=None, accel_pct=None):
        with self.sdk_lock:
            current = self.sdk.GetActualJointPosDegree()[1]
        target = [current[i] + joint_deltas_deg[i] for i in range(6)]
        return self.move_to_joints(target, confirm=confirm, speed_pct=speed_pct, accel_pct=accel_pct)

    def move_to_joints_nearest(self, joint_degrees, confirm=False, speed_pct=None, accel_pct=None):
        """MUTLAK eklem hedefi - standart prosedur (bkz. _ik_adaylariyla_git).

        Verilen acilarin TCP pozu robotun kendi ileri kinematigiyle (SDK
        GetForwardKin) hesaplanir ve o poza en yakin kol durusuyla gidilir.
        Verilen acilar da aday listesindedir: en yakin onlarsa aynen onlara
        gidilir; degilse kol AYNI TCP pozuna FARKLI eklem acilariyla varir.

        Eklem jog'u (move_relative_joints) ve kayitli hareketler
        (replay_sequence) bunu KULLANMAZ - move_to_joints ile verilen acilara
        gider.
        """
        with self.sdk_lock:
            hata, tcp = self.sdk.GetForwardKin([float(v) for v in joint_degrees])
        if hata != 0 or tcp is None:
            return {"success": False, "reason": "fk_failed",
                    "details": (f"verilen eklem acilarinin TCP pozu hesaplanamadi (SDK hata "
                                f"kodu {hata}). Robot hareket etmedi.")}
        return self._ik_adaylariyla_git(*tcp, confirm=confirm, prefer_lin=False,
                                        speed_pct=speed_pct, accel_pct=accel_pct,
                                        ek_adaylar=[joint_degrees])

    def _goal_pose_from_tcp(self, tx, ty, tz, trx, trye, trz):
        from geometry_msgs.msg import PoseStamped
        quat = euler_deg_to_quat(trx, trye, trz)
        qx, qy, qz, qw = quat
        # Hedef yonelimin yerel X/Y/Z eksenlerinin global karsiligi - tam 3B
        # offset vektorunu (compute_tcp_offset) bu hedef yonelime gore global
        # cerceveye tasimak icin. Eskiden sadece Z ekseni kullaniliyordu; bu,
        # olculen offset'te (ox,oy) yanal bir bilesen VARSA onu yok sayiyordu.
        local_x_in_global = [
            1 - 2 * (qy * qy + qz * qz),
            2 * (qx * qy + qz * qw),
            2 * (qx * qz - qy * qw),
        ]
        local_y_in_global = [
            2 * (qx * qy - qz * qw),
            1 - 2 * (qx * qx + qz * qz),
            2 * (qy * qz + qx * qw),
        ]
        local_z_in_global = [
            2 * (qx * qz + qy * qw),
            2 * (qy * qz - qx * qw),
            1 - 2 * (qx * qx + qy * qy),
        ]
        ox, oy, oz = self.tcp_offset_m
        offset_global = [
            ox * local_x_in_global[i] + oy * local_y_in_global[i] + oz * local_z_in_global[i]
            for i in range(3)
        ]
        wrist3_x = tx / 1000.0 - offset_global[0]
        wrist3_y = ty / 1000.0 - offset_global[1]
        wrist3_z = tz / 1000.0 - offset_global[2]

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = wrist3_x
        pose_goal.pose.position.y = wrist3_y
        pose_goal.pose.position.z = wrist3_z
        pose_goal.pose.orientation.x = quat[0]
        pose_goal.pose.orientation.y = quat[1]
        pose_goal.pose.orientation.z = quat[2]
        pose_goal.pose.orientation.w = quat[3]
        return pose_goal

    def move_to_pose_base(self, x, y, z, rx, ry, rz, confirm=False, prefer_lin=False,
                          speed_pct=None, accel_pct=None):
        """Mutlak TCP hedefi (base cercevesi, mm/derece). Standart prosedur:
        en yakin kol durusu (bkz. _ik_adaylariyla_git)."""
        return self._ik_adaylariyla_git(x, y, z, rx, ry, rz, confirm=confirm,
                                        prefer_lin=prefer_lin,
                                        speed_pct=speed_pct, accel_pct=accel_pct)

    def move_to_pose_tool_xyz(self, dx, dy, dz, rx, ry, rz, confirm=False,
                              speed_pct=None, accel_pct=None):
        """frame_choice='Tool': X/Y/Z mevcut Base-frame pozisyonuna eklenir (goreceli),
        RX/RY/RZ HER ZAMAN mutlak hedef olarak alinir - movepy ile ayni davranis."""
        with self.sdk_lock:
            cur = self.sdk.GetActualTCPPose()[1]
        cx, cy, cz = cur[0], cur[1], cur[2]
        # GORECELI hareket = yaklasma/geri cekilme niteligindedir, duz cizgi
        # olmali (bkz. _plan_and_execute icindeki prefer_lin aciklamasi).
        return self.move_to_pose_base(cx + dx, cy + dy, cz + dz, rx, ry, rz,
                                      confirm=confirm, prefer_lin=True,
                                      speed_pct=speed_pct, accel_pct=accel_pct)

    def move_tcp_local(self, dx, dy, dz, confirm=False, speed_pct=None, accel_pct=None):
        """TCP'nin kendi eksenlerine gore goreceli hareket; yonelim degismez."""
        with self.sdk_lock:
            cur = self.sdk.GetActualTCPPose()[1]
        cx, cy, cz, crx, cry, crz = cur[0:6]
        cqx, cqy, cqz, cqw = euler_deg_to_quat(crx, cry, crz)
        cur_x_axis = [1 - 2 * (cqy * cqy + cqz * cqz), 2 * (cqx * cqy + cqz * cqw), 2 * (cqx * cqz - cqy * cqw)]
        cur_y_axis = [2 * (cqx * cqy - cqz * cqw), 1 - 2 * (cqx * cqx + cqz * cqz), 2 * (cqy * cqz + cqx * cqw)]
        cur_z_axis = [2 * (cqx * cqz + cqy * cqw), 2 * (cqy * cqz - cqx * cqw), 1 - 2 * (cqx * cqx + cqy * cqy)]
        tx = cx + dx * cur_x_axis[0] + dy * cur_y_axis[0] + dz * cur_z_axis[0]
        ty = cy + dx * cur_x_axis[1] + dy * cur_y_axis[1] + dz * cur_z_axis[1]
        tz = cz + dx * cur_x_axis[2] + dy * cur_y_axis[2] + dz * cur_z_axis[2]
        # TCP kendi ekseninde goreceli hareket - tipik yaklasma/geri cekilme,
        # duz cizgi olmali (bkz. prefer_lin aciklamasi).
        return self._ik_adaylariyla_git(tx, ty, tz, crx, cry, crz, confirm=confirm,
                                        prefer_lin=True,
                                        speed_pct=speed_pct, accel_pct=accel_pct)

    # _plan_and_execute'un ROBOT HAREKET ETMEDEN (onay ve execute() oncesi)
    # dondugu red sebepleri. _ik_adaylariyla_git yalnizca bunlarda siradaki IK
    # adayina gecer. Basari, kullanici iptali ya da execute() SONRASI hata
    # (execution_not_confirmed) gelirse DURUR - hareket etmis bir robotla
    # baska aday denemek otomatik yeniden deneme olurdu (yasak).
    _HAREKETSIZ_RED_SEBEPLERI = frozenset(
        {"planning_failed", "floor_collision", "self_collision", "excessive_joint_motion"})

    def _ik_adaylariyla_git(self, tx, ty, tz, trx, trye, trz, confirm=False, prefer_lin=False,
                            speed_pct=None, accel_pct=None, ek_adaylar=None):
        """STANDART PROSEDUR (kullanici karari, 14 Eylul): poz ya da mutlak
        eklem hedefli TUM kol hareketleri bu yoldan gider - TCP pozuna git,
        Tool XYZ, TCP local/base jog, mutlak eklem hedefi. Hedef TCP pozuna
        EN YAKIN kol durusuyla gidilir; o planlanamazsa bir sonraki en yakin
        durus denenir. Eklem jog'u, kayitli hareketler ve pick & place bu
        yoldan GECMEZ.

        ek_adaylar: robotun IK'sina ek olarak aday listesine konacak eklem
        acilari (mutlak eklem hedefinde kullanicinin girdigi acilar).

        NEDEN: hedef poz olarak verildiginde IK'yi planlayici kendi icinde
        cozuyor; hangi durusu sectigi ancak plan ciktiktan sonra goruluyordu
        ve OMPL rastgele tohumla baska bir dala atlayabiliyordu. KDL
        (set_from_ik) da tohumla bulamazsa rastgele tohuma geciyor.

        Burada adaylari ROBOTUN KENDI IK'si veriyor: SDK GetInverseKin(type=0,
        tcp_pozu, config) - SDK belgesi: config -1 = mevcut eklem konumuna gore,
        0~7 = belirli eklem konfigurasyonuna gore. Adaylar:
          1. MoveIt modelinin eklem sinirlari disindakiler elenir (FR3 J3
             +-150 - OMPL sinir disi hedefi reddetmeyip kirpiyor, bkz.
             GUNCEL_DURUM 7).
          2. Baslangica yakinliga gore siralanir (ik_adaylarini_sirala).
          3. En yakindan baslayarak EKLEM hedefi olarak planlanir. Robot
             hareket etmeden reddedilirse (_HAREKETSIZ_RED_SEBEPLERI)
             siradakine gecilir; diger her sonucta durulur.
        prefer_lin=True ise LIN yalnizca ilk adayda denenir: LIN eklem
        hedefini FK ile poza cevirip ayni kartezyen dogruyu cizdigi icin her
        adayda ayni sonucu verir.

        EKLEM DONUSU SINIRI YOK (kullanici karari, 14 Eylul): uzak bir durus
        da planlanabilir - acil stop elin altinda.

        OLCULMEDI: GetInverseKin'in 0~7 ile gercekten farkli duruslar verdigi
        canli robotta henuz gorulmedi.
        """
        with self.sdk_lock:
            baslangic = list(self.sdk.GetActualJointPosDegree()[1])
        hedef_tcp = [float(v) for v in (tx, ty, tz, trx, trye, trz)]

        ham_adaylar = [[float(v) for v in a] for a in (ek_adaylar or [])]
        robottan = 0
        for config in (-1, 0, 1, 2, 3, 4, 5, 6, 7):
            with self.sdk_lock:
                hata, eklemler = self.sdk.GetInverseKin(0, hedef_tcp, config)
            if hata == 0 and eklemler is not None:
                ham_adaylar.append([float(v) for v in eklemler])
                robottan += 1
            else:
                print(f"  IK config={config}: cozum yok (hata kodu {hata})")

        jmg = self.robot_model.get_joint_model_group(self.group_name)
        sinir_ici = [a for a in ham_adaylar
                     if jmg.satisfies_position_bounds([math.radians(v) for v in a])]
        sirali = ik_adaylarini_sirala(baslangic, sinir_ici)
        print(f"  IK: robottan {robottan} cozum (+{len(ek_adaylar or [])} verilen), eklem "
              f"siniri icinde {len(sinir_ici)}, farkli {len(sirali)} durus.")

        if not sirali:
            return {"success": False, "reason": "ik_failed",
                    "details": (f"hedef icin kullanilabilir kol durusu yok (robottan "
                                f"{robottan} cozum geldi, toplam {len(ham_adaylar)} adaydan "
                                f"{len(sinir_ici)} tanesi eklem siniri icinde). Planlanmadi - "
                                f"robot hareket etmedi.")}

        son_sonuc = None
        for sira, (aday, fark) in enumerate(sirali, 1):
            print(f"  IK adayi {sira}/{len(sirali)}: {[round(v, 2) for v in aday]} "
                  f"(en cok donen eklem {fark:.1f} derece)")
            sonuc = self.move_to_joints(aday, confirm=confirm, known_start_joints_deg=baslangic,
                                        prefer_lin=(prefer_lin and sira == 1),
                                        speed_pct=speed_pct, accel_pct=accel_pct)
            if sonuc.get("reason") not in self._HAREKETSIZ_RED_SEBEPLERI:
                if sonuc.get("success"):
                    print(f"  IK adayi {sira} kullanildi.")
                return sonuc
            print(f"  IK adayi {sira} kullanilamadi ({sonuc['reason']}) - robot hareket etmedi.")
            son_sonuc = sonuc

        detay = son_sonuc.get("details") or son_sonuc["reason"]
        return {"success": False, "reason": son_sonuc["reason"],
                "details": (f"{len(sirali)} kol durusunun hicbiri planlanamadi; sonuncusu: "
                            f"{detay}. Robot hareket etmedi.")}

    def move_tcp_base(self, dx, dy, dz, confirm=False, speed_pct=None, accel_pct=None):
        """TCP'yi BASE (dunya) eksenlerine gore goreceli hareket ettirir;
        yonelim degismez.

        move_tcp_local'den FARKI: orada eksenler TCP ile birlikte doner, yani
        aleti egdiginde "+Z" artik yukari degil, aletin baktigi yondur. Burada
        eksenler SABIT - "+Z" robot ne durumda olursa olsun yukaridir. Masaya
        dik inip cikmak, tezgah boyunca kaydirmak gibi isler icin dogru olan
        budur.

        Base ekseninde goreceli hareket de tipik yaklasma/geri cekilme
        niteliginde oldugu icin duz cizgi tercih ediliyor (prefer_lin).
        """
        with self.sdk_lock:
            cur = self.sdk.GetActualTCPPose()[1]
        cx, cy, cz, crx, cry, crz = cur[0:6]
        # Donusum YOK - base ekseni zaten TCP pozunun ifade edildigi cerceve.
        return self._ik_adaylariyla_git(cx + dx, cy + dy, cz + dz, crx, cry, crz, confirm=confirm,
                                        prefer_lin=True,
                                        speed_pct=speed_pct, accel_pct=accel_pct)

    def shutdown(self):
        if self._shutdown_called:
            return
        self._shutdown_called = True
        self._executor.shutdown()
        self._rclpy.shutdown()
        self._executor_thread.join(timeout=2)


# ---------------------------------------------------------------------------
# Kucuk girdi yardimcilari (movepy / gripper_interactive.py ile ayni)
# ---------------------------------------------------------------------------

def ask_float(prompt, default=None, allow_back=False):
    suffix = " ('geri' ile don)" if allow_back else ""
    while True:
        raw = input(f"{prompt}" + (f" [{default}]" if default is not None else "") + f"{suffix}: ").strip()
        if allow_back and raw.lower() in BACK_WORDS:
            raise GoBack()
        if raw == "" and default is not None:
            return default
        try:
            return float(raw)
        except ValueError:
            print("  Gecersiz sayi, tekrar dene.")


def ask_int(prompt, default, lo=0, hi=100, allow_back=False):
    suffix = " ('geri' ile don)" if allow_back else ""
    while True:
        raw = input(f"{prompt} [{default}]{suffix}: ").strip()
        if allow_back and raw.lower() in BACK_WORDS:
            raise GoBack()
        if raw == "":
            return default
        try:
            val = int(raw)
        except ValueError:
            print("  Gecersiz sayi, tekrar gir.")
            continue
        if not (lo <= val <= hi):
            print(f"  {lo}-{hi} araliginda olmali, tekrar gir.")
            continue
        return val


def ask_choice(prompt, options, allow_back=False):
    print(prompt)
    for i, opt in enumerate(options, 1):
        print(f"  {i}) {opt}")
    if allow_back:
        print("  0) Geri")
    while True:
        raw = input("Secim: ").strip()
        if allow_back and (raw == "0" or raw.lower() in BACK_WORDS):
            raise GoBack()
        if raw.isdigit() and 1 <= int(raw) <= len(options):
            return int(raw)
        print("  Gecersiz secim, tekrar dene.")


def write_state_cache(arm, gripper, state, gstat):
    """konumcobot'un okuyabilmesi icin son bilinen durumu diske yazar - bkz.
    STATE_CACHE_FILE tanimi. Sessizce basarisiz olur (bu salt bilgi amacli bir
    yan-etki, movecobot'un asil islevini ASLA bozmamali)."""
    try:
        data = {
            "timestamp": time.time(),
            "robot_ip": arm.robot_ip,
            "joints_deg": state["joints_deg"],
            "tcp_pose": state["tcp_pose"],
            "gripper": gstat,
        }
        STATE_CACHE_FILE.write_text(json.dumps(data), encoding="utf-8")
    except Exception:
        pass


def print_state(arm, gripper):
    state = arm.get_current_state()
    print("\n=== Kol durumu ===")
    for name, val in zip(JOINT_NAMES, state["joints_deg"]):
        print(f"  {name.upper()}: {val:8.3f}")
    p = state["tcp_pose"]
    print(f"  TCP: X={p['x']:.3f} Y={p['y']:.3f} Z={p['z']:.3f} RX={p['rx']:.3f} RY={p['ry']:.3f} RZ={p['rz']:.3f}")
    gstat = None
    if gripper is not None:
        try:
            gstat = gripper.status()
            print(f"=== Gripper durumu === pozisyon={gstat['position_pct']} hiz={gstat['speed_pct']} "
                  f"motion_status={gstat['motion_status']}")
        except GripperError as e:
            print(f"UYARI: gripper durumu okunamadi: {e}")
    print()
    write_state_cache(arm, gripper, state, gstat)


def jog_arm(arm):
    try:
        move_type = ask_choice(
            "\nHareket tipi sec:",
            [
                "Eklem acilari (Joint)",
                "Cartesian pozisyon (X,Y,Z + RX,RY,RZ)",
                "Basit hareket (sadece X,Y,Z, TCP'ye gore goreceli)",
            ],
            allow_back=True,
        )
        if move_type == 1:
            joint_mode = ask_choice(
                "\nEklem modu sec:",
                ["Mutlak (kesin aci gir)", "Goreceli (mevcut acinin uzerine ekle/cikar)"],
                allow_back=True,
            )
            if joint_mode == 1:
                print("\nEklem acilarini derece cinsinden gir (mutlak hedef):")
                targets = [ask_float(f"  {j.upper()}", allow_back=True) for j in JOINT_NAMES]
            else:
                print("\nHer eklem icin degisim miktarini gir (derece, 0 = degistirme):")
                deltas = [ask_float(f"  {j.upper()} degisim", default=0.0, allow_back=True) for j in JOINT_NAMES]
                with arm.sdk_lock:
                    current = arm.sdk.GetActualJointPosDegree()[1]
                targets = [current[i] + deltas[i] for i in range(6)]

            # Eklem acilari 6 eksenli kolda TCP pozunu (X,Y,Z,RX,RY,RZ) FK
            # ile ZATEN tek anlamli belirler - RX/RY/RZ'yi AYRICA girdirmek
            # asiri-belirlenmis (imkansiz) bir istek olurdu. Bunun yerine,
            # hareket etmeden ONCE, girilen eklem acilarinin FK ile hangi
            # yonelime/poza karsilik geldigini burada onizliyoruz.
            predicted = arm.predict_tcp_pose(targets)
            print(f"\nFK onizleme (henuz hareket ETMEDI) - bu eklem acilariyla varilacak TCP pozu: "
                  f"X={predicted['x']:.1f} Y={predicted['y']:.1f} Z={predicted['z']:.1f} "
                  f"RX={predicted['rx']:.1f} RY={predicted['ry']:.1f} RZ={predicted['rz']:.1f}")
            if joint_mode == 1:
                # Mutlak eklem hedefi: standart prosedur (en yakin kol durusu).
                result = arm.move_to_joints_nearest(targets, confirm=True)
            else:
                result = arm.move_to_joints(targets, confirm=True)
        elif move_type == 2:
            frame_choice = ask_choice(
                "\nReferans koordinat sistemi:",
                ["Base (mutlak hedef)", "Tool (X,Y,Z goreceli; RX,RY,RZ HER ZAMAN mutlak)"],
                allow_back=True,
            )
            print("\nHedef degerleri gir:")
            x = ask_float("  X (mm)", allow_back=True)
            y = ask_float("  Y (mm)", allow_back=True)
            z = ask_float("  Z (mm)", allow_back=True)
            rx = ask_float("  RX (derece)", default=0.0, allow_back=True)
            ry = ask_float("  RY (derece)", default=0.0, allow_back=True)
            rz = ask_float("  RZ (derece)", default=0.0, allow_back=True)
            if frame_choice == 1:
                result = arm.move_to_pose_base(x, y, z, rx, ry, rz, confirm=True)
            else:
                result = arm.move_to_pose_tool_xyz(x, y, z, rx, ry, rz, confirm=True)
        else:
            print("\nTCP'nin kendi eksenlerine gore (yerel), kac mm hareket edecegini gir (0 = degistirme):")
            dx = ask_float("  X (mm)", default=0.0, allow_back=True)
            dy = ask_float("  Y (mm)", default=0.0, allow_back=True)
            dz = ask_float("  Z (mm)", default=0.0, allow_back=True)
            result = arm.move_tcp_local(dx, dy, dz, confirm=True)
    except GoBack:
        print("Geri donuldu, kol hareket ettirilmedi.")
        return

    if result["success"]:
        print("Tamamlandi.")
    elif result["reason"] == "user_cancelled":
        print("Iptal edildi, kol hareket ettirilmedi.")
    else:
        details = f" ({result['details']})" if result.get("details") else ""
        print(f"HATA: {result['reason']}{details}")


def jog_gripper(gripper, arm=None):
    """arm verilirse, ceneler acildiginda (pozisyon <=10) kavranan nesne
    modelden AYRILIR - yoksa elle acsan bile MoveIt hala nesneyi tasidigini
    sanip planlamayi gereksiz kisitlar (bkz. attach_grasped_object)."""
    print("Pozisyon (0=tam acik, 100=tam kapali) veya 'geri' ile don.")
    while True:
        raw_pos = input("Pozisyon veya geri: ").strip()
        if raw_pos.lower() in ("q", "quit", "exit") + BACK_WORDS:
            return
        try:
            position = int(raw_pos)
        except ValueError:
            print("  Gecersiz deger, sayi gir.")
            continue
        if not (0 <= position <= 100):
            print("  0-100 araliginda olmali.")
            continue
        try:
            speed = ask_int("Hiz (0-100)", 50, allow_back=True)
            force = ask_int("Kuvvet (0-100)", 50, allow_back=True)
        except GoBack:
            print("Geri donuldu, gripper hareket ettirilmedi.")
            continue
        try:
            gripper.move(position=position, speed=speed, force=force)
            if arm is not None and position <= 10:
                arm.detach_object()
            print(f"Tamamlandi. Durum: {gripper.status()}")
        except GripperError as e:
            print(f"HATA: {e}")


# ---------------------------------------------------------------------------
# Manuel (drag-teach) mod: robotu FIZIKSEL olarak elle hareket ettirebilmek.
#
# fairino_hardware_interface.cpp'nin write() fonksiyonu, ros2_control aktif
# oldugu surece HER DONGUDE (100Hz) ServoJ ile robotu son komut edilen
# pozisyonda sabit tutar - WebApp'ten "manuel mod"a gecilse bile bu ROS2
# tarafindaki servo dongusu calismaya devam ettigi icin robot elle itmeye
# direnc gosterir. Cozum: once ros2_control'un donanim bilesenini (asagida)
# 'inactive' yapip bu dongudeki StopMotion()/CloseRPC()'yi tetiklemek, SONRA
# robotu SDK'nin kendi Mode(1)+DragTeachSwitch(1) ile elle-itilebilir hale
# getirmek. Bitince ters sirayla geri alinir; ROS2 kontrolu yeniden
# aktive olurken fairino_hardware_interface.cpp'nin on_activate() fonksiyonu
# komut pozisyonunu robotun O ANKI gercek pozisyonundan okuyarak baslatir,
# yani elle tasidigin yerden ani bir sicrama olmadan devam eder.
# ---------------------------------------------------------------------------

def _respawn_ros2_control(robot_desc_name, package_name, timeout=45):
    """ros2_control_node SURECI TAMAMEN COKMUSSE (sadece 'inactive' degil,
    process yok olmussa - bkz. 'ps aux' / 'Could not contact service')
    set_hardware_component_state artik hicbir ise yaramaz, cunku muhatap
    surec yok. Bu fonksiyon, demo.launch.py'nin normalde yaptigi ile AYNI
    iki adimi (ros2_control_node + spawn_controllers.launch.py - bkz.
    moveit_configs_utils/launches.py generate_demo_launch) move_group/rviz'e
    DOKUNMADAN calistirir. Arka planda, movecobot kapansa bile ayakta
    kalacak sekilde (start_new_session) baslatilir - fairino_launch'in
    KENDI sureciyle iliskisi yoktur, bu YENI ve AYRI bir surectir."""
    launch_path = f"/tmp/movecobot_respawn_hw_{package_name}.launch.py"
    content = f'''from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder({robot_desc_name!r}, package_name={package_name!r}).to_moveit_configs()
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )
    return ld
'''
    try:
        with open(launch_path, "w") as f:
            f.write(content)
    except OSError as e:
        return False, f"gecici launch dosyasi yazilamadi: {e}"

    print("  ros2_control_node COKMUS gorunuyor - move_group/rviz'e dokunmadan yeniden baslatiliyor...")
    try:
        subprocess.Popen(
            ["ros2", "launch", launch_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except FileNotFoundError as e:
        return False, str(e)

    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(2)
        try:
            r = subprocess.run(
                ["ros2", "control", "list_hardware_components"],
                capture_output=True, text=True, timeout=8,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            continue
        if r.returncode == 0 and "active" in r.stdout:
            print("  ros2_control_node yeniden ayakta (YENI, ayri bir surec olarak - "
                  "fairino_launch terminalinde gorunmeyecek).")
            return True, ""
    return False, f"ros2_control_node yeniden baslatildi ama {timeout}sn icinde saglikli duruma gelmedi"


def _ros2_control_set_state(state, robot_desc_name=None, package_name=None, timeout=15):
    def attempt():
        try:
            r = subprocess.run(
                ["ros2", "control", "set_hardware_component_state", DRAG_TEACH_HW_COMPONENT, state],
                capture_output=True, text=True, timeout=timeout,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            return False, str(e)
        if r.returncode != 0:
            return False, (r.stdout + r.stderr).strip()
        return True, ""

    ok, err = attempt()
    if ok:
        return True, ""
    # ros2cli'nin arka planda calisan "daemon" sureci zaman zaman bozuk bir
    # rclpy durumuna takiliyor (xmlrpc.client.Fault: "!rclpy.ok()") - bu
    # robotla degil, salt CLI'nin yerel kesif yardimcisiyla ilgili. Daemon'i
    # bir kere yeniden baslatip tekrar denemek bunu genelde kendiliginden
    # duzeltiyor; bu robotu/ROS2 kontrolunu ETKILEMEZ.
    if "rclpy.ok" in err or "daemon" in err.lower():
        print("  (ros2cli daemon bozuk gorunuyor, yeniden baslatilip tekrar deneniyor...)")
        subprocess.run(["ros2", "daemon", "stop"], capture_output=True, text=True, timeout=10)
        ok, err = attempt()
        if ok:
            return True, ""

    # Hala basarisizsa VE "active" isteniyorsa, servise ULASILAMIYOR olabilir
    # (ros2_control_node surecinin TAMAMEN COKTUGU durum - bkz. docstring).
    # Bu durumda state degistirmek anlamsiz, sureci sifirdan baslatmayi dene.
    if state == "active" and robot_desc_name and package_name:
        ok2, err2 = _respawn_ros2_control(robot_desc_name, package_name)
        if ok2:
            return True, ""
        return False, f"{err}\n  Yeniden baslatma da basarisiz: {err2}"

    return False, err


def enter_drag_teach(arm, gripper):
    """ROS2 kontrol dongusunu durdurur, robotu SDK uzerinden drag-teach
    moduna alir, gripper'i de resetleyip (varsa) tutma torkunu keser.

    KRITIK: movecobot'un kendi arm.sdk baglantisi (Robot.RPC) ROS2'nin
    yeniden baglanabilmesi icin TAMAMEN KAPATILIP (CloseRPC - port 20004
    realtime soketi dahil) sonra manuel mod suresince GECICI, AYRI bir
    baglanti ile yeniden acilir. Aksi halde ros2_control_node'un
    on_activate()'i (fairino_hardware_interface.cpp) robota yeniden
    baglanmaya calisirken movecobot'un ACIK KALAN baglantisiyla catisip
    tamamen kilitleniyor (ilk denemede tam olarak bu oldu: 'ros2 control
    set_hardware_component_state ... active' 20sn'de zaman asimina ugradi,
    robot da bu yuzden WebApp'ten otomatik moda hic donmedi).

    BASTAN SONA arm.sdk_lock tutuluyor - movecobot artik gripper GUI'yi arka
    plan thread'inde calistirabildigi icin, o thread'in arm.sdk KAPANIP
    YENIDEN ACILIRKEN tam ortasinda eski/gecersiz baglantiya komut gondermeye
    calismasi (ya da yeni baglantiyla catisma) boylece engellenir - GUI'nin
    o threadi bu blok bitene kadar sadece bekler, hata almaz."""
    with arm.sdk_lock:
        print("Kendi robot baglantimiz kapatiliyor (ROS2'nin durabilmesi icin)...")
        try:
            arm.sdk.CloseRPC()
        except Exception as e:
            print(f"UYARI: baglanti kapatilirken hata (yine de devam ediliyor): {e}")
        time.sleep(0.5)

        print("ROS2 kontrol dongusu gecici olarak durduruluyor (robot sabit tutulmayacak)...")
        ok, err = _ros2_control_set_state("inactive")
        if not ok:
            print(f"HATA: ROS2 kontrol dongusu durdurulamadi, manuel moda gecilemiyor:\n  {err}")
            print("(fairino_launch'in calisir durumda oldugundan emin ol.) Kendi baglantimiz yeniden aciliyor...")
            arm.sdk = Robot.RPC(arm.robot_ip)
            gripper.robot = arm.sdk
            wait_for_live_joint_reading(arm.sdk, arm.sdk_lock)
            return False

        print("Robotla manuel mod icin gecici bir baglanti kuruluyor...")
        try:
            arm.sdk = Robot.RPC(arm.robot_ip)
            gripper.robot = arm.sdk
            # bkz. wait_for_live_joint_reading - sabit bekleme yerine
            # gercek veri gelene kadar tekrar tekrar okuyup dogruluyoruz.
            wait_for_live_joint_reading(arm.sdk, arm.sdk_lock)
        except Exception as e:
            print(f"HATA: robota baglanilamadi ({e}). ROS2 kontrolu geri aciliyor...")
            _ros2_control_set_state("active", arm.robot_desc_name, arm.package_name)
            return False

        try:
            # DIKKAT: Mode()/DragTeachSwitch() robot komutu REDDEDERSE bile
            # exception FIRLATMAZ - sadece bir hata kodu doner (err=0 basarili).
            # Sadece "except" ile kontrol etmek bu red'i sessizce yutuyordu -
            # WebApp hala Manuel modda goruntuluyor olmasinin sebebi muhtemelen
            # buydu. Simdi donus kodu ACIKCA kontrol ediliyor.
            err = arm.sdk.Mode(1)
            if err != 0:
                raise RuntimeError(f"Mode(1) robot tarafindan reddedildi, err={err}")
            time.sleep(0.3)
            err = arm.sdk.DragTeachSwitch(1)
            if err != 0:
                raise RuntimeError(f"DragTeachSwitch(1) robot tarafindan reddedildi, err={err}")
        except Exception as e:
            print(f"HATA: robot drag-teach moduna alinamadi ({e}). Geri aliniyor...")
            exit_drag_teach(arm, gripper)
            return False

        print("Gripper serbest birakiliyor (tutma torku kesiliyor)...")
        try:
            gripper.deactivate()
            print("Gripper serbest - ceneleri elle acip kapatabilirsin.")
        except GripperError as e:
            print(f"UYARI: gripper serbest birakilamadi ({e}). Jodell bunu desteklemiyor olabilir; "
                  "gripper mevcut pozisyonunda kalacak, elle degistiremeyebilirsin ama okunabilir.")

        print("\nMANUEL MOD ACIK: robotu (ve varsa gripper'i) simdi elinle serbestce hareket ettirebilirsin.")
        return True


def exit_drag_teach(arm, gripper):
    """enter_drag_teach()'in tersi (ters sirayla): gripper'i yeniden aktive
    eder, drag-teach'i kapatir, GECICI baglantiyi kapatir (ROS2'nin
    baglanabilmesi icin), ROS2 kontrol dongusunu yeniden aktive eder, EN SON
    movecobot'un kendi kalici baglantisini tekrar acar.

    bkz. enter_drag_teach - ayni sebeple bastan sona arm.sdk_lock tutuluyor."""
    with arm.sdk_lock:
        print("Gripper yeniden aktive ediliyor...")
        try:
            gripper.activate(force=True)
        except GripperError as e:
            print(f"UYARI: gripper yeniden aktive edilemedi ({e}). "
                  "Gripper hareketleri calismayabilir - gerekirse movecobot'u yeniden baslat.")

        try:
            err = arm.sdk.DragTeachSwitch(0)
            if err != 0:
                print(f"UYARI: DragTeachSwitch(0) robot tarafindan reddedildi, err={err}")
            time.sleep(0.3)
            err = arm.sdk.Mode(0)
            if err != 0:
                print(f"UYARI: Mode(0) robot tarafindan reddedildi, err={err} - "
                      "robot hala MANUEL modda kalmis olabilir! WebApp'ten kontrol et.")
        except Exception as e:
            print(f"UYARI: drag-teach kapatilirken hata: {e}")

        print("Gecici baglantimiz kapatiliyor (ROS2'nin yeniden baglanabilmesi icin)...")
        try:
            arm.sdk.CloseRPC()
        except Exception as e:
            print(f"UYARI: baglanti kapatilirken hata (yine de devam ediliyor): {e}")
        time.sleep(0.5)

        print("ROS2 kontrol dongusu yeniden baslatiliyor...")
        ok, err = _ros2_control_set_state("active", arm.robot_desc_name, arm.package_name)
        if not ok:
            print(f"HATA: ROS2 kontrolu geri acilamadi:\n  {err}")
            print("Jog/replay calismayabilir - gerekirse fairino_launch'i yeniden baslat.")
        else:
            print("MANUEL MOD KAPALI: ROS2 kontrolu geri geldi, jog/replay tekrar kullanilabilir.")

        print("Kendi baglantimiz yeniden kuruluyor...")
        try:
            arm.sdk = Robot.RPC(arm.robot_ip)
            gripper.robot = arm.sdk
            wait_for_live_joint_reading(arm.sdk, arm.sdk_lock)  # bkz. enter_drag_teach
        except Exception as e:
            print(f"HATA: robota yeniden baglanilamadi ({e}). movecobot'u yeniden baslatman gerekebilir.")


def drag_teach_clear_fault(arm):
    """Manuel mod SIRASINDA (ör. elle hizli hareket ettirince 'hiz limiti
    asildi' gibi) robot fault verirse, bunu AYRI bir terminalde 'clearfault'
    ile temizlemeye CALISMA - o, ros2_control'u YENIDEN AKTIVE ediyor, ama
    biz hala Mode(1)+DragTeachSwitch(1) ile manuel moddayken bu iki ayri
    kontrol mekanizmasinin (ros2_control'un 100Hz ServoJ dongusu ile
    movecobot'un bu GECICI drag-teach baglantisi) CATISMASINA yol aciyor -
    robot tekrar elle-itilemez hale geliyor ya da hata hemen geri geliyor.
    Bunun yerine: ros2_control'a HIC DOKUNMADAN (zaten inaktif, oyle kalacak),
    SADECE bu ayni (gecici) arm.sdk baglantisi uzerinden ResetAllError() +
    Mode(1)/DragTeachSwitch(1)'i yeniden assert ederiz - hicbir ayri surece
    gerek yok, kayit ilerlemesi (steps) de zaten hic etkilenmiyor."""
    print("\nWebApp'te 'Clear'a bas, sonra buraya donup Enter'a bas "
          "(ros2_control'a DOKUNULMUYOR, sadece bu manuel mod baglantisi kullanilacak)...")
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print("Iptal edildi, manuel modda kalindi.")
        return False

    with arm.sdk_lock:
        try:
            arm.sdk.ResetAllError()
        except Exception as e:
            print(f"UYARI: ResetAllError basarisiz ({e}), yine de devam ediliyor...")
        time.sleep(0.3)
        try:
            err = arm.sdk.Mode(1)
            if err != 0:
                print(f"HATA: Mode(1) reddedildi, err={err}. Robot hala manuel modda olmayabilir.")
                return False
            time.sleep(0.3)
            err = arm.sdk.DragTeachSwitch(1)
            if err != 0:
                print(f"HATA: DragTeachSwitch(1) reddedildi, err={err}. Robot hala manuel modda olmayabilir.")
                return False
        except Exception as e:
            print(f"HATA: manuel mod yeniden kurulamadi: {e}")
            return False

    print("Hata temizlendi, manuel mod yeniden acik. Elle hareket ettirebilirsin.")
    return True


def drag_teach_add_steps(arm, gripper, steps, name, defaults):
    """Robotu (ve gripper'i) elle hareket ettirip 'adim ekle' ile steps
    listesine (yerinde, in-place) pozisyon ekleme dongusu. Cikista - normal
    ya da Ctrl+C/hata ile - ROS2 kontrolunu ve gripper'i MUTLAKA geri
    acar/aktive eder (finally). name: sadece her adimdan sonra save_draft()
    icin (bkz. DRAFT_FILE). defaults: adim-ekleme yardimcilariyla (bkz.
    add_move_step/add_grip_step/add_wait_step) PAYLASILAN, yerinde
    (in-place) guncellenen son-kullanilan-degerler sozlugu - boylece manuel
    moddan cikip jog moduna donulse bile hiz/kuvvet degerleri korunur."""
    if not enter_drag_teach(arm, gripper):
        return
    try:
        manuel_menu = """
=== Manuel (drag-teach) menusu ===
  [a] Su anki (elle getirilen) pozu adim olarak ekle (gripper sabit pozisyona gider)
  [k] Su anki poz + gripper'i KAVRAT, bunu adim olarak ekle (sabit pozisyon degil)
  [d] Su anki durumu goster
  [l] Eklenen adimlari listele
  [w] Bekleme adimi ekle (saniye cinsinden)
  [x] Bir adimi sil
  [p] Bir adimin hiz/ivme/gripper parametrelerini duzenle (silmeden)
  [c] Robot fault verdiyse (ör. 'hiz limiti asildi'): WebApp'ten Clear'a bas,
      sonra buraya donup bunu sec - launch'i durdurmana/movecobot'u
      yeniden baslatmana GEREK YOK, kayittaki adimlar korunur
  [b] Manuel moddan cik (ROS2 kontrolunu geri devret)
"""
        print(manuel_menu)
        while True:
            choice = input(f"[manuel mod, {len(steps)} adim] Secim (a/k/d/l/w/x/p/c/b): ").strip().lower()
            if choice == "c":
                drag_teach_clear_fault(arm)
            elif choice == "a":
                add_move_step(arm, gripper, steps, name, defaults)
            elif choice == "k":
                add_grip_step(arm, gripper, steps, name, defaults)
            elif choice == "d":
                print_state(arm, gripper)
            elif choice == "l":
                list_steps(steps)
            elif choice == "w":
                add_wait_step(steps, name, defaults)
            elif choice == "x":
                delete_step(steps, name)
            elif choice == "p":
                edit_step_params(steps, name)
            elif choice in BACK_WORDS:
                return
            else:
                print("  Gecersiz secim.")
                continue
            # bkz. _run_step_editor - her komuttan sonra BULUNULAN menuye
            # don, ust menuye degil: robotu/gripper'i etkileyen komutlarda
            # once guncel konumu goster, sonra bu menuyu tekrar bas.
            if choice in ("a", "k", "c"):
                print_state(arm, gripper)
            print(manuel_menu)
    finally:
        exit_drag_teach(arm, gripper)


# ---------------------------------------------------------------------------
# Kayitli hareketler: JSON depo + ~/.bashrc alias senkronizasyonu
# ---------------------------------------------------------------------------

def load_moves():
    if MOVES_FILE.exists():
        return json.loads(MOVES_FILE.read_text(encoding="utf-8"))
    return {}


def save_moves(moves):
    MOVES_FILE.write_text(json.dumps(moves, indent=2, ensure_ascii=False), encoding="utf-8")


def save_draft(name, steps):
    """Hareket kaydi sirasinda her adimdan sonra cagirilir - movecobot
    beklenmedik sekilde kapanirsa/yeniden baslatilirsa ilerlemenin
    kaybolmamasi icin. Sessizce basarisiz olur (bu salt guvenlik amacli bir
    yedekleme, ana akisi asla bozmamali)."""
    try:
        DRAFT_FILE.write_text(
            json.dumps({"name": name, "steps": steps}, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
    except OSError:
        pass


def load_draft():
    if not DRAFT_FILE.exists():
        return None
    try:
        data = json.loads(DRAFT_FILE.read_text(encoding="utf-8"))
        if data.get("steps"):
            return data
    except (json.JSONDecodeError, OSError):
        pass
    return None


def clear_draft():
    DRAFT_FILE.unlink(missing_ok=True)


def sync_bashrc_aliases(moves):
    """~/.bashrc icindeki, movecobot'un kendi yonettigi bloktaki aliaslari
    JSON deponun tam bir yansimasi olacak sekilde yeniden yazar."""
    lines = [ALIAS_MARK_START]
    for name in sorted(moves):
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/ws_moveit2/install/setup.bash && "
            "source ~/ros2_ws/install/setup.bash && "
            f"python3 ~/fairino/client/movecobot.py --replay {name}"
        )
        lines.append(f"alias {name}='{cmd}'")
    lines.append(ALIAS_MARK_END)
    block = "\n".join(lines)

    text = BASHRC.read_text(encoding="utf-8") if BASHRC.exists() else ""
    if ALIAS_MARK_START in text and ALIAS_MARK_END in text:
        pre = text.split(ALIAS_MARK_START)[0]
        post = text.split(ALIAS_MARK_END)[1]
        new_text = pre.rstrip("\n") + "\n\n" + block + post
    else:
        sep = "" if (not text or text.endswith("\n")) else "\n"
        new_text = text + sep + "\n" + block + "\n"
    BASHRC.write_text(new_text, encoding="utf-8")


def valid_move_name(name):
    if not NAME_RE.match(name):
        return False, "isim harfle baslamali, sadece harf/rakam/alt cizgi icerebilir."
    if name in RESERVED_NAMES:
        return False, f"'{name}' zaten baska bir alias tarafindan kullaniliyor."
    return True, ""


# ---------------------------------------------------------------------------
# Adim ekleme/duzenleme yardimcilari - hem "yeni hareket kaydet" (record_new_
# move) hem "manuel mod" (drag_teach_add_steps) hem de "kayitli hareketi
# duzenle" (edit_saved_move) AYNI bu fonksiyonlari kullanir - eskiden bu
# mantik iki yerde (jog kaydi + manuel mod) neredeyse birebir kopyaydi, bir
# yerde yapilan duzeltme digerine tasinmayabiliyordu (nitekim oldu - grip()
# deaktif gripper'da no-op olma hatasi once tek kopyada fark edilmisti).
# ---------------------------------------------------------------------------

def init_step_defaults(arm, steps):
    """steps bos ya da resume edilen bir taslak/duzenleme olabilir - varsa
    SON 'move' adimindaki degerlerden devam eder (kullanici tekrar tekrar
    ayni hiz/kuvveti girmek zorunda kalmasin diye), yoksa oturumun
    hizina/ivmesine ve 50/50 gripper varsayilanina duser."""
    last_move = next((s for s in reversed(steps) if s.get("step_type", "move") == "move"), None)
    if last_move:
        return {
            "g_speed": last_move.get("gripper_speed", 50),
            "g_force": last_move.get("gripper_force", 50),
            "speed_pct": last_move.get("speed_pct", arm.speed_pct),
            "accel_pct": last_move.get("accel_pct", arm.accel_pct),
            "wait_s": 1.0,
        }
    return {"g_speed": 50, "g_force": 50, "speed_pct": arm.speed_pct, "accel_pct": arm.accel_pct, "wait_s": 1.0}


def format_step_line(i, step):
    if step.get("step_type", "move") == "wait":
        return f"  #{i:<3} [bekle]   {step.get('duration_s', 0):.1f} sn"
    joints_str = ",".join(f"{v:.0f}" for v in step.get("joints_deg", [0] * 6))
    if step.get("gripper_action", "position") == "grip":
        g_str = f"KAVRA (hiz=%{step.get('gripper_speed')} kuvvet=%{step.get('gripper_force')})"
    elif step.get("gripper_action") == "none":
        g_str = "DOKUNULMAZ (ne tutuyorsa oyle kalir)"
    else:
        g_str = f"poz={step.get('gripper_position')} (hiz=%{step.get('gripper_speed')} kuvvet=%{step.get('gripper_force')})"
    speed_pct, accel_pct = step.get("speed_pct"), step.get("accel_pct")
    speed_str = f"%{speed_pct:.0f}" if speed_pct is not None else "oturum"
    accel_str = f"%{accel_pct:.0f}" if accel_pct is not None else "oturum"
    return (f"  #{i:<3} [hareket] eklem=[{joints_str}]  gripper={g_str}  "
            f"hiz={speed_str} ivme={accel_str}")


def list_steps(steps):
    if not steps:
        print("\nHenuz adim eklenmedi.")
        return
    print(f"\n=== Eklenen adimlar ({len(steps)}) ===")
    for i, step in enumerate(steps, 1):
        print(format_step_line(i, step))
    print()


def delete_step(steps, name):
    if not steps:
        print("\nSilinecek adim yok.")
        return
    list_steps(steps)
    raw = input("Silinecek adim numarasi ('son' = en sonuncusu, iptal icin 'geri' veya bos birak): ").strip().lower()
    if not raw or raw in BACK_WORDS:
        print("Iptal edildi, hicbir adim silinmedi.")
        return
    if raw in ("son", "last"):
        idx = len(steps)
    elif raw.isdigit():
        idx = int(raw)
    else:
        print("  Gecersiz numara.")
        return
    if not (1 <= idx <= len(steps)):
        print(f"  1-{len(steps)} araliginda olmali.")
        return
    steps.pop(idx - 1)
    save_draft(name, steps)
    print(f"Adim #{idx} silindi. Kalan adim sayisi: {len(steps)}.")


def edit_step_params(steps, name):
    """Var olan bir adimin hiz/ivme (hareket adimlarinda) ya da bekleme
    suresini (bekleme adimlarinda), adimi SILIP YENIDEN EKLEMEDEN
    degistirir - robotu/gripper'i hic hareket ettirmez, sadece steps
    listesindeki sozlugu yerinde gunceller."""
    if not steps:
        print("\nDuzenlenecek adim yok.")
        return
    list_steps(steps)
    raw = input("Duzenlenecek adim numarasi ('son' = en sonuncusu, iptal icin 'geri' veya bos birak): ").strip().lower()
    if not raw or raw in BACK_WORDS:
        print("Iptal edildi, hicbir adim degistirilmedi.")
        return
    if raw in ("son", "last"):
        idx = len(steps)
    elif raw.isdigit():
        idx = int(raw)
    else:
        print("  Gecersiz numara.")
        return
    if not (1 <= idx <= len(steps)):
        print(f"  1-{len(steps)} araliginda olmali.")
        return

    step = steps[idx - 1]
    if step.get("step_type", "move") == "wait":
        try:
            duration = ask_float("Yeni bekleme suresi (saniye)", default=step.get("duration_s", 1.0), allow_back=True)
        except GoBack:
            print("Geri donuldu, adim degistirilmedi.")
            return
        if duration < 0:
            print("  Negatif olamaz, adim degistirilmedi.")
            return
        step["duration_s"] = duration
        print(f"Adim #{idx} guncellendi (bekle {duration:.1f} sn).")
    else:
        try:
            g_speed = ask_int("Gripper hizi %", step.get("gripper_speed", 50), allow_back=True)
            g_force = ask_int("Gripper kuvveti %", step.get("gripper_force", 50), allow_back=True)
            speed_pct = ask_float("Kol hizi %", default=step.get("speed_pct", 30.0), allow_back=True)
            accel_pct = ask_float("Kol ivmesi %", default=step.get("accel_pct", 30.0), allow_back=True)
        except GoBack:
            print("Geri donuldu, adim degistirilmedi.")
            return
        step["gripper_speed"] = g_speed
        step["gripper_force"] = g_force
        step["speed_pct"] = speed_pct
        step["accel_pct"] = accel_pct
        print(f"Adim #{idx} guncellendi (hiz=%{speed_pct:.0f}, ivme=%{accel_pct:.0f}, "
              f"gripper hiz=%{g_speed}, gripper kuvvet=%{g_force}).")
    save_draft(name, steps)


def add_move_step(arm, gripper, steps, name, defaults):
    """Su anki poz + gripper'i SABIT bir pozisyona ayarlayan adim ekler
    ([a] secenegi). defaults sozlugu bu ve diger adim-ekleme yardimcilari
    arasinda PAYLASILIR (in-place guncellenir) - boylece jog/manuel mod
    arasinda gidip gelince bile son girilen hiz/kuvvet degerleri korunur."""
    state = arm.get_current_state()
    try:
        g_pos = gripper.status()["position_pct"]
    except GripperError:
        g_pos = 0

    # GRIPPER'A DOKUNULSUN MU?
    # Bu adim gripper'in O ANKI pozisyonunu MUTLAK hedef olarak kaydeder.
    # Nesne TASIYAN bir adimda bu YANLIS olur: kavramadan sonra okunan
    # pozisyon (ör. 91) o nesnenin KALINLIGIDIR. Ayni hareket baska kalinlikta
    # bir nesneyle oynatilinca 91 ya nesneyi ezer ya da bosta birakip
    # dusurur. Dogrusu "kavradigina tutunmaya devam et" demektir - yani
    # gripper'a hic dokunmamak.
    # Bir onceki adim KAVRA ise varsayilan otomatik olarak "dokunma" olur.
    _prev_is_grip = bool(steps) and steps[-1].get("gripper_action") == "grip"
    _default_touch = "h" if _prev_is_grip else "e"
    if _prev_is_grip:
        print("  Not: bir onceki adim KAVRA - bu adimda gripper'a dokunmamak (tutmaya")
        print("       devam etmek) genelde dogrusudur, yoksa nesnenin kalinligina bagimli")
        print(f"       sabit bir pozisyon ({g_pos}) kaydedilir.")
    _ans = input(f"Bu adimda gripper'a dokunulsun mu? (e=poz={g_pos} kaydet / "
                 f"h=dokunma) [{_default_touch}]: ").strip().lower()
    if _ans == "":
        _ans = _default_touch
    _gripper_action = "position" if _ans == "e" else "none"

    try:
        g_speed = ask_int("Bu adimda gripper hizi %", defaults["g_speed"], allow_back=True)
        g_force = ask_int("Bu adimda gripper kuvveti %", defaults["g_force"], allow_back=True)
        speed_pct = ask_float("Bu adim oynatilirken kol hizi %", default=defaults["speed_pct"], allow_back=True)
        accel_pct = ask_float("Bu adim oynatilirken kol ivmesi %", default=defaults["accel_pct"], allow_back=True)
    except GoBack:
        print("Geri donuldu, adim eklenmedi.")
        return
    defaults["g_speed"], defaults["g_force"] = g_speed, g_force
    defaults["speed_pct"], defaults["accel_pct"] = speed_pct, accel_pct
    steps.append({
        "step_type": "move",
        "joints_deg": state["joints_deg"],
        "tcp_pose": state["tcp_pose"],
        "gripper_action": _gripper_action,
        "gripper_position": g_pos,
        "gripper_speed": g_speed,
        "gripper_force": g_force,
        "speed_pct": speed_pct,
        "accel_pct": accel_pct,
    })
    _g_desc = f"poz={g_pos}" if _gripper_action == "position" else "dokunulmaz"
    print(f"Adim #{len(steps)} eklendi. (eklemler={[round(v, 1) for v in state['joints_deg']]}, "
          f"gripper={_g_desc}, hiz=%{speed_pct:.0f}, ivme=%{accel_pct:.0f})")
    save_draft(name, steps)


def add_grip_step(arm, gripper, steps, name, defaults):
    """Su anki poz + gripper'i GEREKTIGI KADAR kapatip kavratan (adaptif)
    adim ekler ([k] secenegi) - bkz. GripperController.grip()."""
    state = arm.get_current_state()
    try:
        g_speed = ask_int("Kavrama hizi %", defaults["g_speed"], allow_back=True)
        g_force = ask_int("Kavrama kuvveti %", defaults["g_force"], allow_back=True)
        speed_pct = ask_float("Bu adim oynatilirken kol hizi %", default=defaults["speed_pct"], allow_back=True)
        accel_pct = ask_float("Bu adim oynatilirken kol ivmesi %", default=defaults["accel_pct"], allow_back=True)
    except GoBack:
        print("Geri donuldu, adim eklenmedi.")
        return
    defaults["g_speed"], defaults["g_force"] = g_speed, g_force
    defaults["speed_pct"], defaults["accel_pct"] = speed_pct, accel_pct
    try:
        result = gripper.grip(speed=g_speed, force=g_force)
    except GripperError as e:
        print(f"HATA: kavrama basarisiz oldu, adim eklenmedi: {e}")
        return
    if result["gripped"]:
        print(f"Nesne yakalandi (pozisyon={result['position_pct']}).")
    else:
        print(f"UYARI: kapandi ama nesne algilanmadi (pozisyon={result['position_pct']}) "
              f"- yine de adim olarak eklensin mi?")
        if input("Ekle (e/h): ").strip().lower() != "e":
            print("Adim eklenmedi.")
            return

    # NESNE BOYUTU - kamera olmadigi icin BASINCTAN cikariyoruz.
    # Gripper kuvvet geri bildirimiyle nesneye deyince durur; durdugu
    # aciklik = nesnenin CENELER ARASI genisligi. Derinlik ve yukseklik
    # fiziksel olarak algilanamaz (gripper nesnenin cenelerin disina ne
    # kadar tastigini goremez), o ikisi sorulur.
    _derived_w_mm = None
    if result["gripped"] and result["position_pct"] > 0:
        _derived_w_mm = (100 - result["position_pct"]) / 100.0 * GRIPPER_STROKE_M * 1000
        print(f"  Basinctan turetilen nesne genisligi: ~{_derived_w_mm:.0f} mm "
              f"(cene acikligi; yanlissa asagida duzeltebilirsin)")
    try:
        obj_w = ask_float("Nesne genisligi (mm)",
                          default=(_derived_w_mm if _derived_w_mm else defaults.get("obj_w", 30.0)),
                          allow_back=True)
        obj_d = ask_float("Nesne derinligi (mm)", default=defaults.get("obj_d", 30.0), allow_back=True)
        obj_h = ask_float("Nesne yuksekligi (mm)", default=defaults.get("obj_h", 60.0), allow_back=True)
    except GoBack:
        print("Geri donuldu, adim eklenmedi.")
        return
    defaults["obj_w"], defaults["obj_d"], defaults["obj_h"] = obj_w, obj_d, obj_h

    # Kavradik - nesneyi HEMEN modele iliştir ki bundan sonraki her planlama
    # (kayit sirasindaki jog dahil) tasidigimiz nesneyi hesaba katsin.
    arm.attach_grasped_object(obj_w / 1000.0, obj_d / 1000.0, obj_h / 1000.0)

    steps.append({
        "step_type": "move",
        "joints_deg": state["joints_deg"],
        "tcp_pose": state["tcp_pose"],
        "gripper_action": "grip",
        "gripper_position": result["position_pct"],
        "object_size_mm": [obj_w, obj_d, obj_h],
        "gripper_speed": g_speed,
        "gripper_force": g_force,
        "speed_pct": speed_pct,
        "accel_pct": accel_pct,
    })
    print(f"Adim #{len(steps)} eklendi (kavrama eylemi, hiz=%{speed_pct:.0f}, ivme=%{accel_pct:.0f}).")
    save_draft(name, steps)


def add_wait_step(steps, name, defaults):
    """Robotu/gripper'i hareket ettirmeyen, sadece oynatma sirasinda N
    saniye bekleyen bir adim ekler ([w] secenegi) - ör. kavrama sonrasi
    nesnenin oturmasi icin."""
    try:
        duration = ask_float("Bekleme suresi (saniye)", default=defaults.get("wait_s", 1.0), allow_back=True)
    except GoBack:
        print("Geri donuldu, adim eklenmedi.")
        return
    if duration < 0:
        print("  Negatif olamaz, adim eklenmedi.")
        return
    defaults["wait_s"] = duration
    steps.append({"step_type": "wait", "duration_s": duration})
    print(f"Adim #{len(steps)} eklendi (bekle {duration:.1f} sn).")
    save_draft(name, steps)


def _check_resume_draft():
    """Yarim kalmis bir taslak (kesilen kayit YA DA kesilen duzenleme) varsa
    (name, steps) doner, kullanici devam etmek istemezse taslagi siler ve
    (None, None) doner."""
    draft = load_draft()
    if not draft:
        return None, None
    print(f"\nYarim kalmis bir kayit/duzenleme bulundu: '{draft['name']}' ({len(draft['steps'])} adim).")
    answer = input("Devam ettirilsin mi? (e=devam, h=sil ve yoksay): ").strip().lower()
    if answer == "e":
        return draft["name"], draft["steps"]
    clear_draft()
    return None, None


def _run_step_editor(arm, gripper, name, steps, default_speed_pct=None, default_accel_pct=None):
    """Adim ekleme/silme/listeleme ana dongusu - HEM yeni kayit (record_new_
    move) HEM mevcut bir kaydin duzenlenmesi (edit_saved_move) bunu kullanir.
    steps yerinde (in-place) degistirilir. Doner: (steps, replay_speed,
    replay_accel) [b] ile bitirilirse, None [i] ile iptal edilirse."""
    editor_menu = """
=== Adim ekleme menusu ===
  [j] Kolu hareket ettir (jog - komutla)
  [m] Manuel mod: robotu FIZIKSEL olarak elinle hareket ettir (drag-teach)
  [g] Gripper'i hareket ettir (jog)
  [d] Su anki durumu goster
  [l] Eklenen adimlari listele
  [a] Su anki durumu adim olarak ekle (gripper sabit pozisyona gider)
  [k] Gripper'i KAVRAT ve bunu adim olarak ekle (sabit pozisyon degil - nesne
      buyuklugune gore gerektigi kadar kapanip tutar)
  [w] Bekleme adimi ekle (saniye cinsinden)
  [x] Bir adimi sil
  [p] Bir adimin hiz/ivme/gripper parametrelerini duzenle (silmeden)
  [b] Kaydi bitir ve kaydet
  [i] Iptal et (hicbir sey kaydedilmez/degistirilmez)
"""
    print("\nAdim ekleme moduna girildi.")
    print(editor_menu)

    default_speed_pct = arm.speed_pct if default_speed_pct is None else default_speed_pct
    default_accel_pct = arm.accel_pct if default_accel_pct is None else default_accel_pct
    defaults = init_step_defaults(arm, steps)

    while True:
        choice = input(f"[{len(steps)} adim] Secim (j/m/g/d/l/a/k/w/x/p/b/i): ").strip().lower()
        if choice == "j":
            jog_arm(arm)
        elif choice == "m":
            drag_teach_add_steps(arm, gripper, steps, name, defaults)
        elif choice == "g":
            jog_gripper(gripper, arm)
        elif choice == "d":
            print_state(arm, gripper)
        elif choice == "l":
            list_steps(steps)
        elif choice == "a":
            add_move_step(arm, gripper, steps, name, defaults)
        elif choice == "k":
            add_grip_step(arm, gripper, steps, name, defaults)
        elif choice == "w":
            add_wait_step(steps, name, defaults)
        elif choice == "x":
            delete_step(steps, name)
        elif choice == "p":
            edit_step_params(steps, name)
        elif choice == "b":
            if not steps:
                print("  En az 1 adim eklemeden kaydedemezsin.")
                continue
            try:
                replay_speed = ask_float(
                    "\nBu hareket dogrudan alias ile calistirilinca kullanilacak varsayilan hiz %",
                    default=default_speed_pct, allow_back=True)
                replay_accel = ask_float(
                    "Bu hareket dogrudan alias ile calistirilinca kullanilacak varsayilan ivme %",
                    default=default_accel_pct, allow_back=True)
            except GoBack:
                print("Geri donuldu, adim eklemeye devam edebilirsin (adimlar korunuyor).")
                continue
            return steps, replay_speed, replay_accel
        elif choice in ("i",) + BACK_WORDS:
            answer = input("Iptal edilsin mi? Taslak da silinecek (e/h): ").strip().lower()
            if answer == "e":
                clear_draft()
                print("Iptal edildi, hicbir sey kaydedilmedi/degistirilmedi.")
                return None
            print("Iptal vazgecildi, devam edebilirsin.")
        else:
            print("  Gecersiz secim.")
            continue
        # Her komuttan sonra BULUNULAN menuye don: robotu/gripper'i
        # etkileyen komutlarda once guncel konumu goster, sonra bu menuyu
        # tekrar bas - kullanici ust menuye donmeden nerede oldugunu ve
        # hangi seceneklerin oldugunu gorsun. ([d] zaten durumu basiyor.)
        if choice in ("j", "m", "g", "a", "k"):
            print_state(arm, gripper)
        print(editor_menu)


def _teach_pose(arm, gripper, baslik):
    """Kullanici kolu istedigi poza getirene kadar bekler, o pozu doner.
    Iptal edilirse None."""
    print(f"\n{baslik}")
    menu = ("  [j] Kolu komutla hareket ettir (jog)\n"
            "  [m] Manuel mod: robotu ELINLE getir (drag-teach)\n"
            "  [g] Gripper'i hareket ettir\n"
            "  [d] Su anki durumu goster\n"
            "  [o] BURASI TAMAM - bu pozu kullan\n"
            "  [i] Iptal")
    print(menu)
    while True:
        secim = input("Secim (j/m/g/d/o/i): ").strip().lower()
        if secim == "j":
            jog_arm(arm)
        elif secim == "m":
            if enter_drag_teach(arm, gripper):
                try:
                    input("Robotu elinle istedigin poza getir, sonra Enter'a bas...")
                finally:
                    exit_drag_teach(arm, gripper)
        elif secim == "g":
            jog_gripper(gripper, arm)
        elif secim == "d":
            print_state(arm, gripper)
            continue
        elif secim == "o":
            st = arm.get_current_state()
            print(f"  Poz alindi: {[round(v, 1) for v in st['joints_deg']]}")
            return st["joints_deg"]
        elif secim in ("i",) + BACK_WORDS:
            return None
        else:
            print("  Gecersiz secim.")
            continue
        print_state(arm, gripper)
        print(menu)


def record_pick_and_place(arm, gripper):
    """MoveIt Task Constructor ile al-tasi-birak kaydi.

    Kullanicidan SADECE iki poz istenir (alma ve birakma). Yaklasma/geri
    cekilme, nesneyi modele ilistirme/ayirma gibi asamalari MTC'nin kendi
    hazir stage'leri yapar - bkz. pickplace.py."""
    print("\n--- Pick & place ---")
    try:
        from fairino_cobot.pickplace import load_mtc
    except Exception as e:
        print(f"HATA: pickplace modulu yuklenemedi ({e}).")
        return
    if load_mtc() is None:
        print("HATA: MoveIt Task Constructor kurulu degil. Kurulum:")
        print("  sudo apt install ros-humble-moveit-task-constructor-core \\")
        print("       ros-humble-moveit-task-constructor-capabilities")
        return

    try:
        yaklasma = ask_float("Yaklasma/geri cekilme mesafesi (mm)", default=80.0, allow_back=True)
        g_speed = ask_int("Kavrama hizi %", 50, allow_back=True)
        g_force = ask_int("Kavrama kuvveti %", 30, allow_back=True)
        # Nesne olculeri SADECE carpisma kontrolu icin - hassas olmasi
        # gerekmiyor, cömert vermek daha guvenli. Genislik SORULMUYOR:
        # cenelerin kavrama aninda durdugu aciklik zaten onu veriyor
        # (bkz. GRIPPER_STROKE_M), ve her calistirmada yeniden olculuyor ki
        # farkli kalinliktaki nesnelerde de dogru olsun.
        print("\nNesne olculeri (sadece carpisma kontrolu icin, kabaca yeterli):")
        print("  Genislik SORULMUYOR - cenelerin kavrarken durdugu aciklikta olculuyor.")
        obj_d = ask_float("  Derinlik: cene hareketine DIK yatay olcu (mm)",
                          default=40.0, allow_back=True)
        obj_h = ask_float("  Yukseklik: nesnenin gripper'dan disa uzandigi olcu (mm)",
                          default=60.0, allow_back=True)
    except GoBack:
        print("Iptal edildi.")
        return

    def _on_poz_olc(etiket):
        """Su anki pozdan yaklasma_mm kadar GERI cekilip orayi olcer.

        Neden: robot kavrama noktasina DOGRUDAN dalmamali; once guvenli
        mesafede bir 'on-poz'a gidip oradan duz cizgi yaklasmali (MoveIt
        pick&place semasindaki pre_grasp_approach). On-pozu robotun
        kendisine hesaplatiyoruz - hem ters kinematik ugrasi olmuyor hem de
        o pozun GERCEKTEN ulasilabilir oldugu ayni anda dogrulanmis oluyor."""
        print(f"  {etiket}: on-poz icin {yaklasma:.0f}mm geri cekiliyor...")
        r = arm.move_tcp_local(0, 0, -yaklasma, confirm=False)
        if not r["success"]:
            d = f" - {r['details']}" if r.get("details") else ""
            print(f"  HATA: geri cekilemedi ({r['reason']}{d}). Yaklasma mesafesini "
                  f"kucultup tekrar deneyin.")
            return None
        return arm.get_current_state()["joints_deg"]

    pick = _teach_pose(arm, gripper, "1) Kolu nesneyi ALACAGI poza getir "
                                     "(cene nesneyi kavrayacak konumda olsun)")
    if pick is None:
        print("Iptal edildi.")
        return
    pre_pick = _on_poz_olc("Alma")
    if pre_pick is None:
        return

    place = _teach_pose(arm, gripper, "2) Kolu nesneyi BIRAKACAGI poza getir")
    if place is None:
        print("Iptal edildi.")
        return
    pre_place = _on_poz_olc("Birakma")
    if pre_place is None:
        return

    # --- KAYDET ---
    moves = load_moves()
    while True:
        name = input("\nKayit adi (ör: kutu_tasi, kaydetmeden calistirmak icin bos birak): ").strip()
        if not name:
            name = None
            break
        ok, err = valid_move_name(name)
        if not ok:
            print(f"  Gecersiz isim: {err}")
            continue
        if name in moves:
            if input(f"'{name}' zaten kayitli, uzerine yazilsin mi? (e/h): ").strip().lower() != "e":
                continue
        break

    cfg = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "type": "pick_place",
        "pick_joints": pick,
        "place_joints": place,
        # On-pozlar: kavrama/birakma noktasinin yaklasma_mm kadar gerisi.
        # Robot once buraya gidip oradan DUZ CIZGI yaklasiyor.
        "pre_pick_joints": pre_pick,
        "pre_place_joints": pre_place,
        "approach_mm": yaklasma,
        "grip_speed": g_speed,
        "grip_force": g_force,
        "object_dh_mm": [obj_d, obj_h],
        "speed_pct": arm.speed_pct,
        "accel_pct": arm.accel_pct,
    }
    if name:
        moves[name] = cfg
        save_moves(moves)
        sync_bashrc_aliases(moves)
        print(f"\n'{name}' kaydedildi. Yeni terminalde sadece '{name}' yazman yeterli.")
        print("Bu terminalde hemen kullanmak icin: source ~/.bashrc")

    if input("\nSimdi calistirilsin mi? (e/h): ").strip().lower() != "e":
        return
    run_pick_place(arm, gripper, cfg)


def run_pick_place(arm, gripper, cfg, sor=False):
    """Kayitli (ya da yeni olusturulmus) bir pick & place yapilandirmasini
    calistirir. Asamalari MTC yapar - bkz. pickplace.py.

    sor=True ise (menuden oynatma) hiz/ivme ve kavrama hiz/kuvveti
    calistirmadan once SORULUR; kayitli degerler varsayilan olarak gelir.
    Girilen degerler KAYDI DEGISTIRMEZ, sadece bu calistirma icin gecerlidir.
    Alias ile dogrudan calistirmada (sor=False) kayitli degerler kullanilir."""
    from fairino_cobot.pickplace import PickPlace

    if "pre_pick_joints" not in cfg or "pre_place_joints" not in cfg:
        print("HATA: bu kayit ESKI bicimde (on-pozlar yok). Menuden [5] ile ayni "
              "ismi verip yeniden olusturun.")
        return False

    if sor:
        print("\n--- Bu calistirma icin degerler (Enter = kayitli deger) ---")
        try:
            _sp = ask_float("Kol hizi %", default=cfg.get("speed_pct", arm.speed_pct),
                            allow_back=True)
            _ac = ask_float("Kol ivmesi %", default=cfg.get("accel_pct", arm.accel_pct),
                            allow_back=True)
            _gs = ask_int("Kavrama hizi %", cfg["grip_speed"], allow_back=True)
            _gf = ask_int("Kavrama kuvveti %", cfg["grip_force"], allow_back=True)
        except GoBack:
            print("Iptal edildi.")
            return False
        # Kaydi BOZMAMAK icin kopya uzerinde calis.
        cfg = dict(cfg)
        cfg["speed_pct"], cfg["accel_pct"] = _sp, _ac
        cfg["grip_speed"], cfg["grip_force"] = _gs, _gf

    print("\nUYARI: GERCEK ROBOT hareket edecek. Acil stop elinin altinda olsun.")
    try:
        input("Devam etmek icin Enter'a bas (iptal icin Ctrl+C)...")
    except (KeyboardInterrupt, EOFError):
        print("\nIptal edildi.")
        return False

    yaklasma = cfg["approach_mm"]
    g_speed, g_force = cfg["grip_speed"], cfg["grip_force"]
    obj_d, obj_h = cfg["object_dh_mm"]

    try:
        pp = PickPlace(arm.robot_desc_name, arm.package_name, arm.group_name,
                       TIP_LINK, f"/tmp/fairino{arm.model_num}_movecobot.yaml",
                       JOINT_NAMES,
                       speed_pct=cfg.get("speed_pct", arm.speed_pct),
                       accel_pct=cfg.get("accel_pct", arm.accel_pct))
    except Exception as e:
        print(f"HATA: MTC baslatilamadi ({type(e).__name__}: {e})")
        return False

    # --- 1) cene ac, alma pozuna duz cizgi yaklas ---
    try:
        gripper.move(position=0, speed=g_speed, force=g_force)
    except GripperError as e:
        print(f"UYARI: gripper acilamadi: {e}")
    if not pp.alma_pozuna_yaklas(cfg["pre_pick_joints"], yaklasma):
        return False

    # --- 2) KUVVETLE kavra (sabit pozisyon degil - her nesnede calisir) ---
    try:
        sonuc = gripper.grip(speed=g_speed, force=g_force)
    except GripperError as e:
        print(f"HATA: kavrama basarisiz: {e}")
        return False
    if sonuc["gripped"]:
        print(f"  Nesne yakalandi (pozisyon={sonuc['position_pct']}).")
    else:
        print(f"  UYARI: kapandi ama nesne algilanmadi (pozisyon={sonuc['position_pct']}).")
    # Nesne genisligini BASINCTAN turet (cenelerin durdugu aciklik) - kayitta
    # sabit tutulmuyor, HER CALISTIRMADA yeniden olculuyor ki farkli
    # kalinliktaki nesnelerde de dogru olsun.
    obj_w = max((100 - sonuc["position_pct"]) / 100.0 * GRIPPER_STROKE_M * 1000, 5.0)
    print(f"  Basinctan turetilen nesne genisligi: ~{obj_w:.0f} mm")

    # --- 3) ilistir, kaldir, birakma pozuna git, in ---
    if not pp.kaldir_ve_birakma_pozuna_git(cfg["pre_place_joints"], yaklasma, yaklasma,
                                           [obj_w, obj_d, obj_h], arm.tcp_offset_m):
        return False

    # --- 4) birak, ayir, geri cekil ---
    try:
        gripper.move(position=0, speed=g_speed, force=g_force)
    except GripperError as e:
        print(f"UYARI: gripper acilamadi: {e}")
    pp.ayir_ve_geri_cekil(yaklasma)
    print("\nPick & place tamamlandi.")
    return True


def record_new_move(arm, gripper):
    print("\n--- Yeni hareket kaydi ---")

    name, steps = _check_resume_draft()
    if name is not None:
        print(f"'{name}' kaydina {len(steps)} adimla devam ediliyor.")
    else:
        steps = []
        while True:
            name = input("Kayit adi (ör: pick1, iptal icin 'geri' veya bos birak): ").strip()
            if not name or name.lower() in BACK_WORDS:
                print("Iptal edildi.")
                return
            ok, err = valid_move_name(name)
            if not ok:
                print(f"  Gecersiz isim: {err}")
                continue
            break

    moves = load_moves()
    if name in moves:
        answer = input(f"'{name}' zaten kayitli, uzerine yazilsin mi? (e/h): ").strip().lower()
        if answer != "e":
            print("Iptal edildi.")
            return
    else:
        existing_cmd = shutil.which(name)
        if existing_cmd:
            answer = input(
                f"UYARI: '{name}' zaten bir sistem komutu ({existing_cmd}). "
                f"Yine de alias olarak kullanilsin mi? (e/h): "
            ).strip().lower()
            if answer != "e":
                print("Iptal edildi.")
                return

    result = _run_step_editor(arm, gripper, name, steps)
    if result is None:
        return
    steps, replay_speed, replay_accel = result

    moves[name] = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "speed_pct": replay_speed,
        "accel_pct": replay_accel,
        "steps": steps,
    }
    save_moves(moves)
    sync_bashrc_aliases(moves)
    clear_draft()

    print(f"\n'{name}' hareketi {len(steps)} adimla kaydedildi.")
    print("~/.bashrc guncellendi. Bu terminalde hemen kullanmak icin:")
    print("  source ~/.bashrc")
    print(f"Yeni acilan terminallerde sadece '{name}' yazman yeterli olacak.")


def edit_saved_move(arm, gripper):
    print("\n--- Kayitli hareketi duzenle ---")

    draft_name, draft_steps = _check_resume_draft()
    moves = load_moves()
    if draft_name is not None:
        name, steps = draft_name, draft_steps
        print(f"'{name}' duzenlemesine {len(steps)} adimla devam ediliyor.")
    else:
        if not moves:
            print("\nHenuz kayitli hareket yok.")
            return
        list_saved_moves()
        name = input("\nDuzenlenecek hareketin adi (iptal icin 'geri' veya bos birak): ").strip()
        if not name or name.lower() in BACK_WORDS:
            return
        if name not in moves:
            print(f"  '{name}' bulunamadi.")
            return
        if moves[name].get("type") == "pick_place":
            # Pick & place kaydinin adimi yok (iki poz + parametrelerden
            # ibaret, asamalari MTC uretiyor) - adim editoru ile duzenlenemez.
            print(f"  '{name}' bir pick & place kaydi; adim listesi yok, bu editorle "
                  f"duzenlenemez. Degistirmek icin menuden [5] ile yeniden olusturup "
                  f"ayni ismi verin.")
            return
        # Derin kopya - kullanici [b] ile bitirip kaydetmeden orijinal
        # kayitli hareket HIC degismesin (iptal her zaman guvenli olsun).
        steps = json.loads(json.dumps(moves[name]["steps"]))
        print(f"\n--- '{name}' duzenleniyor ({len(steps)} adim) ---")
        list_steps(steps)

    existing = moves.get(name, {})
    result = _run_step_editor(
        arm, gripper, name, steps,
        default_speed_pct=existing.get("speed_pct"), default_accel_pct=existing.get("accel_pct"),
    )
    if result is None:
        print("Duzenleme iptal edildi, kayitli hareket degismedi.")
        return
    steps, replay_speed, replay_accel = result

    moves = load_moves()
    moves[name] = {
        "created": existing.get("created", datetime.now().isoformat(timespec="seconds")),
        "modified": datetime.now().isoformat(timespec="seconds"),
        "speed_pct": replay_speed,
        "accel_pct": replay_accel,
        "steps": steps,
    }
    save_moves(moves)
    sync_bashrc_aliases(moves)
    clear_draft()
    print(f"\n'{name}' hareketi guncellendi ({len(steps)} adim).")


def list_saved_moves():
    moves = load_moves()
    if not moves:
        print("\nHenuz kayitli hareket yok.")
        return
    print("\n=== Kayitli hareketler ===")
    for name, m in sorted(moves.items()):
        modified = f", duzenlendi: {m['modified']}" if m.get("modified") else ""
        ne = "pick & place" if m.get("type") == "pick_place" else f"{len(m.get('steps', []))} adim"
        print(f"  {name:20s}  {ne:12s}  hiz=%{m['speed_pct']:.0f} "
              f"ivme=%{m['accel_pct']:.0f}  ({m['created']}{modified})")


def delete_saved_move():
    moves = load_moves()
    if not moves:
        print("\nHenuz kayitli hareket yok.")
        return
    list_saved_moves()
    name = input("\nSilinecek hareketin adi (iptal icin 'geri' veya bos birak): ").strip()
    if not name or name.lower() in BACK_WORDS:
        return
    if name not in moves:
        print(f"  '{name}' bulunamadi.")
        return
    del moves[name]
    save_moves(moves)
    sync_bashrc_aliases(moves)
    print(f"'{name}' silindi ve alias kaldirildi. Bu terminalde uygulamak icin: source ~/.bashrc")


def replay_sequence(arm, gripper, steps, robot_ip):
    if robot_ip != "127.0.0.1":
        print("\nUYARI: GERCEK ROBOT hareket edecek. Acil stop elinin altinda olsun.")
        try:
            input("Devam etmek icin Enter'a bas (iptal icin Ctrl+C)...")
        except KeyboardInterrupt:
            print("\nIptal edildi.")
            return False

    # Ilk adim SDK'dan taze okumali (robotun GERCEKTE nerede oldugundan
    # emin degiliz - ör. replay yeni baslatildi, robot elle jog edilmis
    # olabilir). Ama SONRAKI her adim, bir ONCEKI adimin _plan_and_execute()
    # icindeki post-execution dogrulamasindan (o da DOGRUDAN SDK okumasi,
    # ROS2 degil) gelen TAZE bir deger zaten elimizde - onu tekrar SDK'ya
    # sormadan kullanmak, adimlar arasindaki ag turu sayisini (ve dolayisiyla
    # "her adimda bekleme" hissini) azaltir, guvenlikten odun vermeden.
    known_start_joints_deg = None
    for i, step in enumerate(steps, 1):
        if step.get("step_type", "move") == "wait":
            duration = step.get("duration_s", 0)
            print(f"[{i}/{len(steps)}] Bekleniyor: {duration:.1f} sn")
            time.sleep(duration)
            continue

        speed_pct, accel_pct = step.get("speed_pct"), step.get("accel_pct")
        speed_str = f"%{speed_pct:.0f}" if speed_pct is not None else f"%{arm.speed_pct:.0f} (oturum)"
        accel_str = f"%{accel_pct:.0f}" if accel_pct is not None else f"%{arm.accel_pct:.0f} (oturum)"
        print(f"[{i}/{len(steps)}] Eklemlere hareket (hiz={speed_str} ivme={accel_str}): "
              f"{[round(v, 1) for v in step['joints_deg']]}")
        result = arm.move_to_joints(step["joints_deg"], confirm=False, speed_pct=speed_pct, accel_pct=accel_pct,
                                     known_start_joints_deg=known_start_joints_deg)
        if not result["success"]:
            details = f" - {result['details']}" if result.get("details") else ""
            print(f"HATA: adim {i} basarisiz oldu ({result['reason']}{details}). Kalan adimlar calistirilmadi.")
            return False
        known_start_joints_deg = result["final_state"]["joints_deg"]
        gripper_action = step.get("gripper_action", "position")
        if gripper_action == "grip":
            print(f"[{i}/{len(steps)}] Gripper -> KAVRA (hiz={step['gripper_speed']} "
                  f"kuvvet={step['gripper_force']})")
            try:
                result = gripper.grip(speed=step["gripper_speed"], force=step["gripper_force"])
                if result["gripped"]:
                    print(f"  Nesne yakalandi (pozisyon={result['position_pct']}).")
                else:
                    print(f"  UYARI: kapandi ama nesne algilanmadi (pozisyon={result['position_pct']}).")
                # MoveIt semasinda kavrama = nesneyi modele ILISTIRME ani.
                # Kayitta boyut varsa onu kullan; yoksa (eski kayitlar)
                # basinctan turetilen genisligi kup gibi varsayarak
                # muhafazakar bir kutu kur.
                _sz = step.get("object_size_mm")
                if _sz is None:
                    _w = (100 - result["position_pct"]) / 100.0 * GRIPPER_STROKE_M * 1000
                    _w = max(_w, 10.0)
                    _sz = [_w, _w, _w]
                    print(f"  (bu eski kayitta nesne boyutu yok - basinctan "
                          f"~{_w:.0f}mm kup varsayildi)")
                arm.attach_grasped_object(_sz[0] / 1000.0, _sz[1] / 1000.0, _sz[2] / 1000.0)
            except GripperError as e:
                print(f"UYARI: kavrama basarisiz oldu: {e}")
        elif gripper_action == "none":
            # Gripper'a HIC dokunma - kavradigi neyse tutmaya devam eder.
            # Bir nesneyi tasiyan adimlarda dogru davranis budur; sabit bir
            # pozisyon emri vermek nesnenin KALINLIGINA bagimli olur.
            print(f"[{i}/{len(steps)}] Gripper -> dokunulmuyor (tutuyorsa tutmaya devam)")
        else:
            print(f"[{i}/{len(steps)}] Gripper -> pozisyon={step['gripper_position']} "
                  f"hiz={step['gripper_speed']} kuvvet={step['gripper_force']}")
            try:
                gripper.move(position=step["gripper_position"], speed=step["gripper_speed"], force=step["gripper_force"])
                # MoveIt semasinda BIRAKMA = nesneyi modelden AYIRMA ani.
                # Ceneler acildiysa (pozisyon ~0) nesne birakilmis demektir.
                if step["gripper_position"] <= 10:
                    arm.detach_object()
            except GripperError as e:
                print(f"UYARI: gripper hareketi basarisiz oldu: {e}")

    print("Hareket tamamlandi.")
    return True


def replay_from_menu(arm, gripper, robot_ip):
    moves = load_moves()
    if not moves:
        print("\nHenuz kayitli hareket yok.")
        return
    list_saved_moves()
    name = input("\nOynatilacak hareketin adi (iptal icin 'geri' veya bos birak): ").strip()
    if not name or name.lower() in BACK_WORDS:
        return
    if name not in moves:
        print(f"  '{name}' bulunamadi.")
        return
    if moves[name].get("type") == "pick_place":
        run_pick_place(arm, gripper, moves[name], sor=True)
        return
    print(f"\n(Bu oturumun hizinda calisacak: %{arm.speed_pct:.0f}/%{arm.accel_pct:.0f}. "
          f"Kayitli hiz %{moves[name]['speed_pct']:.0f}/%{moves[name]['accel_pct']:.0f} sadece "
          f"'{name}' aliasiyla dogrudan calistirinca kullanilir.)")
    replay_sequence(arm, gripper, moves[name]["steps"], robot_ip)


def replay_cli(name):
    moves = load_moves()
    if name not in moves:
        print(f"HATA: '{name}' adinda kayitli bir hareket yok.")
        list_saved_moves()
        sys.exit(1)
    move = moves[name]

    robot_info = detect_robot()
    robot_ip = robot_info["ROBOT_IP"]
    model_num = robot_info["ROBOT_MODEL_NUM"]

    _pp = move.get("type") == "pick_place"
    _ne = "pick & place" if _pp else f"{len(move['steps'])} adim"
    print(f"\n>>> '{name}' oynatiliyor ({_ne}, "
          f"hiz=%{move['speed_pct']:.0f}, ivme=%{move['accel_pct']:.0f})\n")

    arm = CobotArm(robot_ip, model_num, move["speed_pct"], move["accel_pct"], node_name=f"movecobot_replay_{name}")
    gripper = GripperController(sdk=arm.sdk, lock=arm.sdk_lock)
    if not gripper.is_active():
        print("Gripper aktive ediliyor...")
        gripper.activate()

    if _pp:
        ok = run_pick_place(arm, gripper, move)
    else:
        ok = replay_sequence(arm, gripper, move["steps"], robot_ip)
    arm.shutdown()
    sys.exit(0 if ok else 1)


def clear_fault_cli():
    """WebApp'te robot fault verip 'Stopped' oldugunda, ros2_control
    (fairino_launch) HALA CALISIYORSA, 100Hz'lik ServoJ dongusu hatayi
    temizler temizlemez ayni/eski hedefe tekrar komut gonderip hatayi
    ANINDA geri getirebiliyor - WebApp'te 'Clear' hicbir sey yapmiyormus
    gibi gorunmesinin sebebi genelde bu. Bu komut fairino_launch'i (move_
    group/rviz) HIC durdurmadan sadece bu 100Hz dongusunu (enter_drag_teach
    ile AYNI mekanizma, bkz. _ros2_control_set_state) gecici olarak
    durdurur, Clear'a basmani bekler, sonra geri acar - movecobot'un kendi
    robota (SDK) baglantisini hic acmaz/kapatmaz, sadece 'ros2 control
    set_hardware_component_state' cagirir."""
    print("Robot fault WebApp'te 'Clear' ile temizlenmiyor, ros2_control "
          "(fairino_launch) hala calisiyorsa bu genelde onun 100Hz'lik komut "
          "dongusunun hatayi aninda geri getirmesinden kaynaklanir.\n")

    robot_info = detect_robot()
    robot_desc_name = f"fairino{robot_info['ROBOT_MODEL_NUM']}_v6_robot"
    package_name = f"fairino{robot_info['ROBOT_MODEL_NUM']}_v6_moveit2_config"

    print("ROS2 kontrol dongusu gecici olarak durduruluyor (fairino_launch/"
          "move_group/rviz'e DOKUNULMUYOR, sadece komut dongusu duruyor)...")
    ok, err = _ros2_control_set_state("inactive")
    if not ok:
        print(f"HATA: durdurulamadi:\n  {err}")
        sys.exit(1)

    print("Durduruldu. Simdi WebApp'te 'Clear'a bas.")
    try:
        input("Clear'a bastiktan sonra Enter'a bas (ROS2 kontrolunu geri acmak icin)...")
    except (KeyboardInterrupt, EOFError):
        print("\nIptal edildi - ROS2 kontrol dongusu HALA DURDURULMUS durumda, "
              "robotu jog/replay ile hareket ettiremezsin. Geri acmak icin "
              "'movecobot --clear-fault' komutunu tekrar calistirip Enter'a bas.")
        sys.exit(1)

    print("ROS2 kontrol dongusu yeniden aktive ediliyor...")
    ok, err = _ros2_control_set_state("active", robot_desc_name, package_name)
    if not ok:
        print(f"HATA: ROS2 kontrolu geri acilamadi:\n  {err}")
        print("Jog/replay calismayabilir - gerekirse fairino_launch'i yeniden baslat.")
        sys.exit(1)
    print("Tamamlandi. movecobot'u normal sekilde baslatabilirsin.")


# ---------------------------------------------------------------------------
# Interaktif menu
# ---------------------------------------------------------------------------

def interactive_main():
    print("=" * 60)
    print(" movecobot - Fairino kol + Jodell gripper birlesik kontrol")
    print("=" * 60)

    robot_info = detect_robot()
    robot_ip = robot_info["ROBOT_IP"]
    model_num = robot_info["ROBOT_MODEL_NUM"]
    print(f"\n>>> Algilanan: FR{model_num} ({robot_ip})\n")

    speed_pct = ask_float("Bu oturum icin hareket hizi %", default=30.0)
    accel_pct = ask_float("Bu oturum icin ivme %", default=30.0)

    print("\nMoveIt baglantisi kuruluyor (fairino_launch'in calisir durumda olmasi gerekir)...")
    arm = CobotArm(robot_ip, model_num, speed_pct, accel_pct)
    _ox, _oy, _oz = arm.tcp_offset_m
    print(f">>> Olculen wrist3->TCP offset: X={_ox*1000:.1f} Y={_oy*1000:.1f} Z={_oz*1000:.1f} mm\n")

    print("Gripper baglaniyor...")
    try:
        gripper = GripperController(sdk=arm.sdk, lock=arm.sdk_lock)
        if not gripper.is_active():
            print("Aktive ediliyor...")
            gripper.activate()
        print("Gripper hazir.\n")
    except Exception as e:
        print(f"UYARI: gripper baglantisi kurulamadi ({e}). Sadece kol kontrolu kullanilabilir.\n")
        gripper = None

    if gripper is not None:
        # grippergui'yi movecobot'un KENDI arka plan thread'inde, AYNI
        # gripper nesnesiyle (dolayisiyla AYNI sdk baglantisi + AYNI
        # sdk_lock ile) aciyoruz - AYRI bir process/baglanti ACMIYORUZ, bu
        # yuzden konumcobot'ta yasadigimiz "ikinci baglanti robota cevap
        # vermiyor" sorunu burada olusmaz. Pencereyi kapatmak movecobot'u
        # ETKILEMEZ (daemon thread, sadece kendi mainloop()'undan cikar).
        try:
            from gripper_gui import run_gripper_gui
            gui_thread = threading.Thread(
                target=run_gripper_gui, args=(gripper,),
                kwargs={"standalone": True, "busy_event": arm.executing}, daemon=True,
            )
            gui_thread.start()
            print("Gripper GUI penceresi aciliyor (grippergui, ayni baglantiyi paylasarak)...\n")
        except Exception as e:
            print(f"UYARI: Gripper GUI acilamadi ({type(e).__name__}: {e}) - "
                  f"konsoldan [2] ile gripper'i yine de kontrol edebilirsin.\n")

    print_state(arm, gripper)

    menu = """
=== movecobot menu ===
  1) Kolu hareket ettir
  2) Gripper'i hareket ettir
  3) Su anki durumu goster
  4) Yeni hareket kaydet (jog + adim ekle -> isimle kaydet -> otomatik alias)
  5) Pick & place (SADECE alma ve birakma pozu - gerisini MoveIt yapar)
  6) Kayitli bir hareketi duzenle (adim ekle/sil, hiz/ivme degistir)
  7) Kayitli bir hareketi oynat
  8) Kayitli hareketleri listele
  9) Kayitli bir hareketi sil
  10) Cikis
"""
    try:
        while True:
            print(menu)
            choice = input("Secim: ").strip()
            if choice == "1":
                jog_arm(arm)
            elif choice == "2":
                if gripper is None:
                    print("Gripper bagli degil.")
                else:
                    jog_gripper(gripper, arm)
            elif choice == "3":
                print_state(arm, gripper)
            elif choice == "4":
                if gripper is None:
                    print("Gripper bagli olmadan hareket kaydedilemez.")
                else:
                    record_new_move(arm, gripper)
            elif choice == "5":
                if gripper is None:
                    print("Gripper bagli olmadan pick & place yapilamaz.")
                else:
                    record_pick_and_place(arm, gripper)
            elif choice == "6":
                if gripper is None:
                    print("Gripper bagli olmadan hareket duzenlenemez.")
                else:
                    edit_saved_move(arm, gripper)
            elif choice == "7":
                if gripper is None:
                    print("Gripper bagli degil.")
                else:
                    replay_from_menu(arm, gripper, robot_ip)
            elif choice == "8":
                list_saved_moves()
            elif choice == "9":
                delete_saved_move()
            elif choice == "10":
                break
            else:
                print("Gecersiz secim.")
                continue
            # Robotu HAREKET ETTIREBILEN her komuttan sonra guncel konumu
            # goster - kullanici menuye dondugunde kolun/gripper'in nerede
            # oldugunu bir sonraki komutu vermeden once gorsun. (3 zaten
            # durumu basiyor; 7/8 sadece kayit listesiyle ilgili, robotu
            # etkilemiyor.)
            if choice in ("1", "2", "4", "5", "6", "7"):
                print_state(arm, gripper)
    except (KeyboardInterrupt, EOFError):
        print("\nCikiliyor.")
    finally:
        try:
            STATE_CACHE_FILE.unlink(missing_ok=True)
        except Exception:
            pass
        arm.shutdown()


def print_help():
    print(__doc__)


def main():
    args = sys.argv[1:]
    if not args:
        interactive_main()
        return
    if args[0] in ("-h", "--help"):
        print_help()
        return
    if args[0] == "--list":
        list_saved_moves()
        return
    if args[0] == "--replay":
        if len(args) < 2:
            print("Kullanim: movecobot --replay <isim>")
            sys.exit(1)
        replay_cli(args[1])
        return
    if args[0] == "--clear-fault":
        clear_fault_cli()
        return
    print(f"Bilinmeyen secenek: {args[0]}")
    print_help()
    sys.exit(1)


if __name__ == "__main__":
    main()

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
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

CLIENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(CLIENT_DIR, "fairino-python-sdk/linux"))
sys.path.insert(0, CLIENT_DIR)

from fairino import Robot
from gripper_controller import GripperController, GripperError

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
TIP_LINK = "wrist3_link"
ALL_LINKS = ["shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"]
FLOOR_Z_M = 0.0

# Jodell EPG40-050 gripper'in collision kutusu icin boyutlar. Bu gripper
# fairino_description'daki URDF'de HIC modellenmemis (wrist3_link son link,
# bkz. fairino_description/urdf/fairino{N}_v6.urdf) - yani MoveIt'in kendi
# self-collision kontrolu bu geometriler eklenmeden gripper'in var oldugundan
# habersizdir. Govde 40x24x87mm (kaynak: fairino.info/accessories/gripper-
# epg40-050 + welectron.com urun sayfasi, DOGRULANMADI - gercek CAD/olcum
# degil). XY icin genis (govdenin ~2 kati) bir pay bilinclii tercih edildi -
# kullanici onayi: guvenlik once, gerekirse (baslangic pozunda sahte
# carpisma/kilitlenme yasanirsa) kutu asagi cekilir (55x50mm denendi, gerekirse
# tekrar denenebilir). Kilitlenme riski _add_gripper_collision'daki
# baslangic-durumu kontrolu ile karsilaniyor (colliding ise kullaniciya sorup
# gerekirse korumayi o oturum icin kapatiyor). Uzunluk icin ise gercek olcum
# olan tcp_offset_m kullanilir (sabit 87mm yerine - boylece kalibrasyon neyi
# olcerse kutu da onu kapsar).
GRIPPER_WIDTH_M = 0.075
GRIPPER_DEPTH_M = 0.04
# Flanstan itibaren korunan TOPLAM mesafe 120mm olsun diye pay 0'a cekildi -
# bkz. asagidaki GRIPPER_LENGTH_FLOOR_M (120mm zaten kullanicinin belirledigi
# nihai deger, ustune ekstra pay istenmedi).
GRIPPER_LENGTH_MARGIN_M = 0.0
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

MOVES_FILE = Path(CLIENT_DIR) / "movecobot_moves.json"
# Hareket kaydi SIRASINDA (record_new_move/drag_teach_add_steps) eklenen her
# adimdan sonra buraya yazilir. Manuel modda robot bir fault verirse (ör.
# "hiz limiti asildi") kullanicinin launch'i durdurup WebApp'ten temizleyip
# yeniden baslatmasi gerekebiliyor - bu movecobot surecini de oldurup o ana
# kadar eklenen TUM adimlari kaybettiriyordu. Simdi her adim eklendiginde
# taslak buraya kaydediliyor, movecobot yeniden acildiginda "yarim kalan
# kaydi devam ettir" diye sorup kaldigi yerden devam ettiriyor.
DRAFT_FILE = Path(CLIENT_DIR) / ".movecobot_record_draft.json"
# konumcobot (ayri, hafif bir "durum goster" araci) ile movecobot AYNI ANDA
# calisirken robota IKINCI bir SDK baglantisi ACMAMASI icin - Fairino'nun
# gercek zamanli durum soketi (port 20004) tek-tuketicili, iki ayri surecten
# baglanmaya calismak ikisini de kilitleyebiliyor (canli testte yasandi).
# Bunun yerine movecobot, en son okudugu durumu buraya yazar; konumcobot da
# (movecobot calisiyorsa) robota hic baglanmadan bunu okur.
STATE_CACHE_FILE = Path(CLIENT_DIR) / ".movecobot_state_cache.json"
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
    script_path = os.path.expanduser("~/detect_robot.py")
    if not os.path.exists(script_path):
        print(f"HATA: {script_path} bulunamadi.")
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
  pipeline_names: ["ompl"]

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
        if res.collision or res.distance < MIN_CLEARANCE_M:
            violations.append((i, res.distance))
    return violations


# ---------------------------------------------------------------------------
# CobotArm: movepy + fairino_controller.py'nin birlesimi, ama modele/IP'ye
# gore OTOMATIK ayarlanan (hardcoded fairino5/127.0.0.1 degil) versiyonu.
# ---------------------------------------------------------------------------

class CobotArm:
    def __init__(self, robot_ip, model_num, speed_pct, accel_pct, node_name="movecobot"):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from moveit.planning import MoveItPy
        from moveit_configs_utils import MoveItConfigsBuilder
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
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
        self.sdk = Robot.RPC(robot_ip)
        with self.sdk_lock:
            current_joints = self.sdk.robot.GetActualJointPosDegree(0)[1:7]
            current_pose = self.sdk.robot.GetActualTCPPose(0)[1:7]
            current_flange_pose = self.sdk.GetActualToolFlangePose(0)[1]

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
        self._moveit = MoveItPy(node_name=f"{node_name}_core", config_dict=moveit_config.to_dict())
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
                too_close = res.collision or res.distance < MIN_CLEARANCE_M
            print(f"  Gripper collision kutusu eklendi: {GRIPPER_WIDTH_M*1000:.0f}x{GRIPPER_DEPTH_M*1000:.0f}x"
                  f"{length*1000:.0f}mm (flanstan itibaren, {TIP_LINK}'e sabit; flans payi "
                  f"X={fx*1000:.1f} Y={fy*1000:.1f} Z={fz*1000:.1f}mm, canli olculen gripper="
                  f"{abs(gz)*1000:.1f}mm, guvenlik tabani={GRIPPER_LENGTH_FLOOR_M*1000:.0f}mm, "
                  f"buyuk olan + {GRIPPER_LENGTH_MARGIN_M*1000:.0f}mm pay kullanildi)")
            print(f"  Su anki pozisyonda en yakin iki govde arasi mesafe: {res.distance*1000:.1f}mm "
                  f"(guvenlik esigi: {MIN_CLEARANCE_M*1000:.0f}mm)")

            if too_close:
                print(f"\nUYARI: Robotun SU ANKI pozisyonunda en yakin mesafe ({res.distance*1000:.1f}mm) "
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

    def get_current_state(self):
        with self.sdk_lock:
            joints = self.sdk.robot.GetActualJointPosDegree(0)[1:7]
            pose = self.sdk.robot.GetActualTCPPose(0)[1:7]
        return {
            "joints_deg": [round(v, 3) for v in joints],
            "tcp_pose": {
                "x": round(pose[0], 3), "y": round(pose[1], 3), "z": round(pose[2], 3),
                "rx": round(pose[3], 3), "ry": round(pose[4], 3), "rz": round(pose[5], 3),
            },
        }

    def _plan_and_execute(self, confirm, speed_pct=None, accel_pct=None):
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
        # (time-parameterization sonrasi) burada TEKRAR, ayrica kontrol
        # ediyoruz - ikinci, daha guvenilir bir guvenlik agi.
        psm = self._moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            self_collision_steps = check_trajectory_for_self_collision(scene, plan_result.trajectory, self.group_name)
        if self_collision_steps:
            worst_step, worst_dist = min(self_collision_steps, key=lambda v: v[1])
            return {"success": False, "reason": "self_collision",
                    "details": f"plandaki {len(self_collision_steps)} noktada minimum guvenlik "
                                f"mesafesinin ({MIN_CLEARANCE_M*1000:.0f}mm) altina dusuluyor - en "
                                f"kotusu adim {worst_step}/{len(plan_result.trajectory)}, mesafe="
                                f"{worst_dist*1000:.1f}mm"}

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

        self._moveit.execute(self.group_name, plan_result.trajectory)

        final_state = self.get_current_state()
        max_err_deg = max(abs(a - b) for a, b in zip(final_state["joints_deg"], target_joints_deg))
        if max_err_deg > 2.0:
            return {
                "success": False,
                "reason": "execution_not_confirmed",
                "details": (f"hedeften {max_err_deg:.1f} derece uzakta kaldi - robot hareket etmemis "
                            f"olabilir (ROS2/ros2_control tarafinda sorun var mi kontrol et: "
                            f"'ros2 node list' icinde controller_manager/joint_state_broadcaster "
                            f"gorunuyor mu?)"),
                "final_state": final_state,
            }
        return {"success": True, "final_state": final_state}

    def _set_start_state_from_sdk(self):
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
        bu tam olarak boyle bir uyumsuzlukla aciklanabilir."""
        from moveit.core.robot_state import RobotState
        with self.sdk_lock:
            current_joints = self.sdk.robot.GetActualJointPosDegree(0)[1:7]
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

    def move_to_joints(self, joint_degrees, confirm=False, speed_pct=None, accel_pct=None):
        from moveit.core.robot_state import RobotState
        self._set_start_state_from_sdk()
        target_state = RobotState(self.robot_model)
        target_state.set_joint_group_positions(self.group_name, [math.radians(v) for v in joint_degrees])
        self._planning_component.set_goal_state(robot_state=target_state)
        return self._plan_and_execute(confirm, speed_pct=speed_pct, accel_pct=accel_pct)

    def move_relative_joints(self, joint_deltas_deg, confirm=False):
        with self.sdk_lock:
            current = self.sdk.robot.GetActualJointPosDegree(0)[1:7]
        target = [current[i] + joint_deltas_deg[i] for i in range(6)]
        return self.move_to_joints(target, confirm=confirm)

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

    def move_to_pose_base(self, x, y, z, rx, ry, rz, confirm=False):
        self._set_start_state_from_sdk()
        pose_goal = self._goal_pose_from_tcp(x, y, z, rx, ry, rz)
        self._planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=TIP_LINK)
        return self._plan_and_execute(confirm)

    def move_to_pose_tool_xyz(self, dx, dy, dz, rx, ry, rz, confirm=False):
        """frame_choice='Tool': X/Y/Z mevcut Base-frame pozisyonuna eklenir (goreceli),
        RX/RY/RZ HER ZAMAN mutlak hedef olarak alinir - movepy ile ayni davranis."""
        with self.sdk_lock:
            cur = self.sdk.robot.GetActualTCPPose(0)
        cx, cy, cz = cur[1], cur[2], cur[3]
        return self.move_to_pose_base(cx + dx, cy + dy, cz + dz, rx, ry, rz, confirm=confirm)

    def move_tcp_local(self, dx, dy, dz, confirm=False):
        """TCP'nin kendi eksenlerine gore goreceli hareket; yonelim degismez."""
        with self.sdk_lock:
            cur = self.sdk.robot.GetActualTCPPose(0)
        cx, cy, cz, crx, cry, crz = cur[1:7]
        cqx, cqy, cqz, cqw = euler_deg_to_quat(crx, cry, crz)
        cur_x_axis = [1 - 2 * (cqy * cqy + cqz * cqz), 2 * (cqx * cqy + cqz * cqw), 2 * (cqx * cqz - cqy * cqw)]
        cur_y_axis = [2 * (cqx * cqy - cqz * cqw), 1 - 2 * (cqx * cqx + cqz * cqz), 2 * (cqy * cqz + cqx * cqw)]
        cur_z_axis = [2 * (cqx * cqz + cqy * cqw), 2 * (cqy * cqz - cqx * cqw), 1 - 2 * (cqx * cqx + cqy * cqy)]
        tx = cx + dx * cur_x_axis[0] + dy * cur_y_axis[0] + dz * cur_z_axis[0]
        ty = cy + dx * cur_x_axis[1] + dy * cur_y_axis[1] + dz * cur_z_axis[1]
        tz = cz + dx * cur_x_axis[2] + dy * cur_y_axis[2] + dz * cur_z_axis[2]
        return self.move_to_pose_base(tx, ty, tz, crx, cry, crz, confirm=confirm)

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
                    current = arm.sdk.robot.GetActualJointPosDegree(0)[1:7]
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


def jog_gripper(gripper):
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
            time.sleep(1.0)
            return False

        print("Robotla manuel mod icin gecici bir baglanti kuruluyor...")
        try:
            arm.sdk = Robot.RPC(arm.robot_ip)
            gripper.robot = arm.sdk
            # Yeni baglanti kurulduktan hemen sonra okumak, arka plandaki
            # realtime-durum thread'i (port 20004) henuz ilk gercek paketi
            # almadan GetActualJointPosDegree'nin ESKI/varsayilan (sifir)
            # deger dondurmesine yol acabiliyor (bu fonksiyon hep err=0 doner,
            # SDK bu farki kendisi ayirt etmiyor) - kisa bir ek bekleme ile
            # bu yaris durumunu azaltiyoruz.
            time.sleep(1.0)
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
            time.sleep(1.0)  # bkz. enter_drag_teach - realtime durumun durulmasi icin
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
        print("  [a] Su anki (elle getirilen) pozu adim olarak ekle (gripper sabit pozisyona gider)")
        print("  [k] Su anki poz + gripper'i KAVRAT, bunu adim olarak ekle (sabit pozisyon degil)")
        print("  [d] Su anki durumu goster")
        print("  [l] Eklenen adimlari listele")
        print("  [w] Bekleme adimi ekle (saniye cinsinden)")
        print("  [x] Bir adimi sil")
        print("  [c] Robot fault verdiyse (ör. 'hiz limiti asildi'): WebApp'ten Clear'a bas,")
        print("      sonra buraya donup bunu sec - launch'i durdurmana/movecobot'u")
        print("      yeniden baslatmana GEREK YOK, kayittaki adimlar korunur")
        print("  [b] Manuel moddan cik (ROS2 kontrolunu geri devret)\n")
        while True:
            choice = input(f"[manuel mod, {len(steps)} adim] Secim (a/k/d/l/w/x/c/b): ").strip().lower()
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
            elif choice in BACK_WORDS:
                return
            else:
                print("  Gecersiz secim.")
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
        "gripper_action": "position",
        "gripper_position": g_pos,
        "gripper_speed": g_speed,
        "gripper_force": g_force,
        "speed_pct": speed_pct,
        "accel_pct": accel_pct,
    })
    print(f"Adim #{len(steps)} eklendi. (eklemler={[round(v, 1) for v in state['joints_deg']]}, "
          f"gripper={g_pos}, hiz=%{speed_pct:.0f}, ivme=%{accel_pct:.0f})")
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
    steps.append({
        "step_type": "move",
        "joints_deg": state["joints_deg"],
        "tcp_pose": state["tcp_pose"],
        "gripper_action": "grip",
        "gripper_position": result["position_pct"],
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
    print("\nAdim ekleme moduna girildi.")
    print("  [j] Kolu hareket ettir (jog - komutla)")
    print("  [m] Manuel mod: robotu FIZIKSEL olarak elinle hareket ettir (drag-teach)")
    print("  [g] Gripper'i hareket ettir (jog)")
    print("  [d] Su anki durumu goster")
    print("  [l] Eklenen adimlari listele")
    print("  [a] Su anki durumu adim olarak ekle (gripper sabit pozisyona gider)")
    print("  [k] Gripper'i KAVRAT ve bunu adim olarak ekle (sabit pozisyon degil - nesne")
    print("      buyuklugune gore gerektigi kadar kapanip tutar)")
    print("  [w] Bekleme adimi ekle (saniye cinsinden)")
    print("  [x] Bir adimi sil")
    print("  [b] Kaydi bitir ve kaydet")
    print("  [i] Iptal et (hicbir sey kaydedilmez/degistirilmez)\n")

    default_speed_pct = arm.speed_pct if default_speed_pct is None else default_speed_pct
    default_accel_pct = arm.accel_pct if default_accel_pct is None else default_accel_pct
    defaults = init_step_defaults(arm, steps)

    while True:
        choice = input(f"[{len(steps)} adim] Secim (j/m/g/d/l/a/k/w/x/b/i): ").strip().lower()
        if choice == "j":
            jog_arm(arm)
        elif choice == "m":
            drag_teach_add_steps(arm, gripper, steps, name, defaults)
        elif choice == "g":
            jog_gripper(gripper)
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
        print(f"  {name:20s}  {len(m['steps'])} adim  hiz=%{m['speed_pct']:.0f} "
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
        result = arm.move_to_joints(step["joints_deg"], confirm=False, speed_pct=speed_pct, accel_pct=accel_pct)
        if not result["success"]:
            details = f" - {result['details']}" if result.get("details") else ""
            print(f"HATA: adim {i} basarisiz oldu ({result['reason']}{details}). Kalan adimlar calistirilmadi.")
            return False
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
            except GripperError as e:
                print(f"UYARI: kavrama basarisiz oldu: {e}")
        else:
            print(f"[{i}/{len(steps)}] Gripper -> pozisyon={step['gripper_position']} "
                  f"hiz={step['gripper_speed']} kuvvet={step['gripper_force']}")
            try:
                gripper.move(position=step["gripper_position"], speed=step["gripper_speed"], force=step["gripper_force"])
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

    print(f"\n>>> '{name}' oynatiliyor ({len(move['steps'])} adim, "
          f"hiz=%{move['speed_pct']:.0f}, ivme=%{move['accel_pct']:.0f})\n")

    arm = CobotArm(robot_ip, model_num, move["speed_pct"], move["accel_pct"], node_name=f"movecobot_replay_{name}")
    gripper = GripperController(sdk=arm.sdk, lock=arm.sdk_lock)
    if not gripper.is_active():
        print("Gripper aktive ediliyor...")
        gripper.activate()

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
                target=run_gripper_gui, args=(gripper,), kwargs={"standalone": True}, daemon=True,
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
  5) Kayitli bir hareketi duzenle (adim ekle/sil, hiz/ivme degistir)
  6) Kayitli bir hareketi oynat
  7) Kayitli hareketleri listele
  8) Kayitli bir hareketi sil
  9) Cikis
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
                    jog_gripper(gripper)
            elif choice == "3":
                print_state(arm, gripper)
            elif choice == "4":
                if gripper is None:
                    print("Gripper bagli olmadan hareket kaydedilemez.")
                else:
                    record_new_move(arm, gripper)
            elif choice == "5":
                if gripper is None:
                    print("Gripper bagli olmadan hareket duzenlenemez.")
                else:
                    edit_saved_move(arm, gripper)
            elif choice == "6":
                if gripper is None:
                    print("Gripper bagli degil.")
                else:
                    replay_from_menu(arm, gripper, robot_ip)
            elif choice == "7":
                list_saved_moves()
            elif choice == "8":
                delete_saved_move()
            elif choice == "9":
                break
            else:
                print("Gecersiz secim.")
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

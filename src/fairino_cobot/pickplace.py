#!/usr/bin/env python3
"""pickplace.py - MoveIt Task Constructor (MTC) ile pick & place.

NEDEN MTC: Al-tasi-birak dizisinin aşamaları (yaklasma, kavrama, nesneyi
modele ilistirme, kaldirma, birakma, geri cekilme) MoveIt'te HAZIR olarak
var - elle yol noktasi uydurmak yerine kutuphanenin kendi stage'leri
kullaniliyor:

    stages.MoveTo               -> poza git
    stages.MoveRelative         -> DUZ CIZGI yaklasma / geri cekilme
                                   (pre_grasp_approach / post_grasp_retreat)
    stages.ModifyPlanningScene  -> nesneyi ekle / ilistir / ayir (attach/detach)
    core.PipelinePlanner("ompl")-> serbest hareket planlayicisi
    core.CartesianPath          -> duz cizgi planlayicisi

GRIPPER NEDEN MTC'DE DEGIL: MTC'nin kendi kavrama stage'leri (SimpleGrasp)
gripper'i bir EKLEM olarak surer, yani sabit bir pozisyona kapatir. Bu
gripper ROS2'de eklem olarak modellenmis degil; SDK uzerinden KUVVETLE
kapaniyor ve nesnenin kalinligina gore nerede duracagini kendisi buluyor -
her nesnede calismasinin sebebi bu. Bu yuzden kol hareketlerini MTC
yonetiyor, cene kapama/acma anlarinda bizim gripper kodumuz cagriliyor.
Gorev bu yuzden gripper eylemlerinin oldugu yerlerden bolunup her parca
TEK PARCA olarak calistiriliyor.

ONKOSUL: move_group, ExecuteTaskSolutionCapability ile baslatilmis olmali
(auto_fairino_launch.sh bunu 'capabilities:=move_group/
ExecuteTaskSolutionCapability' ile yapiyor). Yoksa gorev planlanir ama
CALISTIRILAMAZ.
"""
import math
import os
import pkgutil
import sys
import tempfile

import yaml

# MTC apt ile /opt/ros/humble altina, moveit_py ise kaynaktan ws_moveit2'ye
# kuruluyor. Ikisi de "moveit" paketi altinda yasiyor ama ws_moveit2'ninki
# NORMAL paket (namespace degil), o yuzden once yuklenince MTC alt paketini
# gizliyor. moveit.__path__'i elle genisletmek bunu cozuyor (canli test
# edildi: ikisi ayni process'te sorunsuz calisiyor, ikisi de MoveIt 2.5.9).
MTC_DIST_PACKAGES = "/opt/ros/humble/local/lib/python3.10/dist-packages"


def load_mtc():
    """MTC'yi (core, stages) doner. Yuklenemezse (kurulu degilse) None."""
    try:
        if MTC_DIST_PACKAGES not in sys.path:
            sys.path.append(MTC_DIST_PACKAGES)
        import moveit
        moveit.__path__ = pkgutil.extend_path(moveit.__path__, "moveit")
        from moveit.task_constructor import core, stages
        return core, stages
    except Exception:
        return None


class PickPlace:
    """MTC gorevlerini kuran/calistiran sarmalayici.

    joints: eklem isimleri (SDK ile AYNI sirada - j1..j6).
    """

    def __init__(self, robot_desc_name, package_name, group_name, tip_link,
                 moveit_cpp_yaml, joint_names, speed_pct=10.0, accel_pct=10.0,
                 node_name="movecobot_mtc"):
        mtc = load_mtc()
        if mtc is None:
            raise RuntimeError(
                "MTC yuklenemedi. Kurulum: sudo apt install "
                "ros-humble-moveit-task-constructor-core "
                "ros-humble-moveit-task-constructor-capabilities")
        self.core, self.stages = mtc

        from moveit_configs_utils import MoveItConfigsBuilder
        import rclcpp
        from geometry_msgs.msg import PoseStamped

        self.group = group_name
        self.tip = tip_link
        self.joint_names = list(joint_names)

        cfg = (MoveItConfigsBuilder(robot_desc_name, package_name=package_name)
               .moveit_cpp(file_path=moveit_cpp_yaml)
               .to_moveit_configs()).to_dict()
        # MTC'nin robot modelini kurabilmesi icin URDF/SRDF/kinematik
        # parametrelerinin NODE UZERINDE olmasi gerekiyor (topic'ten degil,
        # parametreden okuyor). params-file ile veriyoruz.
        #
        # DIKKAT: BUTUN config veriliyor, sadece robot_description* DEGIL.
        # Once sadece robot_description* gecirilmisti ve PipelinePlanner
        # "ompl" ayarlarini bulamayip sessizce CHOMP'a dusuyordu:
        #   "Failed to find 'ompl.planning_plugin' ... Using
        #    'chomp_interface/CHOMPPlanner' for now"
        # yani istedigimiz planlayici hic calismiyordu.
        params = {node_name: {"ros__parameters": dict(cfg)}}
        self._params_file = tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False)
        yaml.safe_dump(params, self._params_file)
        self._params_file.close()

        # DIKKAT: rclpy DEGIL - MTC C++ tarafinda rclcpp::Node bekliyor.
        # py_binding_tools bunun icin bir "rclcpp" Python modulu kuruyor.
        rclcpp.init()
        opts = rclcpp.NodeOptions()
        opts.arguments = ["--ros-args", "--params-file", self._params_file.name]
        self.node = rclcpp.Node(node_name, opts)

        # ROBOT MODELI BIR KEZ yuklenir ve TUM gorevlerde paylasilir.
        # Her gorevde ayri ayri loadRobotModel() cagirmak farkli model
        # nesneleri uretiyor ve planlayici ilk modele bagli kaldigi icin
        # ikinci gorevde su hata aliniyordu:
        #   "The robot model of the planning pipeline isn't the same as the
        #    task's robot model -- use Task::setRobotModel"
        _t = self.core.Task()
        _t.loadRobotModel(self.node)
        self.robot_model = _t.getRobotModel()

        # HIZ/IVME OLCEKLERI MUTLAKA VERILMELI. Verilmezse varsayilan 0
        # kaliyor ve uretilen trajektorinin zamanlamasi da sifir cikiyor;
        # controller boyle bir hedefi REDDEDIYOR:
        #   "Time between points 0 and 1 is not strictly increasing,
        #    it is 0.000000 and 0.000000"
        # Canli testte tam bu yasandi - gorev planlaniyor, "Received new
        # action goal" gorunuyor ama "Accepted" hic gelmiyordu.
        _vel = max(0.01, min(1.0, speed_pct / 100.0))
        _acc = max(0.01, min(1.0, accel_pct / 100.0))

        # POZA GITME planlayicisi: JointInterpolationPlanner.
        #
        # NEDEN OMPL DEGIL: MTC'nin PipelinePlanner'i ile uretilen yol
        # ZAMANLANMADAN geliyor - olculdu: 10 noktanin hepsinin zamani
        # 0.000. Controller boyle bir hedefi reddediyor:
        #   "Time between points 0 and 1 is not strictly increasing"
        # Zamanlamayi normalde AddTimeOptimalParameterization adaptoru
        # yapiyor; yapilandirmada VAR (dogrulandi) ama MTC kendi
        # pipeline'inda uygulamiyor. PipelinePlanner'in bir
        # 'time_parameterization' ozelligi var, ancak ona atanacak sinifin
        # Python baglantisi YOK - yani buradan duzeltilemiyor.
        #
        # JointInterpolationPlanner ise duzgun zamanlanmis yol uretiyor
        # (olculdu: 39 nokta, 0 -> 3.783s) ve zaten sanayi robotlarindaki
        # PTP (dogrudan eklem hareketi) davranisi bu.
        #
        # TAKAS: eklem-uzayinda dogrudan gider, engelin ETRAFINDAN
        # DOLASMAZ. Ama yol yine de carpisma kontrolunden geciyor - engel
        # varsa gorev sessizce carpmak yerine BASARISIZ oluyor.
        self.jip = self.core.JointInterpolationPlanner()
        self.jip.max_velocity_scaling_factor = _vel
        self.jip.max_acceleration_scaling_factor = _acc

        self.cartesian = self.core.CartesianPath()
        self.cartesian.max_velocity_scaling_factor = _vel
        self.cartesian.max_acceleration_scaling_factor = _acc

        # MoveRelative'in hangi noktayi referans alacagi. Duz metin KABUL
        # EDILMIYOR, PoseStamped olmali.
        self.ik_frame = PoseStamped()
        self.ik_frame.header.frame_id = tip_link
        self.ik_frame.pose.orientation.w = 1.0

    def __del__(self):
        try:
            os.unlink(self._params_file.name)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Yardimcilar
    # ------------------------------------------------------------------
    def _task(self, name):
        t = self.core.Task()
        t.name = name
        t.setRobotModel(self.robot_model)   # paylasilan model - bkz. __init__
        t.add(self.stages.CurrentState("mevcut durum"))
        return t

    def _goal_dict(self, joints_deg):
        return {j: math.radians(v) for j, v in zip(self.joint_names, joints_deg)}

    def _move_to(self, task, ad, joints_deg):
        s = self.stages.MoveTo(ad, self.jip)   # bkz. __init__ - OMPL degil
        s.group = self.group
        s.setGoal(self._goal_dict(joints_deg))
        task.add(s)

    def _move_relative_z(self, task, ad, mesafe_m):
        """TCP'nin kendi Z ekseninde DUZ CIZGI hareket (+ ileri, - geri)."""
        from geometry_msgs.msg import Vector3Stamped
        s = self.stages.MoveRelative(ad, self.cartesian)
        s.group = self.group
        s.ik_frame = self.ik_frame
        s.min_distance = abs(mesafe_m) * 0.5   # en az yarisi gerceklesmeli
        s.max_distance = abs(mesafe_m)
        v = Vector3Stamped()
        v.header.frame_id = self.tip
        v.vector.z = mesafe_m
        s.setDirection(v)
        task.add(s)

    def _calistir(self, task, baslik):
        """Gorevi planla ve TEK PARCA calistir."""
        try:
            task.plan()
        except Exception as e:
            print(f"  {baslik}: planlama hatasi ({type(e).__name__}: {e})")
            return False
        if not len(task.solutions):
            print(f"  {baslik}: cozum bulunamadi.")
            return False
        try:
            # DIKKAT: execute() hata firlatmiyor, MoveItErrorCode DONUYOR.
            # Once bu kontrol edilmiyordu ve "Goal was aborted or canceled"
            # logu dusmesine ragmen kod "tamamlandi" diyordu - robot hic
            # hareket etmemisken basarili sanildi.
            ec = task.execute(task.solutions[0])
            val = getattr(ec, "val", None)
            if val == 1:  # moveit_msgs SUCCESS
                print(f"  {baslik}: tamamlandi.")
                return True
            print(f"  {baslik}: CALISTIRILAMADI (hata kodu {val}). "
                  f"fairino_launch loglarina bakin.")
            return False
        except Exception as e:
            print(f"  {baslik}: calistirma hatasi ({type(e).__name__}: {e})")
            return False

    def _nesne_msg(self, boyut_mm, tcp_offset_m):
        """Kavranan nesnenin CollisionObject'i - cenelerin arasinda (TCP'de)."""
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        co = CollisionObject()
        co.header.frame_id = self.tip
        co.id = "grasped_object"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [b / 1000.0 for b in boyut_mm]
        p = Pose()
        p.position.x, p.position.y, p.position.z = tcp_offset_m
        p.orientation.w = 1.0
        co.primitives.append(box)
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD
        return co

    # ------------------------------------------------------------------
    # Asamalar (her biri TEK PARCA calisan bir MTC gorevi)
    # ------------------------------------------------------------------
    def alma_pozuna_yaklas(self, pre_pick_joints_deg, yaklasma_mm):
        """ON-POZA git, sonra DUZ CIZGI ile nesneye yaklas.
        (MoveIt semasindaki pre_grasp_approach)

        ON-POZ, kayit sirasinda alma pozundan yaklasma_mm kadar geri
        cekilerek olculur. Boylece robot kavrama noktasina DOGRUDAN
        DALMAZ - once guvenli mesafede durur, sonra duz cizgi yaklasir.
        (Ilk surumde "alma pozuna git -> geri cekil -> yaklas" yapiliyordu;
        bu hem robotu once nesnenin icine gonderiyordu hem de son yaklasma
        CartesianPath'te min_fraction'a takilip basarisiz oluyordu.)"""
        t = self._task("alma")
        self._move_to(t, "on-poza git", pre_pick_joints_deg)
        self._move_relative_z(t, "duz yaklas", +yaklasma_mm / 1000.0)
        return self._calistir(t, "Alma pozuna yaklasma")

    def kaldir_ve_birakma_pozuna_git(self, place_joints_deg, kaldirma_mm,
                                     inme_mm, boyut_mm, tcp_offset_m):
        """Nesneyi modele ILISTIR (attach), duz cizgi kaldir, birakma pozuna
        git, duz cizgi in. (post_grasp_retreat + transport + place approach)"""
        t = self._task("tasi")
        ms = self.stages.ModifyPlanningScene("nesneyi ilistir")
        ms.addObject(self._nesne_msg(boyut_mm, tcp_offset_m))
        ms.attachObject("grasped_object", self.tip)
        # Gripper govdesiyle temas normaldir - carpisma sayilmasin.
        ms.allowCollisions("grasped_object", [self.tip], True)
        # Robotun TABANI zemin kutusuna oturuyor (robot masaya cıvatalı) -
        # link padding sonrasi bu temas "carpisma" sayilip gorevi
        # dusuruyordu: "nesneyi ilistir (0/1): base_link colliding with
        # floor". Zemin kontrolu KOLUN asagi inmesi icin var, taban icin
        # degil. Sadece bu tek cift serbest.
        ms.allowCollisions("floor", ["base_link"], True)
        t.add(ms)
        self._move_relative_z(t, "nesneyi kaldir", -kaldirma_mm / 1000.0)
        # place_joints_deg burada BIRAKMA ON-POZU (bkz. alma_pozuna_yaklas):
        # once guvenli mesafeye gidilir, sonra duz cizgi inilir.
        self._move_to(t, "birakma on-pozuna git", place_joints_deg)
        self._move_relative_z(t, "duz in", +inme_mm / 1000.0)
        return self._calistir(t, "Tasima ve birakma pozuna inme")

    def ayir_ve_geri_cekil(self, geri_cekilme_mm):
        """Nesneyi modelden AYIR (detach) ve duz cizgi geri cekil.
        (post_place_retreat)"""
        t = self._task("birak")
        ms = self.stages.ModifyPlanningScene("nesneyi ayir")
        ms.detachObject("grasped_object", self.tip)
        ms.removeObject("grasped_object")
        t.add(ms)
        self._move_relative_z(t, "geri cekil", -geri_cekilme_mm / 1000.0)
        return self._calistir(t, "Ayirma ve geri cekilme")

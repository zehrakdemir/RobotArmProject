"""
Fairino FR3 - valiz duzeltme / yerlestirme komutu uretimi ve gonderimi.

GUVENLIK
--------
* ROBOT_ENABLED=false (varsayilan) iken hicbir komut kola GITMEZ; sadece
  loglanir. Goruntu isleme tarafini kol olmadan guvenle test edebilirsiniz.
* Fairino Python SDK (`fairino`) kurulu degilse otomatik olarak kuru-calisma
  (dry-run) moduna duser.

Bu modul, kamera/robot montajiniza ozel el-goz (hand-eye) kalibrasyonunu
VARSAYMAZ. Kavrama noktasi robot taban cercevesine
`src/pipeline.py::image_to_robot_frame` ile cevrilir; o fonksiyon sizin
kalibrasyon dosyanizi (HANDEYE_PATH) bekler.
"""
from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field, asdict
from typing import Optional, Sequence

from .. import config

logger = logging.getLogger(__name__)

Pose6 = Sequence[float]  # (x, y, z, rx, ry, rz)  mm & derece (FR3 konvansiyonu)


@dataclass
class FR3Command:
    action: str                       # none | rotate_upright | reposition
    reason: str
    grasp_pose: Optional[list[float]] = None      # robot taban cercevesi
    lift_pose: Optional[list[float]] = None
    place_pose: Optional[list[float]] = None
    rotation_deg: float = 0.0                      # diklestirme icin gereken donme
    tcp_speed: float = field(default_factory=lambda: config.ROBOT_TCP_SPEED)
    approach_cm: float = field(default_factory=lambda: config.ROBOT_APPROACH_CM)
    waypoints: list[list[float]] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False)


# ----------------------------------------------------------------------
def plan_correction(
    posture_label: str,
    grasp_xyz_robot: Optional[Sequence[float]],
    dims_cm: tuple[float, float, float],
    tilt_deg: float,
    lean_deg: float,
    place_xyz_robot: Optional[Sequence[float]] = None,
) -> FR3Command:
    """
    Duruş sonucuna gore FR3 hareket planini uretir (henuz gondermez).

    posture_label : pose_classify.PostureResult.label
    grasp_xyz_robot : kavrama noktasi (robot taban cerc., mm) - tepe referansi
    dims_cm       : (en, boy, derinlik)
    """
    w, h, d = dims_cm
    notes = [f"olcu en/boy/derinlik = {w:.1f}/{h:.1f}/{d:.1f} cm",
             f"2B yamukluk = {tilt_deg:.1f} deg, 3B egim = {lean_deg:.1f} deg"]

    if posture_label == "standing_on_wheels":
        if abs(tilt_deg) > config.TILT_THRESHOLD_DEG:
            notes.append("Valiz dik ama hafif yamuk; yalnizca yatay hizalama onerildi.")
            return FR3Command(
                action="reposition",
                reason="Dik durus, sadece yonelim duzeltmesi",
                grasp_pose=_mk_pose(grasp_xyz_robot),
                rotation_deg=float(-tilt_deg),
                place_pose=_mk_pose(place_xyz_robot),
                notes=notes,
            )
        notes.append("Valiz normal durusta; robot mudahalesi gerekmiyor.")
        return FR3Command(action="none", reason="Valiz tekerlekleri uzerinde dik", notes=notes)

    if posture_label == "tipped_over":
        # Yatiktan dike: kavra -> kaldir -> ekseni etrafinda ~ (90 - lean) dondur -> birak
        rot = 90.0 - lean_deg if lean_deg < 90.0 else 90.0
        grasp = _mk_pose(grasp_xyz_robot)
        lift = None
        if grasp is not None:
            lift = list(grasp)
            lift[2] += max(dims_cm) * 10.0 * 0.6   # cm -> mm, boyun %60'i kadar kaldir
        notes.append(f"Diklestirme donusu ~ {rot:.0f} deg")
        return FR3Command(
            action="rotate_upright",
            reason="Valiz yatik; tekerlek uzeri dik konuma getirilecek",
            grasp_pose=grasp,
            lift_pose=lift,
            place_pose=_mk_pose(place_xyz_robot),
            rotation_deg=float(rot),
            notes=notes,
        )

    notes.append("Durus belirsiz; guvenlik icin otomatik hareket uretilmedi.")
    return FR3Command(action="none", reason="Belirsiz durus - operator onayi gerekli", notes=notes)


def _mk_pose(xyz_mm: Optional[Sequence[float]]) -> Optional[list[float]]:
    if xyz_mm is None:
        return None
    x, y, z = (float(v) for v in xyz_mm[:3])
    # Varsayilan olarak dikey (tool asagi bakar) yaklasim: rx=180, ry=0, rz=0
    return [x, y, z, 180.0, 0.0, 0.0]


# ----------------------------------------------------------------------
class FairinoFR3:
    """FR3 baglantisi + komut gonderimi (guvenli varsayilan: dry-run).

    NOT: Fairino Python SDK (`Robot.RPC`) SimMachine v3.9.x'te arka plan
    state-thread'i (port 20004) yuzunden ILK `Mode()`/`MoveJ()` cagrisinda
    asili kalabiliyor. Bu yuzden dogrudan HAM XML-RPC (`http://<ip>:20003/RPC2`)
    kullaniyoruz - test edildi, kolu gercekten oynatiyor.
    """

    RPC_PORT = 20003

    @property
    def HOME_JOINTS(self) -> list:
        return [float(v) for v in getattr(config, "ROBOT_HOME_JOINTS",
                                          [0.0, -90.0, 90.0, -90.0, -90.0, 0.0])][:6]

    def __init__(self, ip: Optional[str] = None, enabled: Optional[bool] = None) -> None:
        self.ip = ip or config.ROBOT_IP
        self.enabled = config.ROBOT_ENABLED if enabled is None else enabled
        self._robot = None          # xmlrpc.client.ServerProxy  ( :20003/RPC2 )
        self._connected = False

    # ------------------------------------------------------------------
    def _proxy(self):
        import xmlrpc.client
        if self._robot is None:
            self._robot = xmlrpc.client.ServerProxy(
                f"http://{self.ip}:{self.RPC_PORT}/RPC2", allow_none=True)
        return self._robot

    def _call(self, method: str, *args, timeout: float = 15.0, default=None):
        """Tek ham XML-RPC cagrisi (soket timeout'lu). Fault -> default; soket
        hatasi -> baglantiyi tazele + default."""
        import socket
        import xmlrpc.client
        old = socket.getdefaulttimeout()
        socket.setdefaulttimeout(timeout)
        try:
            return getattr(self._proxy(), method)(*args)
        except xmlrpc.client.Fault as f:
            logger.warning("FR3 %s Fault: %s", method, f)
            return default
        except Exception as exc:  # noqa: BLE001 - soket/timeout -> tazele
            logger.warning("FR3 %s hata: %s", method, exc)
            self._robot = None
            return default
        finally:
            socket.setdefaulttimeout(old)

    def connect(self) -> bool:
        if not self.enabled:
            logger.warning("ROBOT_ENABLED=false -> baglanti kurulmadi (dry-run).")
            return False
        j = self._call("GetActualJointPosDegree", 0, timeout=6.0)
        if isinstance(j, (list, tuple)) and len(j) >= 7:
            self._connected = True
            logger.info("FR3 baglandi (ham XML-RPC): %s:%d/RPC2", self.ip, self.RPC_PORT)
            return True
        logger.error("FR3 RPC baglanamadi (%s:%d/RPC2) -> dry-run", self.ip, self.RPC_PORT)
        self.enabled = False
        self._robot = None
        return False

    # ------------------------------------------------------------------
    def status(self) -> dict:
        """Guvenli, SALT-OKUNUR durum. Hareket ettirmez."""
        if not self._connected and not self.connect():
            return {"bagli": False, "ip": self.ip}
        out = {"bagli": True, "ip": self.ip}
        j = self._call("GetActualJointPosDegree", 0, timeout=6.0)
        if isinstance(j, (list, tuple)) and len(j) >= 7:
            out["eklemler_derece"] = [round(float(v), 2) for v in j[1:7]]
        t = self._call("GetActualTCPPose", 0, timeout=6.0)
        if isinstance(t, (list, tuple)) and len(t) >= 7:
            out["tcp_mm_derece"] = [round(float(v), 2) for v in t[1:7]]
        if "eklemler_derece" not in out:
            out["uyari"] = "durum okunamadi (SimMachine :20003 yanit vermedi)"
        return out

    def error_code(self) -> list:
        """[main, sub]  (0,0 = hata yok). GetRobotErrorCode ham RPC2'de calisiyor."""
        r = self._call("GetRobotErrorCode", timeout=6.0)
        return [int(r[1]), int(r[2])] if isinstance(r, (list, tuple)) and len(r) >= 3 else [-1, -1]

    def _recover(self) -> list:
        """Latch'lenmis denetleyici hatasini temizle (tekillik/config-degisim/err 8-1).
        SimMachine'de guvenilir sira: disable -> ResetAllError -> Mode(0) ->
        DragTeachSwitch(0) -> enable."""
        self._recover_count = getattr(self, "_recover_count", 0) + 1
        steps = []
        self._call("StopMotion", timeout=8.0)
        self._call("RobotEnable", 0, timeout=10.0); time.sleep(0.6)
        steps.append(f"ResetAllError -> {self._call('ResetAllError', timeout=10.0)}")
        time.sleep(0.6)
        steps.append(f"Mode(0) -> {self._call('Mode', 0, timeout=12.0)}")
        time.sleep(0.3)
        self._call("DragTeachSwitch", 0, timeout=8.0)
        steps.append(f"RobotEnable(1) -> {self._call('RobotEnable', 1, timeout=15.0)}")
        time.sleep(1.2)
        return steps

    def prepare(self) -> dict:
        """Hareket oncesi: hata temizle + otomatik mod + servo enable + hiz."""
        if not (self.enabled and self._connected):
            return {"prepared": False, "dry_run": True}
        steps = [f"hata (giris) = {self.error_code()}"]
        steps += self._recover()
        spd = int(config.ROBOT_TCP_SPEED)
        s = self._call("SetSpeed", spd, timeout=8.0); steps.append(f"SetSpeed({spd}) -> {s}")
        if config.GRIPPER_ENABLED:            # gercek gripper: kullanmadan once aktive et
            a = self._call("ActGripper", int(config.GRIPPER_INDEX), 1, timeout=10.0)
            steps.append(f"ActGripper({config.GRIPPER_INDEX},1) -> {a}")
            time.sleep(0.5)
        err1 = self.error_code(); steps.append(f"hata (cikis) = {err1}")
        if err1 == [0, 0]:
            return {"prepared": True, "steps": steps}
        return {"prepared": False, "steps": steps,
                "error": f"hata={err1} temizlenemedi. SimMachine web arayuzunde robot "
                         "'Boot mode'da olabilir - sag alttaki Enter'a basip operasyona alin."}

    # ------------------------------------------------------------------
    # Ham RPC2 imzalari (SDK Robot.py'den birebir):
    #   MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, exaxis[4], blendT, offset_flag, offset_pos[6])
    #   MoveL([jp0..5, dp0..5, tool, user, vel, acc, ovl, blendR, blendMode,
    #          exaxis0..3, search, offset_flag, off0..5, oacc, velAccParamMode])   # 33 eleman
    # FR3 eklem limitleri (derece) - IK cozumu bunun disindaysa gonderme
    _JLIM = ((-175, 175), (-265, 85), (-160, 160), (-265, 85), (-175, 175), (-175, 175))

    def _jlim_bad(self, jp) -> str:
        out = [f"J{i+1}={v:.0f}" for i, (v, (lo, hi)) in enumerate(zip(jp, self._JLIM))
               if not (lo - 2 <= v <= hi + 2)]
        return ", ".join(out)

    # ---- JOG tabanli eklem hareketi ----------------------------------
    # FR3 V6.0 / firmware 3.9.8'de ham MoveJ "Joint command point error" (kod
    # 154/101) veriyor; StartJOG saglam calisiyor. Bu yuzden eklem hareketini
    # KAPALI-DONGU JOG ile yapiyoruz: her ekseni hedefe dogru max_dis kadar
    # jogla, tolerans icine girince dur.
    def _jog_axis(self, nb: int, delta_deg: float, vel: float):
        """nb: 1..6 eklem no. delta_deg: hedefe kalan aci (isaretli)."""
        if abs(delta_deg) < 0.05:
            return
        d = 1 if delta_deg > 0 else 0
        v = float(max(2.0, min(vel, 30.0)))
        # StartJOG(ref=0 eklem, nb, dir, vel, acc, max_dis)  - max_dis sonda
        self._call("StartJOG", 0, int(nb), d, v, 50.0, float(abs(delta_deg)),
                   timeout=8.0, default="ERR")

    def _jog_to_joints(self, target6, vel: float, tol: float = 0.4, timeout_s: float = 45.0):
        tgt = [float(v) for v in target6][:6]
        bad = self._jlim_bad(tgt)
        if bad:
            raise RuntimeError(f"Hedef eklem limiti disinda ({bad})")
        import time as _t
        t0 = _t.time()
        for nb in range(1, 7):                       # eksenleri sirayla getir
            while _t.time() - t0 < timeout_s:
                cur = self._joints_now()
                if not cur:
                    _t.sleep(0.2); continue
                delta = tgt[nb - 1] - cur[nb - 1]
                if abs(delta) <= tol:
                    break
                self._jog_axis(nb, delta, vel)
                _t.sleep(min(0.15 + abs(delta) / max(vel, 1) * 0.9, 3.0))
                self._call("StopJOG", 1, timeout=5.0, default=None)
                _t.sleep(0.15)
        self._call("StopJOG", 1, timeout=5.0, default=None)
        cur = self._joints_now()
        err = max(abs(tgt[i] - cur[i]) for i in range(6)) if cur else 99
        ec = self.error_code()
        if ec != [0, 0]:
            self._recover()
            raise RuntimeError(f"JOG sirasinda hata {ec} hedef={[round(x,1) for x in tgt]}")
        if err > tol * 4:
            raise RuntimeError(f"JOG hedefe ulasamadi (kalan {err:.1f}°) "
                               f"hedef={[round(x,1) for x in tgt]}")
        return 0

    def _movej(self, joints6, vel: float, _retry: bool = True):
        j = [float(v) for v in joints6][:6]
        return self._jog_to_joints(j, vel)

    def _movel(self, pose6, vel: float):
        """SimMachine'de dogrusal MoveL siklikla tekillik/joint-config hatasi
        veriyor ('use PTP command'). Varsayilan: IK + MoveJ (PTP). Gercek dogrusal
        hareket icin ROBOT_LINEAR_MOVES=true."""
        d = [float(v) for v in pose6][:6]
        tool, user = int(config.ROBOT_TOOL_NUM), int(config.ROBOT_USER_NUM)
        ik = self._call("GetInverseKin", 0, d, -1, timeout=8.0)
        jp = None
        if isinstance(ik, (list, tuple)) and len(ik) >= 7 and int(ik[0]) == 0:
            jp = [float(v) for v in ik[1:7]]
        if jp is None:
            raise RuntimeError(
                f"Ters kinematik cozulemedi poz={[round(x, 1) for x in d]} ({ik}). "
                "Poz robotun calisma uzayi disinda - Kavrama Z / PLACE_POSE'u degistir.")
        bad = self._jlim_bad(jp)
        if bad:
            raise RuntimeError(f"IK cozumu eklem limiti disinda ({bad}) "
                               f"poz={[round(x, 1) for x in d]} - pozu degistir.")
        if not getattr(config, "ROBOT_LINEAR_MOVES", False):
            return self._movej(jp, vel)                      # PTP
        arr = (jp + d + [tool, user] + [float(vel), 0.0, 100.0] + [-1.0, 0]
               + [0.0, 0.0, 0.0, 0.0] + [0, 0] + [0.0] * 6 + [100.0, 0])
        r = self._call("MoveL", arr, timeout=30.0, default="ERR")
        if r == 0:
            return r
        logger.warning("ham MoveL kod=%s -> PTP (MoveJ) ile deneniyor", r)
        return self._movej(jp, vel)

    def _joints_now(self) -> list[float]:
        j = self._call("GetActualJointPosDegree", 0, timeout=6.0)
        return [float(v) for v in j[1:7]] if isinstance(j, (list, tuple)) and len(j) >= 7 else []

    # ------------------------------------------------------------------
    def execute(self, cmd: FR3Command) -> dict:
        """Komutu uygular. enabled=false ise sadece plan dondurur."""
        result = {"dispatched": False, "dry_run": not self.enabled, "steps": []}

        if cmd.action == "none":
            result["steps"].append("Hareket gerekmiyor.")
            logger.info("FR3: %s", cmd.reason)
            return result

        if not self.enabled or not self._connected:
            for step in self._plan_steps(cmd):
                result["steps"].append(f"[DRY-RUN] {step}")
                logger.info("[DRY-RUN] %s", step)
            return result

        try:
            spd = cmd.tcp_speed
            if cmd.grasp_pose:
                approach = list(cmd.grasp_pose)
                approach[2] += cmd.approach_cm * 10.0
                self._movel(approach, spd); result["steps"].append(f"MoveL yaklasim {approach}")
                self._movel(cmd.grasp_pose, spd); result["steps"].append("MoveL kavrama")
                self._grip(close=True); result["steps"].append("Gripper KAPAT")
            if cmd.lift_pose:
                self._movel(cmd.lift_pose, spd); result["steps"].append("MoveL kaldir")
            if cmd.rotation_deg:
                jp = self._joints_now()
                jp[5] += cmd.rotation_deg
                self._movej(jp, spd); result["steps"].append(f"MoveJ J6 += {cmd.rotation_deg:.1f} deg")
            for wp in cmd.waypoints:
                self._movel(wp, spd); result["steps"].append(f"MoveL ara nokta {wp}")
            if cmd.place_pose:
                self._movel(cmd.place_pose, spd); result["steps"].append("MoveL yerlestir")
                self._grip(close=False); result["steps"].append("Gripper AC")
            result["dispatched"] = True
            logger.info("FR3 komut tamam: %s", cmd.action)
        except Exception as exc:  # noqa: BLE001
            logger.exception("FR3 hareket hatasi")
            result["error"] = str(exc)
        return result

    # ------------------------------------------------------------------
    def _grip(self, close: bool) -> None:
        self.move_gripper(config.GRIPPER_CLOSE_VALUE if close else config.GRIPPER_OPEN_VALUE)

    def move_gripper(self, value: int) -> tuple[bool, str]:
        """Jodell gripper, poz 0..100 (ham RPC2: MoveGripper).

        Doner: (gonderildi_mi, aciklama). GRIPPER_ENABLED=false iken (SimMachine
        varsayilani) HICBIR sey gonderilmez - MoveGripper cagrisi sim hareket
        denetleyicisini hataya sokuyor.
        """
        if not (self.enabled and self._connected):
            logger.info("[DRY-RUN] Gripper -> %d", value)
            return False, "dry-run"
        if not config.GRIPPER_ENABLED:
            logger.info("Gripper atlandi (GRIPPER_ENABLED=false) -> %d", value)
            return False, "atlandi (GRIPPER_ENABLED=false)"
        # ham RPC2: MoveGripper(index,pos,vel,force,maxtime,block,type,rotNum,rotVel,rotTorque)
        r = self._call("MoveGripper", int(config.GRIPPER_INDEX), int(value),
                       int(config.GRIPPER_SPEED), int(config.GRIPPER_FORCE),
                       10000, 1, 0, 0.0, 0, 0, timeout=15.0, default="ERR")
        if r == 0:
            return True, "MoveGripper ok"
        sd = self._call("SetDO", 0, 0 if value >= 50 else 1, timeout=6.0)
        return (sd == 0), f"MoveGripper={r}, SetDO={sd}"

    # ------------------------------------------------------------------
    def run_program(self, program: "CellProgram") -> dict:
        """CellProgram (grasp.py / align.py planindan uretilen) adimlarini uygular."""
        result = {"dispatched": False, "dry_run": not self.enabled,
                  "action": program.action, "steps": []}
        if not program.feasible:
            result["steps"].append(f"UYGULANAMAZ: {program.reason}")
            return result

        live = self.enabled and self._connected
        spd = config.ROBOT_TCP_SPEED
        self._recover_count = 0
        if live:
            prep = self.prepare()
            result["hazirlik"] = prep
            if not prep.get("prepared"):
                result["steps"].append(f"HAZIRLIK BASARISIZ: {prep.get('error')}")
                return result
            # her calismayi bilinen, tekil-olmayan HOME pozundan baslat (tekrar uretilebilirlik)
            try:
                self._movej(self.HOME_JOINTS, spd)
                result["steps"].append("HOME'a git (baslangic)")
            except Exception as exc:  # noqa: BLE001
                result["steps"].append(f"HOME'a gidilemedi: {exc}")
            result["eklem_baslangic"] = [round(v, 2) for v in self._joints_now()]
        try:
            for kind, arg, note in program.steps:
                tag = "" if live else "[DRY-RUN] "
                result["steps"].append(f"{tag}{note}")
                if not live:
                    continue
                if kind == "movel":
                    self._movel(arg, spd)
                elif kind == "movej":
                    self._movej(arg, spd)
                elif kind == "gripper":
                    _sent, _why = self.move_gripper(int(arg))
                    result["steps"][-1] += f"  [{_why}]"
                elif kind == "wait":
                    time.sleep(float(arg))
                if kind in ("movel", "movej"):
                    ec = self.error_code()
                    if ec != [0, 0]:
                        raise RuntimeError(f"adimdan sonra denetleyici hatasi {ec}")
            result["dispatched"] = live
        except Exception as exc:  # noqa: BLE001
            logger.exception("FR3 program hatasi")
            result["error"] = str(exc)
        if live:
            result["eklem_bitis"] = [round(v, 2) for v in self._joints_now()]
            b, e = result.get("eklem_baslangic"), result.get("eklem_bitis")
            if b and e and len(b) == len(e):
                result["kol_oynadi"] = any(abs(x - y) > 0.5 for x, y in zip(b, e))
            rc = getattr(self, "_recover_count", 0) - 1   # prepare()'deki ilk _recover haric
            if rc > 0:
                result["denetleyici_kurtarma"] = rc
                result["steps"].append(f"NOT: {rc} adimda tekillik/hata -> kurtarma + tekrar")
        return result

    @staticmethod
    def _plan_steps(cmd: FR3Command) -> list[str]:
        steps = [f"action={cmd.action} ({cmd.reason})"]
        if cmd.grasp_pose:
            steps.append(f"yaklasim + kavrama @ {cmd.grasp_pose}")
        if cmd.lift_pose:
            steps.append(f"kaldir @ {cmd.lift_pose}")
        if cmd.rotation_deg:
            steps.append(f"J6 donusu {cmd.rotation_deg:.1f} deg (diklestirme)")
        if cmd.place_pose:
            steps.append(f"yerlestir @ {cmd.place_pose} + birak")
        return steps


# ======================================================================
# HUCRE MODU: grasp.GraspPlan / align.TipUpPlan  ->  FR3 program
# ======================================================================
@dataclass
class CellProgram:
    action: str                        # pick_place | tipup | none
    feasible: bool
    reason: str
    # (kind, arg, note)  kind: movel|movej|gripper|wait
    steps: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "action": self.action, "uygulanabilir": self.feasible, "sebep": self.reason,
            "adimlar": [{"tip": k, "arg": (list(a) if hasattr(a, "__iter__") else a),
                         "aciklama": n} for k, a, n in self.steps],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False)


def _pose(xyz, rpy) -> list[float]:
    return [round(float(xyz[0]), 2), round(float(xyz[1]), 2), round(float(xyz[2]), 2),
            round(float(rpy[0]), 2), round(float(rpy[1]), 2), round(float(rpy[2]), 2)]


def build_pick_place(grasp_plan, place_pose: Optional[Sequence[float]] = None) -> CellProgram:
    """grasp.GraspPlan -> yaklas/kavra/kaldir/tasi/birak FR3 programi."""
    if not grasp_plan.feasible:
        return CellProgram("none", False, grasp_plan.reason)

    place = list(place_pose) if place_pose is not None else list(config.PLACE_POSE)
    rpy = grasp_plan.rpy_deg
    g = grasp_plan.grasp_point
    ap = grasp_plan.approach_point
    lift = grasp_plan.lift_point
    above_place = list(place)
    above_place[2] += config.GRASP_LIFT_MM

    steps = [
        ("gripper", config.GRIPPER_OPEN_VALUE, "Gripper AC"),
        ("movel", _pose(ap, rpy), f"Yaklasim (kavrama ustu +{config.GRASP_APPROACH_MM:.0f}mm)"),
        ("movel", _pose(g, rpy), f"Kavrama noktasi (2 duz yuz ortasi, acilma {grasp_plan.opening_mm:.0f}mm)"),
        ("gripper", config.GRIPPER_CLOSE_VALUE, "Gripper KAPAT (kavra)"),
        ("wait", 0.4, "otur"),
        ("movel", _pose(lift, rpy), f"Kaldir +{config.GRASP_LIFT_MM:.0f}mm"),
        ("movel", above_place, "Hedef ustu"),
        ("movel", place, f"Yerlestir (hedef poz, yaw {config.TARGET_YAW_DEG:.0f})"),
        ("gripper", config.GRIPPER_OPEN_VALUE, "Gripper AC (birak)"),
        ("movel", above_place, "Geri cekil"),
    ]
    return CellProgram("pick_place", True, grasp_plan.reason, steps)


def build_tipup(tipup_plan) -> CellProgram:
    """align.TipUpPlan -> non-prehensile dik konuma getirme programi."""
    if not tipup_plan.feasible:
        return CellProgram("none", False, tipup_plan.reason)

    rpy = tipup_plan.rpy_deg
    c = tipup_plan.contact_point
    pushed = [c[i] + tipup_plan.push_dir[i] * tipup_plan.push_distance_mm for i in range(3)]
    steps = [
        ("gripper", config.GRIPPER_CLOSE_VALUE, "Gripper KAPAT (rijit uc)"),
        ("movel", _pose(tipup_plan.pre_contact, rpy), "Temas oncesi (geri+yukari)"),
        ("movel", _pose(c, rpy), f"Temas: kisa dikey yuz, %{config.TIPUP_PUSH_HEIGHT_FRAC*100:.0f} yukseklik"),
        ("movel", _pose(pushed, rpy), f"IT: {tipup_plan.push_distance_mm:.0f}mm ileri (dik konuma devir)"),
        ("wait", 0.5, "otur"),
        ("movel", _pose(tipup_plan.retract_point, rpy), "Geri cekil"),
    ]
    reason = tipup_plan.reason + " [DRY-RUN dogrula; sonra tekrar goruntu al]"
    return CellProgram("tipup", True, reason, steps)

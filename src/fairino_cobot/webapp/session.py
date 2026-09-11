"""Robot oturumu: CobotArm + GripperController'i tek yerden yonetir.

TASARIM KARARLARI ve NEDENLERI

1. TEK OTURUM, TEK BAGLANTI.
   Fairino'nun gercek-zamanli durum soketi tek-tuketicili; ayni robota iki
   ayri surecten SDK ile baglanmak ikisini de kilitleyebiliyor (canli testte
   yasandi). Bu yuzden web sunucusu SDK baglantisini bir kez acar ve butun
   istekler bu tek oturum uzerinden gider.

2. HAREKETLER SIRAYA GIRER.
   Iki tarayici sekmesi ayni anda "git" derse robota ayni anda iki hedef
   gitmemeli. Butun hareketler tek bir kilidin arkasinda; kilit mesgulken
   gelen istek beklemez, HEMEN "mesgul" doner - kullanici ne oldugunu bilir.

3. HAREKETLER OLAY DONGUSUNU BLOKLAMAZ.
   Bir hareket saniyeler surebiliyor. Hepsi ayri bir is parcaciginda
   kosuyor, boylece telemetri akmaya ve acil durdurma calismaya devam eder.

4. confirm HER ZAMAN False.
   CobotArm.move_*(confirm=True) terminalde input() ile soruyor; web
   sunucusunda stdin yok, oyle bir cagri sonsuza kadar asilirdi. Onay
   arayuzde aliniyor. DIKKAT: guvenlik kontrolleri (zemin carpismasi,
   self-collision, hareket sonrasi dogrulama) confirm'den BAGIMSIZ,
   her zaman calisiyor - onay atlanmasi guvenligi azaltmiyor.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

# SDK YOLU BURADA COZULMUYOR - BILEREK.
#
# Eskiden modul seviyesinde paths.ensure_sdk_on_path() cagriliyordu ve bu,
# SDK'si olmayan bir makinede modulu IMPORT EDILEMEZ yapiyordu: PathError
# ImportError degil, o yuzden testlerdeki importorskip de yakalayamiyordu ve
# butun test toplama asamasi cokuyordu (CI dahil).
#
# Gerek de yok: SDK'ya ihtiyaci olan moduller (movecobot, gripper_controller)
# ensure_sdk_on_path()'i zaten kendileri cagiriyor ve burada ancak
# _baglanmayi_dene() icinde, yani gercekten robota baglanilacagi anda import
# ediliyorlar. Boylece oturumun DAVRANISI (kilit, yeniden deneme, is kuyrugu)
# robot/SDK olmadan da test edilebiliyor.


class Busy(RuntimeError):
    """Baska bir hareket surerken yeni hareket istendi."""


@dataclass
class Job:
    """Suren ya da bitmis bir is (hareket, oynatma, kavrama...)."""

    id: int
    kind: str
    label: str
    started_at: float
    finished_at: float | None = None
    ok: bool | None = None
    detail: str = ""
    log: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
            "label": self.label,
            "running": self.finished_at is None,
            "ok": self.ok,
            "detail": self.detail,
            "log": self.log[-40:],
            "seconds": round((self.finished_at or time.time()) - self.started_at, 1),
        }


class RobotSession:
    """Kola ve gripper'a tek giris noktasi."""

    def __init__(self, speed_pct: float = 10.0, accel_pct: float = 10.0):
        self.speed_pct = speed_pct
        self.accel_pct = accel_pct

        self.arm = None
        self.gripper = None
        self.robot_ip: str | None = None
        self.model_num: str | None = None

        self.ready = False
        self.startup_error: str | None = None
        self.startup_log: list[str] = []

        self._motion_lock = threading.Lock()
        self._job_seq = 0
        self._job: Job | None = None
        self._job_lock = threading.Lock()

    # -- baslangic ---------------------------------------------------------

    def start(self) -> None:
        """Hazir olana kadar DENEMEYE DEVAM EDER.

        Eskiden tek seferlikti: basarisiz olursa arayuz kalici olarak
        "baglanilamadi" ekraninda kaliyor ve container yeniden
        baslatilmadan duzelmiyordu. Oysa sebep cogu zaman gecici:
        robot henuz acilmamis, WebApp'ten enable edilmemis, kablo yeni
        takilmis, ya da move_group hala hazirlaniyor.

        Artik arka planda denemeye devam ediyor; robot hazir olunca arayuz
        kendiliginden acilir - elle mudahale gerekmez.
        """
        bekleme = 10
        while not self.ready:
            self._baglanmayi_dene()
            if self.ready:
                return
            # Hata mesaji arayuzde gorunmeye devam etsin, ama denemeler
            # sussun diye gunluk her turda sifirlanmiyor - sadece kuyruk
            # kirpiliyor (telemetry zaten son 12 satiri gonderiyor).
            time.sleep(bekleme)
            bekleme = min(bekleme * 2, 60)  # 10s -> 60s, sonra sabit

    def _baglanmayi_dene(self) -> None:
        def log(msg: str) -> None:
            self.startup_log.append(msg)
            del self.startup_log[:-40]

        try:
            from fairino_cobot.gripper_controller import GripperController
            from fairino_cobot.movecobot import CobotArm, detect_robot

            log("Robot algilaniyor...")
            info = detect_robot()
            self.robot_ip = info["ROBOT_IP"]
            self.model_num = info["ROBOT_MODEL_NUM"]
            log(f"FR{self.model_num} bulundu: {self.robot_ip}")

            # MoveIt'e baglanma bu tur icinde de birkac kez deneniyor:
            # move_group'un ROS grafinde GORUNMESI ile KULLANIMA HAZIR olmasi
            # ayni an degil - planlama sahnesi monitoru robot_description ve
            # ilk /joint_states'i bekliyor, erken denenince
            #   RuntimeError: Unable to configure planning scene monitor
            # cikiyor. Disaridaki dongu zaten sonsuza kadar tekrar ediyor;
            # buradaki kisa tekrar sadece o ilk birkac saniyeyi yumusatiyor.
            son_hata = None
            for deneme in range(1, 4):
                try:
                    log(f"MoveIt baglantisi kuruluyor (deneme {deneme}/3)...")
                    self.arm = CobotArm(
                        robot_ip=self.robot_ip,
                        model_num=self.model_num,
                        speed_pct=self.speed_pct,
                        accel_pct=self.accel_pct,
                        node_name="movecobot_web",
                    )
                    son_hata = None
                    break
                except Exception as e:
                    son_hata = e
                    log(f"  hazir degil ({type(e).__name__}), 10 sn sonra tekrar...")
                    time.sleep(10)
            if son_hata is not None:
                raise son_hata
            log("Kol hazir.")

            # Gripper KOLUN SDK baglantisini paylasiyor - ikinci bir baglanti
            # acmamak icin (bkz. modul basligi, tek-tuketicili durum soketi).
            self.gripper = GripperController(
                robot_ip=self.robot_ip, sdk=self.arm.sdk, lock=self.arm.sdk_lock
            )
            try:
                if not self.gripper.is_active():
                    log("Gripper aktive ediliyor...")
                    self.gripper.activate()
                log("Gripper hazir.")
            except Exception as e:  # gripper yoksa kol yine de kullanilabilir
                log(f"UYARI: gripper hazirlanamadi ({e}). Kol calismaya devam eder.")

            self.ready = True
            log("Hazir.")
        except TypeError as e:
            # "'_ctypes.CField' object is not subscriptable" = SDK'nin durum
            # paketi HIC olusmamis, yani robotun gercek-zamanli durum akisi
            # (port 20004) gelmiyor. XML-RPC calisiyor olabilir - robot
            # algilanir ama durum okunamaz.
            #
            # BILINEN DURUM: SimMachine simulatoru bu akisi HIC sunmuyor.
            # Ayni hata native tarafta da aliniyor (olculdu), yani container'a
            # ozgu degil. Simulatorle ROS2 yigini (move_group, controller,
            # MTC) tam calisiyor; calismayan sey SDK'nin durum okumasi.
            self.startup_error = (
                f"{type(e).__name__}: {e} - robotun gercek-zamanli durum akisi "
                "(port 20004) gelmiyor. SimMachine simulatoru bu akisi "
                "SUNMUYOR; arayuz gercek robot gerektiriyor."
            )
            log(f"HATA: {self.startup_error}")
        except Exception as e:
            self.startup_error = f"{type(e).__name__}: {e}"
            log(f"HATA: {self.startup_error}")

    def shutdown(self) -> None:
        if self.arm is not None:
            try:
                self.arm.shutdown()
            except Exception:
                pass

    # -- durum -------------------------------------------------------------

    def telemetry(self) -> dict[str, Any]:
        """Arayuzun surekli okudugu canli durum. Hicbir zaman istisna
        firlatmaz - tek bir okuma hatasi arayuzu karartmamali."""
        out: dict[str, Any] = {
            "ready": self.ready,
            "startup_error": self.startup_error,
            "startup_log": self.startup_log[-12:],
            "robot_ip": self.robot_ip,
            "model": f"FR{self.model_num}" if self.model_num else None,
            "speed_pct": self.speed_pct,
            "accel_pct": self.accel_pct,
            "busy": self._motion_lock.locked(),
            "job": self.job_dict(),
        }
        if not self.ready or self.arm is None:
            return out
        try:
            st = self.arm.get_current_state()
            out["joints_deg"] = st.get("joints_deg")
            out["tcp_pose"] = st.get("tcp_pose")
        except Exception as e:
            out["state_error"] = str(e)
        try:
            with self.arm.sdk_lock:
                pkg = self.arm.sdk.robot_state_pkg
                out["fault"] = {"main_code": pkg.main_code, "sub_code": pkg.sub_code}
                out["gripper_active"] = pkg.gripper_active
        except Exception:
            pass
        return out

    def gripper_status(self) -> dict[str, Any]:
        if self.gripper is None:
            return {"error": "gripper yok"}
        try:
            return self.gripper.status()
        except Exception as e:
            return {"error": str(e)}

    # -- isler -------------------------------------------------------------

    def job_dict(self) -> dict[str, Any] | None:
        with self._job_lock:
            return self._job.as_dict() if self._job else None

    def run_job(self, kind: str, label: str, fn: Callable[[Job], Any]) -> Job:
        """fn'i ayri bir is parcaciginda calistirir. Kilit mesgulse Busy
        firlatir - kuyruga alip beklemek yerine hemen soylemek daha iyi,
        cunku kullanici robotun ne yaptigini gormek istiyor."""
        if not self._motion_lock.acquire(blocking=False):
            raise Busy("Baska bir hareket suruyor.")

        with self._job_lock:
            self._job_seq += 1
            job = Job(id=self._job_seq, kind=kind, label=label, started_at=time.time())
            self._job = job

        def runner() -> None:
            try:
                sonuc = fn(job)
                if isinstance(sonuc, dict) and "success" in sonuc:
                    job.ok = bool(sonuc["success"])
                    if not job.ok:
                        job.detail = sonuc.get("details") or sonuc.get("reason") or "basarisiz"
                else:
                    job.ok = True
            except Exception as e:
                job.ok = False
                job.detail = f"{type(e).__name__}: {e}"
                job.log.append(job.detail)
            finally:
                job.finished_at = time.time()
                self._motion_lock.release()

        threading.Thread(target=runner, daemon=True, name=f"job-{job.id}").start()
        return job

    # -- acil durdurma -----------------------------------------------------

    def stop(self) -> dict[str, Any]:
        """Acil durdurma. Hareket kilidini BEKLEMEZ - zaten amaci suren
        hareketi kesmek. Once ROS tarafindaki yorunge iptal edilir, sonra
        SDK'nin kendi durdurma komutu gonderilir."""
        sonuc: dict[str, Any] = {}
        try:
            self.arm.stop_trajectory()  # varsa
            sonuc["ros"] = "iptal edildi"
        except AttributeError:
            sonuc["ros"] = "yok"
        except Exception as e:
            sonuc["ros"] = f"hata: {e}"
        try:
            with self.arm.sdk_lock:
                rc = self.arm.sdk.StopMotion()
            sonuc["sdk"] = f"StopMotion rc={rc}"
        except Exception as e:
            sonuc["sdk"] = f"hata: {e}"
        return sonuc

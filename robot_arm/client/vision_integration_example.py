#!/usr/bin/env python3
"""FoundationPose entegrasyonunun nasil gorunecegini gosteren ornek.
Su an gercek bir vision pipeline yok — 'foundation_pose_stub' fonksiyonu,
FoundationPose'un ileride donduregi turden bir 6D poz sozlugu uretiyor (uydurma
degerler). Gercek entegrasyonda bu fonksiyon, kameradan/FoundationPose'dan gelen
gercek veriyle degistirilecek, FairinoController.move_to_pose() cagrisi ayni kalacak."""
from fairino_controller import FairinoController


def foundation_pose_stub():
    """UYDURMA — gercek FoundationPose burada calisip valizin 6D pozunu doner."""
    return {"x_mm": -450, "y_mm": -150, "z_mm": 400, "rx_deg": 180, "ry_deg": 0, "rz_deg": 90}


def main():
    with FairinoController(speed_pct=30, accel_pct=30) as robot:
        print("Mevcut durum:", robot.get_current_state())

        detected_pose = foundation_pose_stub()
        print(f"\n[Vision'dan gelen poz (uydurma)]: {detected_pose}")

        result = robot.move_to_pose(**detected_pose, confirm=False)

        if result["success"]:
            print("Basarili:", result["final_state"])
        else:
            print("Basarisiz:", result["reason"])


if __name__ == "__main__":
    main()

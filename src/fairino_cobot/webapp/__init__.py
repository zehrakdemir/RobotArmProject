"""movecobot'un web arayuzu.

Bu paket movecobot'un mantigini TEKRAR YAZMAZ - ayni CobotArm ve
GripperController nesnelerini kullanir. Sebep basit: iki ayri uygulama iki
ayri davranisa doner ve guvenlik kontrolleri (zemin, self-collision, hareket
sonrasi dogrulama) birinde eksik kalir. Web arayuzu sadece bir kabuk.
"""

__all__ = ["session", "server"]

"""Fairino FR3 kol kontrol araclari.

Alt moduller ROS2 / MoveIt2 / Fairino SDK gerektirdigi icin BURADA import
EDILMEZ - paketi import etmek (ornegin sadece `paths` icin) robot ortami
kurulu olmayan bir makinede de calismali. Testler ve CI buna dayaniyor.
"""

__version__ = "0.2.0"

__all__ = ["paths", "__version__"]

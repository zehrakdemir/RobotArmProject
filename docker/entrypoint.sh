#!/bin/bash
# Runtime container'in giris noktasi: ROS2 ortamlarini sirayla source eder,
# sonra verilen komutu calistirir.
set -e

source /opt/ros/humble/setup.bash
source /opt/moveit_py_ws/install/setup.bash
source /opt/fairino_ws/install/setup.bash

# RT onceligi alinabiliyor mu? Alinamiyorsa kontrol dongusu 100Hz'i tutturamaz,
# ServoJ akisi duzensizlesir ve robot "axis N command speed exceeds the limit"
# faultu verebilir - canli testte tam bunu yasadik. Sessizce devam etmek yerine
# acikca uyar.
if [ "$(ulimit -r)" = "0" ]; then
    echo "UYARI: RT oncelik izni yok (ulimit -r = 0)."
    echo "       Kontrol dongusu duzensiz calisir, hareket sicramali olabilir."
    echo "       docker-compose.yml icindeki cap_add: SYS_NICE ve"
    echo "       ulimits.rtprio ayarlarinin gecerli oldugundan emin ol."
fi

exec "$@"

# syntax=docker/dockerfile:1
#
# İKİ HEDEF VAR - ne yaptıklarını karıştırma:
#
#   --target dev      Küçük ve hızlı. ROS2 YOK. Lint + test için (CI bunu kullanır).
#                     Saf Python tarafını doğrular: paths, yama aracı, kaynak sağlığı.
#
#   --target runtime  Tam ortam: ROS2 Humble + MoveIt2 + MTC + Fairino sürücüsü.
#                     Gerçek robotu sürebilir AMA host tarafında üç şey doğru
#                     ayarlanmadan ÇALIŞMAZ (host network, RT önceliği, X soketi) -
#                     bkz. docker-compose.yml ve docs/DOCKER.md.
#
# DESTEKLENEN YOL HÂLÂ NATIVE KURULUM (docs/SETUP.md). Runtime imajı
# tekrarlanabilirlik ve yeni geliştiricinin işe başlaması için var; gerçek
# robotla henüz doğrulanmadı.

# ===========================================================================
# dev - ROS2 gerektirmeyen testler ve lint
# ===========================================================================
FROM python:3.10-slim AS dev

# patch: tools/apply_ros2_patches.py vendor .patch dosyasını uygularken kullanıyor.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git patch \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Bağımlılıklar önce - kaynak değişince pip katmanı yeniden kurulmasın.
COPY requirements.txt requirements-dev.txt ./
RUN pip install --no-cache-dir -r requirements-dev.txt

COPY pyproject.toml README.md ./
COPY src/ ./src/
COPY tests/ ./tests/
COPY tools/ ./tools/
COPY ros2/ ./ros2/
COPY examples/ ./examples/
COPY .env.example ./
# Testler reponun temel dosyalarının varlığını da doğruluyor
# (tests/test_kaynak_saglik.py). Bunlar kopyalanmazsa imaj içindeki test
# koşusu host'takinden FARKLI sonuç verir — ki CI tam olarak imajın içinde
# koşuyor. Küçük dosyalar; imajın repoyu sadık temsil etmesi için varlar.
COPY Dockerfile docker-compose.yml ./

RUN pip install --no-cache-dir -e .

CMD ["pytest", "-q"]


# ===========================================================================
# runtime - gerçek robotu sürebilen tam ortam
# ===========================================================================
FROM ros:humble-ros-base AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# --- Sistem bağımlılıkları -------------------------------------------------
# ros-humble-moveit 2.5.9 apt'te VAR; ros-humble-moveit-py YOK (kontrol edildi) -
# native kurulumda MoveIt2'nin kaynaktan derlenmesinin sebebi tam olarak bu.
# Aşağıda sadece moveit_py, apt'ten gelen 2.5.9'a karşı derleniyor: tam kaynak
# derlemesinden çok daha hafif, sürüm de birebir aynı kalıyor.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-moveit \
        ros-humble-moveit-task-constructor-core \
        # -core kütüphaneyi verir ama move_group'un ExecuteTaskSolutionCapability
        # eklentisini VERMEZ; o ayrı paketten geliyor. Eksikken move_group şunu
        # basıp devam ediyor ve pick & place sessizce çalışmıyor:
        #   "Exception while loading move_group capability
        #    'move_group/ExecuteTaskSolutionCapability': ... does not exist"
        ros-humble-moveit-task-constructor-capabilities \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-joint-trajectory-controller \
        ros-humble-pybind11-vendor \
        ros-humble-rclpy \
        ros-humble-rviz2 \
        ros-humble-xacro \
        python3-colcon-common-extensions \
        python3-pip python3-tk \
        build-essential cmake git patch \
    && rm -rf /var/lib/apt/lists/* \
    # MTC çalıştırma eklentisi gerçekten geldi mi? Gelmezse pick & place
    # sessizce çalışmaz — build sırasında yakalansın.
    && test -f /opt/ros/humble/share/moveit_task_constructor_capabilities/capabilities_plugin_description.xml

# DİKKAT — TAG DEĞİL, COMMIT. `2.5.9` tag'inde moveit_py paketi HENÜZ YOK
# (kontrol edildi); native kurulum humble dalında, tag'den 11 commit sonrasında.
# Aşağıdaki commit native ortamda çalışan sürümün ta kendisi — imaj ile native
# birebir aynı moveit_py'yi kullansın diye sabitlendi.
#
# Bu ayrım sinsi bir hataya yol açmıştı: `colcon build --packages-select
# moveit_py` paketi bulamayınca UYARI verip 0 ile çıkıyor, yani build
# "başarılı" görünüyor ama install/ bomboş kalıyordu. Aşağıdaki test'ler
# tam olarak bunu yakalamak için var.
ARG MOVEIT_COMMIT=2b2030dd73881aceeaa759e5622c57c99adf2f9c
# frcobot_ros2'nin bu tag'i, fairino_hardware'in v3.8.2.11'den v3.9.9'a kadar
# TÜM sürümlerini içinde taşıyor. Native kurulumdaki "fairino_official_mirror"
# klasörünün kaynağı da tam olarak budur — özel bir şey değil.
ARG FRCOBOT_TAG=V3.0.0_RobotV3.9.9
# İmajda hangi robot/sürüm için derleneceği. setup_target bu sürüme uyan
# fairino_hardware klasörünü seçip kuruyor.
ARG ROBOT_MODEL=3
ARG ROBOT_VER_MAJ=3
ARG ROBOT_VER_MIN=9
ARG ROBOT_VER_PATCH=9

# --- moveit_py -------------------------------------------------------------
# Bağımlılıklarının hepsi (moveit_core, moveit_ros_planning_interface, rclcpp,
# pybind11_vendor...) apt'te ve tam 2.5.9 — kontrol edildi. Bu yüzden tam
# kaynak derlemesi yerine yalnızca bu paket derleniyor.
WORKDIR /opt/moveit_py_ws
RUN mkdir -p src/moveit2 && cd src/moveit2 \
    && git init -q \
    && git remote add origin https://github.com/moveit/moveit2.git \
    && git fetch -q --depth 1 origin ${MOVEIT_COMMIT} \
    && git checkout -q FETCH_HEAD \
    && cd /opt/moveit_py_ws \
    # Paket gerçekten var mı? Yoksa colcon sessizce hiçbir şey yapmaz.
    && test -d src/moveit2/moveit_py \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select moveit_py --cmake-args -DCMAKE_BUILD_TYPE=Release \
    # Derleme gerçekten bir şey üretti mi?
    && test -d install/moveit_py \
    && rm -rf build log src

# Asıl kanıt: uygulamanın kullandığı import gerçekten çalışıyor mu.
# Bu satır olmasaydı imaj "başarıyla" derlenip robotu hiç süremezdi.
RUN source /opt/ros/humble/setup.bash \
    && source /opt/moveit_py_ws/install/setup.bash \
    && python3 -c "from moveit.planning import MoveItPy; from moveit.core.robot_state import RobotState; print('moveit_py OK')"

# --- Fairino Python SDK ----------------------------------------------------
# PyPI'de yok, klonlanıyor. paths.py bunu FAIRINO_SDK_DIR üzerinden buluyor.
RUN git clone --depth 1 https://github.com/FAIR-INNOVATION/fairino-python-sdk.git \
        /opt/fairino-python-sdk

# --- Vendor kaynak aynası --------------------------------------------------
RUN git clone --depth 1 --branch ${FRCOBOT_TAG} \
        https://github.com/FAIR-INNOVATION/frcobot_ros2.git /opt/fairino_mirror

ENV FAIRINO_SDK_DIR=/opt/fairino-python-sdk/linux
ENV FAIRINO_ROS2_WS=/opt/fairino_ws
# scripts/fairino_launch.sh bunu source ediyor. Verilmezse native varsayılana
# ($HOME/ws_moveit2) düşüyor ve container'da o yol yok — launch tam bu satırda
# "No such file or directory" ile duruyordu.
ENV FAIRINO_MOVEIT_WS=/opt/moveit_py_ws
ENV FAIRINO_MIRROR_DIR=/opt/fairino_mirror
ENV FAIRINO_PATCHES_DIR=/app/ros2/patches

# --- Bizim paket -----------------------------------------------------------
WORKDIR /app
# Ubuntu 22.04'ün pip'i (22.0.2) PEP 660 desteklemiyor: setup.py/setup.cfg
# olmayan, yalnızca pyproject.toml'lu bir projeyi "-e" ile kuramıyor
# ("cannot be installed in editable mode"). Editable kurulum bizde gerekli,
# çünkü paths.py depo kökünü __file__ üzerinden buluyor (.env, ros2/patches).
RUN pip install --no-cache-dir --upgrade pip setuptools wheel
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt
COPY pyproject.toml README.md ./
COPY src/ ./src/
COPY scripts/ ./scripts/
COPY tools/ ./tools/
COPY ros2/ ./ros2/
COPY examples/ ./examples/
COPY .env.example ./
RUN pip install --no-cache-dir -e .

# --- Fairino sürücüsü: NATIVE İLE AYNI KOD YOLU ----------------------------
# Sürücüyü elle kurmak yerine kendi setup_target'ımız çalıştırılıyor: doğru
# sürüm klasörünü seçer, paketi yeniden adlandırır, TAM sürücü yamamızı
# uygular (UDP servo kanalı, ölçülen cmdT, hız sınırlayıcı, boşta servo
# çıkışı) ve derler. Böylece imajın içi ile native kurulum birbirinden
# AYRILAMAZ — ikisi de aynı kodu çalıştırıyor.
#
# robot_ip olarak 127.0.0.1 veriliyor; çalışma zamanında gerçek IP
# .env / FAIRINO_ROBOT_IP ile geliyor, xacro'daki değer yalnızca varsayılan.
RUN mkdir -p /opt/fairino_ws/src/frcobot_ros2 \
    && source /opt/ros/humble/setup.bash \
    && source /opt/moveit_py_ws/install/setup.bash \
    && fairino-setup-target ${ROBOT_MODEL} ${ROBOT_VER_MAJ} ${ROBOT_VER_MIN} ${ROBOT_VER_PATCH} 127.0.0.1

# --- MoveIt config yamaları (tüm model paketlerine) ------------------------
RUN python3 /app/tools/apply_ros2_patches.py --frcobot /opt/fairino_ws/src/frcobot_ros2 \
    && source /opt/ros/humble/setup.bash \
    && source /opt/moveit_py_ws/install/setup.bash \
    && cd /opt/fairino_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Kayıtlı hareketler ve durum önbelleği: imajın içinde DEĞİL, bind-mount edilen
# bir hacimde dursun ki container silinince kaybolmasın (docker-compose.yml).
ENV FAIRINO_STATE_DIR=/state
RUN mkdir -p /state

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

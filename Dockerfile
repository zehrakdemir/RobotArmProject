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
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-joint-trajectory-controller \
        ros-humble-rviz2 \
        ros-humble-xacro \
        python3-colcon-common-extensions \
        python3-pip python3-tk python3-vcstool \
        build-essential cmake git patch \
    && rm -rf /var/lib/apt/lists/*

ARG MOVEIT_TAG=2.5.9
ARG FRCOBOT_TAG=V3.0.0_RobotV3.9.3

# --- moveit_py -------------------------------------------------------------
WORKDIR /opt/moveit_py_ws
RUN git clone --depth 1 --branch ${MOVEIT_TAG} https://github.com/moveit/moveit2.git src/moveit2 \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select moveit_py --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log src

# --- Fairino ROS2 sürücüsü + bizim yamalarımız -----------------------------
# Yamalar tools/apply_ros2_patches.py ile uygulanıyor: aynı araç, aynı sonuç -
# imajın içi ile native kurulumun birbirinden ayrılmaması için.
WORKDIR /opt/fairino_ws
RUN git clone --depth 1 --branch ${FRCOBOT_TAG} \
        https://github.com/FAIR-INNOVATION/frcobot_ros2.git src/frcobot_ros2

COPY ros2/ /opt/fairino_cobot/ros2/
COPY tools/ /opt/fairino_cobot/tools/
RUN python3 /opt/fairino_cobot/tools/apply_ros2_patches.py \
        --frcobot /opt/fairino_ws/src/frcobot_ros2 \
    && source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# --- Fairino Python SDK ----------------------------------------------------
# PyPI'de yok, klonlanıyor. paths.py bunu FAIRINO_SDK_DIR üzerinden buluyor.
RUN git clone --depth 1 https://github.com/FAIR-INNOVATION/fairino-python-sdk.git \
        /opt/fairino-python-sdk
ENV FAIRINO_SDK_DIR=/opt/fairino-python-sdk/linux
ENV FAIRINO_ROS2_WS=/opt/fairino_ws

# --- Bizim paket -----------------------------------------------------------
WORKDIR /app
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt
COPY pyproject.toml README.md ./
COPY src/ ./src/
COPY scripts/ ./scripts/
COPY examples/ ./examples/
COPY .env.example ./
RUN pip install --no-cache-dir -e .

# Kayıtlı hareketler ve durum önbelleği: imajın içinde DEĞİL, bind-mount edilen
# bir hacimde dursun ki container silinince kaybolmasın (docker-compose.yml).
ENV FAIRINO_STATE_DIR=/state
RUN mkdir -p /state

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

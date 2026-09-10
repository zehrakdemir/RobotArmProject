# Ubuntu tabanli, CPU. GPU icin: FROM pytorch/pytorch:...-cuda ve
# opencv-contrib-python-headless yerine tam surum.
FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    MPLBACKEND=Agg

# OpenCV + ultralytics runtime bagimliliklari
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 libgomp1 \
        wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Once torch'u CPU tekeriyle kur (imaj kucuk kalsin)
COPY requirements.txt .
RUN pip install --upgrade pip && \
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip install -r requirements.txt

COPY . .

# YOLO26-seg agirligi: varsa image'a kopyalanir; yoksa ilk calistirmada
# fallback (yolov8s-seg.pt) otomatik indirilir.
EXPOSE 8501
HEALTHCHECK CMD wget -qO- http://localhost:8501/_stcore/health || exit 1

CMD ["streamlit", "run", "app.py", \
     "--server.address=0.0.0.0", "--server.port=8501", \
     "--server.headless=true", "--server.maxUploadSize=50"]

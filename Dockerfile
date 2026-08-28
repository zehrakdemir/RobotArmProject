FROM pytorch/pytorch:2.2.0-cuda12.1-cudnn8-runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libopencv-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# PyTorch 2.2.0 ile tam uyumlu torchvision ve transformers sürümlerini ekliyoruz
RUN pip install --no-cache-dir \
    "typing_extensions<=4.10.0" \
    "numpy<2" \
    torchvision==0.17.0 \
    ultralytics \
    opencv-python \
    matplotlib \
    open3d \
    transformers==4.40.0

WORKDIR /workspace
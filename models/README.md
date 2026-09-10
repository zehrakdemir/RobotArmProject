# models/

Kendi egittiginiz **YOLO26-seg** agirligini buraya koyun, orn:

    models/yolo26s-seg.pt

Sonra `.env` icinde:

    SEG_WEIGHTS=models/yolo26s-seg.pt

Agirlik yoksa sistem otomatik olarak `SEG_WEIGHTS_FALLBACK`
(`yolov8s-seg.pt`, COCO 'suitcase' sinifi 28) indirir ve onunla calisir.

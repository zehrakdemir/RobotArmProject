"""MTC (MoveIt Task Constructor) cozumunu onizleme yoluna cevirmenin SAF
(bagimliliksiz) kismi.

NEDEN AYRI MODUL: movecobot.py import edilirken rclpy, moveit_py ve Fairino
SDK'si gerekiyor. Bu donusum robot olmadan test edilebilsin diye burada
(bkz. yol_onizleme.py, ik_secimi.py, safety.py ile ayni gerekce).

MTC'nin cozumu (bkz. pickplace.py'deki cozum_asamalari) her alt-gorevi
(stage) ayri bir "sub_trajectory" olarak veriyor; bazilari (FixedState,
ModifyPlanningScene) hareket icermez (0 nokta). Zaman her alt-gorevde
sifirdan basliyor ve eklem adi sirasi garanti degil. Bu modul TUM gorevleri
(pick&place zinciri 3 MTC Task'a bolunmus - bkz. pickplace.py) tek bir
(points_deg, times_s) dizisine cevirir; bu, yol_onizleme.secenek_ozeti'nin
ve viewer.js'in zaten bildigi bicimin AYNISI.
"""

from __future__ import annotations

import math

EKLEM_ADLARI = ("j1", "j2", "j3", "j4", "j5", "j6")

# Iki ardisik noktanin zamani esit (ya da EPS'ten daha az farkli) cikarsa
# controller'in "Time between points is not strictly increasing" reddini
# (bkz. pickplace.py:120-125) tekrar etmemek icin bu kadar ileri itilir.
ZAMAN_EPSILON_S = 1e-3

# Ayni noktanin (bir gorevin sonu = sonraki gorevin baslangici) tekrar
# gorunmesini eleme toleransi (derece).
DIKIS_TOLERANS_DEG = 1e-3


class MtcYoluHatasi(ValueError):
    """Bir alt-gorevin eklem adlari eklem_adlari kumesini kapsamiyor."""


def _stage_noktalarini_deg_cevir(stage: dict, eklem_adlari: tuple[str, ...]) -> list[tuple[list[float], float]]:
    """Bir alt-gorevin (stage) noktalarini (derece, saniye) cifti listesine
    cevirir. Stage hareketsizse (joint_names bos ya da nokta yok) bos liste
    doner - bu bir hata degil (FixedState/ModifyPlanningScene icin normal)."""
    joint_names = list(stage.get("joint_names") or [])
    noktalar = stage.get("noktalar") or []
    if not joint_names or not noktalar:
        return []
    eksik = [j for j in eklem_adlari if j not in joint_names]
    if eksik:
        raise MtcYoluHatasi(
            f"stage {stage.get('ad')!r} eklem(ler)i eksik: {eksik} "
            f"(mevcut: {joint_names})")
    idx = [joint_names.index(j) for j in eklem_adlari]
    return [([math.degrees(pos[i]) for i in idx], float(t)) for pos, t in noktalar]


def asamalari_birlestir(
    gorevler: list[list[dict]],
    eklem_adlari: tuple[str, ...] = EKLEM_ADLARI,
    bosluk_s: float = 0.0,
) -> tuple[list[list[float]], list[float], list[dict]]:
    """MTC zincirinin (birden fazla Task'a bolunmus) cozümünü TEK bir
    (noktalar_deg, zamanlar_s, asama_bilgisi) dizisine cevirir.

    gorevler: her biri BIR MTC Task'in stage listesi (bkz. pickplace.py
      cozum_asamalari), sirayla [pick_task, transfer_task, place_task] gibi.
      Her stage: {"ad": str, "joint_names": [...], "noktalar": [([rad,...], t_s), ...]}.
    bosluk_s: gorevler ARASINA (gripper'in kapanacagi/acilacagi an) eklenen
      bekleme suresi. Gorev ICINDEKI stage'ler arasinda bosluk YOK (ayni
      hareketin devami).

    _plani_acilara_cevir ile AYNI sozlesme: dereceler, baslangictan
    itibaren biriken saniye, strictly-increasing zaman.
    """
    noktalar_deg: list[list[float]] = []
    zamanlar_s: list[float] = []
    asama_bilgisi: list[dict] = []
    zaman_ofset = 0.0

    for gorev_sirasi, stage_listesi in enumerate(gorevler):
        for stage in stage_listesi:
            cift = _stage_noktalarini_deg_cevir(stage, eklem_adlari)
            if not cift:
                # Hareketsiz stage (FixedState/ModifyPlanningScene) - yol
                # noktasi yok ama gorunur olsun diye sifir-uzunluklu bilgi.
                asama_bilgisi.append({
                    "ad": stage.get("ad") or "stage",
                    "tip": "sahne",
                    "ilk": len(noktalar_deg), "son": len(noktalar_deg),
                    "t0": round(zaman_ofset, 3), "t1": round(zaman_ofset, 3),
                })
                continue

            ilk_index = len(noktalar_deg)
            for i, (deg, t_yerel) in enumerate(cift):
                t_mutlak = zaman_ofset + t_yerel
                # Dikis noktasi: bu stage'in ilk noktasi, onceki noktayla
                # (ayni zaman, ayni acilar) aynıysa atla - cift sayilmasin.
                if i == 0 and noktalar_deg:
                    onceki = noktalar_deg[-1]
                    ayni_aci = all(abs(a - b) <= DIKIS_TOLERANS_DEG for a, b in zip(deg, onceki))
                    ayni_zaman = abs(t_mutlak - zamanlar_s[-1]) <= ZAMAN_EPSILON_S
                    if ayni_aci and ayni_zaman:
                        continue
                if zamanlar_s and t_mutlak <= zamanlar_s[-1]:
                    t_mutlak = zamanlar_s[-1] + ZAMAN_EPSILON_S
                noktalar_deg.append(deg)
                zamanlar_s.append(t_mutlak)

            son_index = len(noktalar_deg) - 1
            if son_index >= ilk_index:
                asama_bilgisi.append({
                    "ad": stage.get("ad") or "stage",
                    "tip": "kol",
                    "ilk": ilk_index, "son": son_index,
                    "t0": round(zamanlar_s[ilk_index], 3),
                    "t1": round(zamanlar_s[son_index], 3),
                })
            # Bu stage'in son zamanini bir sonraki stage'in ofseti yap -
            # zamanlar zaten MUTLAK olarak biriktirildigi icin ofset,
            # yalnizca stage'in KENDI ic saatinin sifirlanma noktasi.
            if cift:
                zaman_ofset = zamanlar_s[-1] if zamanlar_s else zaman_ofset

        if bosluk_s > 0 and gorev_sirasi < len(gorevler) - 1:
            t0 = zamanlar_s[-1] if zamanlar_s else 0.0
            zaman_ofset = t0 + bosluk_s
            asama_bilgisi.append({
                "ad": "gripper", "tip": "gripper",
                "ilk": len(noktalar_deg) - 1, "son": len(noktalar_deg) - 1,
                "t0": round(t0, 3), "t1": round(zaman_ofset, 3),
            })

    return noktalar_deg, zamanlar_s, asama_bilgisi

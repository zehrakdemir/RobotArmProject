"""
GIRIS NOKTASI - sayfa yonlendirme (st.navigation). Gercek icerik pages/ altinda.

2026-09-11: eski 2-fotograf demosu nav'dan kaldirildi (kod hala calisir durumda,
legacy/app_2foto_demo.py) - artik kalibreli 2-kamera hucresi + Otomatik Hucre
kullaniliyor. Geri eklemek icin asagidaki listeye bir st.Page daha eklemen yeterli.

Calistirma:
    streamlit run app.py
Docker:
    docker compose up  ->  http://localhost:8501
"""
from __future__ import annotations

import logging

import streamlit as st

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

pg = st.navigation([
    st.Page("pages/3_Otomatik_Hucre.py", title="Otomatik Hucre", icon="🛄", default=True),
    st.Page("pages/2_Hucre_Modu.py", title="Hucre Modu", icon="🤖"),
    st.Page("pages/1_Hizli_Stereo.py", title="Hizli Stereo", icon="⚡"),
])
pg.run()

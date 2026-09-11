"""
ORTAK ARAYUZ TEMASI - tum Streamlit sayfalarinda kullanilir.

SADECE GORSEL katman. Hicbir is mantigina / veri akisina dokunmaz - sadece
CSS enjekte eder ve birkac kucuk HTML bilesen (hero baslik, rozet/bagaj etiketi,
bant durum isigi) sunar. Her sayfa basinda bir kere:

    from src.ui import inject_theme, page_header
    st.set_page_config(...)
    inject_theme()
    page_header("🛄", "Otomatik Hucre", "...")

Konsept: HAVALIMANI BAGAJ BANDI. Proje zaten bunu yapiyor - kameralar valiz/
kutuyu olcup siniflandiriyor, robot en buyukten kucuge "bant"tan alip
yerlestiriyor. Tema bunu yansitir:
  - Sicak amber/turuncu ana renk = "gelis-gidis panosu" (split-flap departure
    board) hissi, JetBrains Mono ile.
  - Ince, hafif hareket eden SARI-SIYAH TEHLIKE SERIDI (bant kenar seridi)
    sayfa en ustunde sabit bir cizgi olarak - konsepti aninda soyler.
  - Rozetler bagaj etiketi gibi: sol kenarda delik + kesikli cizgi, hafif egik.
  - Ikincil renk soguk camgobegi (cyan) = tarama/ekran parlamasi.
"""
from __future__ import annotations

import streamlit as st

ACCENT = "#f5a623"      # amber/turuncu - gelis-gidis panosu, ana marka rengi
ACCENT_2 = "#22d3ee"    # camgobegi - ekran/tarama parlamasi, ikincil vurgu
OK = "#34d399"
WARN = "#f5a623"
BAD = "#f87171"


def inject_theme() -> None:
    """Sayfa basina bir kez CSS enjekte eder (session_state ile tekrari onler -
    Otomatik Hucre gibi 2sn'de bir yenilenen sayfalarda DOM'u sismemesi icin)."""
    if st.session_state.get("_vh_theme_injected"):
        return
    st.session_state["_vh_theme_injected"] = True
    st.markdown(_CSS, unsafe_allow_html=True)


def page_header(icon: str, title: str, subtitle: str = "") -> None:
    """st.title + st.caption yerine: parlayan ikon rozeti + gradyan baslik +
    kalkis-panosu tadinda alt cizgi."""
    sub_html = f'<div class="vh-sub">{subtitle}</div>' if subtitle else ""
    st.markdown(
        f"""
        <div class="vh-hero">
          <div class="vh-hero-icon">{icon}</div>
          <div class="vh-hero-text">
            <div class="vh-title">{title}</div>
            {sub_html}
          </div>
        </div>
        """,
        unsafe_allow_html=True,
    )


def badge(text: str, kind: str = "info") -> str:
    """Bagaj etiketi gorunumlu kucuk rozet HTML'i dondurur."""
    color = {"ok": OK, "warn": WARN, "bad": BAD, "info": ACCENT_2}.get(kind, ACCENT)
    return f'<span class="vh-badge" style="--c:{color}">{text}</span>'


def badge_row(*items: tuple[str, str]) -> None:
    """(metin, kind) ciftlerinden bir bagaj-etiketi sirasi cizer."""
    html = "".join(badge(t, k) for t, k in items)
    st.markdown(f'<div class="vh-badge-row">{html}</div>', unsafe_allow_html=True)


def live_dot(active: bool, label_on: str = "BANT ÇALIŞIYOR",
            label_off: str = "BANT BEKLEMEDE") -> None:
    """Nabız atan bant-durum isigi (Otomatik Hucre gibi canli sayfalar icin)."""
    cls = "vh-dot-on" if active else "vh-dot-off"
    label = label_on if active else label_off
    st.markdown(
        f'<div class="vh-live"><span class="vh-dot {cls}"></span>{label}</div>',
        unsafe_allow_html=True,
    )


def section(icon: str, title: str) -> None:
    """st.subheader yerine: ikonlu, gradyan-alt-cizgili bolum basligi."""
    st.markdown(f'<div class="vh-section">{icon} {title}</div>', unsafe_allow_html=True)


_CSS = """
<style>
@import url('https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@500;600;700&family=Inter:wght@400;500;600&family=JetBrains+Mono:wght@400;600;700&display=swap');

:root{
  --vh-bg0:#0a0906; --vh-bg1:#0d0c09; --vh-panel:#151109; --vh-panel2:#1a1610;
  --vh-border:#2e2312; --vh-text:#f2ece1; --vh-muted:#a99a80;
  --vh-accent:#f5a623; --vh-accent2:#22d3ee;
  --vh-ok:#34d399; --vh-warn:#f5a623; --vh-bad:#f87171;
  --vh-stripe-dark:#12100c;
}

html, body, [class*="css"]{ font-family:'Inter', -apple-system, sans-serif; }

/* --- bant kenar seridi: sayfanin en ustunde sabit, hafifce kayan tehlike seridi --- */
[data-testid="stApp"]::before{
  content:""; position:fixed; top:0; left:0; right:0; height:5px; z-index:999;
  background:repeating-linear-gradient(135deg,
    var(--vh-accent) 0 14px, var(--vh-stripe-dark) 14px 28px);
  background-size:200% 100%;
  animation:vh-belt 5s linear infinite;
  box-shadow:0 0 14px rgba(245,166,35,.45);
}
@keyframes vh-belt{ from{ background-position:0 0; } to{ background-position:-56px 0; } }

/* --- zemin: hafif izgara + sicak/soguk kose parlakligi (pano + ekran hissi) --- */
[data-testid="stAppViewContainer"]{
  background:
    radial-gradient(1100px 620px at 90% -8%, rgba(245,166,35,.09), transparent 60%),
    radial-gradient(900px 500px at -10% 108%, rgba(34,211,238,.06), transparent 55%),
    linear-gradient(180deg, var(--vh-bg0), var(--vh-bg1) 60%);
}
[data-testid="stAppViewContainer"]::before{
  content:""; position:fixed; inset:0; pointer-events:none; opacity:.3; z-index:0;
  background-image:
    linear-gradient(rgba(169,154,128,.05) 1px, transparent 1px),
    linear-gradient(90deg, rgba(169,154,128,.05) 1px, transparent 1px);
  background-size:42px 42px;
  mask-image:radial-gradient(1200px 700px at 70% 0%, black, transparent 75%);
}
[data-testid="stHeader"]{ background:rgba(10,9,6,.55); backdrop-filter:blur(6px); top:5px; }
[data-testid="stAppViewContainer"] > .main{ position:relative; z-index:1; }

/* --- kenar cubugu --- */
[data-testid="stSidebar"]{
  background:linear-gradient(180deg, var(--vh-panel), var(--vh-bg0) 140%);
  border-right:1px solid var(--vh-border); margin-top:5px;
}
[data-testid="stSidebar"] h1, [data-testid="stSidebar"] h2, [data-testid="stSidebar"] h3{
  font-family:'Space Grotesk', sans-serif; color:var(--vh-text); letter-spacing:.01em;
}
[data-testid="stSidebar"] hr{ border-color:var(--vh-border); }
[data-testid="stSidebarNav"] a{ border-radius:8px !important; }
[data-testid="stSidebarNav"] a:hover{ background:rgba(245,166,35,.10) !important; }

/* --- baslik / metin --- */
h1,h2,h3{ font-family:'Space Grotesk', sans-serif !important; color:var(--vh-text) !important; }
p, li, label, .stMarkdown{ color:var(--vh-text); }
.stCaption, [data-testid="stCaptionContainer"]{ color:var(--vh-muted) !important; }
code, .stCode, [data-testid="stMetricValue"]{ font-family:'JetBrains Mono', monospace !important; }

/* --- hero baslik: kalkis panosu tadinda kart --- */
.vh-hero{ display:flex; align-items:center; gap:16px; padding:18px 22px; margin:10px 0 18px 0;
  background:linear-gradient(135deg, rgba(245,166,35,.10), rgba(34,211,238,.05) 70%);
  border:1px solid var(--vh-border); border-radius:18px; position:relative; overflow:hidden; }
.vh-hero::before{ content:""; position:absolute; left:0; top:0; bottom:0; width:6px;
  background:repeating-linear-gradient(45deg, var(--vh-accent) 0 8px, var(--vh-stripe-dark) 8px 16px); }
.vh-hero::after{ content:""; position:absolute; inset:0; border-radius:18px; padding:1px;
  background:linear-gradient(120deg, var(--vh-accent), transparent 35%, transparent 65%, var(--vh-accent2));
  -webkit-mask:linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0);
  -webkit-mask-composite:xor; mask-composite:exclude; opacity:.5; pointer-events:none; }
.vh-hero-icon{ font-size:34px; line-height:1; width:60px; height:60px; min-width:60px;
  display:flex; align-items:center; justify-content:center; border-radius:16px; margin-left:10px;
  background:radial-gradient(circle at 35% 30%, rgba(245,166,35,.35), rgba(21,17,9,.9));
  box-shadow:0 0 0 1px var(--vh-border), 0 8px 24px -8px rgba(245,166,35,.5); }
.vh-title{ font-family:'Space Grotesk', sans-serif; font-weight:700; font-size:1.65rem;
  background:linear-gradient(90deg, #fbf3e6, var(--vh-accent) 65%, var(--vh-accent2));
  -webkit-background-clip:text; background-clip:text; color:transparent; letter-spacing:.01em; }
.vh-sub{ color:var(--vh-muted); font-size:.92rem; margin-top:3px; max-width:900px; }

/* --- bolum basligi: ikon + gradyan (amber->cyan) alt cizgi --- */
.vh-section{ font-family:'Space Grotesk', sans-serif; font-weight:600; font-size:1.05rem;
  color:var(--vh-text); margin:20px 0 10px 0; padding-bottom:8px; position:relative; }
.vh-section::after{ content:""; position:absolute; left:0; right:0; bottom:0; height:2px;
  background:linear-gradient(90deg, var(--vh-accent), var(--vh-border) 55%); }

/* --- rozetler: bagaj etiketi (delik + kesikli kenar) --- */
.vh-badge-row{ display:flex; flex-wrap:wrap; gap:10px; margin:6px 0 16px 0; }
.vh-badge{ --c:#f5a623; display:inline-flex; align-items:center; gap:6px; padding:5px 12px 5px 18px;
  border-radius:3px 10px 10px 3px; font-size:.78rem; font-weight:600; letter-spacing:.02em;
  color:var(--c); background:color-mix(in srgb, var(--c) 14%, transparent);
  border:1px dashed color-mix(in srgb, var(--c) 55%, transparent); position:relative; }
.vh-badge::before{ content:""; position:absolute; left:6px; top:50%; transform:translateY(-50%);
  width:5px; height:5px; border-radius:50%; background:var(--vh-bg0);
  border:1px solid color-mix(in srgb, var(--c) 55%, transparent); }

/* --- bant durum isigi --- */
.vh-live{ display:inline-flex; align-items:center; gap:9px; font-weight:700; font-size:.82rem;
  letter-spacing:.04em; color:var(--vh-text); padding:6px 14px; border:1px solid var(--vh-border);
  border-radius:999px; background:var(--vh-panel); margin-bottom:10px; text-transform:uppercase; }
.vh-dot{ width:9px; height:9px; border-radius:50%; display:inline-block; }
.vh-dot-on{ background:var(--vh-ok); box-shadow:0 0 0 0 rgba(52,211,153,.6); animation:vh-pulse 1.4s infinite; }
.vh-dot-off{ background:#5a4d38; }
@keyframes vh-pulse{
  0%{ box-shadow:0 0 0 0 rgba(52,211,153,.55); }
  70%{ box-shadow:0 0 0 9px rgba(52,211,153,0); }
  100%{ box-shadow:0 0 0 0 rgba(52,211,153,0); }
}

/* --- metrik kartlari: split-flap pano hissi --- */
[data-testid="stMetric"]{ background:linear-gradient(180deg, var(--vh-panel2), var(--vh-panel));
  border:1px solid var(--vh-border); border-radius:10px; padding:14px 16px 12px 16px;
  transition:border-color .15s, transform .15s; }
[data-testid="stMetric"]:hover{ border-color:var(--vh-accent); transform:translateY(-1px); }
[data-testid="stMetricLabel"]{ color:var(--vh-muted) !important; font-size:.76rem !important;
  text-transform:uppercase; letter-spacing:.07em; }
[data-testid="stMetricValue"]{ color:var(--vh-accent) !important; font-size:1.55rem !important;
  letter-spacing:.02em; }

/* --- butonlar: parlak amber, hover'da isik kaymasi (bant parlamasi) --- */
.stButton>button{ border-radius:10px !important; border:1px solid var(--vh-border) !important;
  font-weight:700 !important; letter-spacing:.02em; transition:transform .12s, box-shadow .15s, border-color .15s !important;
  background:var(--vh-panel2) !important; color:var(--vh-text) !important; position:relative; overflow:hidden; }
.stButton>button:hover{ transform:translateY(-1px); border-color:var(--vh-accent) !important;
  box-shadow:0 6px 18px -6px rgba(245,166,35,.35); }
.stButton>button[kind="primary"]{
  background:linear-gradient(120deg, var(--vh-accent), #d9820f) !important;
  color:#1c1204 !important; border:none !important; box-shadow:0 8px 22px -8px rgba(245,166,35,.6) !important; }
.stButton>button[kind="primary"]::after{ content:""; position:absolute; inset:0;
  background:linear-gradient(120deg, transparent 40%, rgba(255,255,255,.35) 50%, transparent 60%);
  background-size:220% 100%; background-position:120% 0; transition:background-position .5s; }
.stButton>button[kind="primary"]:hover::after{ background-position:-20% 0; }
.stButton>button[kind="primary"]:hover{ filter:brightness(1.05); transform:translateY(-1px); }
.stDownloadButton>button{ border-radius:10px !important; }

/* --- sekmeler --- */
[data-baseweb="tab-list"]{ gap:4px; border-bottom:1px solid var(--vh-border) !important; }
[data-baseweb="tab"]{ border-radius:10px 10px 0 0 !important; color:var(--vh-muted) !important;
  font-weight:600 !important; }
[data-baseweb="tab"][aria-selected="true"]{ color:var(--vh-accent) !important;
  background:rgba(245,166,35,.08) !important; }
[data-baseweb="tab-highlight"]{ background:var(--vh-accent) !important; height:2.5px !important; }

/* --- uyari kutulari --- */
[data-testid="stAlert"]{ border-radius:10px !important; border:1px solid var(--vh-border) !important;
  background:var(--vh-panel) !important; }

/* --- expander / dataframe --- */
[data-testid="stExpander"]{ border:1px solid var(--vh-border) !important; border-radius:10px !important;
  background:var(--vh-panel); overflow:hidden; }
[data-testid="stDataFrame"], [data-testid="stTable"]{ border-radius:10px; overflow:hidden;
  border:1px solid var(--vh-border); }

/* --- slider / input --- */
.stSlider [data-baseweb="slider"] div[role="slider"]{ background:var(--vh-accent) !important; }
div[data-baseweb="input"], div[data-baseweb="select"]>div, div[data-baseweb="textarea"]{
  border-radius:8px !important; border-color:var(--vh-border) !important; }

/* --- gorseller --- */
[data-testid="stImage"] img{ border-radius:10px; border:1px solid var(--vh-border); }

/* --- kaydirma cubugu --- */
::-webkit-scrollbar{ width:10px; height:10px; }
::-webkit-scrollbar-track{ background:var(--vh-bg0); }
::-webkit-scrollbar-thumb{ background:#3a2c15; border-radius:6px; }
::-webkit-scrollbar-thumb:hover{ background:var(--vh-accent); }

/* --- yumusak giris --- */
.main .block-container{ animation:vh-fade .35s ease-out; }
@keyframes vh-fade{ from{ opacity:0; transform:translateY(4px);} to{ opacity:1; transform:none;} }
</style>
"""

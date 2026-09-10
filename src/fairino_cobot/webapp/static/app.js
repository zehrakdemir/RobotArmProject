"use strict";

const EKLEMLER = ["j1", "j2", "j3", "j4", "j5", "j6"];
const $ = (s) => document.querySelector(s);
const el = (t, c, x) => { const n = document.createElement(t); if (c) n.className = c; if (x !== undefined) n.textContent = x; return n; };

let sonDurum = {};
let sonIsId = null;

// ---------------------------------------------------------------------------
// Gunluk
// ---------------------------------------------------------------------------
function yaz(mesaj, sinif) {
  const kutu = $("#log");
  const satir = el("div");
  const saat = new Date().toLocaleTimeString("tr-TR", { hour12: false });
  satir.appendChild(el("span", "t", saat + "  "));
  satir.appendChild(el("span", sinif || "", mesaj));
  kutu.appendChild(satir);
  kutu.scrollTop = kutu.scrollHeight;
  while (kutu.children.length > 300) kutu.removeChild(kutu.firstChild);
}

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------
async function api(yol, secenek) {
  const cevap = await fetch(yol, {
    method: (secenek && secenek.method) || "GET",
    headers: { "Content-Type": "application/json" },
    body: secenek && secenek.body ? JSON.stringify(secenek.body) : undefined,
  });
  let veri = null;
  try { veri = await cevap.json(); } catch (_) { /* govde yok */ }
  if (!cevap.ok) {
    const mesaj = (veri && veri.detail) || cevap.statusText;
    throw new Error(mesaj);
  }
  return veri;
}

async function isGonder(yol, govde, etiket) {
  try {
    await api(yol, { method: "POST", body: govde });
    yaz(etiket + " gönderildi");
  } catch (e) {
    yaz(etiket + " REDDEDİLDİ: " + e.message, "err");
  }
}

// ---------------------------------------------------------------------------
// Onay penceresi
// ---------------------------------------------------------------------------
function onayla(baslik, mesaj) {
  return new Promise((coz) => {
    $("#m-title").textContent = baslik;
    $("#m-msg").textContent = mesaj;
    $("#modal").classList.add("open");
    const kapat = (sonuc) => {
      $("#modal").classList.remove("open");
      $("#m-yes").onclick = null; $("#m-no").onclick = null;
      coz(sonuc);
    };
    $("#m-yes").onclick = () => kapat(true);
    $("#m-no").onclick = () => kapat(false);
  });
}

// ---------------------------------------------------------------------------
// Arayuz kurulumu
// ---------------------------------------------------------------------------
function kurJog() {
  const kap = $("#jog");
  EKLEMLER.forEach((ad, i) => {
    const satir = el("div", "jogrow");
    satir.appendChild(el("span", "lbl", ad));
    [-1, 1].forEach((yon) => {
      const b = el("button", null, (yon < 0 ? "− " : "+ ") + ad);
      b.onclick = () => {
        const adim = parseFloat($("#jstep").value) || 0;
        const farklar = [0, 0, 0, 0, 0, 0];
        farklar[i] = adim * yon;
        isGonder("/api/arm/jog", { deltas_deg: farklar }, `${ad} ${yon > 0 ? "+" : "−"}${adim}°`);
      };
      satir.appendChild(b);
    });
    kap.appendChild(satir);
  });

  document.querySelectorAll("[data-t]").forEach((b) => {
    b.onclick = () => {
      const adim = (parseFloat($("#tstep").value) || 0) * parseInt(b.dataset.s, 10);
      const govde = { dx_mm: 0, dy_mm: 0, dz_mm: 0 };
      govde["d" + b.dataset.t + "_mm"] = adim;
      isGonder("/api/arm/tcp_local", govde, `TCP ${b.dataset.t.toUpperCase()} ${adim}mm`);
    };
  });

  document.querySelectorAll("[data-g]").forEach((b) => {
    b.onclick = () => isGonder("/api/gripper", {
      action: b.dataset.g,
      speed: parseInt($("#g-speed").value, 10) || 50,
      force: parseInt($("#g-force").value, 10) || 50,
    }, "Gripper " + b.textContent);
  });
}

function kurMutlak() {
  const kap = $("#abs");
  EKLEMLER.forEach((ad) => {
    const l = el("label", "field");
    l.appendChild(el("span", null, ad));
    const inp = el("input"); inp.type = "number"; inp.step = "0.1";
    inp.id = "abs-" + ad; inp.value = "0";
    l.appendChild(inp);
    kap.appendChild(l);
  });

  const oku = () => EKLEMLER.map((a) => parseFloat($("#abs-" + a).value) || 0);

  $("#abs-now").onclick = () => {
    const j = sonDurum.joints_deg;
    if (!j) { yaz("Şu anki pozisyon henüz okunmadı", "warn"); return; }
    EKLEMLER.forEach((a, i) => { $("#abs-" + a).value = j[i].toFixed(2); });
    yaz("Alanlara şu anki pozisyon yazıldı");
  };

  $("#abs-preview").onclick = async () => {
    try {
      const r = await api("/api/arm/preview", { method: "POST", body: { joints_deg: oku() } });
      const p = r.tcp_pose;
      $("#abs-out").textContent =
        `Bu açılarda TCP: x=${p.x} y=${p.y} z=${p.z}  rx=${p.rx} ry=${p.ry} rz=${p.rz}`;
      yaz("FK önizleme hesaplandı");
    } catch (e) { yaz("Önizleme hatası: " + e.message, "err"); }
  };

  $("#abs-go").onclick = async () => {
    const hedef = oku();
    const ok = await onayla("Mutlak eklem hedefi",
      "Kol şu açılara gidecek:\n" + EKLEMLER.map((a, i) => `${a} = ${hedef[i]}°`).join("\n"));
    if (!ok) return;
    isGonder("/api/arm/joints", { joints_deg: hedef }, "Mutlak hedef");
  };
}

// ---------------------------------------------------------------------------
// Kayitli hareketler
// ---------------------------------------------------------------------------
async function hareketleriYukle() {
  const kap = $("#moves");
  kap.innerHTML = "";
  let veri;
  try { veri = await api("/api/moves"); } catch (e) {
    kap.appendChild(el("div", "note", "Yüklenemedi: " + e.message)); return;
  }
  if (!veri.moves.length) {
    kap.appendChild(el("div", "note", "Henüz kayıtlı hareket yok."));
    return;
  }
  veri.moves.forEach((m) => {
    const satir = el("div", "move");
    satir.appendChild(el("span", "name", m.name));
    if (m.type === "pick_place") satir.appendChild(el("span", "tag", "pick & place"));
    else satir.appendChild(el("span", "meta", m.steps + " adım"));
    satir.appendChild(el("span", "sep"));

    const oynat = el("button", "primary", "Oynat");
    oynat.onclick = async () => {
      const ok = await onayla("Hareketi oynat",
        `'${m.name}' oynatılacak. Robot hareket edecek — acil stop elinin altında olsun.`);
      if (!ok) return;
      const sayi = (id) => { const v = parseFloat($(id).value); return isNaN(v) ? null : v; };
      isGonder(`/api/moves/${encodeURIComponent(m.name)}/play`, {
        speed_pct: sayi("#p-speed"), accel_pct: sayi("#p-accel"),
        grip_speed: sayi("#p-gspeed"), grip_force: sayi("#p-gforce"),
      }, `'${m.name}' oynat`);
    };
    satir.appendChild(oynat);

    const sil = el("button", "danger", "Sil");
    sil.onclick = async () => {
      const ok = await onayla("Kaydı sil", `'${m.name}' kalıcı olarak silinecek.`);
      if (!ok) return;
      try {
        await api(`/api/moves/${encodeURIComponent(m.name)}`, { method: "DELETE" });
        yaz(`'${m.name}' silindi`); hareketleriYukle();
      } catch (e) { yaz("Silinemedi: " + e.message, "err"); }
    };
    satir.appendChild(sil);
    kap.appendChild(satir);
  });
}

// ---------------------------------------------------------------------------
// Telemetri
// ---------------------------------------------------------------------------
function kutu(kap, anahtar, deger, birim) {
  const d = el("div", "kv");
  d.appendChild(el("span", "k", anahtar));
  d.appendChild(el("span", "v", deger === undefined || deger === null ? "—" : deger + (birim || "")));
  kap.appendChild(d);
}

function durumCiz(s) {
  sonDurum = s;

  const bagli = !!s.ready;
  $("#d-conn").className = "dot " + (bagli ? "ok" : (s.startup_error ? "err" : "warn"));
  $("#t-conn").textContent = bagli ? "bağlı" : (s.startup_error ? "hata" : "hazırlanıyor");
  $("#t-robot").textContent = s.model ? `${s.model} · ${s.robot_ip} · hız %${s.speed_pct}` : "—";

  const f = s.fault;
  const fvar = f && (f.main_code !== 0 || f.sub_code !== 0);
  $("#d-fault").className = "dot " + (f ? (fvar ? "err" : "ok") : "");
  $("#t-fault").textContent = f ? (fvar ? `fault ${f.main_code}/${f.sub_code}` : "fault yok") : "fault ?";

  $("#d-busy").className = "dot " + (s.busy ? "busy" : "ok");
  $("#t-busy").textContent = s.busy ? "hareket ediyor" : "boşta";

  const je = $("#joints"); je.innerHTML = "";
  (s.joints_deg || []).forEach((v, i) => kutu(je, EKLEMLER[i], v.toFixed(2), "°"));

  const te = $("#tcp"); te.innerHTML = "";
  const p = s.tcp_pose;
  if (p) {
    kutu(te, "x", p.x, " mm"); kutu(te, "y", p.y, " mm"); kutu(te, "z", p.z, " mm");
    kutu(te, "rx", p.rx, "°"); kutu(te, "ry", p.ry, "°"); kutu(te, "rz", p.rz, "°");
  }

  // Is bitince sonucu bir kez gunluge yaz.
  const j = s.job;
  if (j && !j.running && j.id !== sonIsId) {
    sonIsId = j.id;
    if (j.ok) yaz(`${j.label} — tamam (${j.seconds}s)`, "ok");
    else yaz(`${j.label} — BAŞARISIZ: ${j.detail}`, "err");
    hareketleriYukle();
  }

  // Acilis ekrani
  const ov = $("#overlay");
  if (bagli) ov.classList.add("hidden");
  else {
    ov.classList.remove("hidden");
    $("#ovtitle").textContent = s.startup_error ? "Bağlanılamadı" : "Robota bağlanılıyor…";
    $("#ovlog").textContent = (s.startup_log || []).join("\n");
  }
}

async function gripperDurum() {
  if (!sonDurum.ready) return;
  try {
    const g = await api("/api/gripper/status");
    const kap = $("#gstat"); kap.innerHTML = "";
    if (g.error) { kutu(kap, "durum", g.error); return; }
    const anlam = { 0: "hareket ediyor", 1: "bitti · nesne yok", 2: "bitti · nesne var" };
    kutu(kap, "durum", anlam[g.motion_status] || g.motion_status);
    kutu(kap, "fault", g.fault);
    kutu(kap, "pozisyon", g.position_pct, " %");
    kutu(kap, "hız", g.speed_pct, " %");
  } catch (_) { /* gripper okunamadi - arayuz kararmasin */ }
}

// ---------------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------------
function baglan() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(`${proto}//${location.host}/ws/state`);
  ws.onmessage = (olay) => { try { durumCiz(JSON.parse(olay.data)); } catch (_) {} };
  ws.onclose = () => {
    $("#d-conn").className = "dot err";
    $("#t-conn").textContent = "bağlantı koptu";
    setTimeout(baglan, 1500);   // sunucu yeniden basladiysa kendiliginden donsun
  };
}

// ---------------------------------------------------------------------------
// Baslangic
// ---------------------------------------------------------------------------
kurJog();
kurMutlak();
hareketleriYukle();
baglan();
setInterval(gripperDurum, 1200);

$("#estop").onclick = async () => {
  try {
    const r = await api("/api/stop", { method: "POST" });
    yaz("ACİL DUR: " + JSON.stringify(r), "warn");
  } catch (e) { yaz("Acil dur hatası: " + e.message, "err"); }
};

$("#clearfault").onclick = async () => {
  const ok = await onayla("Fault temizle",
    "ResetAllError() GLOBAL bir reset — kolun servo oturumunu düşürür.\n" +
    "Sonrasında fairino_launch yeniden başlatılmalı.\n\nDevam edilsin mi?");
  if (!ok) return;
  try {
    const r = await api("/api/fault/clear", { method: "POST" });
    yaz("Fault temizlendi (rc=" + r.rc + "). " + r.warning, "warn");
  } catch (e) { yaz("Temizlenemedi: " + e.message, "err"); }
};

yaz("Arayüz hazır.");

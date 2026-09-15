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
    let mesaj = (veri && veri.detail) || cevap.statusText;
    // 422 dogrulama hatasinda detail bir liste
    if (Array.isArray(mesaj)) mesaj = mesaj.map((d) => d.msg).join("; ");
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

  // Arac eksenleri: eksenler aletle birlikte doner.
  document.querySelectorAll("[data-t]").forEach((b) => {
    b.onclick = () => {
      const adim = (parseFloat($("#tstep").value) || 0) * parseInt(b.dataset.s, 10);
      const govde = { dx_mm: 0, dy_mm: 0, dz_mm: 0 };
      govde["d" + b.dataset.t + "_mm"] = adim;
      isGonder("/api/arm/tcp_local", govde, `TCP araç ${b.dataset.t.toUpperCase()} ${adim}mm`);
    };
  });

  // Base eksenleri: eksenler SABIT - alet nasil durursa dursun +Z yukari.
  document.querySelectorAll("[data-b]").forEach((b) => {
    b.onclick = () => {
      const adim = (parseFloat($("#bstep").value) || 0) * parseInt(b.dataset.s, 10);
      const govde = { dx_mm: 0, dy_mm: 0, dz_mm: 0 };
      govde["d" + b.dataset.b + "_mm"] = adim;
      isGonder("/api/arm/tcp_base", govde, `TCP base ${b.dataset.b.toUpperCase()} ${adim}mm`);
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

// Mutlak TCP pozu - hepsi BASE cercevesinde.
//
// NEDEN "Su anki" butonu once: alti alanin hepsini elle doldurmak, robotu
// beklenmedik bir yonelime birden dondurmenin en kolay yolu. Once mevcut poz
// yazilip TEK bir deger degistirilince hareket ongorulebilir oluyor.
const POZ_ALAN = [
  ["x", "X (mm)"], ["y", "Y (mm)"], ["z", "Z (mm)"],
  ["rx", "RX (°)"], ["ry", "RY (°)"], ["rz", "RZ (°)"],
];

function kurTcpMutlak() {
  const kap = $("#pose");
  POZ_ALAN.forEach(([ad, etiket]) => {
    const l = el("label", "field");
    l.appendChild(el("span", null, etiket));
    const inp = el("input"); inp.type = "number"; inp.step = "0.1";
    inp.id = "pose-" + ad; inp.value = "0";
    l.appendChild(inp);
    kap.appendChild(l);
  });

  const oku = () => {
    const o = {};
    POZ_ALAN.forEach(([ad]) => { o[ad] = parseFloat($("#pose-" + ad).value) || 0; });
    return o;
  };

  const doldur = () => {
    const p = sonDurum.tcp_pose;
    if (!p) { yaz("TCP pozu henüz okunmadı", "warn"); return false; }
    POZ_ALAN.forEach(([ad]) => { $("#pose-" + ad).value = Number(p[ad]).toFixed(2); });
    return true;
  };

  $("#pose-now").onclick = () => { if (doldur()) yaz("Alanlara şu anki TCP pozu yazıldı"); };

  $("#pose-go").onclick = async () => {
    const h = oku();
    const p = sonDurum.tcp_pose;
    // Mevcut pozdan farki da goster - "sadece Z'yi 20mm indiriyorum" demek
    // isteyen biri, yanlislikla yonelimi de degistirdiyse burada gorur.
    let fark = "";
    if (p) {
      const degisen = POZ_ALAN
        .map(([ad]) => [ad, +(h[ad] - p[ad]).toFixed(2)])
        .filter(([, d]) => d !== 0)
        .map(([ad, d]) => `${ad.toUpperCase()} ${d > 0 ? "+" : ""}${d}`);
      fark = degisen.length
        ? "\n\nŞu ankinden fark:\n" + degisen.join("   ")
        : "\n\n(şu anki pozla aynı)";
    }
    const ok = await onayla("Mutlak TCP pozu (base)",
      `TCP şuraya gidecek:\nx=${h.x} y=${h.y} z=${h.z}\n` +
      `rx=${h.rx} ry=${h.ry} rz=${h.rz}` + fark);
    if (!ok) return;
    h.prefer_lin = $("#pose-lin").checked;
    isGonder("/api/arm/pose", h, "Mutlak TCP pozu");
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
// Ayarlar penceresi
// ---------------------------------------------------------------------------
// Eklem jog, TCP jog ve mutlak hedeflerde kullanilir. Kayitli hareketler ve
// Gripper paneli kendi alanlarini gonderir; bu ayrim sunucuda (server.py).
const AYAR_ALAN = [
  ["speed_pct", "#s-speed", "Hız"], ["accel_pct", "#s-accel", "İvme"],
];

function kurAyarlar() {
  const hata = (mesaj) => { $("#s-err").textContent = mesaj; $("#s-err").hidden = !mesaj; };
  const kapat = () => { $("#settings").classList.remove("open"); hata(""); };

  $("#settings-open").onclick = async () => {
    try {
      const a = await api("/api/settings");
      AYAR_ALAN.forEach(([ad, id]) => { $(id).value = a[ad]; });
      hata("");
      $("#settings").classList.add("open");
    } catch (e) { yaz("Ayarlar okunamadı: " + e.message, "err"); }
  };

  $("#s-no").onclick = kapat;

  $("#s-yes").onclick = async () => {
    const govde = {};
    for (const [ad, id, etiket] of AYAR_ALAN) {
      const v = parseFloat($(id).value);
      if (isNaN(v) || v < 1 || v > 100) { hata(`${etiket} 1 ile 100 arasında olmalı.`); return; }
      govde[ad] = v;
    }
    try {
      const a = await api("/api/settings", { method: "POST", body: govde });
      kapat();
      yaz(`Ayarlar kaydedildi: hız %${a.speed_pct}, ivme %${a.accel_pct}`, "ok");
    } catch (e) { hata("Kaydedilemedi: " + e.message); }
  };
}

// ---------------------------------------------------------------------------
// Pick & place kaydi
// ---------------------------------------------------------------------------
// Terminaldeki record_pick_and_place ile ayni akis: kolu kavrama pozuna getir,
// pozu kaydet (kol yaklasma mesafesi kadar GERI cekilir, on-poz da kaydedilir),
// ayni sey birakma icin, sonra kaydet. Taslak sunucuda (/api/pickplace/draft).
async function ppTaslakGoster() {
  try {
    const t = await api("/api/pickplace/draft");
    const satir = (ad, etiket) =>
      `${etiket}: ${t[ad] ? t[ad].map((v) => Number(v).toFixed(1)).join(", ") : "—"}`;
    $("#pp-out").textContent = [
      satir("pick", "Alma"), satir("pre_pick", "Alma ön-poz"),
      satir("place", "Bırakma"), satir("pre_place", "Bırakma ön-poz"),
    ].join("\n") + (t.approach_mm ? `\nYaklaşma: ${t.approach_mm} mm` : "");
  } catch (e) { $("#pp-out").textContent = "Taslak okunamadı: " + e.message; }
}

function kurPickPlace() {
  const pozKaydet = async (etiket, ad) => {
    const mm = parseFloat($("#pp-approach").value);
    if (isNaN(mm) || mm <= 0) { yaz("Yaklaşma mesafesi 0'dan büyük olmalı", "err"); return; }
    const ok = await onayla(`Pick & place: ${ad} pozu`,
      `Kolun şu anki pozu ${ad.toLowerCase()} pozu olarak kaydedilecek.\n` +
      `Sonra kol kendi Z ekseninde ${mm} mm GERİ çekilecek (robot hareket eder) ` +
      `ve ön-poz kaydedilecek.`);
    if (!ok) return;
    isGonder("/api/pickplace/pose", { etiket, approach_mm: mm }, `${ad} pozu kaydı`);
  };
  $("#pp-pick").onclick = () => pozKaydet("pick", "Alma");
  $("#pp-place").onclick = () => pozKaydet("place", "Bırakma");

  $("#pp-reset").onclick = async () => {
    try {
      await api("/api/pickplace/draft", { method: "DELETE" });
      yaz("Pick & place taslağı sıfırlandı");
      ppTaslakGoster();
    } catch (e) { yaz("Sıfırlanamadı: " + e.message, "err"); }
  };

  $("#pp-save").onclick = async () => {
    const govde = {
      name: $("#pp-name").value.trim(),
      approach_mm: parseFloat($("#pp-approach").value),
      grip_speed: parseInt($("#pp-gspeed").value, 10),
      grip_force: parseInt($("#pp-gforce").value, 10),
      object_d_mm: parseFloat($("#pp-objd").value),
      object_h_mm: parseFloat($("#pp-objh").value),
      overwrite: false,
    };
    try {
      await api("/api/pickplace/save", { method: "POST", body: govde });
    } catch (e) {
      if (!e.message.includes("zaten kayıtlı")) { yaz("Kaydedilemedi: " + e.message, "err"); return; }
      const ok = await onayla("Kaydın üzerine yaz", `'${govde.name}' zaten kayıtlı. Üzerine yazılsın mı?`);
      if (!ok) return;
      govde.overwrite = true;
      try { await api("/api/pickplace/save", { method: "POST", body: govde }); } catch (e2) {
        yaz("Kaydedilemedi: " + e2.message, "err"); return;
      }
    }
    yaz(`'${govde.name}' pick & place olarak kaydedildi`, "ok");
    ppTaslakGoster();
    hareketleriYukle();
  };

  ppTaslakGoster();
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
    if (j.label.startsWith("Pick & place")) ppTaslakGoster();
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
kurTcpMutlak();
kurAyarlar();
kurPickPlace();
hareketleriYukle();
baglan();
setInterval(gripperDurum, 1200);

$("#stopmove").onclick = async () => {
  try {
    const r = await api("/api/stop", { method: "POST" });
    if (r.durduruldu) yaz("Hareket durduruldu.", "warn");
    else yaz("Hareket durdurulamadı: " + (r.hata || JSON.stringify(r)), "err");
  } catch (e) { yaz("Hareket durdurma hatası: " + e.message, "err"); }
};

yaz("Arayüz hazır.");

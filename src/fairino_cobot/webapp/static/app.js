"use strict";

const EKLEMLER = ["j1", "j2", "j3", "j4", "j5", "j6"];
const $ = (s) => document.querySelector(s);
const el = (t, c, x) => { const n = document.createElement(t); if (c) n.className = c; if (x !== undefined) n.textContent = x; return n; };

let sonDurum = {};
let sonIsId = null;

// Sayi alani oku. Bos ya da gecersiz deger robota 0 olarak GITMESIN: alanin
// yaninda tarayici uyarisi cikar, null doner ve cagiran komutu gondermez.
// pozitif=true: adim alanlari - eksi adim "+" butonunu ters yone goturuyordu.
function sayiAlan(inp, pozitif) {
  const v = parseFloat(inp.value);
  let hata = "";
  if (inp.value.trim() === "" || isNaN(v)) hata = "Enter a number.";
  else if (pozitif && v <= 0) hata = "Must be greater than 0.";
  if (!hata) { inp.setCustomValidity(""); return v; }
  inp.setCustomValidity(hata);
  inp.reportValidity();
  inp.addEventListener("input", () => inp.setCustomValidity(""), { once: true });
  return null;
}

// ---------------------------------------------------------------------------
// Gunluk
// ---------------------------------------------------------------------------
function yaz(mesaj, sinif) {
  const kutu = $("#log");
  const satir = el("div");
  const saat = new Date().toLocaleTimeString("en-GB", { hour12: false });
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

// Basarida null, reddedilirse hata mesaji doner (panelin yaninda gostermek icin).
async function isGonder(yol, govde, etiket) {
  try {
    await api(yol, { method: "POST", body: govde });
    yaz(etiket + " sent");
    return null;
  } catch (e) {
    yaz(etiket + " REJECTED: " + e.message, "err");
    return e.message;
  }
}

// Ust cubuktaki "Preview before moving" aciksa hareket butonlari robotu
// HAREKET ETTIRMEZ: ayni istek /api/preview'e gider, sunucu secenekleri
// planlar. Robot ancak "Run this path" ve onayla hareket eder (kurOnizleme).
const onizlemeAcik = () => $("#pv-mode").checked;

async function onizlemeIste(kind, values, prefer_lin, etiket) {
  onizlemeHata("");
  try {
    await api("/api/preview", { method: "POST", body: { kind, values, prefer_lin } });
    yaz(etiket + ": planning paths (the robot does not move)");
    return null;
  } catch (e) {
    yaz(etiket + " preview REJECTED: " + e.message, "err");
    onizlemeHata(etiket + " preview rejected: " + e.message);
    return e.message;
  }
}

// Hareket butonlarinin ortak yolu: onizleme aciksa planla, degilse gonder.
function gonderVeyaOnizle(yol, govde, etiket, onizleme) {
  if (onizlemeAcik()) {
    return onizlemeIste(onizleme.kind, onizleme.values, onizleme.prefer_lin !== false, etiket);
  }
  return isGonder(yol, govde, etiket);
}

// ---------------------------------------------------------------------------
// Acilir pencereler
// ---------------------------------------------------------------------------
// Onay ve hiz/ivme penceresi sayfanin ortasinda degil, basilan butonun
// ALTINDA acilir. Disari tiklamak ya da Esc vazgecmek demek. Onayda odak
// "Cancel"da baslar: Enter'a yanlislikla basmak robotu hareket ettirmesin.

// Tek seferde tek dinleyici: kapatinca document'tan kaldirilir.
function disariKapat(kap, kapat) {
  const fare = (e) => { if (!kap.contains(e.target)) kapat(); };
  const tus = (e) => { if (e.key === "Escape") kapat(); };
  document.addEventListener("mousedown", fare, true);
  document.addEventListener("keydown", tus, true);
  return () => {
    document.removeEventListener("mousedown", fare, true);
    document.removeEventListener("keydown", tus, true);
  };
}

// Pencereyi butonun hemen altina, ekran icinde kalacak sekilde koyar.
// Sayfa kaydirilinca butonla birlikte kayar (belge koordinatlari).
function butonunAltina(pop, capa) {
  const r = capa.getBoundingClientRect();
  const genislik = pop.offsetWidth;
  const ekran = document.documentElement.clientWidth;
  const sol = Math.max(12, Math.min(r.left, ekran - genislik - 12));
  pop.style.left = (sol + window.scrollX) + "px";
  pop.style.top = (r.bottom + window.scrollY + 6) + "px";
  pop.scrollIntoView({ block: "nearest" });
}

let acikOnayiKapat = null;

function onayla(baslik, mesaj, capa) {
  if (acikOnayiKapat) acikOnayiKapat(false);
  return new Promise((coz) => {
    const pop = $("#confirm");
    $("#m-title").textContent = baslik;
    $("#m-msg").textContent = mesaj;
    pop.hidden = false;
    butonunAltina(pop, capa);

    let birak = null;
    const kapat = (sonuc) => {
      // Odak penceredeyse basilan butona don (klavyeyle kullanan yerini kaybetmesin).
      if (pop.contains(document.activeElement)) capa.focus();
      pop.hidden = true;
      $("#m-yes").onclick = null; $("#m-no").onclick = null;
      if (birak) birak();
      acikOnayiKapat = null;
      coz(sonuc);
    };
    $("#m-yes").onclick = () => kapat(true);
    $("#m-no").onclick = () => kapat(false);
    // Ayni butona tekrar basmak pencereyi yeniden acar (onclick), disari sayma.
    birak = disariKapat({ contains: (n) => pop.contains(n) || capa.contains(n) }, () => kapat(false));
    acikOnayiKapat = kapat;
    $("#m-no").focus();
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
        const adim = sayiAlan($("#jstep"), true);
        if (adim === null) { yaz("Joint jog not sent: step must be greater than 0", "err"); return; }
        const farklar = [0, 0, 0, 0, 0, 0];
        farklar[i] = adim * yon;
        gonderVeyaOnizle("/api/arm/jog", { deltas_deg: farklar }, `${ad} ${yon > 0 ? "+" : "−"}${adim}°`,
          { kind: "jog", values: farklar });
      };
      satir.appendChild(b);
    });
    kap.appendChild(satir);
  });

  // Arac eksenleri: eksenler aletle birlikte doner.
  document.querySelectorAll("[data-t]").forEach((b) => {
    b.onclick = () => {
      const deger = sayiAlan($("#tstep"), true);
      if (deger === null) { yaz("Tool jog not sent: step must be greater than 0", "err"); return; }
      const adim = deger * parseInt(b.dataset.s, 10);
      const govde = { dx_mm: 0, dy_mm: 0, dz_mm: 0 };
      govde["d" + b.dataset.t + "_mm"] = adim;
      gonderVeyaOnizle("/api/arm/tcp_local", govde,
        `Tool jog ${b.dataset.t.toUpperCase()} ${adim} mm`,
        { kind: "tcp_local", values: [govde.dx_mm, govde.dy_mm, govde.dz_mm] });
    };
  });

  // Base eksenleri: eksenler SABIT - alet nasil durursa dursun +Z yukari.
  document.querySelectorAll("[data-b]").forEach((b) => {
    b.onclick = () => {
      const deger = sayiAlan($("#bstep"), true);
      if (deger === null) { yaz("Base jog not sent: step must be greater than 0", "err"); return; }
      const adim = deger * parseInt(b.dataset.s, 10);
      const govde = { dx_mm: 0, dy_mm: 0, dz_mm: 0 };
      govde["d" + b.dataset.b + "_mm"] = adim;
      gonderVeyaOnizle("/api/arm/tcp_base", govde,
        `Base jog ${b.dataset.b.toUpperCase()} ${adim} mm`,
        { kind: "tcp_base", values: [govde.dx_mm, govde.dy_mm, govde.dz_mm] });
    };
  });

  document.querySelectorAll("[data-g]").forEach((b) => {
    b.onclick = () => isGonder("/api/gripper", {
      action: b.dataset.g,
      speed: parseInt($("#g-speed").value, 10) || 50,
      force: parseInt($("#g-force").value, 10) || 50,
    }, "Gripper " + b.textContent.toLowerCase());
  });
}

function kurMutlak() {
  const kap = $("#abs");
  EKLEMLER.forEach((ad) => {
    const l = el("label", "field");
    l.appendChild(el("span", null, ad));
    const inp = el("input"); inp.type = "number"; inp.step = "0.1";
    // Bos baslar (0 degil): "Current" basmadan Go denirse hedef tum eklemler
    // 0 = tekil poz oluyordu. Bos alan asagida hata verir.
    inp.id = "abs-" + ad;
    l.appendChild(inp);
    kap.appendChild(l);
  });

  // Biri bos/gecersizse null (alanin yaninda uyari cikar), yoksa 6 aci.
  const oku = () => {
    const acilar = [];
    for (const a of EKLEMLER) {
      const v = sayiAlan($("#abs-" + a), false);
      if (v === null) return null;
      acilar.push(v);
    }
    return acilar;
  };

  $("#abs-now").onclick = () => {
    const j = sonDurum.joints_deg;
    if (!j) { yaz("Current joint angles not read yet", "warn"); return; }
    EKLEMLER.forEach((a, i) => { $("#abs-" + a).value = j[i].toFixed(2); });
    yaz("Fields filled with the current joint angles");
  };

  $("#abs-preview").onclick = async () => {
    const acilar = oku();
    if (!acilar) { yaz("Preview: fill all six joint fields (use Current)", "err"); return; }
    try {
      const r = await api("/api/arm/preview", { method: "POST", body: { joints_deg: acilar } });
      const p = r.tcp_pose;
      $("#abs-out").textContent =
        `TCP at these angles: x=${p.x} y=${p.y} z=${p.z}  rx=${p.rx} ry=${p.ry} rz=${p.rz}`;
      yaz("FK preview computed");
    } catch (e) { yaz("Preview error: " + e.message, "err"); }
  };

  $("#abs-go").onclick = async () => {
    const hedef = oku();
    if (!hedef) { yaz("Absolute joint target not sent: fill all six fields (use Current)", "err"); return; }
    // Onizleme aciksa robot hareket etmez; onay "Run this path"te alinir.
    if (onizlemeAcik()) {
      onizlemeIste("joints", hedef, true, "Absolute joint target");
      return;
    }
    const ok = await onayla("Absolute joint target",
      "The arm will move to:\n" + EKLEMLER.map((a, i) => `${a} = ${hedef[i]}°`).join("\n"),
      $("#abs-go"));
    if (!ok) return;
    isGonder("/api/arm/joints", { joints_deg: hedef }, "Absolute joint target");
  };
}

// Mutlak TCP pozu - hepsi BASE cercevesinde.
//
// NEDEN "Current" butonu once: alti alanin hepsini elle doldurmak, robotu
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
    inp.id = "pose-" + ad;   // bos baslar; bos alan 0 (ornegin z=0) olarak gitmez
    l.appendChild(inp);
    kap.appendChild(l);
  });

  // Biri bos/gecersizse null (alanin yaninda uyari cikar).
  const oku = () => {
    const o = {};
    for (const [ad] of POZ_ALAN) {
      const v = sayiAlan($("#pose-" + ad), false);
      if (v === null) return null;
      o[ad] = v;
    }
    return o;
  };

  const doldur = () => {
    const p = sonDurum.tcp_pose;
    if (!p) { yaz("TCP pose not read yet", "warn"); return false; }
    POZ_ALAN.forEach(([ad]) => { $("#pose-" + ad).value = Number(p[ad]).toFixed(2); });
    return true;
  };

  $("#pose-now").onclick = () => { if (doldur()) yaz("Fields filled with the current TCP pose"); };

  $("#pose-go").onclick = async () => {
    const h = oku();
    if (!h) { yaz("Absolute TCP pose not sent: fill all six fields (use Current)", "err"); return; }
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
        ? "\n\nChange from current:\n" + degisen.join("   ")
        : "\n\n(same as the current pose)";
    }
    // Onizleme aciksa robot hareket etmez; onay "Run this path"te alinir.
    if (onizlemeAcik()) {
      onizlemeIste("pose", [h.x, h.y, h.z, h.rx, h.ry, h.rz], $("#pose-lin").checked,
        "Absolute TCP pose");
      return;
    }
    const ok = await onayla("Absolute TCP pose (base)",
      `The TCP will move to:\nx=${h.x} y=${h.y} z=${h.z}\n` +
      `rx=${h.rx} ry=${h.ry} rz=${h.rz}` + fark,
      $("#pose-go"));
    if (!ok) return;
    h.prefer_lin = $("#pose-lin").checked;
    isGonder("/api/arm/pose", h, "Absolute TCP pose");
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
    kap.appendChild(el("div", "note", "Could not load: " + e.message)); return;
  }
  if (!veri.moves.length) {
    kap.appendChild(el("div", "note", "No saved moves yet."));
    return;
  }
  veri.moves.forEach((m) => {
    const satir = el("div", "move");
    const adEl = el("span", "name", m.name);
    adEl.title = m.name;   // uzun ad CSS'te kisaltiliyor, tami ustune gelince gorunsun
    satir.appendChild(adEl);
    if (m.type === "pick_place") satir.appendChild(el("span", "tag", "pick & place"));
    else satir.appendChild(el("span", "meta", m.steps + (m.steps === 1 ? " step" : " steps")));
    satir.appendChild(el("span", "sep"));

    const oynat = el("button", "primary", "Play");
    oynat.onclick = async () => {
      const ok = await onayla("Play move",
        `'${m.name}' will be played. The robot will move — keep the emergency stop within reach.`,
        oynat);
      if (!ok) return;
      const sayi = (id) => { const v = parseFloat($(id).value); return isNaN(v) ? null : v; };
      isGonder(`/api/moves/${encodeURIComponent(m.name)}/play`, {
        speed_pct: sayi("#p-speed"), accel_pct: sayi("#p-accel"),
        grip_speed: sayi("#p-gspeed"), grip_force: sayi("#p-gforce"),
      }, `Play '${m.name}'`);
    };
    satir.appendChild(oynat);

    const sil = el("button", "danger", "Delete");
    sil.onclick = async () => {
      const ok = await onayla("Delete move", `'${m.name}' will be permanently deleted.`, sil);
      if (!ok) return;
      try {
        await api(`/api/moves/${encodeURIComponent(m.name)}`, { method: "DELETE" });
        yaz(`'${m.name}' deleted`); hareketleriYukle();
      } catch (e) { yaz("Could not delete: " + e.message, "err"); }
    };
    satir.appendChild(sil);
    kap.appendChild(satir);
  });
}

// ---------------------------------------------------------------------------
// Hiz / ivme penceresi
// ---------------------------------------------------------------------------
// Eklem jog, TCP jog ve mutlak hedeflerde kullanilir. Kayitli hareketler ve
// Gripper paneli kendi alanlarini gonderir; bu ayrim sunucuda (server.py).
const AYAR_ALAN = [
  ["speed_pct", "#s-speed", "Speed"], ["accel_pct", "#s-accel", "Acceleration"],
];

function kurAyarlar() {
  const pop = $("#settings");
  const buton = $("#settings-open");
  const hata = (mesaj) => { $("#s-err").textContent = mesaj; $("#s-err").hidden = !mesaj; };

  let birak = null;
  const kapat = () => {
    if (pop.contains(document.activeElement)) buton.focus();
    pop.hidden = true;
    buton.setAttribute("aria-expanded", "false");
    hata("");
    if (birak) { birak(); birak = null; }
  };

  buton.onclick = async () => {
    if (!pop.hidden) { kapat(); return; }
    try {
      const a = await api("/api/settings");
      AYAR_ALAN.forEach(([ad, id]) => { $(id).value = a[ad]; });
      hata("");
      pop.hidden = false;
      buton.setAttribute("aria-expanded", "true");
      birak = disariKapat($("#settings-wrap"), kapat);
      $("#s-speed").focus();
    } catch (e) { yaz("Could not read speed/accel: " + e.message, "err"); }
  };

  $("#s-no").onclick = kapat;

  $("#s-yes").onclick = async () => {
    const govde = {};
    for (const [ad, id, etiket] of AYAR_ALAN) {
      const v = parseFloat($(id).value);
      if (isNaN(v) || v < 1 || v > 100) { hata(`${etiket} must be between 1 and 100.`); return; }
      govde[ad] = v;
    }
    try {
      const a = await api("/api/settings", { method: "POST", body: govde });
      kapat();
      yaz(`Speed/accel saved: speed ${a.speed_pct}%, accel ${a.accel_pct}%`, "ok");
    } catch (e) { hata("Could not save: " + e.message); }
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
      satir("pick", "Pick"), satir("pre_pick", "Pick approach"),
      satir("place", "Place"), satir("pre_place", "Place approach"),
    ].join("\n") + (t.approach_mm ? `\nApproach: ${t.approach_mm} mm` : "");
  } catch (e) { $("#pp-out").textContent = "Could not read draft: " + e.message; }
}

// Hata panelin butonlarinin hemen altinda da gorunsun: Log sag sutunun en
// altinda, ekranda olmayabilir. Bos mesaj kutuyu gizler.
function ppHata(mesaj) {
  $("#pp-err").textContent = mesaj || "";
  $("#pp-err").hidden = !mesaj;
}

function kurPickPlace() {
  const pozKaydet = async (etiket, ad, buton) => {
    const mm = parseFloat($("#pp-approach").value);
    if (isNaN(mm) || mm <= 0) {
      yaz("Approach distance must be greater than 0", "err");
      ppHata("Approach distance must be greater than 0.");
      return;
    }
    ppHata("");
    const ok = await onayla(`Pick & place: ${ad} pose`,
      `The arm's current pose will be saved as the ${ad.toLowerCase()} pose.\n` +
      `The arm then retracts ${mm} mm along its own Z axis (the robot moves) ` +
      `and the approach pose is saved.`,
      buton);
    if (!ok) return;
    const red = await isGonder("/api/pickplace/pose", { etiket, approach_mm: mm }, `Save ${ad.toLowerCase()} pose`);
    if (red) ppHata(`Save ${ad.toLowerCase()} pose rejected: ${red}`);
  };
  $("#pp-pick").onclick = () => pozKaydet("pick", "Pick", $("#pp-pick"));
  $("#pp-place").onclick = () => pozKaydet("place", "Place", $("#pp-place"));

  $("#pp-reset").onclick = async () => {
    try {
      await api("/api/pickplace/draft", { method: "DELETE" });
      yaz("Pick & place draft reset");
      ppTaslakGoster();
    } catch (e) { yaz("Could not reset: " + e.message, "err"); }
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
    const kaydedilemedi = (mesaj) => { yaz("Could not save: " + mesaj, "err"); ppHata("Could not save: " + mesaj); };
    ppHata("");
    try {
      await api("/api/pickplace/save", { method: "POST", body: govde });
    } catch (e) {
      // server.py pp_kaydet: ayni isim varsa 409 "'ad' already exists."
      if (!e.message.includes("already exists")) { kaydedilemedi(e.message); return; }
      const ok = await onayla("Overwrite move", `'${govde.name}' already exists. Overwrite it?`, $("#pp-save"));
      if (!ok) return;
      govde.overwrite = true;
      try { await api("/api/pickplace/save", { method: "POST", body: govde }); } catch (e2) {
        kaydedilemedi(e2.message); return;
      }
    }
    yaz(`'${govde.name}' saved as pick & place`, "ok");
    ppTaslakGoster();
    hareketleriYukle();
  };

  ppTaslakGoster();
}

// ---------------------------------------------------------------------------
// Yol onizleme paneli
// ---------------------------------------------------------------------------
// Sunucudaki onizlemeyi (/api/preview) gosterir: secenek kartlari ve 3D gorunum
// (viewer.js, "onizleme-secimi" olayiyla). Robot yalnizca "Run this path" ve
// onaydan sonra hareket eder; calistirilan plan ekranda gosterilenin AYNISI
// (sunucu sakliyor, tarayici yalnizca numara gonderiyor).

let onizleme = null;              // /api/preview ozeti ya da null
let seciliSecenek = null;
let gosterilenOnizlemeId = null;
let onizlemeYukleniyor = false;

function onizlemeHata(mesaj) {
  $("#pv-err").textContent = mesaj || "";
  $("#pv-err").hidden = !mesaj;
}

// viewer.js bu dosyadan SONRA yuklenir; son secim pencerede de tutulur ki
// modul yuklenirken gecen secimi kacirmasin.
//
// Pick & place onizlemesi (ppOnizleme, asagida) VARSA o gosterilir, yoksa
// normal yol onizlemesi - sunucu ikisini karsilikli disliyor (bkz.
// session.py pp_onizleme_kaydet/onizleme_kaydet), burada JS'de ayrica
// arbitraj gerekmiyor.
function secimiBildir() {
  const d = ppOnizleme ? { onizleme: ppOnizleme, secenek: ppSecenek }
                       : { onizleme, secenek: seciliSecenek };
  window.sonOnizlemeSecimi = d;
  window.dispatchEvent(new CustomEvent("onizleme-secimi", { detail: d }));
}

function secenekSatiri(s) {
  const satir = el("div", "opt" + (s === seciliSecenek ? " sel" : "") + (s.ok ? "" : " bad"));
  satir.tabIndex = 0;
  satir.setAttribute("role", "option");
  satir.setAttribute("aria-selected", s === seciliSecenek ? "true" : "false");
  satir.appendChild(el("span", "num", String(s.index)));
  satir.appendChild(el("span", "name", s.label));
  satir.appendChild(el("span", "state", s.ok ? "OK" : "Rejected"));
  const bilgi = [s.method || "no plan"];
  if (s.max_joint && s.max_deg !== null) bilgi.push(`max ${s.max_joint} ${s.max_deg}°`);
  if (s.duration_s !== null) bilgi.push(`${s.duration_s} s`);
  satir.appendChild(el("span", "meta", bilgi.join(" · ")));
  if (!s.ok) {
    satir.appendChild(el("span", "why",
      (s.reason || "rejected") + (s.details ? ": " + s.details : "")));
  }
  const sec = () => { seciliSecenek = s; onizlemeCiz(); };
  satir.onclick = sec;
  satir.onkeydown = (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); sec(); } };
  return satir;
}

function onizlemeCiz() {
  const kap = $("#pv-options");
  kap.innerHTML = "";
  const planliyor = sonDurum.job && sonDurum.job.running && sonDurum.job.kind === "preview";
  if (!onizleme) {
    $("#pv-info").textContent = planliyor
      ? "Planning paths… (the robot does not move)"
      : "No preview. Turn on “Preview before moving”, then use a jog button or Go.";
    $("#pv-run").disabled = true;
    $("#pv-clear").disabled = true;
    secimiBildir();
    return;
  }
  const secenekler = onizleme.options || [];
  const bilgi = [`${secenekler.length} option(s)`];
  if (onizleme.speed_pct != null) {
    bilgi.push(`planned at speed ${onizleme.speed_pct}% · acceleration ${onizleme.accel_pct}%`);
  }
  const h = onizleme.target_tcp;
  if (h) {
    bilgi.push(`target TCP x=${h.x.toFixed(1)} y=${h.y.toFixed(1)} z=${h.z.toFixed(1)}  ` +
      `rx=${h.rx.toFixed(1)} ry=${h.ry.toFixed(1)} rz=${h.rz.toFixed(1)}`);
  }
  if (onizleme.success === false) {
    bilgi.push(`No usable path: ${onizleme.reason || "rejected"}` +
      (onizleme.details ? " — " + onizleme.details : ""));
  }
  $("#pv-info").textContent = bilgi.join("\n");
  secenekler.forEach((s) => kap.appendChild(secenekSatiri(s)));
  if (!secenekler.length) kap.appendChild(el("div", "note", "No options."));
  $("#pv-run").disabled = !(seciliSecenek && seciliSecenek.ok);
  $("#pv-clear").disabled = false;
  secimiBildir();
}

async function onizlemeYukle(beklenenId) {
  if (onizlemeYukleniyor) return;
  onizlemeYukleniyor = true;
  gosterilenOnizlemeId = beklenenId;   // ayni durum icin tekrar tekrar sormasin
  try {
    const o = await api("/api/preview");
    onizleme = o && o.id ? o : null;
    gosterilenOnizlemeId = onizleme ? onizleme.id : null;
    const secenekler = onizleme ? (onizleme.options || []) : [];
    seciliSecenek = secenekler.find((s) => s.ok) || secenekler[0] || null;
  } catch (e) {
    onizleme = null;
    seciliSecenek = null;
    onizlemeHata("Could not read the preview: " + e.message);
  } finally {
    onizlemeYukleniyor = false;
  }
  onizlemeCiz();
}

function kurOnizleme() {
  $("#pv-mode").onchange = () => {
    yaz(onizlemeAcik()
      ? "Preview mode on: jog and Go only plan paths; nothing moves until you run one."
      : "Preview mode off: jog and Go move the robot again.", "warn");
  };

  $("#pv-clear").onclick = async () => {
    try {
      await api("/api/preview", { method: "DELETE" });
      onizlemeHata("");
      yaz("Preview discarded");
      onizlemeYukle(null);
    } catch (e) { onizlemeHata("Could not discard: " + e.message); }
  };

  $("#pv-run").onclick = async () => {
    if (!onizleme || !seciliSecenek || !seciliSecenek.ok) return;
    const s = seciliSecenek;
    const ok = await onayla("Run previewed path",
      `The robot will follow path ${s.index} (${s.label}, ${s.method}).\n` +
      (s.max_joint && s.max_deg !== null ? `Largest joint rotation: ${s.max_joint} ${s.max_deg}°.\n` : "") +
      "Keep the emergency stop within reach.",
      $("#pv-run"));
    if (!ok) return;
    onizlemeHata("");
    const red = await isGonder("/api/preview/run", { id: onizleme.id, option: s.index },
      `Run previewed path ${s.index}`);
    if (red) onizlemeHata("Run rejected: " + red);
  };

  onizlemeYukle(null);
}

// ---------------------------------------------------------------------------
// Pick & place (kamera + nesne tespiti + MTC)
// ---------------------------------------------------------------------------
// "Detect" kameradan (vision API koprusu) nesneleri ister - robota
// dokunmaz. "Plan pick & place" secili nesne + slot icin TAM zinciri
// (yaklas->kavra->kaldir->tasi->in->birak->geri cekil) robot hareket
// etmeden planlar; 3D gorunumde (viewer.js, ayni "onizleme-secimi" olayi -
// bkz. secimiBildir) gosterilir. Yalnizca "Run this plan" ve onaydan sonra
// robot hareket eder; calistirilan zincir ekranda gosterilenin AYNISI.

let pnpNesneler = [];
let pnpSeciliNesne = null;
let ppOnizleme = null;          // /api/pickplace/preview ozeti ya da null
let ppSecenek = null;           // ppOnizleme.options[0] (tek secenek - tum zincir)
let gosterilenPpId = null;
let ppYukleniyor = false;
let pnpVizyonBilgi = null;      // /api/vision/info sonucu (sonDurum'a KARISTIRILMAZ -
                                // telemetri her turda sonDurum'u bastan yaziyor)

function pnpHata(mesaj) {
  $("#pnp-err").textContent = mesaj || "";
  $("#pnp-err").hidden = !mesaj;
}
function pnpVizyonHata(mesaj) {
  $("#pnp-vision-err").textContent = mesaj || "";
  $("#pnp-vision-err").hidden = !mesaj;
}

function pnpKameralar() {
  const acik = $("#pnp-cams").checked;
  const taban = (pnpVizyonBilgi && pnpVizyonBilgi.cam_url)
    ? pnpVizyonBilgi.cam_url.replace(/\/$/, "")
    : `${location.protocol}//${location.hostname}:8600`;
  $("#pnp-camL").src = acik ? `${taban}/stream/L` : "";
  $("#pnp-camR").src = acik ? `${taban}/stream/R` : "";
}

async function pnpBilgiYukle() {
  try {
    pnpVizyonBilgi = await api("/api/vision/info");
    pnpVizyonHata(pnpVizyonBilgi.handeye ? ""
      : "Hand-eye calibration missing or invalid - object positions may be wrong.");
    const sec = $("#pnp-slot");
    sec.innerHTML = "";
    (pnpVizyonBilgi.slots || []).forEach((s, i) => {
      const o = document.createElement("option");
      o.value = String(i);
      o.textContent = `Slot ${i + 1} — x=${s[0].toFixed(0)} y=${s[1].toFixed(0)} z=${s[2].toFixed(0)}`;
      sec.appendChild(o);
    });
    pnpKameralar();
  } catch (e) {
    pnpVizyonHata("Vision API not reachable: " + e.message);
  }
}

function pnpNesneSatiri(o, i) {
  const secili = o === pnpSeciliNesne;
  const satir = el("div", "opt" + (secili ? " sel" : "") + (o.graspable ? "" : " bad"));
  satir.tabIndex = 0;
  satir.setAttribute("role", "option");
  satir.setAttribute("aria-selected", secili ? "true" : "false");
  satir.appendChild(el("span", "num", String(i + 1)));
  satir.appendChild(el("span", "name", `${o.dims_cm[0]}×${o.dims_cm[1]}×${o.dims_cm[2]} cm`));
  const saga = el("span", "state-wrap");
  saga.style.display = "flex"; saga.style.gap = "6px"; saga.style.alignItems = "center";
  saga.appendChild(el("span", "state", o.graspable ? "graspable" : "not graspable"));
  const sil = el("button", "opt-remove", "✕");
  sil.type = "button";
  sil.title = "Remove from list (false detection, e.g. the arm itself)";
  sil.setAttribute("aria-label", "Remove this object from the list");
  sil.onclick = (e) => { e.stopPropagation(); pnpNesneSil(i); };
  saga.appendChild(sil);
  satir.appendChild(saga);
  satir.appendChild(el("span", "meta", `${o.posture} · vol ${o.volume_l} L`));
  if (!o.graspable || o.reason) satir.appendChild(el("span", "why", o.reason || ""));
  const sec = () => { pnpSeciliNesne = o; pnpNesneleriCiz(); };
  satir.onclick = sec;
  satir.onkeydown = (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); sec(); } };
  return satir;
}

// Yanlis tespit (ör. robotun kendi kolu, arka plan) listeden ve 3D
// gorunumden kaldirilir - COLLISION KONTROLUNU ETKILEMEZ (gercek nesneler
// zaten MoveIt sahnesinde degil, bkz. panel notu), yalnizca gorunumu ve
// secimi temizler. Kaldirilan nesne icin bir plan varsa (tutarsiz olabilir)
// atilir - kullanici "Plan pick & place"i tekrar basar.
function pnpNesneSil(i) {
  const silinen = pnpNesneler[i];
  pnpNesneler = pnpNesneler.filter((_, idx) => idx !== i);
  if (pnpSeciliNesne === silinen) {
    pnpSeciliNesne = pnpNesneler.find((o) => o.graspable) || pnpNesneler[0] || null;
  }
  pnpNesneleriCiz();
  if (ppOnizleme) {
    api("/api/pickplace/preview", { method: "DELETE" }).catch(() => {});
    ppOnizlemeYukle(null);
  }
}

function pnpNesneleriCiz() {
  const kap = $("#pnp-objects");
  kap.innerHTML = "";
  pnpNesneler.forEach((o, i) => kap.appendChild(pnpNesneSatiri(o, i)));
  if (!pnpNesneler.length) kap.appendChild(el("div", "note", "No objects detected yet."));
  window.sonAlgilananNesneler = pnpNesneler.map((o) => ({
    center_mm: o.box ? o.box.center_mm : [0, 0, 0],
    size_mm: o.box ? o.box.size_mm : [10, 10, 10],
    yaw_deg: o.box ? o.box.yaw_deg : 0,
    graspable: o.graspable,
    secili: o === pnpSeciliNesne,
  }));
  window.dispatchEvent(new CustomEvent("algilanan-nesneler", { detail: window.sonAlgilananNesneler }));
}

// 3D gorunumde bir kutuya tiklamak, listeden bir satira tiklamakla AYNI -
// viewer.js sirayi (nesneGrubu.children == pnpNesneler ile ayni sira) bildirir.
window.addEventListener("nesne-tiklandi", (e) => {
  const n = pnpNesneler[e.detail];
  if (!n) return;
  pnpSeciliNesne = n;
  pnpNesneleriCiz();
});

async function pnpAlgila() {
  const btn = $("#pnp-detect");
  btn.disabled = true;
  pnpVizyonHata("");
  yaz("Detecting objects… (the robot does not move)");
  try {
    const r = await api("/api/vision/detect", { method: "POST" });
    pnpNesneler = r.objects || [];
    pnpSeciliNesne = pnpNesneler.find((o) => o.graspable) || pnpNesneler[0] || null;
    pnpNesneleriCiz();
    yaz(`Detected ${pnpNesneler.length} object(s)`, "ok");
  } catch (e) {
    pnpVizyonHata("Detect failed: " + e.message);
    yaz("Detect FAILED: " + e.message, "err");
  } finally {
    btn.disabled = false;
  }
}

function ppOnizlemeCiz() {
  const planliyor = sonDurum.job && sonDurum.job.running && sonDurum.job.kind === "preview"
    && sonDurum.job.label === "Plan pick & place chain";
  if (!ppOnizleme) {
    $("#pnp-info").textContent = planliyor
      ? "Planning pick & place chain… (the robot does not move)"
      : "No plan. Select a detected object and a place slot, then press “Plan pick & place”.";
    $("#pnp-run").disabled = true;
    secimiBildir();
    return;
  }
  const s = (ppOnizleme.options || [])[0];
  if (!s) {
    // IK/MTC baslama noktasinda basarisiz oldu - hic secenek yok (bkz.
    // server.py pp_onizle "basarisiz" yardimcisi, yollari_onizle ile ayni desen).
    ppSecenek = null;
    $("#pnp-info").textContent =
      `No usable plan: ${ppOnizleme.reason || "rejected"}` +
      (ppOnizleme.details ? " — " + ppOnizleme.details : "");
    $("#pnp-run").disabled = true;
    secimiBildir();
    return;
  }
  ppSecenek = s;
  const satirlar = [`${s.ok ? "Runnable" : "REJECTED (" + s.reason + ")"} · ${s.duration_s ?? "?"} s`];
  if (s.max_joint && s.max_deg !== null) satirlar.push(`Largest joint rotation: ${s.max_joint} ${s.max_deg}°`);
  if (s.stages && s.stages.length) {
    satirlar.push(...s.stages.filter((a) => a.tip !== "sahne")
      .map((a) => `  ${a.t0}–${a.t1}s  ${a.tip === "gripper" ? "[gripper]" : a.ad}`));
  }
  $("#pnp-info").textContent = satirlar.join("\n");
  $("#pnp-run").disabled = !s.ok;
  secimiBildir();
}

async function ppOnizlemeYukle(beklenenId) {
  if (ppYukleniyor) return;
  ppYukleniyor = true;
  gosterilenPpId = beklenenId;
  try {
    const o = await api("/api/pickplace/preview");
    ppOnizleme = o && o.id ? o : null;
    gosterilenPpId = ppOnizleme ? ppOnizleme.id : null;
  } catch (e) {
    ppOnizleme = null;
    pnpHata("Could not read the plan: " + e.message);
  } finally {
    ppYukleniyor = false;
  }
  ppOnizlemeCiz();
}

function kurNesneAlBirak() {
  $("#pnp-cams").onchange = pnpKameralar;
  $("#pnp-detect").onclick = pnpAlgila;

  $("#pnp-discard").onclick = async () => {
    try {
      await api("/api/pickplace/preview", { method: "DELETE" });
      pnpHata("");
      yaz("Pick & place plan discarded");
      ppOnizlemeYukle(null);
    } catch (e) { pnpHata("Could not discard: " + e.message); }
  };

  $("#pnp-plan").onclick = async () => {
    pnpHata("");
    if (!pnpSeciliNesne || !pnpSeciliNesne.grasp) {
      pnpHata("Select a graspable object first (Detect, then click one in the list).");
      return;
    }
    const slotIdx = parseInt($("#pnp-slot").value, 10);
    const slot = (pnpVizyonBilgi && pnpVizyonBilgi.slots || [])[slotIdx];
    if (!slot) { pnpHata("No place slot available (check the vision API)."); return; }
    const approach = sayiAlan($("#pnp-approach"), true);
    const objh = sayiAlan($("#pnp-objh"), true);
    if (approach === null || objh === null) return;
    // box.size_mm = [genislik (uzun yatay), kalinlik (kapanma yonu), yukseklik]
    // (bkz. detect_api.py _nesne_sozlugu / OBB.extents sirasi) - dims_cm
    // BUYUKTEN KUCUGE siralanmis oldugu icin eksenlere geri eslenemez.
    const boyut = pnpSeciliNesne.box ? pnpSeciliNesne.box.size_mm : [null, null, null];
    const govde = {
      grasp_point: pnpSeciliNesne.grasp.point_mm,
      grasp_rpy: pnpSeciliNesne.grasp.rpy_deg,
      place_pose: slot,
      approach_mm: approach,
      grip_speed: parseInt($("#pnp-gspeed").value, 10) || 50,
      grip_force: parseInt($("#pnp-gforce").value, 10) || 5,
      object_w_mm: boyut[1] || null,   // kapanma yonundeki kalinlik - TAHMIN (bkz. panel notu)
      object_d_mm: boyut[0] || 50,
      object_h_mm: objh,
    };
    yaz("Planning pick & place chain… (the robot does not move)");
    try {
      await api("/api/pickplace/preview", { method: "POST", body: govde });
    } catch (e) {
      pnpHata("Plan rejected: " + e.message);
      yaz("Pick & place plan REJECTED: " + e.message, "err");
    }
  };

  $("#pnp-run").onclick = async () => {
    if (!ppOnizleme || !ppSecenek || !ppSecenek.ok) return;
    const s = ppSecenek;
    const ok = await onayla("Run pick & place",
      "The robot will follow the plan shown: approach → grip → lift → transfer → " +
      "lower → release → retreat.\n" +
      (s.max_joint && s.max_deg !== null ? `Largest joint rotation: ${s.max_joint} ${s.max_deg}°.\n` : "") +
      "Keep the emergency stop within reach.",
      $("#pnp-run"));
    if (!ok) return;
    pnpHata("");
    try {
      await api("/api/pickplace/preview/run", { method: "POST", body: { id: ppOnizleme.id } });
      yaz("Run pick & place: sent");
    } catch (e) {
      pnpHata("Run rejected: " + e.message);
      yaz("Run pick & place REJECTED: " + e.message, "err");
    }
  };

  pnpKameralar();
  pnpBilgiYukle();
  ppOnizlemeYukle(null);
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
  $("#t-conn").textContent = bagli ? "connected" : (s.startup_error ? "error" : "starting");
  // Hiz/ivme: Ayarlar penceresinin degeri (s.settings). s.speed_pct oturumun
  // acilis varsayilani, pencere onu degistirmiyor (bkz. session.py ayarlar).
  const ayar = s.settings || { speed_pct: s.speed_pct, accel_pct: s.accel_pct };
  $("#t-robot").textContent = s.model
    ? `${s.model} · ${s.robot_ip} · speed ${ayar.speed_pct}% · accel ${ayar.accel_pct}%`
    : "—";

  const f = s.fault;
  const fvar = f && (f.main_code !== 0 || f.sub_code !== 0);
  $("#d-fault").className = "dot " + (f ? (fvar ? "err" : "ok") : "");
  $("#t-fault").textContent = f ? (fvar ? `fault ${f.main_code}/${f.sub_code}` : "no fault") : "fault ?";

  // Onizleme planlanirken de kilit mesgul; "moving" yazmasin, robot duruyor.
  const planliyor = s.job && s.job.running && s.job.kind === "preview";
  $("#d-busy").className = "dot " + (s.busy ? "busy" : "ok");
  $("#t-busy").textContent = s.busy ? (planliyor ? "planning" : "moving") : "idle";

  const je = $("#joints"); je.innerHTML = "";
  (s.joints_deg || []).forEach((v, i) => kutu(je, EKLEMLER[i], v.toFixed(2), "°"));
  // 3D gorunum (viewer.js) kolun canli durusunu bu olaydan aliyor.
  if (s.joints_deg) {
    window.dispatchEvent(new CustomEvent("kol-durumu", { detail: s.joints_deg }));
  }

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
    if (j.ok) yaz(`${j.label} — done (${j.seconds}s)`, "ok");
    else yaz(`${j.label} — FAILED: ${j.detail}`, "err");
    hareketleriYukle();
    if (j.label.startsWith("Pick & place")) {
      ppTaslakGoster();
      ppHata(j.ok ? "" : `${j.label} failed: ${j.detail}`);
    }
  }

  // Onizleme degistiyse (planlama bitti, bir secenek calistirildi ya da
  // silindi) paneli yenile; sunucu numarasini telemetride bildiriyor.
  if ((s.preview_id || null) !== gosterilenOnizlemeId) {
    onizlemeYukle(s.preview_id || null);
  }
  if ((s.pp_preview_id || null) !== gosterilenPpId) {
    ppOnizlemeYukle(s.pp_preview_id || null);
  }

  // Acilis ekrani
  const ov = $("#overlay");
  if (bagli) ov.classList.add("hidden");
  else {
    ov.classList.remove("hidden");
    $("#ovtitle").textContent = s.startup_error ? "Connection failed" : "Connecting to robot…";
    $("#ovlog").textContent = (s.startup_log || []).join("\n");
  }
}

async function gripperDurum() {
  if (!sonDurum.ready) return;
  try {
    const g = await api("/api/gripper/status");
    const kap = $("#gstat"); kap.innerHTML = "";
    if (g.error) { kutu(kap, "state", g.error); return; }
    const anlam = { 0: "moving", 1: "done · no object", 2: "done · object held" };
    kutu(kap, "state", anlam[g.motion_status] || g.motion_status);
    kutu(kap, "fault", g.fault);
    kutu(kap, "position", g.position_pct, " %");
    kutu(kap, "speed", g.speed_pct, " %");
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
    $("#t-conn").textContent = "connection lost";
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
kurOnizleme();
kurNesneAlBirak();
hareketleriYukle();
baglan();
setInterval(gripperDurum, 1200);

$("#stopmove").onclick = async () => {
  try {
    const r = await api("/api/stop", { method: "POST" });
    if (r.durduruldu) yaz("Motion stopped.", "warn");
    else yaz("Could not stop motion: " + (r.hata || JSON.stringify(r)), "err");
  } catch (e) { yaz("Stop error: " + e.message, "err"); }
};

yaz("Interface ready.");

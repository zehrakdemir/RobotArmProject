// Yol onizlemesi 3D gorunumu (three.js 0.186.0, static/lib/three, MIT).
//
// YALNIZCA GORUNTU: robota hicbir istek gondermez. Veriyi app.js'ten olaylarla
// alir:
//   "kol-durumu"       detail = kolun su anki 6 eklem acisi (derece)
//   "onizleme-secimi"  detail = { onizleme, secenek } (ikisi de null olabilir)
// Model /api/model'den gelir: MoveIt'in KULLANDIGI URDF + container'daki STL
// dosyalari. Internetten hicbir sey yuklenmez (robot PC'si internetsiz olabilir).
//
// Renkler: gri = kolun su anki durusu, --accent = secili calistirilabilir yol,
// --err = reddedilen yol, --ok = hedef TCP noktasi.

import * as THREE from "three";
import { OrbitControls } from "/static/lib/three/OrbitControls.js";
import { STLLoader } from "/static/lib/three/STLLoader.js";

const kap = document.getElementById("pv-view");
const mesajEl = document.getElementById("pv-view-msg");
const oynatBtn = document.getElementById("pv-play");
const kaydirici = document.getElementById("pv-scrub");
const hizSec = document.getElementById("pv-rate");
const zamanEl = document.getElementById("pv-time");

const css = getComputedStyle(document.documentElement);
const renk = (ad) => new THREE.Color(css.getPropertyValue(ad).trim());
const hareketAzalt = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
// Yol sonunda animasyon bastan baslamadan once bekleme (saniye).
const SON_BEKLEME_S = 0.8;
// Eksen ucgenlerinin uzunlugu (metre): base (dunya) cercevesi ve TCP cercevesi.
// Renkler three.js AxesHelper'in kendi renkleri: X kirmizi, Y yesil, Z mavi.
const BASE_EKSEN_M = 0.15;
const TCP_EKSEN_M = 0.06;

function mesaj(metin) {
  mesajEl.textContent = metin || "";
  mesajEl.hidden = !metin;
}

// URDF rpy: sabit eksenli X-Y-Z, R = Rz(yaw) * Ry(pitch) * Rx(roll). three.js'te
// "ZYX" sirasi ayni matrisi verir.
const rpy = (a) => new THREE.Euler(a[0], a[1], a[2], "ZYX");

let cizici = null;
try {
  cizici = new THREE.WebGLRenderer({ antialias: true });
} catch (e) {
  mesaj("3D view unavailable: WebGL could not start (" + e.message + "). The options list still works.");
}

if (cizici) kur();

function kur() {
  const sahne = new THREE.Scene();
  sahne.background = renk("--bg");

  const kamera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
  kamera.up.set(0, 0, 1);                 // ROS/MoveIt: Z yukari
  kamera.position.set(1.0, -1.25, 0.9);
  cizici.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  kap.appendChild(cizici.domElement);

  const kontrol = new OrbitControls(kamera, cizici.domElement);
  kontrol.target.set(0, 0, 0.25);
  kontrol.enableDamping = true;
  kontrol.update();

  sahne.add(new THREE.HemisphereLight(0xffffff, 0x2a2f3a, 2.0));
  const gunes = new THREE.DirectionalLight(0xffffff, 2.0);
  gunes.position.set(1.5, -1.0, 2.5);
  sahne.add(gunes);

  // Zemin z = 0: MoveIt'teki zemin carpisma kutusunun ust yuzeyi. GridHelper
  // XZ duzleminde cizildigi icin X ekseninde 90 derece dondurulur.
  const izgara = new THREE.GridHelper(2, 20, renk("--dim"), renk("--line"));
  izgara.rotation.x = Math.PI / 2;
  sahne.add(izgara);
  // Base (dunya) cercevesi: Base jog yonleri bu eksenlere gore. Sabittir.
  sahne.add(new THREE.AxesHelper(BASE_EKSEN_M));

  const boyutla = () => {
    const w = kap.clientWidth;
    const h = kap.clientHeight;
    if (!w || !h) return;
    cizici.setSize(w, h, false);          // canvas boyutu CSS'te %100
    kamera.aspect = w / h;
    kamera.updateProjectionMatrix();
  };
  new ResizeObserver(boyutla).observe(kap);
  boyutla();

  // --- malzemeler ---------------------------------------------------------
  const canliMalzeme = new THREE.MeshStandardMaterial({
    color: 0xc9ced6, roughness: 0.6, metalness: 0.1, transparent: true, opacity: 1,
  });
  const hayaletMalzeme = new THREE.MeshStandardMaterial({
    color: renk("--accent"), roughness: 0.5, metalness: 0.0,
    transparent: true, opacity: 0.6, depthWrite: false,
  });
  const kutuCanli = new THREE.MeshBasicMaterial({ color: renk("--dim"), wireframe: true });
  const kutuHayalet = new THREE.MeshBasicMaterial({ color: renk("--accent"), wireframe: true });

  const yolCizgisi = new THREE.Line(new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: renk("--accent") }));
  yolCizgisi.visible = false;
  const hedefNokta = new THREE.Mesh(new THREE.SphereGeometry(0.012, 20, 12),
    new THREE.MeshBasicMaterial({ color: renk("--ok") }));
  hedefNokta.visible = false;
  sahne.add(yolCizgisi, hedefNokta);

  // Kameranin tespit ettigi nesneler: tel-kafes kutular. app.js "Detect"
  // sonucunu "algilanan-nesneler" olayiyla gonderir - robota/plana bagli
  // degil, salt gorsel. Renk: secili --accent, kavranamaz --err, digerleri --dim.
  const nesneGrubu = new THREE.Group();
  sahne.add(nesneGrubu);

  function nesneleriCiz(liste) {
    while (nesneGrubu.children.length) {
      const c = nesneGrubu.children.pop();
      c.geometry.dispose(); c.material.dispose();
    }
    (liste || []).forEach((n) => {
      const [sx, sy, sz] = n.size_mm.map((v) => Math.max(v, 1) / 1000);
      const renkAdi = n.secili ? "--accent" : (n.graspable ? "--dim" : "--err");
      const kutu = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz),
        new THREE.MeshBasicMaterial({ color: renk(renkAdi), wireframe: true }));
      kutu.position.set(n.center_mm[0] / 1000, n.center_mm[1] / 1000, n.center_mm[2] / 1000);
      kutu.rotation.z = (n.yaw_deg || 0) * Math.PI / 180;
      nesneGrubu.add(kutu);
    });
  }
  window.addEventListener("algilanan-nesneler", (e) => nesneleriCiz(e.detail));
  if (window.sonAlgilananNesneler) nesneleriCiz(window.sonAlgilananNesneler);

  // Kutuya tiklayinca app.js'e "nesne-tiklandi" (detail: sira) gonderilir -
  // liste ile senkron secim. Surukleyerek dondurme (OrbitControls) TIKLAMA
  // sayilmasin diye basi/sonu 5px'ten fazla kaymissa yoksayilir.
  const isaretci = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  let basiX = 0, basiY = 0;
  cizici.domElement.addEventListener("pointerdown", (e) => { basiX = e.clientX; basiY = e.clientY; });
  cizici.domElement.addEventListener("pointerup", (e) => {
    if (Math.hypot(e.clientX - basiX, e.clientY - basiY) > 5) return;
    if (!nesneGrubu.children.length) return;
    const r = cizici.domElement.getBoundingClientRect();
    ndc.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    ndc.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    isaretci.setFromCamera(ndc, kamera);
    const vurus = isaretci.intersectObjects(nesneGrubu.children)[0];
    if (vurus) {
      window.dispatchEvent(new CustomEvent("nesne-tiklandi",
        { detail: nesneGrubu.children.indexOf(vurus.object) }));
    }
  });

  // --- model --------------------------------------------------------------
  let model = null;
  let canli = null;
  let hayalet = null;
  let modelYukleniyor = false;
  let sonDeneme = 0;
  let sonAcilar = null;

  function robotKur(m, geometriler, malzeme, kutuMalzeme) {
    const linkler = {};
    for (const l of m.links) {
      const g = new THREE.Group();
      linkler[l.name] = g;
      const geo = geometriler[l.name];
      if (geo) {
        const parca = new THREE.Mesh(geo, malzeme);
        parca.position.fromArray(l.xyz);
        parca.quaternion.setFromEuler(rpy(l.rpy));
        parca.scale.fromArray(l.scale);
        g.add(parca);
      }
    }
    // Her eklem: ust link -> sabit donusum (origin) -> eklem ekseninde donen grup -> alt link.
    const eklemler = {};
    const kayarlar = [];
    const cocuklar = new Set();
    for (const j of m.joints) {
      const ust = linkler[j.parent];
      const alt = linkler[j.child];
      if (!ust || !alt) continue;
      const sabit = new THREE.Group();
      sabit.position.fromArray(j.xyz);
      sabit.quaternion.setFromEuler(rpy(j.rpy));
      const donen = new THREE.Group();
      ust.add(sabit);
      sabit.add(donen);
      donen.add(alt);
      cocuklar.add(j.child);
      if (j.type === "revolute" || j.type === "continuous") {
        eklemler[j.name] = { grup: donen, eksen: new THREE.Vector3().fromArray(j.axis).normalize() };
      } else if (j.type === "prismatic") {
        // Gripper parmaklari: kol eklemi degil, degeri gripper durumundan gelir.
        kayarlar.push({ ad: j.name, grup: donen, eksen: new THREE.Vector3().fromArray(j.axis).normalize(),
                        limit: j.limit, mimic: j.mimic });
      }
    }
    const kok = new THREE.Group();
    for (const l of m.links) if (!cocuklar.has(l.name)) kok.add(linkler[l.name]);

    // TCP isareti: ucun (wrist3_link) KENDI ekseninde olculen TCP ofseti.
    const tcp = new THREE.Object3D();
    const uc = linkler[m.tip_link];
    if (uc) {
      tcp.position.fromArray(m.tcp_offset_m);
      // TCP cercevesi: aletle birlikte doner, yani Tool jog yonlerini gosterir.
      // Yonelim wrist3_link'in yonelimidir - olculen TCP ofseti yalnizca
      // oteleme (bkz. compute_tcp_offset); predict_tcp_pose da TCP'nin
      // RX/RY/RZ'si olarak ayni yonelimi veriyor.
      tcp.add(new THREE.AxesHelper(TCP_EKSEN_M));
      uc.add(tcp);
    }
    // Gripper carpisma kutusu: planlayicinin kullandigi kutu (movecobot.py).
    const k = m.gripper_box;
    if (k && linkler[k.link]) {
      const kutu = new THREE.Mesh(new THREE.BoxGeometry(k.size[0], k.size[1], k.size[2]), kutuMalzeme);
      kutu.position.fromArray(k.xyz);
      linkler[k.link].add(kutu);
    }
    return { kok, eklemler, kayarlar, tcp };
  }

  // Gripper acikligi: position 0 = tam acik, 100 = tam kapali (gripper_controller).
  // Surucu olan eklem (mimic'siz) limitin ustune gore orantilanir, mimic'ler ona baglanir.
  function parmaklariUygula(robot, konumYuzde) {
    if (!robot || !Number.isFinite(konumYuzde)) return;
    const degerler = {};
    for (const k of robot.kayarlar) {
      if (k.mimic || !k.limit) continue;
      const oran = Math.min(Math.max((100 - konumYuzde) / 100, 0), 1);
      degerler[k.ad] = k.limit.lower + oran * (k.limit.upper - k.limit.lower);
    }
    for (const k of robot.kayarlar) {
      const v = k.mimic ? k.mimic.multiplier * (degerler[k.mimic.joint] ?? 0) + k.mimic.offset
                        : degerler[k.ad];
      if (Number.isFinite(v)) k.grup.position.copy(k.eksen).multiplyScalar(v);
    }
  }

  function acilariUygula(robot, derece) {
    model.joint_names.forEach((ad, i) => {
      const e = robot.eklemler[ad];
      if (e && Number.isFinite(derece[i])) {
        e.grup.quaternion.setFromAxisAngle(e.eksen, THREE.MathUtils.degToRad(derece[i]));
      }
    });
  }

  async function modeliYukle() {
    if (model || modelYukleniyor || Date.now() - sonDeneme < 5000) return;
    modelYukleniyor = true;
    sonDeneme = Date.now();
    mesaj("Loading robot model…");
    try {
      const cevap = await fetch("/api/model");
      if (!cevap.ok) {
        const govde = await cevap.json().catch(() => ({}));
        throw new Error(govde.detail || cevap.statusText);
      }
      const m = await cevap.json();
      const yukleyici = new STLLoader();
      const geometriler = {};
      await Promise.all(m.links.filter((l) => l.mesh).map(async (l) => {
        geometriler[l.name] = await yukleyici.loadAsync(l.mesh);
      }));
      model = m;
      canli = robotKur(m, geometriler, canliMalzeme, kutuCanli);
      hayalet = robotKur(m, geometriler, hayaletMalzeme, kutuHayalet);
      hayalet.kok.visible = false;
      sahne.add(canli.kok, hayalet.kok);
      mesaj(Object.keys(geometriler).length ? "" : "Robot meshes were not found; only the path line is shown.");
      if (sonAcilar) acilariUygula(canli, sonAcilar);
      if (window.sonGripperKonumu != null) {
        parmaklariUygula(canli, window.sonGripperKonumu);
        parmaklariUygula(hayalet, window.sonGripperKonumu);
      }
      secimiCiz();
    } catch (e) {
      mesaj("Could not load the robot model: " + e.message + " (retrying)");
    } finally {
      modelYukleniyor = false;
    }
  }

  // --- secili yol ve animasyon -------------------------------------------
  let onizleme = null;
  let secenek = null;
  let anim = null;   // { noktalar, zamanlar, sure, t, oynuyor }

  function kontrolleriAc(acik) {
    oynatBtn.disabled = !acik;
    kaydirici.disabled = !acik;
    if (!acik) { zamanEl.textContent = "—"; kaydirici.value = 0; oynatBtn.textContent = "Play"; }
  }

  function kareyiUygula(t) {
    const { noktalar, zamanlar, sure } = anim;
    let i = 1;
    while (i < zamanlar.length && zamanlar[i] < t) i++;
    if (i >= zamanlar.length) {
      acilariUygula(hayalet, noktalar[noktalar.length - 1]);
    } else {
      const t0 = zamanlar[i - 1];
      const t1 = zamanlar[i];
      const a = t1 > t0 ? Math.min(Math.max((t - t0) / (t1 - t0), 0), 1) : 1;
      const p = noktalar[i - 1];
      const q = noktalar[i];
      acilariUygula(hayalet, p.map((v, k) => v + (q[k] - v) * a));
    }
    const gosterilen = Math.min(t, sure);
    zamanEl.textContent = `${gosterilen.toFixed(1)} / ${sure.toFixed(1)} s`;
    kaydirici.value = sure > 0 ? Math.round((gosterilen / sure) * 1000) : 1000;
  }

  function secimiCiz() {
    const hedef = onizleme && onizleme.target_tcp;
    hedefNokta.visible = !!hedef;
    if (hedef) hedefNokta.position.set(hedef.x / 1000, hedef.y / 1000, hedef.z / 1000);

    const noktalar = secenek && secenek.points && secenek.points.length ? secenek.points : null;
    if (!model) return;

    const renkAdi = secenek && !secenek.ok ? "--err" : "--accent";
    hayaletMalzeme.color = renk(renkAdi);
    kutuHayalet.color = renk(renkAdi);
    yolCizgisi.material.color = renk(renkAdi);
    // Yol gosterilirken su anki kol soluk: hayalet onun icinden gorulsun.
    canliMalzeme.opacity = noktalar ? 0.35 : 1;
    canliMalzeme.depthWrite = !noktalar;
    hayalet.kok.visible = !!noktalar;
    yolCizgisi.visible = !!noktalar;
    if (!noktalar) {
      anim = null;
      kontrolleriAc(false);
      return;
    }

    // TCP'nin yol boyunca konumu: hayalet her noktaya konup TCP isaretinin
    // dunya konumu okunur (ayni URDF kinematigi).
    const konumlar = new Float32Array(noktalar.length * 3);
    const v = new THREE.Vector3();
    noktalar.forEach((n, i) => {
      acilariUygula(hayalet, n);
      hayalet.kok.updateMatrixWorld(true);
      hayalet.tcp.getWorldPosition(v);
      v.toArray(konumlar, i * 3);
    });
    yolCizgisi.geometry.dispose();
    yolCizgisi.geometry = new THREE.BufferGeometry();
    yolCizgisi.geometry.setAttribute("position", new THREE.BufferAttribute(konumlar, 3));

    const zamanlar = secenek.times && secenek.times.length === noktalar.length
      ? secenek.times : noktalar.map((_, i) => i * 0.1);
    const sure = zamanlar[zamanlar.length - 1] || 0;
    anim = { noktalar, zamanlar, sure, t: hareketAzalt ? sure : 0, oynuyor: !hareketAzalt };
    kontrolleriAc(true);
    oynatBtn.textContent = anim.oynuyor ? "Pause" : "Play";
    kareyiUygula(anim.t);
  }

  oynatBtn.onclick = () => {
    if (!anim) return;
    if (!anim.oynuyor && anim.t >= anim.sure) anim.t = 0;
    anim.oynuyor = !anim.oynuyor;
    oynatBtn.textContent = anim.oynuyor ? "Pause" : "Play";
  };
  kaydirici.oninput = () => {
    if (!anim) return;
    anim.oynuyor = false;
    oynatBtn.textContent = "Play";
    anim.t = (Number(kaydirici.value) / 1000) * anim.sure;
    kareyiUygula(anim.t);
  };

  // --- olaylar ------------------------------------------------------------
  window.addEventListener("kol-durumu", (e) => {
    sonAcilar = e.detail;
    if (!model) { modeliYukle(); return; }
    acilariUygula(canli, sonAcilar);
  });
  // Gripper'in OKUDUGU konum (app.js gripperDurum). Okuma guvenilmez olabilir
  // (GUNCEL_DURUM 9-6) - gorunum ne okunduysa onu gosterir.
  window.addEventListener("gripper-durumu", (e) => {
    parmaklariUygula(canli, e.detail);
    parmaklariUygula(hayalet, e.detail);
  });
  const secimAl = (d) => { onizleme = d.onizleme; secenek = d.secenek; secimiCiz(); };
  window.addEventListener("onizleme-secimi", (e) => secimAl(e.detail));
  // Bu modul app.js'ten SONRA yuklenir; o arada gelen son secim kacmasin.
  if (window.sonOnizlemeSecimi) secimAl(window.sonOnizlemeSecimi);

  let onceki = performance.now();
  function dongu(simdi) {
    const dt = (simdi - onceki) / 1000;
    onceki = simdi;
    if (anim && anim.oynuyor) {
      anim.t += dt * Number(hizSec.value || 1);
      if (anim.t > anim.sure + SON_BEKLEME_S) anim.t = 0;
      kareyiUygula(anim.t);
    }
    kontrol.update();
    cizici.render(sahne, kamera);
    requestAnimationFrame(dongu);
  }
  requestAnimationFrame(dongu);
}

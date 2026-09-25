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
//
// Pick & place onizlemesinde (onizleme.object) nesne de canlandirilir:
// kavramaya kadar alma yerinde, gripper kapaninca uc linke bagli, acilinca
// birakma yerinde. Zaman cizelgesi secenegin "gripper" asamalarindan gelir.
//
// "Show safety layers" (#pv-safety): planlayicinin carpisma modeli - zemin
// kutusu, link padding kabugu, gripper kutusu + padding, tasinan nesnenin
// kutusu + padding. YALNIZCA GORSEL; hicbir kontrolu acip kapatmaz.

import * as THREE from "three";
import { OrbitControls } from "/static/lib/three/OrbitControls.js";
import { STLLoader } from "/static/lib/three/STLLoader.js";

const kap = document.getElementById("pv-view");
const mesajEl = document.getElementById("pv-view-msg");
const oynatBtn = document.getElementById("pv-play");
const kaydirici = document.getElementById("pv-scrub");
const hizSec = document.getElementById("pv-rate");
const zamanEl = document.getElementById("pv-time");
const guvenlikKutu = document.getElementById("pv-safety");

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

// Link padding kabugu: MoveIt'in mesh padding'inin aynisi (geometric_shapes
// 2.3.4 Mesh::scaleAndPadd + computeVertexNormals): her kose, komsu
// ucgenlerin koseye dusen ACIYLA agirliklandirilmis normali boyunca "pay"
// kadar disari itilir. STL ucgenleri ayri kose tasir; once ayni konumdaki
// koseler birlestirilir (yoksa her ucgen kendi normaliyle ayrilir).
function sisirilmisGeometri(geo, pay) {
  const p = geo.getAttribute("position");
  const anahtarlar = new Map();
  const koseler = [];
  const indeks = new Uint32Array(p.count);
  for (let i = 0; i < p.count; i++) {
    const x = p.getX(i), y = p.getY(i), z = p.getZ(i);
    const k = `${Math.round(x * 1e6)},${Math.round(y * 1e6)},${Math.round(z * 1e6)}`;
    let j = anahtarlar.get(k);
    if (j === undefined) { j = koseler.length / 3; anahtarlar.set(k, j); koseler.push(x, y, z); }
    indeks[i] = j;
  }
  const normaller = new Float64Array(koseler.length);
  const v = (j) => new THREE.Vector3(koseler[j * 3], koseler[j * 3 + 1], koseler[j * 3 + 2]);
  for (let f = 0; f + 2 < indeks.length; f += 3) {
    const ids = [indeks[f], indeks[f + 1], indeks[f + 2]];
    const [a, b, c] = ids.map(v);
    const n = new THREE.Vector3().subVectors(b, a).cross(new THREE.Vector3().subVectors(c, a));
    if (n.lengthSq() === 0) continue;
    n.normalize();
    const acilar = [
      new THREE.Vector3().subVectors(b, a).angleTo(new THREE.Vector3().subVectors(c, a)),
      new THREE.Vector3().subVectors(a, b).angleTo(new THREE.Vector3().subVectors(c, b)),
      new THREE.Vector3().subVectors(a, c).angleTo(new THREE.Vector3().subVectors(b, c)),
    ];
    ids.forEach((j, s) => {
      normaller[j * 3] += n.x * acilar[s];
      normaller[j * 3 + 1] += n.y * acilar[s];
      normaller[j * 3 + 2] += n.z * acilar[s];
    });
  }
  const konum = new Float32Array(koseler.length);
  for (let j = 0; j < koseler.length / 3; j++) {
    const n = new THREE.Vector3(normaller[j * 3], normaller[j * 3 + 1], normaller[j * 3 + 2]);
    if (n.lengthSq() > 0) n.normalize();
    konum[j * 3] = koseler[j * 3] + n.x * pay;
    konum[j * 3 + 1] = koseler[j * 3 + 1] + n.y * pay;
    konum[j * 3 + 2] = koseler[j * 3 + 2] + n.z * pay;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(konum, 3));
  g.setIndex(new THREE.BufferAttribute(indeks, 1));
  g.computeVertexNormals();
  return g;
}

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
  // Eksen ucgenleri "Show axes" (#pv-axes) acikken gorunur.
  const eksenParcalari = [];
  const eksenKutu = document.getElementById("pv-axes");
  function eksenleriGoster() {
    const acik = !!(eksenKutu && eksenKutu.checked);
    for (const e of eksenParcalari) e.visible = acik;
  }
  if (eksenKutu) eksenKutu.onchange = eksenleriGoster;
  // Base (dunya) cercevesi: Base jog yonleri bu eksenlere gore. Sabittir.
  const baseEksen = new THREE.AxesHelper(BASE_EKSEN_M);
  sahne.add(baseEksen);
  eksenParcalari.push(baseEksen);
  eksenleriGoster();

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
  // Guvenlik katmanlari (yalnizca gorsel): padding kabuklari --warn, zemin --err.
  const kabukMalzeme = new THREE.MeshStandardMaterial({
    color: renk("--warn"), roughness: 0.8, metalness: 0.0,
    transparent: true, opacity: 0.22, depthWrite: false,
  });
  const zeminMalzeme = new THREE.MeshBasicMaterial({
    color: renk("--err"), transparent: true, opacity: 0.10, depthWrite: false,
  });
  const planKutuMalzeme = new THREE.MeshBasicMaterial({ color: renk("--warn"), wireframe: true });
  // Pick & place: tasinan nesne (dolu) ve birakma/alma yerleri (tel kafes).
  const nesneMalzeme = new THREE.MeshStandardMaterial({
    color: renk("--ok"), roughness: 0.7, metalness: 0.0, transparent: true, opacity: 0.85,
  });
  const birakmaMalzeme = new THREE.LineBasicMaterial({ color: renk("--ok") });
  const almaMalzeme = new THREE.LineBasicMaterial({ color: renk("--dim") });

  // Guvenlik katmani parcalari: { nesne, kosul } - kosul() false ise kutucuk
  // acik olsa da gizli (ör. canli kolun kabugu yol gosterilirken).
  const guvenlikParcalari = [];
  let yolGosteriliyor = false;
  const guvenlikAcik = () => !!(guvenlikKutu && guvenlikKutu.checked);
  function guvenlikGoster() {
    const acik = guvenlikAcik();
    for (const p of guvenlikParcalari) p.nesne.visible = acik && (!p.kosul || p.kosul());
  }
  if (guvenlikKutu) {
    guvenlikKutu.onchange = () => {
      guvenlikGoster();
      if (anim) kareyiUygula(anim.t);
    };
  }

  const yolCizgisi = new THREE.Line(new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: renk("--accent") }));
  yolCizgisi.visible = false;
  const hedefNokta = new THREE.Mesh(new THREE.SphereGeometry(0.012, 20, 12),
    new THREE.MeshBasicMaterial({ color: renk("--ok") }));
  hedefNokta.visible = false;
  // Pick & place: birakmada parmak uclarinin noktasi (hedefNokta = kavrama).
  const birakmaNokta = new THREE.Mesh(new THREE.SphereGeometry(0.012, 20, 12),
    new THREE.MeshBasicMaterial({ color: renk("--ok") }));
  birakmaNokta.visible = false;
  sahne.add(yolCizgisi, hedefNokta, birakmaNokta);

  // Pick & place nesnesi ve alma/birakma yerleri. Matrisler elle kurulur.
  const elleKonumlu = (o) => { o.matrixAutoUpdate = false; o.visible = false; sahne.add(o); return o; };
  const ppNesne = elleKonumlu(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), nesneMalzeme));
  const ppAlma = elleKonumlu(new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), almaMalzeme));
  const ppBirakma = elleKonumlu(new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), birakmaMalzeme));
  // Planlayicinin tasinan nesne kutusu ve attached padding'li hali.
  const ppPlanKutu = elleKonumlu(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), planKutuMalzeme));
  const ppPlanKabuk = elleKonumlu(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), kabukMalzeme));
  let pp = null;   // pick & place zaman cizelgesi (ppKur), yoksa null

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

  // kabukKosulu: bu robotun guvenlik kabuklari ne zaman gorunsun (bkz.
  // guvenlikParcalari).
  function robotKur(m, geometriler, malzeme, kutuMalzeme, kabukGeometrileri, kabukKosulu) {
    const linkler = {};
    const guvenlikEkle = (nesne) => {
      nesne.visible = false;
      guvenlikParcalari.push({ nesne, kosul: kabukKosulu });
    };
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
      const kabukGeo = kabukGeometrileri[l.name];
      if (kabukGeo) {
        const kabuk = new THREE.Mesh(kabukGeo, kabukMalzeme);
        kabuk.position.fromArray(l.xyz);
        kabuk.quaternion.setFromEuler(rpy(l.rpy));
        kabuk.scale.fromArray(l.scale);
        g.add(kabuk);
        guvenlikEkle(kabuk);
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
      const tcpEksen = new THREE.AxesHelper(TCP_EKSEN_M);
      tcp.add(tcpEksen);
      eksenParcalari.push(tcpEksen);
      uc.add(tcp);
    }
    // Gripper carpisma kutusu: planlayicinin kullandigi kutu (movecobot.py).
    const k = m.gripper_box;
    if (k && linkler[k.link]) {
      const kutu = new THREE.Mesh(new THREE.BoxGeometry(k.size[0], k.size[1], k.size[2]), kutuMalzeme);
      kutu.position.fromArray(k.xyz);
      linkler[k.link].add(kutu);
      // Kutu da bir guvenlik katmani: yalnizca "Show safety layers" acikken.
      guvenlikEkle(kutu);
      // Kutu ilisik (attached) cisim: attached padding her yone eklenir
      // (kutu primitifinde olcu + 2 x pay).
      const pay = m.safety && m.safety.padding_m ? m.safety.padding_m.attached : 0;
      if (pay > 0) {
        const kabuk = new THREE.Mesh(new THREE.BoxGeometry(
          k.size[0] + 2 * pay, k.size[1] + 2 * pay, k.size[2] + 2 * pay), kabukMalzeme);
        kabuk.position.fromArray(k.xyz);
        linkler[k.link].add(kabuk);
        guvenlikEkle(kabuk);
      }
    }
    return { kok, eklemler, kayarlar, tcp, linkler };
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
      // Link padding kabugu: yalnizca carpisma geometrisi gorsel mesh'le
      // AYNI olan linklere (sunucu URDF'ten soyler); MoveIt padding'i
      // carpisma geometrisine uyguluyor. Pay metre; mesh olcekliyse olcege bolunur.
      const guvenlik = m.safety || {};
      const linkPayi = guvenlik.padding_m ? guvenlik.padding_m.link : 0;
      const kabukGeometrileri = {};
      if (linkPayi > 0) {
        for (const l of m.links) {
          if (geometriler[l.name] && l.collision_mesh_is_visual) {
            kabukGeometrileri[l.name] = sisirilmisGeometri(geometriler[l.name], linkPayi / (l.scale[0] || 1));
          }
        }
      }
      model = m;
      canli = robotKur(m, geometriler, canliMalzeme, kutuCanli, kabukGeometrileri, () => !yolGosteriliyor);
      hayalet = robotKur(m, geometriler, hayaletMalzeme, kutuHayalet, kabukGeometrileri, null);
      hayalet.kok.visible = false;
      sahne.add(canli.kok, hayalet.kok);
      // Zemin carpisma kutusu (ust yuzu z = 0; taban_link ile temasi serbest).
      const z = guvenlik.floor_box;
      if (z) {
        const zemin = new THREE.Mesh(new THREE.BoxGeometry(z.size[0], z.size[1], z.size[2]), zeminMalzeme);
        zemin.position.fromArray(z.xyz);
        zemin.visible = false;
        sahne.add(zemin);
        guvenlikParcalari.push({ nesne: zemin, kosul: null });
      }
      guvenlikGoster();
      eksenleriGoster();
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

  // --- pick & place nesnesi -----------------------------------------------
  const olcekli = (M, boyut) => M.clone().multiply(new THREE.Matrix4().makeScale(...boyut));
  function elleKoy(o, M) { o.matrix.copy(M); o.matrixWorldNeedsUpdate = true; }

  function ppTemizle() {
    if (pp && hayalet && window.sonGripperKonumu != null) {
      parmaklariUygula(hayalet, window.sonGripperKonumu);   // canli gripper'a don
    }
    pp = null;
    for (const o of [ppNesne, ppAlma, ppBirakma, ppPlanKutu, ppPlanKabuk, birakmaNokta]) o.visible = false;
  }

  // Secenegin zaman cizelgesinden nesnenin yolunu kurar. Hayalet kol bu
  // sirada yol noktalarina konur; cagiran sonra kareyiUygula ile geri alir.
  function ppKur(noktalar) {
    ppTemizle();
    const n = onizleme && onizleme.object;
    const asamalar = (secenek && secenek.stages) || [];
    const gripperlar = asamalar.filter((a) => a.tip === "gripper");
    const uc = n && hayalet.linkler[n.tip_link];
    if (!n || !uc || gripperlar.length < 2) return;
    const [g1, g2] = gripperlar;
    if (!noktalar[g1.ilk] || !noktalar[g2.ilk]) return;
    const boyut = n.size_mm.map((v) => Math.max(v, 1) / 1000);

    // Parmaklarin kapanma ekseni (uc link cercevesinde): gripper modelinin
    // surucu parmak ekleminin ekseni. Gercek nesnenin kalinligi (size_mm[0])
    // bu eksende. Model yoksa uc linkin X'i (planlayici kutusuyla ayni).
    acilariUygula(hayalet, noktalar[g1.ilk]);
    hayalet.kok.updateMatrixWorld(true);
    const ucQ = uc.getWorldQuaternion(new THREE.Quaternion());
    let aci = 0;
    const surucu = hayalet.kayarlar.find((k) => !k.mimic);
    if (surucu && surucu.grup.parent) {
      const eksen = surucu.eksen.clone()
        .applyQuaternion(surucu.grup.parent.getWorldQuaternion(new THREE.Quaternion()))
        .applyQuaternion(ucQ.clone().invert());
      if (Math.hypot(eksen.x, eksen.y) > 1e-6) aci = Math.atan2(eksen.y, eksen.x);
    }
    const merkez = new THREE.Vector3().fromArray(n.center_tip_m);
    const yerel = new THREE.Matrix4().compose(merkez,
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), aci), new THREE.Vector3(1, 1, 1));
    // Planlayicinin kutusu uc link eksenlerinde (pickplace._nesne_msg, yonelim birim).
    const planYerel = new THREE.Matrix4().makeTranslation(merkez.x, merkez.y, merkez.z);

    const ucMatrisi = (nokta) => {
      acilariUygula(hayalet, nokta);
      hayalet.kok.updateMatrixWorld(true);
      return uc.matrixWorld.clone();
    };
    const Malma = ucMatrisi(noktalar[g1.ilk]).multiply(yerel);
    const Mbirakma = ucMatrisi(noktalar[g2.ilk]).multiply(yerel);

    const pay = model.safety && model.safety.padding_m ? model.safety.padding_m.attached : 0;
    const genislik = n.size_mm[0];
    const kapaliYuzde = n.stroke_mm > 0
      ? 100 - Math.min(Math.max(genislik / n.stroke_mm, 0), 1) * 100 : 100;
    pp = { g1, g2, uc, yerel, planYerel, boyut, pay, kapaliYuzde,
           Malma, Mbirakma };

    elleKoy(ppAlma, olcekli(Malma, boyut));
    elleKoy(ppBirakma, olcekli(Mbirakma, boyut));
    ppAlma.visible = true;
    ppBirakma.visible = true;
    const b = n.place_point_mm;
    if (b) {
      birakmaNokta.position.set(b[0] / 1000, b[1] / 1000, b[2] / 1000);
      birakmaNokta.visible = true;
    }
  }

  const ara = (a, b, s) => a + (b - a) * Math.min(Math.max(s, 0), 1);
  const oran = (t, a) => (a.t1 > a.t0 ? (t - a.t0) / (a.t1 - a.t0) : 1);

  // Hayalet kol t anindaki acilarda; nesne, parmaklar ve planlayici kutusu.
  function ppKare(t) {
    if (!pp) return;
    const { g1, g2, uc, kapaliYuzde } = pp;
    hayalet.kok.updateMatrixWorld(true);
    // Parmaklar: acik -> (1. gripper) nesne genisligine kapanir -> (2.) acilir.
    let yuzde = 0;
    if (t >= g1.t0 && t < g2.t0) yuzde = ara(0, kapaliYuzde, oran(t, g1));
    else if (t >= g2.t0) yuzde = ara(kapaliYuzde, 0, oran(t, g2));
    parmaklariUygula(hayalet, yuzde);

    // Nesne: kavramadan once alma yerinde, birakmadan sonra birakma yerinde,
    // arada uc linke bagli.
    const tasiniyor = t >= g1.t0 && t < g2.t0;
    const M = tasiniyor ? uc.matrixWorld.clone().multiply(pp.yerel)
                        : (t < g1.t0 ? pp.Malma : pp.Mbirakma);
    elleKoy(ppNesne, olcekli(M, pp.boyut));
    ppNesne.visible = true;

    // Planlayicinin kutusu yalnizca ilisikken var (kapanmadan acilmanin sonuna).
    const ilisik = guvenlikAcik() && t >= g1.t1 && t < g2.t1;
    ppPlanKutu.visible = ilisik;
    ppPlanKabuk.visible = ilisik && pp.pay > 0;
    if (ilisik) {
      const P = uc.matrixWorld.clone().multiply(pp.planYerel);
      elleKoy(ppPlanKutu, olcekli(P, pp.boyut));
      elleKoy(ppPlanKabuk, olcekli(P, pp.boyut.map((v) => v + 2 * pp.pay)));
    }
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
    ppKare(t);
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
    yolGosteriliyor = !!noktalar;
    guvenlikGoster();
    if (!noktalar) {
      ppTemizle();
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
    ppKur(noktalar);
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
    // Pick & place onizlemesinde hayaletin parmaklari zaman cizelgesinden (ppKare).
    if (!pp) parmaklariUygula(hayalet, e.detail);
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

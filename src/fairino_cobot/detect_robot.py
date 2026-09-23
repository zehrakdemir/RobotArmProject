#!/usr/bin/env python3
"""
Hangi Fairino robotuna bağlı olduğumuzu (IP + model + sürüm) TAMAMEN
OTOMATIK öğrenir. Hiçbir IP hardcoded değil.

Kullanım:
  python3 detect_robot.py
"""
import ipaddress
import re
import socket
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from fairino_cobot import paths

HOME = Path.home()
RPC_PORT = 20003
SCAN_TIMEOUT = 0.25
MAX_WORKERS = 100
STATE_FILE = paths.last_target_file()
CACHE_PORT_TIMEOUT = 0.5
CACHE_RPC_TIMEOUT = 2.0
# Tek denemede RPC cevap vermemesi (gecici yavaslik/yuk) gercek robotun
# yanlislikla "cevapsiz" sayilip oncelik sirasinda daha dusuk oncelikli
# adaylara (orn. localhost simulator) dusulmesine yol acmasin diye, her aday
# birden fazla kez denenir once vazgecilir.
PROBE_RETRIES = 3


def log(msg):
    print(msg, file=sys.stderr)


def find_sdk_path():
    """SDK'nin yeri. Arama mantigi paths.sdk_dir()'e tasindi - once
    FAIRINO_SDK_DIR, sonra bilinen konumlar, son care ev dizini taramasi."""
    try:
        return str(paths.sdk_dir())
    except paths.PathError as e:
        log(str(e))
        return None


def get_windows_host_subnets():
    subnets = set()
    try:
        raw = subprocess.run(
            ["ipconfig.exe"], capture_output=True, timeout=15
        ).stdout
        try:
            out = raw.decode("cp857")
        except Exception:
            out = raw.decode("utf-8", errors="replace")
    except Exception as e:
        log(f"UYARI: ipconfig.exe çalıştırılamadı ({e}), bu kaynak atlanıyor.")
        return subnets

    current_ip = None
    for raw_line in out.splitlines():
        line = raw_line.strip()
        m_ip = re.search(r"IPv4 Adres[a-z]*[.\s]*:\s*([\d.]+)", line, re.IGNORECASE)
        if not m_ip:
            m_ip = re.search(r"IPv4 Address[.\s]*:\s*([\d.]+)", line, re.IGNORECASE)
        if m_ip:
            current_ip = m_ip.group(1)
            continue
        m_mask = re.search(r"Maske[a-z]*[.\s]*:\s*([\d.]+)", line, re.IGNORECASE)
        if not m_mask:
            m_mask = re.search(r"Subnet Mask[.\s]*:\s*([\d.]+)", line, re.IGNORECASE)
        if m_mask and current_ip:
            mask = m_mask.group(1)
            if current_ip.startswith("127.") or current_ip.startswith("169.254."):
                current_ip = None
                continue
            try:
                iface = ipaddress.ip_interface(f"{current_ip}/{mask}")
                prefix = max(iface.network.prefixlen, 24)
                net = ipaddress.ip_interface(f"{current_ip}/{prefix}").network
                subnets.add(net)
                log(f"  Windows tarafında bulunan ağ: {current_ip}/{mask} -> taranacak: {net}")
            except Exception:
                pass
            current_ip = None
    return subnets


def get_local_subnets():
    wsl_subnets = set()
    try:
        out = subprocess.run(
            ["ip", "-4", "-o", "addr", "show"],
            capture_output=True, text=True, timeout=10
        ).stdout
        for line in out.splitlines():
            m = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)/(\d+)", line)
            if not m:
                continue
            ip_str, prefix = m.group(1), int(m.group(2))
            if ip_str.startswith("127."):
                continue
            try:
                eff_prefix = max(prefix, 24)
                iface = ipaddress.ip_interface(f"{ip_str}/{eff_prefix}")
                wsl_subnets.add(iface.network)
            except Exception:
                continue
    except Exception as e:
        log(f"UYARI: 'ip' komutu çalışmadı ({e}).")

    windows_subnets = get_windows_host_subnets()
    return windows_subnets, wsl_subnets


def port_open(ip, port=RPC_PORT, timeout=SCAN_TIMEOUT):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            return s.connect_ex((str(ip), port)) == 0
    except Exception:
        return False


def scan_subnet_for_port(network):
    hosts = list(network.hosts())
    found = []
    log(f"  {network} taranıyor ({len(hosts)} host, port {RPC_PORT})...")
    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as ex:
        futures = {ex.submit(port_open, h): h for h in hosts}
        for fut in as_completed(futures):
            h = futures[fut]
            try:
                if fut.result():
                    found.append(str(h))
            except Exception:
                pass
    return found


def discover_candidates():
    priority_by_ip = {}

    def add(ip, prio):
        if ip not in priority_by_ip or prio < priority_by_ip[ip]:
            priority_by_ip[ip] = prio

    windows_subnets, wsl_subnets = get_local_subnets()

    for net in windows_subnets:
        for ip in scan_subnet_for_port(net):
            add(ip, 0)

    for net in wsl_subnets:
        for ip in scan_subnet_for_port(net):
            add(ip, 1)

    if port_open("127.0.0.1"):
        add("127.0.0.1", 2)

    ordered = sorted(priority_by_ip.items(), key=lambda kv: kv[1])
    return [ip for ip, _prio in ordered]


def probe(ip, Robot):
    # NOT: Burada bilerek stdout'u contextlib.redirect_stdout ile bastirmiyoruz.
    # discover edilen adaylar birden fazla thread'de (biri diger bitmeden
    # 5sn join timeout'u dolunca) COKLU calisabiliyor; redirect_stdout global
    # sys.stdout'u degistirdigi icin bu thread'ler birbirinin stdout'unu
    # ezip main()'in en sonundaki ROBOT_IP=... satirlarinin gercek stdout'a
    # hic yazilmamasina (bos stdout, returncode=0) yol aciyordu. SDK'nin
    # birkac zararsiz "ServerProxy(...)" / "RPC connection closed." satiri
    # ekrana sizsa da main()'deki regex parse'i (^[A-Z_][A-Z0-9_]*=...) bunu
    # yok sayar, o yuzden zararsiz.
    try:
        robot = Robot.RPC(ip)
        result = robot.GetSoftwareVersion()
        try:
            robot.CloseRPC()
        except Exception:
            pass
        if result[0] != 0:
            return None
        return {
            "ip": ip,
            "model": result[1],
            "web_version": result[2],
            "controller_version": result[3],
        }
    except Exception:
        return None


def probe_bekleyerek(ip, Robot, saniye=5, denemeler=PROBE_RETRIES):
    """probe()'u thread icinde calistirir; asili kalirsa terk edip devam eder.

    probe icindeki XML-RPC cagrisi, port acik ama karsi taraf sessizse
    sonsuza kadar bekleyebiliyor (main()'deki setdefaulttimeout notuna bak).
    daemon=True: sure dolunca thread terk edilir, script beklemez.
    """
    for _ in range(denemeler):
        kutu = {}

        def _isci(kutu=kutu):
            kutu["info"] = probe(ip, Robot)

        t = threading.Thread(target=_isci, daemon=True)
        t.start()
        t.join(timeout=saniye)
        if kutu.get("info"):
            return kutu["info"]
    return None


def emit_result(chosen):
    robot_model = chosen["model"]
    web_version = chosen["web_version"]
    controller_version = chosen.get("controller_version", "?")
    reachable_ip = chosen["ip"]

    m = re.search(r"FR\s*(\d+)", str(robot_model), re.IGNORECASE)
    if not m:
        log(f"HATA: robot modeli '{robot_model}' içinden numara çıkarılamadı.")
        sys.exit(1)
    model_num = m.group(1)

    log(f"Not: controller_version={controller_version} bilgi amaçlı, sürüm eşleştirmesinde kullanılmıyor.")
    v = re.search(r"(\d+)\.(\d+)\.(\d+)", str(web_version))
    if not v:
        log(f"HATA: web sürümü '{web_version}' parse edilemedi.")
        sys.exit(1)
    maj, minr, min2 = v.group(1), v.group(2), v.group(3)

    package_name = f"fairino{model_num}_v6_moveit2_config"
    version_tag = f"v{maj}{minr}{min2}"

    log("")
    log(f">>> ALGILANAN ROBOT: FR{model_num}, sürüm {maj}.{minr}.{min2} ({reachable_ip})")
    log(f">>> Kullanılacak paket: {package_name}")
    log(f">>> Kullanılacak fairino_hardware kaynağı: {version_tag}")
    log("")

    print(f"ROBOT_MODEL_NUM={model_num}")
    print(f"ROBOT_VERSION_MAJ={maj}")
    print(f"ROBOT_VERSION_MIN={minr}")
    print(f"ROBOT_VERSION_MIN2={min2}")
    print(f"ROBOT_VERSION_TAG={version_tag}")
    print(f"MOVEIT_PACKAGE={package_name}")
    print(f"ROBOT_IP={reachable_ip}")


def read_cached_target():
    """fairino_launch (auto_fairino_launch.sh) her calistiginda ~/.fairino_last_target'a
    en son bulunan ROBOT_IP + MOVEIT_PACKAGE'i yazar. Burada onu okuyoruz - henuz
    dogrulamiyoruz, sadece "denemeye deger bir aday var mi" diye bakiyoruz."""
    if not STATE_FILE.exists():
        return None
    info = {}
    try:
        for line in STATE_FILE.read_text().splitlines():
            m = re.match(r"^([A-Z_][A-Z0-9_]*)=(.*)$", line)
            if m:
                info[m.group(1)] = m.group(2)
    except Exception:
        return None

    ip = info.get("ROBOT_IP")
    pkg = info.get("MOVEIT_PACKAGE")
    if not ip or not pkg:
        return None
    m = re.match(r"^fairino(\d+)_v6_moveit2_config$", pkg)
    if not m:
        return None
    return {"ip": ip, "expected_model_num": m.group(1)}


def try_fast_path(Robot):
    """~/.fairino_last_target'taki IP'yi TEK basina, kisa bir sure ile yoklar.
    Hala ayakta ve beklenen modelle eslesiyorsa tam ag taramasini (5 subnet,
    yuzlerce host) atlayip saniyeler icinde sonuc doner. Herhangi bir sapma
    (dosya yok, IP cevap vermiyor, farkli model) olursa sessizce None doner
    ve cagiran taraf normal tam taramaya devam eder - bu yuzden burasi asla
    yanlis/eski bir robotu "dogrulanmis" gibi sunmaz."""
    cached = read_cached_target()
    if not cached:
        return None
    ip = cached["ip"]

    log(f"Onbellek bulundu (~/.fairino_last_target): {ip} - hizlica dogrulaniyor...")

    if not port_open(ip, timeout=CACHE_PORT_TIMEOUT):
        log(f"  -> {ip}: port {RPC_PORT} kapali/cevap vermiyor, onbellek gecersiz. Tam taramaya donuluyor.")
        return None

    info = None
    for attempt in range(1, PROBE_RETRIES + 1):
        result_box = {}

        def _worker(result_box=result_box):
            result_box["info"] = probe(ip, Robot)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        t.join(timeout=CACHE_RPC_TIMEOUT)
        info = result_box.get("info")
        if info:
            break
        log(f"  -> {ip}: RPC cevap vermedi (deneme {attempt}/{PROBE_RETRIES}, {CACHE_RPC_TIMEOUT}sn icinde)...")

    if not info:
        log(f"  -> {ip}: {PROBE_RETRIES} denemede de RPC cevap vermedi, onbellek gecersiz. Tam taramaya donuluyor.")
        return None

    m = re.search(r"FR\s*(\d+)", str(info["model"]), re.IGNORECASE)
    detected_model_num = m.group(1) if m else None
    if detected_model_num != cached["expected_model_num"]:
        log(f"  -> {ip}: model degismis ({info['model']}), onbellek gecersiz. Tam taramaya donuluyor.")
        return None

    log(f"  -> {ip}: DOĞRULANDI (onbellekten hizli yol), model={info['model']}, "
        f"web={info['web_version']}, controller={info['controller_version']}")
    return info


def main():
    # XML-RPC istemcisinin kendi zaman asimi yok - port acik ama karsi
    # taraf cevap vermezse SONSUZA KADAR bekleyebiliyor. Global bir
    # varsayilan socket zaman asimi koyup bunu engelliyoruz.
    socket.setdefaulttimeout(3.0)

    sdk_path = find_sdk_path()
    if not sdk_path:
        log("HATA: fairino-python-sdk/linux klasörü bulunamadı.")
        sys.exit(1)
    log(f"SDK bulundu: {sdk_path}")
    sys.path.insert(0, sdk_path)

    try:
        from fairino import Robot
    except Exception as e:
        log(f"HATA: fairino SDK import edilemedi: {e}")
        sys.exit(1)

    # Robotun IP'si makineye/aga ozgu bir deger. Kodun icinde sabit
    # DEGIL; .env icindeki FAIRINO_ROBOT_IP ile verilebilir. Verilirse
    # tum ag taramasi atlanir (ve --full bile taramaya zorlamaz -
    # kullanici acikca hangi robotu istedigini soylemis olur).
    # Virgulle birden fazla adres verilebilir; sirayla denenir ve CEVAP VEREN
    # ILK adres kullanilir. Boylece ayni kurulum hem gercek robota hem
    # simulatore baglanabilir, oncelik yazma sirasidir:
    #   FAIRINO_ROBOT_IP=192.168.57.2,192.168.58.2
    # (once gercek robot; o kapaliysa SimMachine)
    forced_ip = paths.robot_ip_override()
    if forced_ip:
        adaylar = [p.strip() for p in forced_ip.split(",") if p.strip()]
        if len(adaylar) == 1:
            log(f"FAIRINO_ROBOT_IP={adaylar[0]} verildi - ag taramasi atlaniyor.")
        else:
            log(f"FAIRINO_ROBOT_IP {len(adaylar)} adres veriyor - ag taramasi "
                f"atlaniyor, oncelik sirasiyla denenecek: {', '.join(adaylar)}")
        for sira, ip in enumerate(adaylar, 1):
            # Tek adres verilmisse kullanici acikca onu istemis: eski davranis
            # aynen korunur (dogrudan probe, sure siniri yok).
            if len(adaylar) == 1:
                info = probe(ip, Robot)
            else:
                # Ulasilamayan adrese RPC denemek hem pahali (3 x 5 sn) hem de
                # ZARARLI: sure dolunca terk edilen daemon thread'ler SDK'yi
                # bozuyor ve SONRAKI aday da cevapsiz gorunuyor (olculdu:
                # 57.2 sonrasi calisir durumdaki 58.2 "cevap yok" verdi).
                # Once ucuz port kontrolu - hepsi 0.3 sn.
                if not port_open(ip):
                    log(f"  -> {ip}: port {RPC_PORT} kapali, atlaniyor")
                    continue
                info = probe_bekleyerek(ip, Robot)
            if info:
                if len(adaylar) > 1:
                    log(f"  -> {ip}: DOGRULANDI (aday {sira}/{len(adaylar)}), "
                        f"model={info['model']}, web={info['web_version']}")
                emit_result(info)
                return
            log(f"  -> {ip}: cevap yok"
                + (", sirakine geciliyor..." if sira < len(adaylar) else ""))
        log("HATA: verilen adreslerin hicbirinde robota ulasilamadi: "
            f"{', '.join(adaylar)}")
        sys.exit(1)

    if "--full" in sys.argv:
        log("--full verildi: onbellek atlanip her zaman tam ag taramasi yapiliyor.")
    else:
        fast_result = try_fast_path(Robot)
        if fast_result:
            emit_result(fast_result)
            return

    log("================================================")
    log(" Ağ taranıyor (port 20003 açık olan cihazlar aranıyor)")
    log("================================================")
    candidates = discover_candidates()

    if not candidates:
        log("HATA: Port 20003 açık hiçbir cihaz bulunamadı.")
        # Container icinde tarama ZATEN calisamaz; bunu ayrica soyle.
        # Robotun agi (or. 192.168.57.0/24) container'da yerel bir ag degil,
        # yonlendirilmis bir ag - "ip route get" ile gorulur. Native tarafta
        # bu bilgi Windows'un arayuzlerinden (ipconfig.exe) geliyor;
        # container'dan o erisim yok, dolayisiyla taranacak aralik
        # bulunamiyor. Tek cozum IP'yi acikca vermek.
        if Path("/.dockerenv").exists():
            log("")
            log("Container içindesin: robotun ağı burada YEREL DEĞİL (yönlendirilmiş),")
            log("bu yüzden tarama ile bulunamaz. Robotun IP'sini vermen gerekiyor:")
            log("")
            log("  .env dosyasına:   FAIRINO_ROBOT_IP=192.168.x.y")
            log("  ya da:            docker run -e FAIRINO_ROBOT_IP=192.168.x.y ...")
            log("")
            log("IP'yi bilmiyorsan native tarafta 'detect-robot' çalıştırıp öğrenebilirsin.")
        else:
            log("Robot açık mı / ağa bağlı mı? SimMachine ayakta mı?")
        sys.exit(1)

    log(f"Aday IP'ler (port 20003 açık): {candidates}")
    log("Her biri RPC ile doğrulanıyor (her biri en fazla 5sn bekletilir)...")

    found = []
    for ip in candidates:
        info = None
        for attempt in range(1, PROBE_RETRIES + 1):
            result_box = {}

            def _probe_worker(ip=ip, result_box=result_box):
                result_box["info"] = probe(ip, Robot)

            # daemon=True: 5sn icinde bitmezse thread'i terk ediyoruz, script
            # onu beklemeden devam ediyor.
            t = threading.Thread(target=_probe_worker, daemon=True)
            t.start()
            t.join(timeout=5)
            info = result_box.get("info")
            if info:
                break
            log(f"  -> {ip}: cevap yok (deneme {attempt}/{PROBE_RETRIES})...")

        if info:
            log(f"  -> {ip}: DOĞRULANDI, model={info['model']}, web={info['web_version']}, controller={info['controller_version']}")
            found.append(info)
        else:
            log(f"  -> {ip}: port açıktı ama Fairino RPC cevap vermedi ({PROBE_RETRIES} denemede de), atlanıyor")

    if not found:
        log("HATA: Port 20003 açık cihazlar vardı ama hiçbiri Fairino RPC'sine cevap vermedi.")
        sys.exit(1)

    chosen = found[0]

    if len(found) > 1:
        others = ", ".join(f["ip"] for f in found[1:])
        log(f"UYARI: Birden fazla cihaz bulundu, seçilen: {chosen['ip']} (diğerleri: {others})")

    emit_result(chosen)


if __name__ == "__main__":
    main()

# ROS2 / MoveIt2 vendor yamaları

`frcobot_ros2` (FAIR-INNOVATION'ın resmi ROS2 sürücü paketi, bkz. [`docs/SETUP.md`](../docs/SETUP.md))
üzerine uyguladığımız override'lar. Vendor deposu bizim değil — bu yüzden değişiklikler
orada tutulmuyor, buradan **yeniden uygulanabilir** biçimde saklanıyor.

## Uygulama

```bash
python3 tools/apply_ros2_patches.py --dry-run
```

```bash
python3 tools/apply_ros2_patches.py && cd ~/ros2_ws && colcon build
```

Farklı bir konumdaysa: `--frcobot /başka/yol/frcobot_ros2`.
Araç **idempotent** — tekrar çalıştırılabilir, uygulanmış olanı atlar.

### Neden bir script, neden elle kopyalama değil

`frcobot_ros2` desteklediği **her** Fairino modeli için ayrı bir
`fairino<N>_v6_moveit2_config` paketi içeriyor (FR3, FR5, FR10, FR16, FR20, FR30, FR3-MT).
`detect_robot.py` hangi robot takılıysa onu otomatik seçtiği için, düzeltmelerin **hepsine**
uygulanmış olması gerekiyor — yoksa başka bir robot takıldığında düzeltmesiz varsayılan
config'e geri düşülür.

Ayrıca eklenen ayarların bir kısmı **modele özgü isimler** içeriyor
(`fairino3_v6_group:`, `fairino3_controller:`), yani tek bir dosyayı her pakete kopyalamak
artık yetmiyor. Araç bu isimleri her paketin **kendi** SRDF ve yaml dosyasından okuyor.

## Yamalar

### `patches/fairino_hardware_interface.patch`

Sürücünün ros2_control donanım arayüzü. Bakir kaynağa karşı üretildi (`fairino_hardware`
v3.9.7 ve v3.9.9 bu dosyada birebir aynı, ikisi de geçerli referans). Dört değişiklik:

| Değişiklik | Neden |
|---|---|
| `ServoMoveStart(1)` — UDP kanalı (port 20007), XML-RPC'ye geri düşüşle | Asıl kök sebep: XML-RPC üzerinden servo akışı düzensizdi. |
| `cmdT` sabit değil, **ölçülen** kontrol periyodundan (0.001–0.016 s'ye kırpılmış) | SDK'nın dokümante ettiği aralık 1–16 ms; sabit değer hareket sırasında yanlış. |
| Eklem hızı sınırlayıcı (170 °/s ≈ limitin %94'ü) | Yüksek hızda robot "axis N command speed exceeds the limit" faultu veriyordu. |
| Boşta 0.5 sn sonra `ServoMoveEnd`, yeni hedefte `ServoMoveStart` | Robot servo modundayken **gripper komutları bloke oluyor** (aktüasyon robotun hareket kuyruğundan geçiyor). Kol dururken moddan çıkınca gripper çalışıyor. Elle oynatılmışsa (fark > 2°) mod otomatik açılmıyor — sıçramayı önlemek için `clearfault` isteniyor. |

Ayrıntılı gerekçeler yamanın kendi içindeki yorumlarda ve
[`docs/journal/2026-09-09.md`](../docs/journal/2026-09-09.md)'de.

### `patches/ompl_planning.yaml`

İki ayrı düzeltme:

**1. Jerk sınırlaması.** Varsayılan config `request_adapters` listesinde sadece
`AddTimeOptimalParameterization` içeriyordu — jerk (ani ivme değişimi) hiç sınırlanmıyordu.
Küçük (< 10°) eklem hareketlerinde bile robotun kendi tork tabanlı çarpışma algılaması
yanlış-pozitif "Shaft collision fault" veriyordu. `AddRuckigTrajectorySmoothing`, TOTG'nin
**yerine değil onunla birlikte** ve listede **en başta** olmalı (adaptör zinciri dışarıdan
içeriye çalışır; Ruckig zaten-zamanlanmış bir trajectory üzerine jerk sınırı ekler — ham
girdi verilince "invalid input" hatası veriyordu).

**2. Planlama-zamanı çarpışma kontrolü sıklığı** (`longest_valid_segment_fraction: 0.005`).
Bu paketlerde hiç grup bölümü yoktu; `model_based_planning_context.cpp:278`'e göre config
boşsa `useConfig()` hemen dönüyor ve OMPL'in kendi varsayılanı (%1) geçerli oluyor. 6 eksenli
bir kolda bu çok kaba — iki kontrol noktası arasından ince bir yakın-geçiş sızabiliyor.
%0.5'e çekildi. Grup adı modele özgü olduğu için bu satırı araç SRDF'ten okuyup ekliyor.

> `planner_configs` listesi **bilerek eklenmedi**: `ompl_interface.cpp:199-215`'e göre bir
> grup için `planner_configs` ya da `default_planner_config` verilmezse MoveIt varsayılan
> yapılandırmayı kendisi üretip `type`'ı `geometric::RRTConnect` yapıyor — yani fiilen
> çalışan planlayıcı bu. Liste eklemek o seçimi değiştirme riski taşırdı.

### `ros2_controllers.yaml` — hedef toleransları

Dosyada `constraints` bölümü hiç yoktu. Tanımlanmayınca JointTrajectoryController **hiçbir**
tolerans kontrolü yapmıyor: yörünge bitince goal'ü koşulsuz `SUCCEEDED` işaretliyor — robot
hedefe hiç ulaşmamış olsa bile.

Araç her modelin kendi controller adını ve eklem listesini okuyup ekliyor:
`goal: 0.05` rad (~2.9°), `goal_time: 2.0` s.

- `goal`, `movecobot`'un SDK üzerinden yaptığı 2°'lik kontrolden **kasıtlı olarak biraz
  gevşek** — gerçek bir sapmada önce o konuşsun (daha açıklayıcı, ayrıca robotun fault
  kodunu basıyor); bu tolerans onun kaçırdığı durumlar için ikinci ağ.
- `trajectory: 0.0` (hareket **sırasındaki** takip hatası kontrolü) kapalı: ServoJ komutu ile
  gerçek pozisyon arasındaki gecikme normal çalışmada da var, eşik koymak sağlıklı
  hareketleri ortasında kestirirdi.
- `stopped_velocity_tolerance: 0.0` kapalı: bu kurulumda velocity *state* arayüzü yok
  (`state_interfaces: [position]`), JTC hızı pozisyon farkından tahmin etmek zorunda kalır.

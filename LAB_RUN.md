# LAB RUN — Ubuntu Lab PC rehberi (Docker YOK)

Bu rehber lab bilgisayarinda (Ubuntu, ROS 2 Jazzy native kurulu) 3 act'i
calistirmak icindir. **Adim adim, sirayla takip edin. Atlama yapmayin.**

> **Windows/Docker icin:** `RUN.md` dosyasina bakin.
> Bu dosya SADECE lab PC icindir.

---

## KURALLAR (OKUMADAN BASLAMAYIN)

1. **Her yeni terminal actiginda** asagidaki source komutlarini calistir — yoksa `ros2: command not found` alirstn
2. `src/` klasorune sadece `my_tb3_world` klasoru gidecek — baska bir sey koymayIn
3. **Tum repo'yu `src/` icine klonlamayin** — sadece `my_tb3_world` klasorunu kopyalayin
4. `setup.py` icinde `package_name = 'my_tb3_world'` oldugundan emin olun (SONDA S OLMAYACAK)

---

## A) TEMIZLIK — her sey sifirdan (5 dakika)

Daha once bir seyler denediyseniz, ONCE temizleyin. Ilk kez yapiyorsaniz da
calIstirIn, zarar vermez:

```bash
# 1. Eski build artIklarini sil
rm -rf /home/team06/turtlebot3_ws/build
rm -rf /home/team06/turtlebot3_ws/install
rm -rf /home/team06/turtlebot3_ws/log

# 2. src icindeki YANLIS dosyalari sil (repo artiklari)
rm -rf "/home/team06/turtlebot3_ws/src/Simulation Files (1)"
rm -rf /home/team06/turtlebot3_ws/src/Cbl
rm -rf /home/team06/turtlebot3_ws/src/docs
rm -f  /home/team06/turtlebot3_ws/src/README.md
rm -f  /home/team06/turtlebot3_ws/src/RUN.md
rm -f  /home/team06/turtlebot3_ws/src/LAB_RUN.md

# 3. Eski my_tb3_world'u de sil (taze kopyalayacagiz)
rm -rf /home/team06/turtlebot3_ws/src/my_tb3_world

# 4. Eski clone'u sil
rm -rf /home/team06/Cbl

# 5. Bozuk environment variable'lari temizle
unset COLCON_PREFIX_PATH
```

---

## B) KLONLA VE KOPYALA (2 dakika)

```bash
# 1. Repo'yu HOME klasorune klonla (src ICINE DEGIL!)
cd /home/team06
git clone https://github.com/TwentySubsTBP/Cbl.git
```

> Eger password sorarsa: GitHub sifren DEGIL, Personal Access Token lazim.
> github.com > Settings > Developer settings > Personal access tokens > Generate

```bash
# 2. SADECE my_tb3_world klasorunu workspace'e kopyala
cp -r /home/team06/Cbl/my_tb3_world /home/team06/turtlebot3_ws/src/
```

**KONTROL — src icinde sadece su olmali:**
```bash
ls /home/team06/turtlebot3_ws/src/
```
Beklenen cikti:
```
my_tb3_world
```
Baska bir sey varsa (docs, README.md, Simulation Files, Cbl, vs.) YANLIS.
Yukaridaki temizlik adimlarini tekrar calistir.

---

## C) SETUP.PY KONTROL (30 saniye)

Bu adimi ATLAMAYIN. Gecen sefer burada hata vardi.

```bash
head -5 /home/team06/turtlebot3_ws/src/my_tb3_world/setup.py
```

Beklenen cikti:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_tb3_world'
```

> **EGER `my_tb3_worlds` (sonunda S) YAZIYORSA**, duzelt:
> ```bash
> sed -i "s/package_name = 'my_tb3_worlds'/package_name = 'my_tb3_world'/" /home/team06/turtlebot3_ws/src/my_tb3_world/setup.py
> ```

---

## D) BUILD (2 dakika)

```bash
# 1. ROS'u source'la
source /opt/ros/jazzy/setup.bash

# 2. Workspace'e git
cd /home/team06/turtlebot3_ws

# 3. Build et
colcon build
```

**Beklenen cikti:**
```
Starting >>> my_tb3_world
Finished <<< my_tb3_world [X.XXs]
Summary: 1 package finished [X.XXs]
```

> **EGER HATA ALIRSAN:**
> - `Duplicate package names` → Temizlik adimina don, `src/` icinde fazla klasor var
> - `package directory does not exist` → setup.py kontrol adimina don
> - `colcon: command not found` → `source /opt/ros/jazzy/setup.bash` unuttun

```bash
# 4. Build sonrasini source'la
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# 5. Paketin gorunuyor mu kontrol et
ros2 pkg list | grep my_tb3
```

Beklenen cikti:
```
my_tb3_world
```

---

## E) CALISTIRMA — 3 Terminal gerekli

### ONEMLI: Her terminalde su 3 satiri ONCE calistir

```bash
source /opt/ros/jazzy/setup.bash
cd /home/team06/turtlebot3_ws && source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Bu satIrlar olmadan `ros2: command not found` alIrsIn.

---

### T1 — Gazebo dunyasi

```bash
ros2 launch my_tb3_world new_world.launch.py
```
~40 saniye bekle, Gazebo penceresi acilacak.

---

### T2 — Twin node'lari (yeni terminal ac)

Once source komutlarini calistir (yukarida), sonra:

```bash
# odom calisiyor mu kontrol et (sayi gelmeli):
ros2 topic echo --once --qos-reliability best_effort /odom
```

Sayi geldiyse:

```bash
ros2 launch my_tb3_world waverider.launch.py start_world:=false hazard_x:=0.8 hazard_y:=0.0
```

---

### T3 — Kontrol terminali (yeni terminal ac)

Once source komutlarini calistir (yukarida), sonra act'leri calistir.

---

## ACT 1 — State sync (pH leak) [Gazebo'da KIRMIZI bolge]

```bash
# leak'i tetikle:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: true}"

# kanit:
ros2 topic echo --once /alerts
ros2 topic echo --once --field data /mode

# sifirla:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"
```

Beklenen: alerts'te `RAISED`, mode'da `ALERT`, Gazebo'da kirmizi bolge.

---

## ACT 2 — Hazard re-route [Gazebo'da MAVI bolge, robot etrafinda doner]

**ONCE Act 1'i sifirladigindan emin ol** (yoksa robot hareket etmez):

```bash
# leak kapatildi mi kontrol et:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"

# hazard'i olustur:
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: true}"

# robotu hazard'in otesine gonder (QoS flag'leri ZORUNLU):
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 1.8, y: 0.0}, orientation: {w: 1.0}}}"

# sifirla:
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: false}"
```

---

## ACT 3 — Comms safety halt (twin olurse robot durur)

```bash
# robotu surdur:
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"

# twin canli mi kontrol et:
ros2 topic echo --once --field data /twin_alive
# -> True olmali

# twin'i oldur:
pkill -9 -f lib/my_tb3_world/dt_supervisor

# 5 saniye bekle, robot DURUR, sonra:
ros2 topic echo --once --field data /twin_alive
# -> False olmali
```

**ONEMLI — Act 3'ten sonra twin'i GERI GETIR:**
```bash
ros2 run my_tb3_world dt_supervisor --ros-args -p cmd_vel_topic:=cmd_vel_override
```
> Bunu ayri bir terminalde calistir (T3'te calistirirsan T3 mesgul olur).
> Twin geri gelmeden robot HAREKET ETMEZ.

---

## OTONOM DEVRIYE (istege bagli)

Robot'un haritayi otomatik gezmesini istiyorsan:

```bash
# ONCE twin canli olmali (twin_alive = True)
# Yeni terminalde (source komutlarini unutma):
ros2 run my_tb3_world sector_nav
```

Robot 2x2 grid'de otomatik gezer. Baska bir terminalden anomali tetikleyebilirsin.

---

## HATA REHBERI

| Hata | Neden | Cozum |
|------|-------|-------|
| `ros2: command not found` | ROS source edilmemis | `source /opt/ros/jazzy/setup.bash` |
| `Package 'my_tb3_world' not found` | Install source edilmemis veya yanlis klasordeysin | `cd /home/team06/turtlebot3_ws && source install/setup.bash` |
| `Duplicate package names` | `src/` icinde iki tane `my_tb3_world` var | Eski kopyayi sil: `rm -rf "src/Simulation Files (1)"` |
| `package directory 'my_tb3_worlds' does not exist` | setup.py'de typo | `sed -i "s/my_tb3_worlds/my_tb3_world/" src/my_tb3_world/setup.py` |
| `git clone` password hatasi | GitHub sifre kabul etmiyor | Personal Access Token kullan |
| Robot hareket etmiyor (cmd_vel = 0) | twin_alive False veya mode ALERT | 1) `ros2 topic echo --once --field data /twin_alive` kontrol et. False ise supervisor'u yeniden baslat. 2) `ros2 topic echo --once --field data /mode` kontrol et. ALERT ise pH'i sifirla |
| `/odom` bos / Gazebo donmus | Gazebo crash | T1'i kapat, `ros2 launch my_tb3_world new_world.launch.py` tekrar calistir |
| `COLCON_PREFIX_PATH` uyarisi | Eski bozuk environment variable | `unset COLCON_PREFIX_PATH` veya yeni terminal ac |

---

## KLASOR YAPISI (BOYLE OLMALI)

```
/home/team06/
├── Cbl/                          <-- repo BURAYA klonlandi (src icine DEGIL)
│   ├── my_tb3_world/
│   ├── docs/
│   ├── RUN.md
│   ├── LAB_RUN.md
│   └── README.md
│
└── turtlebot3_ws/                <-- workspace
    ├── src/
    │   └── my_tb3_world/         <-- SADECE bu klasor burada olmali
    │       ├── launch/
    │       ├── my_tb3_world/     <-- Python node dosyalari (.py)
    │       ├── worlds/
    │       ├── resource/
    │       ├── test/
    │       ├── package.xml
    │       ├── setup.py          <-- package_name = 'my_tb3_world' (S YOK!)
    │       └── setup.cfg
    ├── build/                    <-- colcon build olusturur
    ├── install/                  <-- colcon build olusturur (setup.bash burada)
    └── log/                      <-- colcon build olusturur
```

> **YANLIS olan yapilar:**
> - `src/Cbl/my_tb3_world/` — tum repo src icine klonlanmis
> - `src/Simulation Files (1)/` — eski kopya
> - `src/README.md` — repo dosyasi src'de olmamali

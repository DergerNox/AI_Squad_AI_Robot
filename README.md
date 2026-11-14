# Használati útmutató: a kód használata a Raspberry Pi-n

Ez az útmutató azt írja le, hogyan kerül a repó kódja a Pi-re, hogyan kell
lefordítani, elindítani és kipróbálni. A Pi-n Ubuntu 22.04 és ROS 2 Humble van, ez így
marad. A mi kódunk a saját workspace-ébe kerül (`~/squad_ws`), a csapat `~/picar_ws`
mappájához nem nyúl. Az MI-ágens nem a Pi-n fut, hanem egy laptopon (7. pont).

## Néhány szó, ami előjön

- **Node**: egy futó program, amely ROS 2-n keresztül kommunikál.
- **Topik**: névvel ellátott csatorna, amelyen egy program folyamatosan küld adatot, és bárki olvashatja (például `/cmd_vel`: menetparancsok).
- **Szolgáltatás (service)**: kérés és válasz (például `/picar/move`: „menj 1 másodpercig előre” → „kész”).
- **Csomag (package)**: egy mappa kóddal; a `colcon build` parancs fordítja le.
- **Workspace**: egy mappa, amelynek az `src` mappájában vannak a csomagok. A Pi-n a mi
  kódunké a `~/squad_ws`, a csapat sajátja a `~/picar_ws`; a PC-n és a laptopon a mienk
  a `~/picar_ws`.

## A repó mappái

- `shared/`: a közös kód, amelyet mindkét autó használ: az MI-ágens (`picar_agent`),
  a `picar_base` (motorok, kormány, kameratartó, biztonsági megállás), a `Move` és `Look`
  szolgáltatás leírása (`picar_interfaces`), a parancsfogadó node (`picar_listener`),
  a Gazebo-beli digitális iker (`picar_gazebo`) és a `sim` parancs (`squad_gazebo`).
- `picar/`: csak az Adeept PiCar-B-é: az autó modellje (`picar_description`) és a
  Pi-n futó node-ok (`picar_car`).
- `pccar/`: a második autó (táblagép + ESP32). Egyelőre csak a leírása van meg.
- A repó gyökerében két telepítő: `setup_car.sh` (egy autó gépe, 1. pont) és
  `setup_laptop.sh` (a laptop, amelyen az MI-ágens fut, 7. pont).

A csomagok neve nem változott, ezért a `colcon build`, `ros2 run` és `ros2 launch`
parancsok ugyanazok. Csak a mappák helye más: az ágens például a
`<workspace>/src/picar/shared/picar_agent` mappában van.

Ha a kódot még a régi mappákkal fordítottad le, a `git pull` után egyszer töröld a
régi fordítást, és fordíts újra (1. pont). A Pi-n:

```bash
cd ~/squad_ws
rm -rf build install log
```

## 1. A kód letöltése és fordítása (egyszer)

A legegyszerűbb: a `setup_car.sh` mindent telepít és beállít, ami a Pi-nek kell (a
hiányzó csomagokat, az SPI kikapcsolását, a csoportokat és a `~/.bashrc`-t). Ami már
megvan, azt kihagyja, így bármikor újra lehet futtatni. A Pi-n már fent levő ROS 2
Humble-höz és a rendszerhez nem nyúl (nem frissít), és az ágenst sem teszi fel (az a
laptopon fut). A mi kódunk külön workspace-be kerül, a `~/squad_ws`-be:

```bash
mkdir -p ~/squad_ws/src && cd ~/squad_ws/src
git clone -b Ferenc_Csiszér https://github.com/DergerNox/AI_Squad_AI_Robot.git picar
cd picar
./setup_car.sh picar
```

(A `-b Ferenc_Csiszér` addig kell, amíg ez a kód nincs a `main` ágban.) A végén kiírja,
mi van még hátra (például újraindítás). Ha csak meg akarod nézni, mit csinálna:
`./setup_car.sh picar --dry-run`. Ha olyan workspace-ben indítod, amelyben más is van
(például a csapat `~/picar_ws`-ében), nem fordít, hanem megmondja, hogyan kell külön.

Kézzel ugyanez:

```bash
mkdir -p ~/squad_ws/src
cd ~/squad_ws/src
git clone -b Ferenc_Csiszér https://github.com/DergerNox/AI_Squad_AI_Robot.git picar
sudo apt install python3-gpiozero python3-lgpio python3-rpi.gpio python3-smbus python3-pytest \
    ros-humble-camera-ros ros-humble-compressed-image-transport
cd ~/squad_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select picar_interfaces picar_base picar_listener picar_car
```

Ezután, hogy minden új terminál ismerje a mi csomagjainkat is (a ROS-t és a csapat
workspace-ét a `~/.bashrc` már betölti):

```bash
echo 'source ~/squad_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 2. A parancsfogadó node kipróbálása

A `command_listener` fogadja a menetparancsokat a `/cmd_vel` topikon, kiírja
őket, és a `/odom` topikra küldi, hogy a parancsok alapján hol kellene lennie
az autónak. Minden parancsot külön terminálban futtass (például több SSH-ablakban).

1. terminál, a node indítása:
   ```bash
   ros2 run picar_listener command_listener
   ```
2. terminál, parancsok küldése (Ctrl+C állítja le):
   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
   ```
   Az 1. terminálban ilyen sorok jelennek meg: `Command: speed +0.20 m/s, turn +29 deg/s`.
   A `linear.x` a sebesség m/s-ban (pozitív = előre), az `angular.z` a fordulás
   rad/s-ban (pozitív = balra).
3. terminál, a becsült helyzet figyelése:
   ```bash
   ros2 topic echo /odom --field pose.pose.position
   ```

Billentyűzetes vezetéshez (a Pi-n már fent van; máshol: `sudo apt install
ros-humble-teleop-twist-keyboard`): `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

Az autó adatait (tengelytáv, legnagyobb kormányszög, időkorlát) a node ROS-paraméterként
kapja, az alapértékek a digitális iker értékei. Ha a valódi autón mást mértek, indításkor
megadhatók, például:

```bash
ros2 run picar_listener command_listener --ros-args -p wheelbase_m:=0.15
```

## 3. Az autó node-jainak indítása

Az autón külön node a kamera (`camera`), a távolságérzékelő (`ultrasonic`) és a
`picar_base` (motorok, kormány, kameratartó, biztonsági megállás). Egy paranccsal indulnak:

```bash
ros2 launch picar_car car.launch.py                              # mindhárom
ros2 launch picar_car car.launch.py camera:=false base:=false    # csak a távolságérzékelő
```

Amíg a `camera_ros` nincs telepítve, add hozzá a `camera:=false`-t, különben az indítás
a „package 'camera_ros' not found” hibával leáll.

**Vigyázat, a `picar_base` rögtön mozgatja az autót:** induláskor középre állítja a
kormányt és a kamerát, utána pedig azt csinálja, amit a `/cmd_vel` vagy a `/picar/move`
kér. Először tedd az autót egy dobozra, hogy a kerekei ne érjenek le.

**A csapat `picar_control` node-jai ne fussanak vele egyszerre** (`motor_steering_node`,
`ultrasonic_node`, `picar_control.launch.py`): ugyanazokat a lábakat és ugyanazt a
szervóvezérlőt használják, a két program összeveszne az autón.

Ellenőrzés egy másik terminálban:

```bash
ros2 topic echo /ultrasonic/range --field range   # méter; .inf: 4 m-en belül nincs semmi, .nan: nem jött visszhang
ros2 topic hz /camera/image_raw                    # kb. 10 kép másodpercenként
```

Ha a távolságérzékelő azt írja, hogy a lábai foglaltak (busy): az Ubuntu bekapcsolja az
SPI-t, ami ugyanezeket a lábakat (GPIO 8 és 11) használja. A `/boot/firmware/config.txt`
fájlban írd át a `dtparam=spi=on` sort `dtparam=spi=off`-ra, mentsd, és indítsd újra a Pi-t:

```bash
sudo nano /boot/firmware/config.txt
sudo reboot
```

Ha a kamera nem indul: a ROS-csomag kamerakönyvtára (libcamera) nem ismer minden
Raspberry Pi kamerát. Ilyenkor a camera_ros leírása szerint a Raspberry Pi-s libcamera-t
kell lefordítani: https://github.com/christianrauch/camera_ros

A beállítások (a kamera képmérete, az érzékelő lábai, a sebesség- és biztonsági
értékek, a motorok és szervók adatai) a
`~/squad_ws/src/picar/picar/picar_car/config/car.yaml` fájlban vannak.

### A távolságérzékelő node próbája érzékelő nélkül

Ez a próba szimulált GPIO-lábakkal futtatja a node-ot, így bekötés nélkül is megmutatja,
hogy a node jól számol: 10 cm és 3,9 m közötti távolságokat, a 4 m-nél messzebbit és a
hiányzó visszhangot. Ugyanígy próbálja a `picar_base` hardveres részét is, szimulált
motorokkal és szervóvezérlővel. A valódi lábakhoz és az autóhoz nem nyúl, ezért
bekötött autón is futtatható. Körülbelül 10 másodpercig tart:

```bash
cd ~/squad_ws/src/picar/picar/picar_car
python3 -m pytest test
```

Ha minden rendben van, a végén csak `passed` áll (`failed` nem). Ha `skipped` is, a
`python3-gpiozero` nincs telepítve; ha a `pytest` hiányzik: `sudo apt install python3-pytest`.

## 4. Mi fut éppen?

```bash
ros2 node list       # a futó programok
ros2 topic list      # a topikok
ros2 service list    # a szolgáltatások
```

## 5. Frissítés a GitHubról

```bash
cd ~/squad_ws/src/picar
git pull
cd ~/squad_ws
colcon build --symlink-install --packages-select picar_interfaces picar_base picar_listener picar_car
```

Az új mappaszerkezet utáni első frissítésnél a fordítás előtt egyszer töröld a régi
fordítást (lásd „A repó mappái”).

## 6. A saját módosításaitok feltöltése

Első alkalommal:

```bash
git config --global user.name "Neved"
git config --global user.email "a-github-email-cimed@pelda.hu"
cd ~/squad_ws/src/picar
git switch -c sajat-ag-neve
```

Utána minden alkalommal:

```bash
cd ~/squad_ws/src/picar
git add .
git commit -m "Röviden, mit változtattam"
git push -u origin sajat-ag-neve
```

A GitHubon a „Compare & pull request” gombbal kérhetitek, hogy a módosítás
bekerüljön a `main` ágba. Jelszó helyett GitHub tokent kér
(GitHub → Settings → Developer settings → Personal access tokens).

## 7. Az MI-ágens a laptopon

Az MI-ágens (és ha helyben fut, a modell is) Ferenc laptopján fut, nem a Pi-n: a Pi 3B+
memóriája kevés hozzá. A Pi csak az autó node-jait futtatja (3. pont). A laptop és a Pi
ugyanazon a Wi-Fin van (egy hotspoton, amelynek a nevét a Pi ismeri), és ROS 2-n
beszélnek egymással. A laptopon ugyanaz a ROS fut, mint a Pi-n (Humble), hogy biztosan
megértsék egymást.

Telepítés a laptopon (Fedora), egyszer:

```bash
sudo dnf install -y git
mkdir -p ~/picar_ws/src && cd ~/picar_ws/src
git clone -b Ferenc_Csiszér https://github.com/DergerNox/AI_Squad_AI_Robot.git picar
cd picar
./setup_laptop.sh
```

A `setup_laptop.sh`:
- telepíti az Ollamát és a modellt (`qwen3-vl:4b-instruct`);
- mivel a ROS 2 Humble csak Ubuntu 22.04-en fut, készít egy Ubuntu 22.04-es „dobozt”
  (distrobox, neve `picar`), és abban telepíti a ROS-t és az ágenst;
- a tűzfalon átengedi a ROS 2 forgalmát.

Ha a gép eleve Ubuntu 22.04, elég ennyi: `./setup_car.sh laptop`.

A modellt az ágens `.env` fájlja választja
(`~/picar_ws/src/picar/shared/picar_agent/.env`):
- alapból a laptop saját Ollamája;
- felhő-modellhez: `./setup_laptop.sh --no-model`, és a `.env`-ben `PICAR_LLM`,
  `PICAR_MODEL` és a kulcs;
- ha a modell a PC-n fut (8. pont): `PICAR_BASE_URL=http://<a PC IP-címe>:11434`.

Indítás, miután a Pi-n elindultak az autó node-jai (3. pont):

```bash
picar-agent
```

Látja-e a laptop az autót:

```bash
distrobox enter picar
ros2 topic list     # legyen benne a /camera/image_raw és az /ultrasonic/range
```

Még nincs kész:
- a `picar_base` hardveres része megvan, de a valódi autón még nem próbáltuk;
- a kamera képei most tömörítetlenül mennek (kb. 74 Mbit/s), ami a Pi Wi-Fijének túl
  sok. Ehhez tömörített képek kellenek.

## 8. Ollama a PC-n (ha a modell a PC-n fut)

Akkor kell, ha a modell nem a laptopon, hanem a PC-n fut. A PC és a laptop legyen
ugyanazon a hálózaton.

1. Telepítsd az Ollamát Windowsra: https://ollama.com/download/windows
2. Töltsd le a modellt (PowerShellben): `ollama pull qwen3-vl:4b-instruct`
3. Engedd, hogy a hálózatról is elérhető legyen: lépj ki az Ollamából a tálcán,
   keresd meg a Start menüben a „környezeti változók” beállítást, add hozzá a
   fiókodhoz: `OLLAMA_HOST` = `0.0.0.0:11434`, majd indítsd újra az Ollamát a Start menüből.
4. Engedélyezd a 11434-es portot a tűzfalon (PowerShell rendszergazdaként):
   ```powershell
   New-NetFirewallRule -DisplayName "Ollama" -Direction Inbound -Protocol TCP -LocalPort 11434 -Action Allow
   ```
5. A PC IP-címe: `ipconfig`, az aktív hálózati kártya „IPv4 Address” sora.
6. Próba a laptopról:
   ```bash
   curl http://<a PC IP-címe>:11434/api/tags
   ```
   Ha a válaszban szerepel a `qwen3-vl:4b-instruct`, a laptop eléri a modellt.

## Ami még nem használható a Pi-n

- `picar_base`: a valódi autó hardveres része (motorok, kormány, kameratartó) megvan, de az autón még nem próbáltuk: az első próbánál legyen az autó dobozon (3. pont)
- `picar_gazebo`, `picar_description`: a digitális iker, csak a PC-n fut
- `squad_gazebo` (a `sim` parancs): a digitális ikert és az MI-t indítja, csak a PC-n fut
- `picar_agent`: az MI nem a Pi-n fut, hanem a laptopon (7. pont), a digitális ikerrel a PC-n

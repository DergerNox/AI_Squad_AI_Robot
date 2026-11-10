# Használati útmutató: a kód használata a Raspberry Pi-n

Ez az útmutató azt írja le, hogyan kerül a repó kódja a Pi-re, hogyan kell
lefordítani, elindítani és kipróbálni. Feltétel: a Pi-n Ubuntu 24.04 és
ROS 2 Jazzy (`ros-jazzy-ros-base` és `ros-dev-tools`) van telepítve.

## Néhány szó, ami előjön

- **Node**: egy futó program, amely ROS 2-n keresztül kommunikál.
- **Topik**: névvel ellátott csatorna, amelyen egy program folyamatosan küld adatot, és bárki olvashatja (például `/cmd_vel`: menetparancsok).
- **Szolgáltatás (service)**: kérés és válasz (például `/picar/move`: „menj 1 másodpercig előre” → „kész”).
- **Csomag (package)**: egy mappa kóddal; a `colcon build` parancs fordítja le.
- **Workspace**: a `~/picar_ws` mappa; a csomagok az `src` mappában vannak.

## 1. A kód letöltése és fordítása (egyszer)

```bash
mkdir -p ~/picar_ws/src
cd ~/picar_ws/src
git clone https://github.com/DergerNox/AI_Squad_AI_Robot.git picar
sudo apt install python3-gpiozero python3-lgpio ros-jazzy-camera-ros
cd ~/picar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select picar_interfaces picar_base picar_listener picar_car
```

Ezután, hogy minden új terminál ismerje a ROS-t és a mi csomagjainkat:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/picar_ws/install/setup.bash' >> ~/.bashrc
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

Billentyűzetes vezetéshez telepítsd: `sudo apt install ros-jazzy-teleop-twist-keyboard`,
majd: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## 3. Az autó node-jainak indítása

Az autón külön node a kamera (`camera`), a távolságérzékelő (`ultrasonic`) és a
`picar_base` (motorok, kormány, kameratartó, biztonsági megállás). Egy paranccsal indulnak:

```bash
ros2 launch picar_car car.launch.py                              # mindhárom
ros2 launch picar_car car.launch.py camera:=false base:=false    # csak a távolságérzékelő
```

A `picar_base` hardveres része még nincs megírva, addig `base:=false`-szal indítsd.
Amíg a `camera_ros` nincs telepítve, add hozzá a `camera:=false`-t is, különben az
indítás a „package 'camera_ros' not found” hibával leáll.

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
értékek) a `picar_car/config/car.yaml` fájlban vannak.

## 4. Mi fut éppen?

```bash
ros2 node list       # a futó programok
ros2 topic list      # a topikok
ros2 service list    # a szolgáltatások
```

## 5. Frissítés a GitHubról

```bash
cd ~/picar_ws/src/picar
git pull
cd ~/picar_ws
colcon build --symlink-install --packages-select picar_interfaces picar_base picar_listener picar_car
```

## 6. A saját módosításaitok feltöltése

Első alkalommal:

```bash
git config --global user.name "Neved"
git config --global user.email "a-github-email-cimed@pelda.hu"
cd ~/picar_ws/src/picar
git switch -c sajat-ag-neve
```

Utána minden alkalommal:

```bash
cd ~/picar_ws/src/picar
git add .
git commit -m "Röviden, mit változtattam"
git push -u origin sajat-ag-neve
```

A GitHubon a „Compare & pull request” gombbal kérhetitek, hogy a módosítás
bekerüljön a `main` ágba. Jelszó helyett GitHub tokent kér
(GitHub → Settings → Developer settings → Personal access tokens).

## 7. Az MI-ágens futtatása a Pi-n

Egyszeri telepítés (sok csomagot tölt le, eltarthat egy ideig):

```bash
sudo apt install ros-jazzy-rai-interfaces
python3 -m venv ~/rai_venv --system-site-packages
source ~/rai_venv/bin/activate
pip install rai-core
```

Beállítás: hozd létre a `.env` fájlt a mintából, és írd bele, hol fut a modell:

```bash
cd ~/picar_ws/src/picar/picar_agent
cp .env.example .env
nano .env
```

- Ha a modell a PC-n fut (Ollama): `PICAR_BASE_URL=http://<a PC IP-címe>:11434`
- Ha a felhőben fut: `PICAR_LLM`, `PICAR_MODEL` és a kulcs

Indítás. A Pi-n ne a `run.sh`-t használd, mert az a Gazebót is elindítja:

```bash
source ~/rai_venv/bin/activate
cd ~/picar_ws/src/picar/picar_agent
python3 agent.py
```

Az ágens csak akkor tud vezetni, ha az autó node-jai is futnak (3. pont). A `picar_base`
hardveres része még nincs kész.

## 8. Ollama a PC-n (ha a modell a PC-n fut)

A PC és a Pi legyen ugyanazon a hálózaton.

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
6. Próba a Pi-ről:
   ```bash
   curl http://<a PC IP-címe>:11434/api/tags
   ```
   Ha a válaszban szerepel a `qwen3-vl:4b-instruct`, a Pi eléri a modellt.

## Ami még nem használható a Pi-n

- `picar_base`: most csak a Gazebo-beli digitális ikert vezérli; a valódi autó hardveres része (motorok, kormány, kameratartó) még nincs megírva
- `picar_gazebo`, `picar_description`: a digitális iker, csak a PC-n fut
- `picar_agent`: az MI, egyelőre a PC-n fut

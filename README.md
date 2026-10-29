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
cd ~/picar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select picar_interfaces picar_base picar_listener
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

## 3. Mi fut éppen?

```bash
ros2 node list       # a futó programok
ros2 topic list      # a topikok
ros2 service list    # a szolgáltatások
```

## 4. Frissítés a GitHubról

```bash
cd ~/picar_ws/src/picar
git pull
cd ~/picar_ws
colcon build --symlink-install --packages-select picar_interfaces picar_base picar_listener
```

## 5. A saját módosításaitok feltöltése

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

## Ami még nem használható a Pi-n

- `picar_base`: most csak a Gazebo-beli digitális ikert vezérli; a valódi autó hardveres része még nincs megírva
- `picar_gazebo`, `picar_description`: a digitális iker, csak a PC-n fut
- `picar_agent`: az MI, egyelőre a PC-n fut

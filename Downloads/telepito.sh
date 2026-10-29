mkdir -p ~/Downloads

cat << 'EOF' > ~/Downloads/telepito.sh
#!/bin/bash
set -e

echo "=== PiCar-B ROS 2 + Gemini projekt csomagok telepitese ==="

# 1. Rendszer es ROS 2 Humble alapszintu csomagok
echo "[1/4] APT frissites es ROS 2 csomagok telepitese..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server

# 2. Python mesterseges intelligencia es adatfeldolgozo konyvtarak
echo "[2/4] Python AI es eszkozcsomagok telepitese..."
pip install --break-system-packages \
    google-genai \
    opencv-python \
    numpy \
    python-dotenv

# 3. PiCar fizikai hardver konyvtarak (Raspberry Pi eseten szukseges, VM-en lefut hiba nelkul)
echo "[3/4] Robot HAT hardveres vezerlok (fizikai robot eseten)..."
pip install --break-system-packages robot-hat || true

echo "=== Telepites sikeresen befejezodott! ==="
EOF

chmod +x ~/Downloads/telepito.sh

cat << 'EOF' > ~/Downloads/README_TELEPITES.txt
PiCar-B ROS 2 + Gemini Rendszer Telepitesi Lista
=================================================

1. Rendszercsomagok (APT):
   - python3-pip
   - python3-colcon-common-extensions (colcon build-hez)
   - ros-humble-teleop-twist-keyboard (kezi billentyuzet-vezerleshez)
   - ros-humble-cv-bridge (kamera Image uzenetek OpenCV konvertalasahoz)
   - ros-humble-image-transport (kepfolyam kezeleshez)
   - ros-humble-gazebo-ros-pkgs es gazebo-plugins (szimulaciohoz es szenzorokhoz)
   - ros-humble-slam-toolbox (2D terkepkesziteshez)
   - ros-humble-nav2-map-server (terkep kepkent valo kimentesehez)

2. Python konyvtarak (PIP):
   - google-genai (hivatalos Gemini kliens a donteshozo agent.py-hoz)
   - opencv-python (kepfeldolgozashoz)
   - numpy (szamitasokhoz)
   - python-dotenv (kornyezeti valtozokhoz)
   - robot-hat (PiCar fizikai ultrahang- es motorvezerleshez)

Futtatas egyben:
   bash ~/Downloads/telepito.sh
EOF

echo "A telepito fajlok elkeszultek a ~/Downloads mappaban:"
ls -l ~/Downloads/telepito.sh ~/Downloads/README_TELEPITES.txt
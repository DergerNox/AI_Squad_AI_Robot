cat << 'EOF' > ~/Downloads/README_TELEPITES.txt
====================================================================
PiCar-B ROS 2 + Gemini Projekt - Teljes Telepitesi es Inditasi Utmutato
====================================================================

1. TELEPITES (Csak egyszer kell lefuttatni)
-------------------------------------------
Nyiss egy terminalt, es futtasd:
   bash ~/Downloads/telepito.sh


2. SZUKSEGES FAJLOK ES HELYUK
-----------------------------
A kovetkezo ket fajlnak kell leteznie a munkateruletben:
- picar_bridge.py  -> Mozgasvezerles, odometria, ultrahang kezelo
- agent.py         -> Gemini AI donteshozo modul


3. RENDSZER INDITASA (3 kulon terminal ablakban!)
-------------------------------------------------

>>> 1. TERMINAL: Bridge (Kinematika es szenzorok)
   source /opt/ros/humble/setup.bash
   source ~/picar_gazebo_ws/install/setup.bash   # vagy ahol a buildelt csomagod van
   ros2 run minirobot_bringup picar_bridge

>>> 2. TERMINAL: Gemini Agent (AI Agy)
   source /opt/ros/humble/setup.bash
   export GEMINI_API_KEY="IDE_MASOLD_BE_A_SAJAT_API_KULCSODAT"
   python3 agent.py

>>> 3. TERMINAL: Utasitasok kuldese a robotnak
   source /opt/ros/humble/setup.bash

   # Pelda: Eloremenet
   ros2 topic pub --once /from_human std_msgs/msg/String "{data: 'Menj elore egy kicsit'}"

   # Pelda: Kanyarodas
   ros2 topic pub --once /from_human std_msgs/msg/String "{data: 'Fordulj balra'}"
   ros2 topic pub --once /from_human std_msgs/msg/String "{data: 'Kanyarodj jobbra'}"

   # Pelda: Megallas
   ros2 topic pub --once /from_human std_msgs/msg/String "{data: 'Allj meg most'}"


4. HASZNOS MONITORING PARANCSOK (Opcionalis)
--------------------------------------------
- Robot szoveges valasza:       ros2 topic echo /to_human
- Ultrahangos tavolsagmeres:   ros2 topic echo /ultrasonic_distance
- Odometria ellenorzese:       ros2 topic echo /odom
====================================================================
EOF
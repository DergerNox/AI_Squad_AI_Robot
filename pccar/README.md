# PC-s autó (tervezett)

Egy második autó a PiCar-B mellé (nem helyette). Még semmi sincs megépítve: ez a mappa a
tervet tartalmazza. Ugyanazt az MI-t, ugyanazokat a ROS-neveket és biztonsági szabályokat
használja, mint a PiCar-B (`shared/`), csak más hardverrel.

## A hardver (adott, nem változtatható)

| Alkatrész | Feladata |
|---|---|
| Panasonic Toughbook CF-20 mk2 (i5/m5-7Y57, 8 GB RAM, Intel grafika) | Lubuntu 24.04, ROS 2 Jazzy, az MI |
| ESP32-S3, Arduino IDE-vel programozva | a motorok, a szervók és a 3 ultrahangos távolságérzékelő |
| 16 csatornás szervóvezérlő | „16/4”-ként van feljegyezve; nem tudjuk, mit jelent |
| Samsung Galaxy A12 USB-n | a kamera |

## Hogyan áll össze (elvben megbeszélve)

| Node | Feladata | A PiCar-B-n |
|---|---|---|
| `camera` (`v4l2_camera`) | DroidCam USB-n → `/dev/videoN` → `/camera/image_raw`, 640×480 | `camera_ros` |
| `esp32_bridge` | egyedül ő használja a soros portot: közzéteszi a távolságokat, és továbbítja a `picar_base` motor- és szervóparancsait az ESP32-nek | a digitális ikerben a `sim_bridge` csinálja ezt |
| `picar_base`, `esp32` backend | biztonsági szűrő (SafetyGate), motorok, kormány | `hw` backend |
| MI, `command_listener` | ugyanaz | ugyanaz |

- **Lubuntu 24.04, nem 26.04.** A ROS 2 Jazzy csak a 24.04-en támogatott teljesen
  (Tier 1); a 26.04-hez a Lyrical tartozik, minden más pedig Jazzyn és Gazebo Harmonicon fut.
- **Kamera.** Az A12 utolsó frissítése az Android 12, az Android saját USB-s webkamera
  módjához viszont Android 14 QPR1 kell. Ezért: DroidCam USB-n (adb + v4l2loopback).
- **ESP32-kapcsolat.** Egyszerű szöveges üzenetek USB-s soros vonalon, micro-ROS nélkül:
  a Jazzyhoz előre lefordított micro_ros_arduino nehezen használható az Arduino IDE-ben.
- **Biztonság az ESP32-n.** A firmware magától leállítja a motorokat, ha 0,5 másodpercig
  nem jön parancs.
- **Érzékelők.** A `/ultrasonic/range` a középső érzékelő (ugyanaz a topik, mint a
  PiCar-B-n), mellette `/ultrasonic/left` és `/ultrasonic/right`. A SafetyGate azokat az
  érzékelőket használja, amelyek az adott autón vannak.
- **Az autó adatai.** A sebesség, a méretek, a kormányzás és a kamera látószöge a
  `picar_tools.py`-ból autónkénti beállításfájlba kerül, a `picar_base` pedig modulnév
  alapján tölti be a backendjét (az `esp32` backend a `pccar/pccar_car` csomagban lesz).

## Az építéshez

- A HC-SR04 echo lába 5 V-ot ad, az ESP32 viszont csak 3,3 V-ot bír: minden echo
  vezetékre tegyetek feszültségosztót.
- A 3 érzékelő mérését kb. 60 ms-os eltolással indítsátok, hogy egyik se hallja a
  másik visszhangját.
- A motorok és a szervók saját akkumulátorról menjenek, de a földjük (GND) legyen közös
  az ESP32-ével.

## Tervezett csomagok

| Csomag | Mi lesz benne |
|---|---|
| `pccar_description` | a Gazebo-modell (ehhez kellenek a lenti méretek) |
| `pccar_gazebo` | a bridge-beállításai, hogy a `sim` be tudja tölteni (a `cars.yaml`-ban már van rá váró `pccar` bejegyzés) |
| `pccar_car` | az `esp32_bridge` és a `picar_base` `esp32` backendje |
| `esp32_firmware` | az Arduino-program és a soros üzenetek leírása |

## Nyitott kérdések

- Kormányzás: szervóval kormányzott első kerekek (Ackermann), vagy tankszerű (a két
  oldal kerekei külön hajtva)?
- Mit hajt a szervóvezérlő, és mit jelent a „16/4”?
- Merre néz a 3 érzékelő?
- Méretek: kerékátmérő, nyomtáv, karosszéria, a kamera magassága.

import os
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from google import genai
from google.genai import types

VALID_ACTIONS = {"ELORE", "HATRA", "BALRA", "JOBBRA", "STOP"}
DEFAULT_DURATIONS = {
    "ELORE": 2.0,
    "HATRA": 1.0,
    "BALRA": 1.0,
    "JOBBRA": 1.0,
    "STOP": 0.0,
}
MAX_DURATION_SEC = 5.0


class PiCarAIAgent(Node):
    def __init__(self):
        super().__init__("picar_ai_agent")

        self.cmd_pub = self.create_publisher(String, "/robot_command", 10)
        self.reply_pub = self.create_publisher(String, "/to_human", 10)

        self.create_subscription(String, "/from_human", self.on_human_input, 10)
        self.create_subscription(Float32, "/ultrasonic_distance", self.on_distance_update, 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_camera_frame, 10)

        self.current_distance = 1.5
        self.last_frame = None
        self._seq_lock = threading.Lock()
        self._cancel_event = threading.Event()

        api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
        if not api_key:
            self.get_logger().error("HIÁNYZIK A GEMINI_API_KEY KÖRNYEZETI VÁLTOZÓ!")
            return

        self.client = genai.Client(api_key=api_key)
        self.model_id = os.getenv("GEMINI_MODEL", "gemini-3.5-flash-lite").strip()

        self.system_prompt = (
            "Te egy intelligens PiCar-B robotasszisztens vagy. "
            "A válaszod elején adj meg egy vagy több parancssort az alábbi formátumban:\n"
            "CMD:PARANCS[:MASODPERC]\n"
            "Engedélyezett PARANCS értékek: ELORE, HATRA, BALRA, JOBBRA, STOP.\n"
            "A MASODPERC opcionális (0.2 és 5.0 között). "
            "Ha a felhasználó azt kéri, hogy fordulj meg (180 fok), adj ki egymás után több "
            "CMD:JOBBRA:1.1 sort (pl. 2 lépést), hogy tényleg megforduljon a robot, ne csak egy kicsit kanyarodjon. "
            "A CMD sorok után maximum 1 rövid magyar mondatban válaszolj."
        )

        self.get_logger().info(f"[agent] PiCar AI Agent aktív | modell: {self.model_id}")

    def on_distance_update(self, msg: Float32):
        self.current_distance = msg.data

    def on_camera_frame(self, msg: Image):
        self.last_frame = msg

    def _parse_cmd_line(self, line: str):
        raw = line.strip()
        if not raw.upper().startswith("CMD:"):
            return None

        payload = raw[4:].strip()
        parts = [p.strip().upper() for p in payload.split(":")]
        action = parts[0] if parts else ""
        if action not in VALID_ACTIONS:
            return None

        duration = DEFAULT_DURATIONS.get(action, 1.0)
        if len(parts) >= 2 and parts[1]:
            try:
                parsed = float(parts[1])
                duration = max(0.2, min(MAX_DURATION_SEC, parsed))
            except ValueError:
                pass

        return action, duration

    def _publish_cmd(self, action: str):
        cmd_msg = String()
        cmd_msg.data = action
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"[agent] Kiadva (/robot_command): {action}")

    def _execute_sequence(self, steps: list[tuple[str, float]], cancel_event: threading.Event):
        with self._seq_lock:
            for idx, (action, duration) in enumerate(steps):
                if cancel_event.is_set():
                    self.get_logger().info("[agent] Előző parancssor megszakítva új utasítás miatt.")
                    return

                self._publish_cmd(action)

                if idx < len(steps) - 1 and duration > 0.0:
                    if cancel_event.wait(timeout=duration):
                        return
                    stop_msg = String()
                    stop_msg.data = "STOP"
                    self.cmd_pub.publish(stop_msg)
                    if cancel_event.wait(timeout=0.15):
                        return

    def on_human_input(self, msg: String):
        prompt = msg.data.strip()
        if not prompt:
            return

        self._cancel_event.set()
        new_cancel = threading.Event()
        self._cancel_event = new_cancel

        self.get_logger().info(
            f"[agent] Utasítás: '{prompt}' | Távolság: {self.current_distance:.2f} m | Modell: {self.model_id}"
        )

        try:
            full_context = f"{prompt}\n[Ultrahangos elülső távolság: {self.current_distance:.2f} m]"
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=full_context,
                config=types.GenerateContentConfig(
                    system_instruction=self.system_prompt,
                    temperature=0.1,
                    max_output_tokens=120,
                ),
            )

            lines = (response.text or "").strip().splitlines()
            steps: list[tuple[str, float]] = []
            reply_lines: list[str] = []

            for line in lines:
                parsed = self._parse_cmd_line(line)
                if parsed is not None:
                    steps.append(parsed)
                elif not line.strip().upper().startswith("CMD:"):
                    reply_lines.append(line.strip())

            if steps:
                threading.Thread(
                    target=self._execute_sequence,
                    args=(steps, new_cancel),
                    daemon=True,
                ).start()

            reply_text = " ".join(part for part in reply_lines if part).strip()
            if not reply_text and steps:
                summary = ", ".join(f"{act} ({dur:.1f}s)" for act, dur in steps)
                reply_text = f"Végrehajtva: {summary}"

            if reply_text:
                reply_msg = String()
                reply_msg.data = reply_text
                self.reply_pub.publish(reply_msg)

        except Exception as e:
            self.get_logger().error(f"[agent] Gemini API kommunikációs hiba: {e}")


def main(args=None):
    rclpy.init(args=args)
    agent = PiCarAIAgent()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent._cancel_event.set()
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

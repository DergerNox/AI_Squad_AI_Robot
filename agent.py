import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from google import genai
from google.genai import types


class PiCarAIAgent(Node):
    def __init__(self):
        super().__init__('picar_ai_agent')

        self.cmd_pub = self.create_publisher(String, '/robot_command', 10)
        self.reply_pub = self.create_publisher(String, '/to_human', 10)

        self.create_subscription(String, '/from_human', self.on_human_input, 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.on_distance_update, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_camera_frame, 10)

        self.current_distance = 1.5
        self.last_frame = None

        api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
        if not api_key:
            self.get_logger().error("HIÁNYZIK A GEMINI_API_KEY KÖRNYEZETI VÁLTOZÓ!")
            return

        self.client = genai.Client(api_key=api_key)
        self.model_id = 'gemini-3.5-flash'

        self.system_prompt = (
            "Te egy intelligens PiCar-B robotasszisztens vagy. "
            "A valaszod elso soraban KIZAROLAG egyetlen parancs allhat:\n"
            "CMD:ELORE\n"
            "CMD:HATRA\n"
            "CMD:BALRA\n"
            "CMD:JOBBRA\n"
            "CMD:STOP\n"
            "A masodik sortol kezdve maximum 1 rovid magyar mondatban valaszolj."
        )

        self.get_logger().info("[agent] PiCar AI Agent aktiv es optimalizalva!")

    def on_distance_update(self, msg: Float32):
        self.current_distance = msg.data

    def on_camera_frame(self, msg: Image):
        self.last_frame = msg

    def on_human_input(self, msg: String):
        prompt = msg.data.strip()
        if not prompt:
            return

        self.get_logger().info(f"[agent] Utasitas: '{prompt}' | Tavolsag: {self.current_distance:.2f} m")

        forward_words = ["elore", "előre", "menj", "indulj", "lassan"]
        if self.current_distance < 0.25 and any(w in prompt.lower() for w in forward_words) and "hatra" not in prompt.lower():
            warn = f"Akadaly van elottem ({self.current_distance * 100:.0f} cm), nem mehetek elore! Fordulj el!"
            self.get_logger().warn(f"[agent] {warn}")
            rep = String()
            rep.data = warn
            self.reply_pub.publish(rep)
            return

        try:
            full_context = f"{prompt}\n[Ultrahangos elulso tavolsag: {self.current_distance:.2f} m]"
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=full_context,
                config=types.GenerateContentConfig(
                    system_instruction=self.system_prompt,
                    temperature=0.1,
                    max_output_tokens=60 
                )
            )

            lines = response.text.strip().splitlines()
            cmd_sent = None
            reply_lines = []

            for line in lines:
                if line.startswith("CMD:"):
                    action = line.replace("CMD:", "").strip().upper()
                    if action in {'ELORE', 'HATRA', 'BALRA', 'JOBBRA', 'STOP'}:
                        cmd_msg = String()
                        cmd_msg.data = action
                        self.cmd_pub.publish(cmd_msg)
                        self.get_logger().info(f"[agent] Kiadva a kerekeknek: {action}")
                        cmd_sent = action
                else:
                    reply_lines.append(line)

            reply_text = " ".join(reply_lines).strip()
            if not reply_text and cmd_sent:
                reply_text = f"Parancs vegrehajtva: {cmd_sent}"

            reply_msg = String()
            reply_msg.data = reply_text
            self.reply_pub.publish(reply_msg)

        except Exception as e:
            self.get_logger().error(f"[agent] Gemini API kommunikacios hiba: {e}")


def main(args=None):
    rclpy.init(args=args)
    agent = PiCarAIAgent()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
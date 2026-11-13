import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

try:
    from robot_hat import Ultrasonic, Pin, PWM, Servo
    HARDWARE_AVAILABLE = True
except (ImportError, RuntimeError):
    HARDWARE_AVAILABLE = False


class PiCarBridge(Node):
    def __init__(self):
        super().__init__('picar_bridge')
        self.hardware = HARDWARE_AVAILABLE
        self.get_logger().info(f"[picar_bridge] Inicializalva. Hardver jelenlet: {self.hardware}")

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/robot_command', self.command_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.feedback_pub = self.create_publisher(String, '/robot_feedback', 10)
        self.distance_pub = self.create_publisher(Float32, '/ultrasonic_distance', 10)

        self.current_cmd = "STOP"
        self.speed = 0.0
        self.steering_angle = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheelbase = 0.14
        self.sim_distance = 1.5

        # Akadalykikerulo allapotgep valtozoi
        self.avoid_state = "IDLE"   # IDLE, BRAKE, REVERSE, TURN
        self.avoid_timer = 0.0

        # Hardver komponensek
        self.sonar = None
        self.servo = None
        self.motor_dir_left = None
        self.motor_pwm_left = None
        self.motor_dir_right = None
        self.motor_pwm_right = None

        if self.hardware:
            try:
                self.sonar = Ultrasonic(Pin("D2"), Pin("D3"))

                self.servo = Servo("P0")
                self.servo.angle(0)

                self.motor_dir_left = Pin("D4")
                self.motor_pwm_left = PWM("P12")
                self.motor_dir_right = Pin("D5")
                self.motor_pwm_right = PWM("P13")

                for pwm in [self.motor_pwm_left, self.motor_pwm_right]:
                    pwm.period(4095)
                    pwm.prescaler(10)
                    pwm.pulse_width_percent(0)

                self.get_logger().info("[picar_bridge] Fizikai motorok, szervo es ultrahang beallitva!")
            except Exception as e:
                self.get_logger().error(f"[picar_bridge] Hiba a hardver csatlakozasakor: {e}")
                self.hardware = False

        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.update_odometry)
        self.create_timer(0.1, self.read_ultrasonic)

    def apply_actuators(self):
        """Kiküldi a vezerlest a fizikai szervora es motorokra."""
        if not self.hardware:
            return

        try:
            deg = math.degrees(self.steering_angle)
            safe_deg = max(-30.0, min(30.0, deg))
            self.servo.angle(safe_deg)

            if abs(self.speed) < 0.01:
                self.motor_pwm_left.pulse_width_percent(0)
                self.motor_pwm_right.pulse_width_percent(0)
            elif self.speed > 0:
                self.motor_dir_left.high()
                self.motor_dir_right.low()
                self.motor_pwm_left.pulse_width_percent(65)
                self.motor_pwm_right.pulse_width_percent(65)
            else:
                self.motor_dir_left.low()
                self.motor_dir_right.high()
                self.motor_pwm_left.pulse_width_percent(65)
                self.motor_pwm_right.pulse_width_percent(65)

        except Exception as e:
            self.get_logger().error(f"Hiba a fizikai motorvezerlesben: {e}")

    def command_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.current_cmd = cmd

        # Uj manualis parancs megszakitja az esetleges kikerulo manovert
        self.avoid_state = "IDLE"

        if cmd == "ELORE":
            self.speed = 0.2
            self.steering_angle = 0.0
        elif cmd == "HATRA":
            self.speed = -0.2
            self.steering_angle = 0.0
        elif cmd == "BALRA":
            self.speed = 0.15
            self.steering_angle = 0.45
        elif cmd == "JOBBRA":
            self.speed = 0.15
            self.steering_angle = -0.45
        elif cmd == "STOP":
            self.speed = 0.0
            self.steering_angle = 0.0

        self.apply_actuators()

        ack = String()
        ack.data = f"ACK: {cmd}"
        self.feedback_pub.publish(ack)

    def cmd_vel_callback(self, msg: Twist):
        self.avoid_state = "IDLE"
        self.speed = msg.linear.x
        self.steering_angle = msg.angular.z
        self.apply_actuators()

    def process_avoidance(self, dist: float):
        """Nem-blokkolo autonom akadalykikerulo allapotgep (10 Hz-en fut)."""
        now = time.monotonic()

        if self.avoid_state == "IDLE":
            # Ha elore haladunk es 18 cm ala er az akadaly, indul a kikerules
            if dist < 0.18 and self.speed > 0:
                self.get_logger().warn(f"[picar_bridge] AKADALY ({dist*100:.0f} cm)! Kikerulo manover inditasa...")
                self.avoid_state = "BRAKE"
                self.avoid_timer = now
                self.speed = 0.0
                self.steering_angle = 0.0
                self.apply_actuators()

        elif self.avoid_state == "BRAKE":
            # 0.3 mp megallas, hogy ne rantsa le a fogaskerekeket az iranyvaltas
            if now - self.avoid_timer >= 0.3:
                self.avoid_state = "REVERSE"
                self.avoid_timer = now
                self.speed = -0.18
                self.steering_angle = 0.0
                self.apply_actuators()
                self.get_logger().info("[picar_bridge] Kikerules 1/3: Tolatas...")

        elif self.avoid_state == "REVERSE":
            # 0.7 mp tolatas egyenes kormannyal
            if now - self.avoid_timer >= 0.7:
                self.avoid_state = "TURN"
                self.avoid_timer = now
                self.speed = 0.16
                self.steering_angle = 0.45  # Balra kanyarodas (~26 fok)
                self.apply_actuators()
                self.get_logger().info("[picar_bridge] Kikerules 2/3: Elfordulas balra...")

        elif self.avoid_state == "TURN":
            # Ha kanyarodas kozben is nekimegy valaminek (< 14 cm), azonnal ujrakezdi
            if dist < 0.14:
                self.avoid_state = "BRAKE"
                self.avoid_timer = now
                self.speed = 0.0
                self.apply_actuators()
                return

            # 0.9 mp kanyarodas utan kiegyenesit es halad tovabb az uj iranyba
            if now - self.avoid_timer >= 0.9:
                self.avoid_state = "IDLE"
                self.speed = 0.2
                self.steering_angle = 0.0
                if not self.hardware:
                    self.sim_distance = 1.5  # Szimulacioban uj szabad ut szimulalasa
                self.apply_actuators()
                self.get_logger().info("[picar_bridge] Kikerules 3/3: Ut szabad, haladas tovabb!")

    def read_ultrasonic(self):
        dist = 0.0
        if self.hardware and self.sonar:
            try:
                val = float(self.sonar.read())
                if val <= 0 or val > 400:
                    dist = 2.0
                else:
                    dist = val / 100.0
            except Exception:
                dist = 0.2
        else:
            if self.speed > 0:
                self.sim_distance = max(0.05, self.sim_distance - (self.speed * 0.1))
            elif self.speed < 0:
                self.sim_distance = min(3.0, self.sim_distance + 0.04)
            dist = self.sim_distance

        # Autonom kikerules futtatasa
        self.process_avoidance(dist)

        dist_msg = Float32()
        dist_msg.data = float(dist)
        self.distance_pub.publish(dist_msg)

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        angular_velocity = (self.speed / self.wheelbase) * math.tan(self.steering_angle)
        self.theta += angular_velocity * dt
        self.x += self.speed * math.cos(self.theta) * dt
        self.y += self.speed * math.sin(self.theta) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = float(self.speed)
        odom.twist.twist.angular.z = float(angular_velocity)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PiCarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.hardware:
            try:
                node.motor_pwm_left.pulse_width_percent(0)
                node.motor_pwm_right.pulse_width_percent(0)
                node.servo.angle(0)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

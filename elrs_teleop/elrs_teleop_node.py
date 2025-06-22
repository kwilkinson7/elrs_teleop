import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped
from .telemetry_sender import CRSFTelemetrySender
import serial
import math

class ELRSTeleop(Node):
    def __init__(self):
        super().__init__('elrs_teleop_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(UInt8, '/motion_mode', 10)
        self.ser = serial.Serial('/dev/ttyTHS1', 420000, timeout=0.01)
        self.timer = self.create_timer(0.02, self.loop)
        self.last_mode = None
        self.telemetry = CRSFTelemetrySender(self.ser)
        self.create_subscription(Float32, '/battery_voltage', self.voltage_cb, 10)
        # self.create_subscription(Vector3Stamped, '/velocity', self.velocity_cb, 10)
        # self.create_subscription(MagneticField, '/imu/mag', self.mag_cb, 10)

    def voltage_cb(self, msg: Float32):
        self.telemetry.send_voltage(msg.data)

    def velocity_cb(self, msg: Vector3Stamped):
        speed = (msg.vector.x**2 + msg.vector.y**2)**0.5
        self.telemetry.send_cpu_percent(speed * 100)  # Placeholder use of CPU % field

    def mag_cb(self, msg: MagneticField):
        mx, my = msg.magnetic_field.x, msg.magnetic_field.y
        heading = math.atan2(my, mx) * 180 / math.pi
        heading = heading if heading >= 0 else heading + 360
        self.telemetry.send_heading(heading)

    def parse_crsf_packet(self, data):
        i = 0
        while i < len(data) - 1:
            if data[i] == 0xC8:  # CRSF address (receiver → FC)
                length = data[i + 1]
                type_byte = data[i + 2]
                if type_byte == 0x16:  # RC Channels
                    payload = data[i + 3:i + 3 + 22]
                    channels = []
                    raw = int.from_bytes(payload, 'little')
                    for j in range(16):
                        ch = (raw >> (11 * j)) & 0x7FF
                        channels.append(ch)
                    return channels
            i += 1
        return None

    def loop(self):
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            channels = self.parse_crsf_packet(data)
            if channels:
                linear_raw_x = channels[2]  
                linear_raw_y = channels[3]
                angular_raw = channels[0]  
                robot_mode = channels[5]

                # Map CRSF value range (172–1811) to -1.0 to 1.0
                def scale(val): return (val - 992) / 820.0  # center = 0

                twist = Twist()
                twist.linear.x = scale(linear_raw_x)
                twist.linear.y = -scale(linear_raw_y)
                twist.angular.z = -scale(angular_raw)
                self.pub.publish(twist)
                
                if robot_mode > 1500:
                    mode_index = 2  # goto
                elif robot_mode > 800:
                    mode_index = 1  # explore
                else:
                    mode_index = 0  # manual

                if mode_index != self.last_mode:
                    self.mode_pub.publish(UInt8(data=mode_index))
                    self.get_logger().info(f"Switched mode to index: {mode_index}")
                    self.last_mode = mode_index


def main(args=None):
    rclpy.init(args=args)
    node = ELRSTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

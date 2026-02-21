#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState
import serial
import threading
import time
import math


class SaditSerialController(Node):
    def __init__(self):
        super().__init__('sadit_serial_controller')
        
        # Parametreler
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # Basit dönüşüm parametreleri
        self.declare_parameter('max_pwm', 140)
        self.declare_parameter('min_pwm', 5)
        self.declare_parameter('max_cmd_linear', 2.0)
        self.declare_parameter('max_cmd_angular', 1.5)
        
        # SADIT robot parametreleri (2WD)
        self.declare_parameter('wheel_separation', 0.5)
        self.declare_parameter('wheel_radius', 0.048)
        
        # Parametreleri al
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value
        self.min_pwm = self.get_parameter('min_pwm').get_parameter_value().integer_value
        self.max_cmd_linear = self.get_parameter('max_cmd_linear').get_parameter_value().double_value
        self.max_cmd_angular = self.get_parameter('max_cmd_angular').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        
        # Seri bağlantı
        self.serial_connection = None
        self.connect_to_arduino()
        
        # ROS2 aboneleri
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # ROS2 yayıncıları
        self.status_pub = self.create_publisher(String, 'arduino_status', 10)
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.current_speed_pub = self.create_publisher(Float32, 'current_speed', 10)
        self.battery_voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        
        # Kontrol değişkenleri
        self.current_speeds = {'sol_arka': 0.0, 'sag_arka': 0.0}
        self.current_positions = {'sol_arka': 0.0, 'sag_arka': 0.0}
        self.last_command_time = time.time()
        self.battery_voltage = 0.0
        
        # Seri okuma iş parçacığını başlat
        self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.serial_thread.start()
        
        self.get_logger().info(f'SADIT Serial Controller started on {self.serial_port}')

    def connect_to_arduino(self):
        """Arduino ile seri bağlantı kurar"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.serial_connection = None

    def cmd_vel_callback(self, msg):
        """Twist mesajını tekerlek hızlarına çevirir ve Arduino'ya gönderir"""
        if self.serial_connection is None:
            self.get_logger().warn('No serial connection available')
            return
        
        linear_input = max(-self.max_cmd_linear, min(self.max_cmd_linear, msg.linear.x))
        angular_input = max(-self.max_cmd_angular, min(self.max_cmd_angular, msg.angular.z))
        
        wheel_speeds = self.calculate_wheel_speeds(linear_input, angular_input)
        
        pwm_values = self.convert_to_pwm(wheel_speeds)
        
        self.send_command(pwm_values)
        self.last_command_time = time.time()
        
        self.get_logger().debug(f'Input: linear={linear_input:.2f}, angular={angular_input:.2f} -> '
                               f'Wheel speeds: {wheel_speeds} -> PWM: {pwm_values}')

    def calculate_wheel_speeds(self, linear_vel, angular_vel):
        """Özel dönüş mantığına göre tekerlek hızlarını hesaplar"""
        
        # Hızları dönüştürmek için bir oran kullan
        linear_scale = self.max_pwm / self.max_cmd_linear
        angular_scale = self.max_pwm / self.max_cmd_angular
        
        v_sol = 0.0
        v_sag = 0.0
        
        if linear_vel != 0 and abs(angular_vel) < 0.1:
            # Düz gitme
            v_sol = linear_vel * linear_scale
            v_sag = linear_vel * linear_scale
        elif angular_vel < 0:
            # Sola dönüş için sağ tekerlek ileri
            v_sol = 0
            v_sag = abs(angular_vel) * angular_scale
            self.get_logger().info("Sola dönülüyor: Sağ tekerlek ileri")
        elif angular_vel > 0:
            # Sağa dönüş için sol tekerlek ileri
            v_sol = abs(angular_vel) * angular_scale
            v_sag = 0
            self.get_logger().info("Sağa dönülüyor: Sol tekerlek ileri")
        
        return {
            'sol_arka': -v_sol,
            'sag_arka': v_sag
        }

    def convert_to_pwm(self, wheel_speeds):
        """Tekerlek hızlarını PWM değerlerine çevirir"""
        pwm_values = {}
        
        for wheel, speed in wheel_speeds.items():
            if abs(speed) < self.min_pwm:
                pwm_values[wheel] = 0
            else:
                pwm_values[wheel] = int(speed)
        
        return pwm_values

    def is_valid_message(self, message):
        """Gelen mesajın geçerli olup olmadığını kontrol eder"""
        if not message or len(message) < 3:
            return False
        
        valid_prefixes = ['STATUS:', 'CMD_ACK:', 'ERROR:', 'SADIT Robot Arduino Ready']
        
        for prefix in valid_prefixes:
            if message.startswith(prefix):
                return True
        
        return False

    def send_command(self, pwm_values):
        """Arduino'ya 2 tekerlek komutunu gönderir"""
        if self.serial_connection is None:
            return
        
        try:
            # Arduino kodundaki motor sıralaması: Motor 1: Sağ arka, Motor 2: Sol arka
            # Komut formatı: "MOVE:motor1_pwm,motor2_pwm\n"
            command = f"MOVE:{pwm_values['sag_arka']},{pwm_values['sol_arka']}\n"
            self.serial_connection.write(command.encode())
            self.get_logger().debug(f'Sent command: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {str(e)}')

    def serial_reader(self):
        """Arduino'dan gelen mesajları okur"""
        while rclpy.ok() and self.serial_connection:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if line and len(line) > 0:
                        if self.is_valid_message(line):
                            self.parse_arduino_message(line)
                        else:
                            self.get_logger().debug(f'Invalid message received: {repr(line)}')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {str(e)}')
                time.sleep(0.1)

    def parse_arduino_message(self, message):
        """Arduino mesajlarını ayrıştırır"""
        if message.startswith("STATUS:"):
            self.status_pub.publish(String(data=message))
            
            parts = message.replace("STATUS: ", "").split(",")
            for part in parts:
                if "motor1=" in part:
                    self.current_speeds['sag_arka'] = float(part.split("=")[1])
                elif "motor2=" in part:
                    self.current_speeds['sol_arka'] = float(part.split("=")[1])
                elif "battery=" in part:
                    self.battery_voltage = float(part.split("=")[1])
                    self.battery_voltage_pub.publish(Float32(data=self.battery_voltage))
            
            self.publish_joint_states()
            
            avg_speed = sum(self.current_speeds.values()) / len(self.current_speeds.values())
            self.current_speed_pub.publish(Float32(data=avg_speed))
            
        elif message.startswith("CMD_ACK:"):
            self.get_logger().debug(f'Arduino ACK: {message}')
        elif message.startswith("ERROR:"):
            self.get_logger().warn(f'Arduino Error: {message}')
        else:
            self.get_logger().info(f'Arduino: {message}')

    def publish_joint_states(self):
        """Joint states mesajını yayınlar"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        joint_state.name = ['sol_arka', 'sag_arka']
        
        joint_state.position = [
            self.current_positions['sol_arka'],
            self.current_positions['sag_arka']
        ]
        
        joint_state.velocity = [
            self.current_speeds['sol_arka'] * self.wheel_radius,
            self.current_speeds['sag_arka'] * self.wheel_radius
        ]
        
        self.joint_states_pub.publish(joint_state)

    def destroy_node(self):
        """Node kapanırken temizlik yapar"""
        if self.serial_connection:
            stop_command = {'sol_arka': 0, 'sag_arka': 0}
            self.send_command(stop_command)
            time.sleep(0.1)
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SaditSerialController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()


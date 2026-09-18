#!/usr/bin/env python3
"""
Nodo puente de ROS 2 para el Brazo Robótico.
Se suscribe a /joint_states, calcula la cinemática directa cartesiana
y transmite las coordenadas por puerto serie a la placa MKS Gen v1.4 a 9600 baudios.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

class RobotArmSerialBridge(Node):
    def __init__(self):
        super().__init__('robot_arm_serial_bridge')

        # Declaración de parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('use_hardware', True)
        self.declare_parameter('max_rate_hz', 15.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.use_hardware = self.get_parameter('use_hardware').get_parameter_value().bool_value
        self.max_rate_hz = self.get_parameter('max_rate_hz').get_parameter_value().double_value

        self.min_interval = 1.0 / self.max_rate_hz
        self.last_send_time = 0.0

        # Conexión serie
        self.ser = None
        if self.use_hardware:
            self.init_serial()

        # Suscriptor al tópico estándar de articulaciones
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info("Nodo puente serie ROS 2 iniciado y escuchando en /joint_states")

    def init_serial(self):
        try:
            self.get_logger().info(f"Conectando al hardware en {self.port} a {self.baudrate} baudios...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(1.8)  # Esperar reinicio DTR del ATmega2560
            if self.ser.in_waiting:
                resp = self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace').strip()
                self.get_logger().info(f"Respuesta inicial de la placa: {resp}")

            # Activar motores (M17)
            self.send_gcode("M17")
            self.get_logger().info("Motores activados con M17.")
        except Exception as e:
            self.get_logger().error(f"Fallo al abrir el puerto serie {self.port}: {e}")
            self.get_logger().warn("Continuando en modo simulación (sin hardware).")
            self.ser = None

    def send_gcode(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            return
        try:
            line = (cmd.strip() + "\r\n").encode('ascii')
            self.ser.write(line)
        except Exception as e:
            self.get_logger().error(f"Error transmitiendo comando serie: {e}")

    def joint_state_callback(self, msg: JointState):
        now = time.time()
        if now - self.last_send_time < self.min_interval:
            return  # Limitar tasa para no saturar el buffer serie de Arduino

        # Extraer nombres y posiciones de articulaciones
        joint_dict = dict(zip(msg.name, msg.position))

        th_rot = joint_dict.get('joint_base_rotate', 0.0)
        th_low = joint_dict.get('joint_lower_arm', 0.0)
        th_high = joint_dict.get('joint_upper_arm', -math.pi / 2.0)
        th_wrist = joint_dict.get('joint_wrist_rotate', 0.0)
        grip_pos = joint_dict.get('joint_gripper_left', 0.0)

        # Cinemática directa (eslabones de 120 mm)
        L = 120.0
        # Radio horizontal proyectado en plano XY
        # Hombro y codo articulados en cadena plana
        r_side = 2.0 * L * math.sin(abs(th_high) * 0.5)
        # Aproximación cinemática cartesiana para el efector
        phi = th_low
        y_side = r_side * math.cos(phi)
        z_side = r_side * math.sin(phi)

        x_mm = y_side * math.sin(th_rot)
        y_mm = max(20.0, y_side * math.cos(th_rot))
        z_mm = z_side + 65.0  # offset de base

        e_mm = math.degrees(th_wrist)

        # Enviar comando G1
        cmd = f"G1 X{x_mm:.1f} Y{y_mm:.1f} Z{z_mm:.1f} E{e_mm:.1f}"
        self.send_gcode(cmd)
        self.last_send_time = now

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.send_gcode("M18")  # Desactivar motores al apagar
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

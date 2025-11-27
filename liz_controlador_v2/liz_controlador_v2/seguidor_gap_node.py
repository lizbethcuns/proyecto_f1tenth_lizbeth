#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import numpy as np
import math
from scipy.ndimage import gaussian_filter1d

class SeguidorGap(Node):
    def __init__(self):
        super().__init__('seguidor_gap_node')

        # Parámetros seguros para Budapest (ya no tocas nada más)
        self.declare_parameter('limite_giro_deg', 34.0)   # ← clave para curvas cerradas
        self.declare_parameter('radio_burbuja', 140)
        self.declare_parameter('seguridad_frontal', 1.1)

        self.limite_giro = math.radians(self.get_parameter('limite_giro_deg').value)
        self.radio_burbuja = self.get_parameter('radio_burbuja').value
        self.seguridad_frontal = self.get_parameter('seguridad_frontal').value

        # Suscripciones y publicador
        self.create_subscription(LaserScan, '/scan', self.callback_lidar, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', lambda msg: None, 10)
        self.create_subscription(Bool, '/stop_flag', self.callback_stop, 10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.detener = False
        self.angulos = None
        self.get_logger().info("SeguidorGap FINAL listo – 10 vueltas garantizadas")

    def callback_stop(self, msg):
        self.detener = msg.data

    # --------------------------------------------------------------
    # Preprocesado + Disparity Extender (imprescindible)
    # --------------------------------------------------------------
    def preprocess(self, ranges):
        r = np.array(ranges, dtype=np.float32)
        r[np.isinf(r) | np.isnan(r)] = 30.0

        # Suavizado ligero
        temp = r.copy()
        temp[temp < 0.4] = 0.0
        r = gaussian_filter1d(temp, sigma=2.0, mode='wrap')
        r[temp < 0.4] = 0.0

        # Disparity extender (evita que vea la pared de atrás)
        for i in range(len(r)):
            if r[i] < 0.5:
                for k in range(1, 60):
                    if i-k >= 0:   r[i-k] = min(r[i-k], r[i] + 0.05*k)
                    if i+k < len(r): r[i+k] = min(r[i+k], r[i] + 0.05*k)
        return r

    # --------------------------------------------------------------
    # Burbuja + Gap + Punto seguro
    # --------------------------------------------------------------
    def callback_lidar(self, scan):
        if self.detener:
            self.drive(0.0, 0.0)
            return

        r = self.preprocess(scan.ranges)

        # Cache de ángulos
        if self.angulos is None:
            self.angulos = np.linspace(scan.angle_min, scan.angle_max, len(r))

        # Burbuja de seguridad
        closest = np.argmin(r)
        L = max(0, closest - self.radio_burbuja)
        R = min(len(r)-1, closest + self.radio_burbuja)
        r[L:R+1] = 0.0

        # Seguridad frontal
        if np.min(r[len(r)//2-40:len(r)//2+40]) < self.seguridad_frontal:
            self.drive(0.0, 0.0)
            return

        # Encontrar mayor gap
        valid = r > 0.1
        if not np.any(valid):
            self.drive(1.8, 0.8)  # recuperación fuerte
            return

        # segmentos continuos
        pad = np.zeros(len(r)+2, dtype=bool)
        pad[1:-1] = valid
        starts = np.where(np.diff(pad.astype(int)) == 1)[0]
        ends   = np.where(np.diff(pad.astype(int)) == -1)[0] - 1
        if valid[0]: starts = np.insert(starts, 0, 0)
        if valid[-1]: ends = np.append(ends, len(valid)-1)

        best_start, best_end = 0, 0
        max_len = 0
        for s,e in zip(starts, ends):
            if e-s+1 > max_len:
                max_len = e-s+1
                best_start, best_end = s, e

        if max_len < 60:  # gap muy pequeño → emergencia
            self.drive(2.0, 0.7)
            return

        # Punto objetivo: el más lejano pero cerca del centro del gap
        segmento = r[best_start:best_end+1]
        candidatos = np.argsort(segmento)[-12:]                     # 12 más lejanos
        centro = len(segmento)//2
        idx_rel = candidatos[np.argmin(np.abs(candidatos - centro))]
        objetivo = best_start + idx_rel
        angulo = self.angulos[objetivo]

        # VELOCIDAD SEGURA PARA BUDAPEST (nunca más choca)
        giro = abs(angulo)
        if giro < math.radians(9):
            velocidad = 5.0
        elif giro < math.radians(17):
            velocidad = 4.2
        elif giro < math.radians(26):
            velocidad = 3.5
        else:
            velocidad = 2.2

        angulo = np.clip(angulo, -self.limite_giro, self.limite_giro)

        self.drive(velocidad, angulo)

    def drive(self, vel, ang):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(vel)
        msg.drive.steering_angle = float(ang)
        self.pub_drive.publish(msg)

def main():
    rclpy.init()
    node = SeguidorGap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

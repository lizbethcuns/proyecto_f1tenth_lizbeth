#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class ContadorVueltas(Node):
    def __init__(self):
        super().__init__('contador_vueltas_node')

        # Suscripción a la odometría del carrito
        self.sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.callback_odom,
            10
        )

        # Publicador del flag de STOP
        self.pub_stop = self.create_publisher(
            Bool, '/stop_flag', 10
        )

        # Variables internas
        self.pos_inicial = None
        self.en_zona_meta = True
        self.vueltas = 0
        self.tiempo_inicio_vuelta = None
        self.tiempo_total = 0.0

        # Número máximo de vueltas
        self.max_vueltas = 5   # <-- puedes cambiarlo luego

        self.get_logger().info("Nodo ContadorVueltas listo. Esperando la primera posición...")

    # -------------------------------
    # Formatear tiempo mm:ss
    # -------------------------------
    def formato(self, segundos):
        minutos = int(segundos // 60)
        seg = segundos % 60
        return f"{minutos:02d}:{seg:05.2f}"

    # -------------------------------
    # Callback ODOMETRÍA
    # -------------------------------
    def callback_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Registrar primera posición como línea de meta
        if self.pos_inicial is None:
            self.pos_inicial = (x, y)
            self.get_logger().info(f"Meta registrada en x={x:.2f}, y={y:.2f}")
            return

        x0, y0 = self.pos_inicial
        distancia = math.sqrt((x - x0)**2 + (y - y0)**2)
        en_meta = distancia < 3.5   # tolerancia

        # 1. Mientras está en meta → no cuenta vuelta
        if en_meta and self.en_zona_meta:
            return

        # 2. Sale de meta → inicia vuelta
        if not en_meta and self.en_zona_meta:
            self.en_zona_meta = False
            self.tiempo_inicio_vuelta = self.get_clock().now()
            self.get_logger().info(
                f"\n=== Iniciando vuelta {self.vueltas + 1} ==="
            )
            return

        # 3. Regresa a meta → termina vuelta
        if en_meta and not self.en_zona_meta:
            self.vueltas += 1
            ahora = self.get_clock().now()
            duracion = (ahora - self.tiempo_inicio_vuelta).nanoseconds / 1e9
            self.tiempo_total += duracion

            self.get_logger().info(
                f"\n===== VUELTA {self.vueltas} COMPLETADA =====\n"
                f"Tiempo vuelta: {duracion:.2f} s\n"
                f"Tiempo total : {self.formato(self.tiempo_total)}\n"
                f"============================================="
            )

            # Verificar si llegó al límite
            if self.vueltas >= self.max_vueltas:
                msg_stop = Bool()
                msg_stop.data = True
                self.pub_stop.publish(msg_stop)
                self.get_logger().warn("Máximo de vueltas alcanzado → DETENIENDO vehículo")

            self.en_zona_meta = True


def main(args=None):
    rclpy.init(args=args)
    nodo = ContadorVueltas()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

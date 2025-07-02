import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from collections import deque
import csv
from datetime import datetime


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.evitacion_activa = False
        self.create_subscription(String, '/activar_evitacion', self.activar_callback, 10)

        self.subscription_lidar = self.create_subscription(
            PointStamped,
            '/utlidar/range_info',
            self.lidar_callback,
            10
        )

        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        self.lidar_recibido = False
        self.peligro_derecha = False
        self.peligro_izquierda = False
        self.mucha_dist = False
        self.distancia_critica = False

        self.distancia_frontal_buffer = deque(maxlen=5)
        self.distancia_izquierda_buffer = deque(maxlen=5)
        self.distancia_derecha_buffer = deque(maxlen=5)

        # Medición y logging
        self.csv_file = open('laberinto_derecha.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'frontal_m', 'izquierda_m', 'derecha_m'])

        # Variables para detectar bloqueo
        self.estado_atrapado = False
        self.contador_bloqueo = 0
        self.max_bloqueo = 20  # 20 ciclos = 2 segundos si el timer es 0.1s
        self.ultimo_giro = None  # "izq" o "der"

        self.get_logger().info("Nodo de detección de obstáculos iniciado")

    def activar_callback(self, msg):
        contenido = msg.data.lower().strip()
        if contenido == "true":
            self.evitacion_activa = True
            self.get_logger().info("✅ Evitación de obstáculos ACTIVADA por interfaz.")
        elif contenido == "false":
            self.evitacion_activa = False
            self.get_logger().info("⛔ Evitación de obstáculos DESACTIVADA por interfaz.")

    def lidar_callback(self, msg):
        self.lidar_recibido = True
        self.distancia_frontal_buffer.append(msg.point.x)
        self.distancia_izquierda_buffer.append(msg.point.y)
        self.distancia_derecha_buffer.append(msg.point.z)

        distancia_frontal = sum(self.distancia_frontal_buffer) / len(self.distancia_frontal_buffer)
        distancia_izquierda = sum(self.distancia_izquierda_buffer) / len(self.distancia_izquierda_buffer)
        distancia_derecha = sum(self.distancia_derecha_buffer) / len(self.distancia_derecha_buffer)

        self.peligro_derecha = distancia_derecha < 0.4
        self.peligro_izquierda = distancia_izquierda < 0.4
        self.mucha_dist = distancia_frontal > 1.5
        self.distancia_critica = distancia_frontal < 0.7

        # Guardar en CSV
        timestamp = datetime.now().isoformat()
        self.csv_writer.writerow([timestamp, distancia_frontal, distancia_izquierda, distancia_derecha])
        self.csv_file.flush()

    def publish_velocity(self):
        twist = Twist()
        if not self.evitacion_activa or not self.lidar_recibido:
            return

        # Detectar bloqueo (obstáculo delante y sin salida lateral)
        if self.distancia_critica and self.peligro_derecha and self.peligro_izquierda:
            self.contador_bloqueo += 1
        else:
            self.contador_bloqueo = 0

        if self.contador_bloqueo >= self.max_bloqueo:
            self.estado_atrapado = True
            self.get_logger().warn("⚠️ Atrapado detectado, forzando escape")

        if self.estado_atrapado:
            twist.linear.x = 0.0
            twist.angular.z = -0.5 if self.ultimo_giro != "der" else 0.5
            self.ultimo_giro = "izq" if self.ultimo_giro == "der" else "der"
            self.publisher_cmd_vel.publish(twist)
            self.get_logger().info("🔄 Escape: giro amplio para salir de zona estrecha")
            if self.contador_bloqueo >= self.max_bloqueo + 10:
                self.estado_atrapado = False
                self.contador_bloqueo = 0
            return

        # --- Lógica estándar de evitación ---
        if self.mucha_dist:
            twist.linear.x = 0.3
            twist.angular.z = -0.05
            self.get_logger().info("NO HAY OBSTACULO")
        elif not self.mucha_dist and not self.distancia_critica:
            twist.linear.x = 0.2
            self.get_logger().info("POSIBLE OBSTACULO")
            if not self.peligro_derecha:
                twist.angular.z = -0.3
                self.ultimo_giro = "der"
                self.get_logger().info("Giro derecha + posible obstaculo")
            else:
                twist.angular.z = 0.3
                self.ultimo_giro = "izq"
                self.get_logger().info("Giro izquierda + posible obstaculo")
        elif self.distancia_critica:
            twist.linear.x = 0.0
            self.get_logger().info("OBSTACULO DELANTE")
            if self.peligro_derecha and not self.peligro_izquierda:
                twist.angular.z = 0.4
                self.ultimo_giro = "izq"
                self.get_logger().info("Giro izquierda, peligro a la derecha")
            elif self.peligro_izquierda and not self.peligro_derecha:
                twist.angular.z = -0.4
                self.ultimo_giro = "der"
                self.get_logger().info("Giro derecha, peligro a la izquierda")
            else:
                twist.angular.z = 0.4
                self.ultimo_giro = "izq"
                self.get_logger().info("Giro aleatorio por bloqueo frontal")

        # Refuerzo de giro lateral si hay un lado con peligro claro
        if self.peligro_derecha and not self.peligro_izquierda:
            twist.angular.z = 0.2
            self.ultimo_giro = "izq"
            self.get_logger().info("PELIGRO LATERAL DERECHA")
        elif self.peligro_izquierda and not self.peligro_derecha:
            twist.angular.z = -0.2
            self.ultimo_giro = "der"
            self.get_logger().info("PELIGRO LATERAL IZQUIERDA")

        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info(f"Velocidad publicada: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

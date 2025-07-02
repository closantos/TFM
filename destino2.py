import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from collections import deque
import json
import math
import numpy as np
from shapely.geometry import LineString, Point
from pyproj import Transformer
from sklearn.decomposition import PCA

class NavegacionAutonoma(Node):
    def __init__(self):
        super().__init__('navegacion_autonoma')

        # Subscripciones
        self.sub_gps = self.create_subscription(String, '/gps', self.gps_callback, 10)
        self.sub_imu = self.create_subscription(Float32, '/brujula', self.imu_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update_navigation)

        # Estado
        self.gps_actual = None
        self.yaw_actual = None
        self.buffer = deque(maxlen=20)
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
        self.target_idx = 0
        self.girando = False
        self.yaw_objetivo = None
        self.reached_threshold = 2.0

        # Puntos objetivo
        self.puntos_control = self.load_waypoints('/home/carmen/ros2_ws/src/go2_robot_sdk/go2_robot_sdk/prueba.json')
        if len(self.puntos_control) < 1:
            self.get_logger().error("El archivo JSON no contiene puntos objetivo.")
            self.destino = True
        else:
            print(self.puntos_control[0])
            self.target_actual = self.transform_to_xy(self.puntos_control[0])
            self.destino = False

    def load_waypoints(self, ruta):
        try:
            with open(ruta, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Error al cargar waypoints: {e}")
            return []

    def transform_to_xy(self, punto):
        return np.array(self.transformer.transform(punto['longitud'], punto['latitud']))

    def gps_callback(self, msg):
        self.gps_actual = msg.data
        #print(msg.data)
        partes = self.gps_actual.split(',')
        #print(partes)
        latitud = float(partes[0].strip())
        longitud = float(partes[1].strip())
        punto_xy = self.transformer.transform(longitud, latitud)
        self.buffer.append(punto_xy)

    def imu_callback(self, msg):
        self.yaw_actual = math.degrees(msg.data) % 360
        #print(self.yaw_actual)
        

    def get_heading_vector(self):
        if len(self.buffer) < 2:
            return None
        points = np.array(list(self.buffer))
        pca = PCA(n_components=2)
        pca.fit(points)
        direction = pca.components_[0]
        error = pca.explained_variance_ratio_[0]/pca.explained_variance_[1]
        origin = self.buffer[0]
        target_vector = self.target_actual - origin
        if np.dot(direction, target_vector) < 0:
            direction = -direction
        return direction, error

    def compute_angle(self, v1, v2):
        dot = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        cos_theta = np.clip(dot / (norm_v1 * norm_v2), -1.0, 1.0)
        angle_rad = math.acos(cos_theta)
        angle_deg = math.degrees(angle_rad)
        cross = v1[0]*v2[1] - v1[1]*v2[0]
        return -angle_deg if cross < 0 else angle_deg

    def compute_distance_to_trajectory(self):
        if not self.buffer:
            return 0
        x1, y1 = self.buffer[0]
        x2, y2 = self.target_actual
        xp, yp = self.buffer[-1]
        line = LineString([(x1, y1), (x2, y2)])
        point = Point(xp, yp)
        return point.distance(line)

    def compute_distance_to_target(self):
        if not self.buffer:
            return float('inf')
        return np.linalg.norm(np.array(self.target_actual) - np.array(self.buffer[-1]))

    def update_navigation(self):
        if self.destino or self.gps_actual is None or self.yaw_actual is None or len(self.buffer) < 2:
            print("GPS desconectado u obteniendo puntos iniciales")
            return

        heading_vector, error = self.get_heading_vector()
        if heading_vector is None:
            print("probando2")
            return

        desired_vector = self.target_actual - self.buffer[0]
        angle = self.compute_angle(heading_vector, desired_vector)
        dist_lateral = self.compute_distance_to_trajectory()
        dist_to_target = self.compute_distance_to_target()
        reached = dist_to_target < self.reached_threshold

        self.get_logger().info(
            f"\n🧭 Heading: {heading_vector}\n📐 Ángulo con trayectoria: {angle:.2f}°"
            f"\n📏 Distancia lateral: {dist_lateral:.2f} m\n🎯 Distancia al objetivo: {dist_to_target:.2f} m"
            f"\n✅ ¿Llegó? {reached}\n{'-'*40}"
        )

        twist = Twist()
        print("error: ", error)
        if error > 100:
            print("Palante")
            twist.linear.x = 0.5
            self.cmd_vel_pub.publish(twist)
            return 


        if reached:
            self.get_logger().info("📍 Punto alcanzado.")
            self.target_idx += 1
            if self.target_idx >= len(self.puntos_control):
                self.get_logger().info("✅ Ruta finalizada. Deteniendo robot.")
                self.cmd_vel_pub.publish(Twist())
                self.destino = True
                return
            self.target_actual = self.transform_to_xy(self.puntos_control[self.target_idx])
            return

        # Giros grandes en parado
        """if self.girando:
            print("girando")
            error_yaw = (self.yaw_objetivo - self.yaw_actual + 540) % 360 - 180
            print("Error yaw", error_yaw)
            if abs(error_yaw) < 3:
                self.get_logger().info("Giro completado. Reanudando marcha recta.")
                self.girando = False
            else:
                twist.angular.z = 0.3 if error_yaw > 0 else -0.3
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                return"""
        if abs(angle) > 10:
            #self.yaw_objetivo = (self.yaw_actual + angle) % 360
            self.girando = True
            if angle > 0:
                print("mayor que 0")
                twist.angular.z = 0.3
            else:
                print("menor que 0")
                twist.angular.z = -0.3
            twist.linear.x = 0.5
            #self.get_logger().info(f"Iniciando giro. Objetivo: {self.yaw_objetivo:.2f}°, Actual: {self.yaw_actual:.2f}°")
            print("girando...", twist.angular.z)
            

        # Corrección suave en marcha
        twist.linear.x += 0.3
        """if abs(angle) > 2:
            twist.angular.z += -0.05 if angle > 0 else 0.05
        else:
            twist.angular.z = 0.0"""

        print("mensaje", twist)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavegacionAutonoma()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

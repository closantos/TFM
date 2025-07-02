import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from std_msgs.msg import String, Float32, Float64
from geometry_msgs.msg import Twist, PointStamped
from collections import deque
import math
import json

class RobotNavigation(Node):
    def __init__(self):
        super().__init__('robot_navigation')
        
        # Suscripciones
        #self.create_subscription(SportModeState, '/sportmodestate_2', self.imu_callback, 10)
        self.create_subscription(Float64, '/brujula_arduino', self.imu_callback, 10)
        #self.create_subscription(Float32, '/brujula', self.imu_callback, 10)
        self.create_subscription(String, 'gps', self.gps_callback, 10)
        self.create_subscription(PointStamped, '/utlidar/range_info', self.lidar_callback, 10)
        
        # Publicador de velocidad
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Estado de detección de obstáculos
        self.lidar_recibido = False
        self.peligro_derecha = False
        self.peligro_izquierda = False
        self.mucha_dist = False
        self.distancia_critica = False

        # Variables de estado
        self.yaw_degrees = 0.0
        self.gps_coordinates = None
        #self.offset_yaw = None
        self.offset_yaw = -230
        self.obstacle_data = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}
        
        # Orientacion robot
        self.punto_control_actual = None
        self.contador_actualizar_angulo = 0
        self.redirigiendo = False
        self.puntos_de_control = []
        self.indice_punto_control = 0
        self.destino = False
        self.umbral_orientacion_grande = 10
        self.umbral_orientacion_peque = 6

        # Buffers de los últimos valores del LiDAR
        self.distancia_frontal_buffer = deque(maxlen=5)
        self.distancia_izquierda_buffer = deque(maxlen=5)
        self.distancia_derecha_buffer = deque(maxlen=5)
        
        self.get_logger().info("Nodo de navegación iniciado")
    
    '''
    def imu_callback(self, msg):
        q = msg.imu_state.quaternion
        w, x, y, z = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        yaw_deg = yaw_deg % 360 if yaw_deg >= 0 else 360 + yaw_deg
        yaw_deg = (360 - yaw_deg) % 360
        if self.offset_yaw is None:
            self.offset_yaw = yaw_deg
            self.get_logger().info(f"Offset inicial guardado: {self.offset_yaw:.2f}°")
        self.yaw_degrees = (yaw_deg - self.offset_yaw + 360) % 360

        # Mostrar la orientación corregida
        #self.get_logger().info(f'[YAW corregido] Orientación del robot: {yaw_degrees:.2f}°')
    '''
    def imu_callback(self, msg):
        self.yaw_degrees = msg.data
        #self.get_logger().info(f"[IMU_CALLBACK] Recibido ángulo: {self.yaw_degrees}")


    '''
    def imu_callback(self, msg):
        self.yaw_degrees = (msg.data + self.offset_yaw)%360
    '''

    def gps_callback(self, msg):
        self.gps_coordinates = msg.data
        self.get_logger().info(f'Coordenadas GPS: {self.gps_coordinates}')
        self.partes_gps = self.gps_coordinates.split(',')
        latitud_actual = float(self.partes_gps[0].split(':')[1].strip())
        longitud_actual = float(self.partes_gps[1].split(':')[1].strip())

        # Si no hay puntos de control, salir
        if not self.puntos_de_control:
            self.get_logger().warn("No hay puntos de control definidos.")
            return

        # Si ya te pasaste, mantente en el último punto
        if self.indice_punto_control >= len(self.puntos_de_control):
            self.indice_punto_control = len(self.puntos_de_control) - 1

        # Obtener punto de control actual
        punto_control_actual = self.puntos_de_control[self.indice_punto_control]

        # Calcular distancia
        distancia = self.calcular_distancia(
            latitud_actual,
            longitud_actual,
            punto_control_actual['latitud'],
            punto_control_actual['longitud']
        )

        self.get_logger().info(f"Distancia al checkpoint: {distancia:.2f} metros")

        # Verificar si llegaste al destino final
        if self.indice_punto_control == len(self.puntos_de_control) - 1 and distancia < 10:
            if not self.destino:
                self.get_logger().info("HAS LLEGADO A TU DESTINO")
                self.destino = True
            return

        # Si no es el último, y ya llegaste, avanza al siguiente
        if distancia < 1:
            self.indice_punto_control += 1
            self.get_logger().info("Checkpoint alcanzado. Pasando al siguiente punto de control.")

    def lidar_callback(self, msg):
        """ Callback que recibe datos del LiDAR y calcula la media móvil """
        self.lidar_recibido = True

        self.distancia_frontal_buffer.append(msg.point.x)
        self.distancia_izquierda_buffer.append(msg.point.y)
        self.distancia_derecha_buffer.append(msg.point.z)

        # Calcular la media de los últimos valores
        distancia_frontal = sum(self.distancia_frontal_buffer) / len(self.distancia_frontal_buffer)
        distancia_izquierda = sum(self.distancia_izquierda_buffer) / len(self.distancia_izquierda_buffer)
        distancia_derecha = sum(self.distancia_derecha_buffer) / len(self.distancia_derecha_buffer)


        # Determinar estado de detección de obstáculos
        self.peligro_derecha = distancia_derecha < 0.4
        self.peligro_izquierda = distancia_izquierda < 0.4
        self.mucha_dist = distancia_frontal > 1.5
        self.distancia_critica = distancia_frontal < 0.7

        #self.get_logger().info(f"Distancia frontal: {distancia_frontal:.2f} m")
        #self.get_logger().info(f"Distancia izquierda: {distancia_izquierda:.2f} m - Distancia derecha: {distancia_derecha:.2f} m")
    
    def cargar_puntos_de_control(self, prueba):
        """Carga los puntos de control desde un archivo JSON."""
        try:
            with open(prueba, "r") as archivo:
                puntos_de_control = json.load(archivo)
                self.get_logger().info(f"Puntos de control cargados: {puntos_de_control}")
                return puntos_de_control
        except Exception as e:
            self.get_logger().error(f"Error al cargar el archivo JSON: {e}")
            return None

    def orientar_robot(self, punto_control):
        """Calcula la diferencia de ángulo entre la orientación del robot y el punto de control."""
        self.get_logger().info(f"[DEBUG] gps_coordinates = {self.gps_coordinates}")
        
        if self.gps_coordinates is None:
            self.get_logger().warn("No se han recibido coordenadas GPS aún.")
            return None

        #self.get_logger().info(f"[DEBUG] gps_coordinates raw = '{self.gps_coordinates}'")
        self.partes_gps = self.gps_coordinates.split(',')
        latitud_actual = float(self.partes_gps[0].split(':')[1].strip())
        longitud_actual = float(self.partes_gps[1].split(':')[1].strip())
        #self.get_logger().info(f"[DEBUG] latitud_actual = {latitud_actual}, longitud_actual = {longitud_actual}")
        
        delta_lat = punto_control['latitud'] - latitud_actual
        delta_lon = punto_control['longitud'] - longitud_actual

        # Calcular ángulo entre el robot y el punto de control
        angulo_punto_control = math.atan2(delta_lon, delta_lat) * (180 / math.pi)
        if angulo_punto_control < 0:
            angulo_punto_control += 360
        self.get_logger().info(f"Angulo a punto de control: {angulo_punto_control}")
        self.get_logger().info(f"Angulo brujula: {self.yaw_degrees}")

        diferencia_angulo = (angulo_punto_control - self.yaw_degrees + 180) % 360 -180
        
        #self.get_logger().info(f"Diferencia angulo: {diferencia_angulo}")

        return diferencia_angulo
    
    def calcular_distancia(self, lat1, lon1, lat2, lon2):
        R = 6371.0  # Radio de la Tierra en km
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distancia = R * c
        return distancia * 1000  # Lo devolvemos en metros


    def publish_velocity(self):
        """ Publica velocidad según la detección de obstáculos """
        '''
        if not self.lidar_recibido:
            self.get_logger().warn("Esperando datos del LiDAR...")
            return  # No hace nada hasta que lleguen datos
        '''

        if self.destino:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_cmd_vel.publish(twist)
            self.get_logger().info("Robot ha llegado al destino. Detenido.")
            return

        twist = Twist()

        # NO PELIGRO de obstáculos
        #if self.mucha_dist:
        #self.get_logger().info("NO HAY OBSTACULO")
        if self.punto_control_actual is not None and self.gps_coordinates is not None:
            self.contador_actualizar_angulo += 1

            if self.contador_actualizar_angulo >= 5:  # Cada 0.5 s si timer = 0.1 s
                diferencia_angulo = self.orientar_robot(self.punto_control_actual)
                self.get_logger().info(f"Diferencia angulo: {diferencia_angulo}")
                if diferencia_angulo is not None:
                    self.redirigiendo = True
                    twist.linear.x = 0.0

                    if self.redirigiendo:
                        if abs(diferencia_angulo) <= self.umbral_orientacion_peque:
                            self.redirigiendo = False
                            twist.linear.x = 0.3
                            twist.angular.z = -0.05  # Compensar rumbo
                            self.get_logger().info("Robot orientado")
                        else:
                            twist.linear.x = 0.0
                            if diferencia_angulo > 0:
                                twist.angular.z = -0.2
                                self.get_logger().info("Reorientacion DERECHA (manteniendo)")
                            else:
                                twist.angular.z = 0.2
                                self.get_logger().info("Reorientacion IZQUIERDA (manteniendo)")
                    else:
                        if abs(diferencia_angulo) > self.umbral_orientacion_grande:
                            self.redirigiendo = True
                            twist.linear.x = 0.0
                            if diferencia_angulo > 0:
                                twist.angular.z = -0.2
                                self.get_logger().info("Reorientacion DERECHA (activada)")
                            else:
                                twist.angular.z = 0.2
                                self.get_logger().info("Reorientacion IZQUIERDA (activada)")
                        else:
                            twist.linear.x = 0.3
                            twist.angular.z = -0.05  # Compensar rumbo
                            self.get_logger().info("Robot orientado2")

                    self.publisher_cmd_vel.publish(twist)
                    self.get_logger().info(f"Velocidad publicada: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
                    #else:
                    #    twist.angular.z = 0.0
                    #    twist.linear.x = 0.3
                    #    self.get_logger().info("Orientacion correcta, avanzando")
                
                self.contador_actualizar_angulo = 0
        '''
        # POSIBLE PELIGRO
        elif not self.mucha_dist and not self.distancia_critica:
            twist.linear.x = 0.2  # Velocidad moderada
            self.get_logger().info("POSIBLE OBSTACULO")
            
            if not self.peligro_derecha:
                twist.angular.z = -0.3  # Giro a la derecha
                self.get_logger().info("Giro derecha + posible obstaculo")
            else:
                twist.angular.z = 0.1  # Giro a la izquierda suave
                self.get_logger().info("Giro izquierda + posible obstaculo")
        # PELIGRO
        elif self.distancia_critica:
            twist.linear.x = 0.0
            self.get_logger().info("OBSTACULO DELANTE")
            
            if self.peligro_derecha and not self.peligro_izquierda:
                twist.angular.z = 0.3  # Giro izquierda
                self.get_logger().info("Giro izquierda, peligro a la derecha")
            elif self.peligro_izquierda and not self.peligro_derecha:
                twist.angular.z = -0.3  # Giro derecha
                self.get_logger().info("Giro derecha, peligro a la izquierda")
            else:
                twist.angular.z = 0.3  # Giro izquierda por defecto
                self.get_logger().info("Giro aleatorio")
        # PELIGRO LATERAL
        if self.peligro_derecha and not self.peligro_izquierda:
            twist.angular.z = 0.1
            self.get_logger().info("PELIGRO LATERAL DERECHA")
        elif self.peligro_izquierda and not self.peligro_derecha:
            self.get_logger().info("PELIGRO LATERAL IZQUIERDA")            

        '''
        #self.publisher_cmd_vel.publish(twist)
        #self.get_logger().info(f"Velocidad publicada: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

        
    

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigation()
    puntos_de_control = node.cargar_puntos_de_control("/home/carmen/ros2_ws/src/go2_robot_sdk/go2_robot_sdk/prueba.json")
    if puntos_de_control:
        node.puntos_de_control = puntos_de_control   # ASIGNA LA LISTA
        node.indice_punto_control = 0                # COMIENZA DESDE EL PRIMER PUNTO
        node.punto_control_actual = puntos_de_control[0]  # Opcional si lo usas directamente
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

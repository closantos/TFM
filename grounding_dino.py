import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PointStamped
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from groundingdino.util.inference import load_model, predict, annotate

matplotlib.use('TkAgg')

# === Configuración de Grounding DINO ===
CONFIG_PATH = "/home/carmen/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
WEIGHTS_PATH = "/home/carmen/weights/groundingdino_swint_ogc.pth"
device = "cuda" if torch.cuda.is_available() else "cpu"
model = load_model(CONFIG_PATH, WEIGHTS_PATH).to(device)

# === Parámetros ===
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
BOX_THRESHOLD = 0.5
TEXT_THRESHOLD = 0.25
UMBRAL_DETECCIONES = 3
DETECTION_INTERVAL = 3
ESPERA_CENTRADO_FRAMES = 20  # 2 segundos si el timer es 0.1s

class DinoBot(Node):
    def __init__(self):
        super().__init__('dino_bot')
        self.bridge = CvBridge()
        self.prompt = None
        self.recibido = False
        self.avanzando = False
        self.distancia_critica = False
        self.detecciones_consecutivas = 0
        self.frame_count = 0
        self.last_frame = None
        self.last_annotated = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
        self.frames_espera = 0
        self.en_espera_para_avanzar = False
        self.plot_inicializado = False


        # Para brujula robusta
        self.ultimo_angulo = None
        self.angulo_acumulado = 0.0

        # ROS: topics
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.prompt_sub = self.create_subscription(String, '/buscar_objeto', self.prompt_callback, 10)
        self.lidar_sub = self.create_subscription(PointStamped, '/utlidar/range_info_2', self.lidar_callback, 10)
        self.brujula_sub = self.create_subscription(Float32, '/brujula', self.brujula_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para comportamiento
        self.timer = self.create_timer(0.1, self.procesar_comportamiento)

    def prompt_callback(self, msg):
        self.prompt = msg.data.strip()
        self.recibido = True
        self.avanzando = False
        self.en_espera_para_avanzar = False
        self.frames_espera = 0
        self.detecciones_consecutivas = 0
        self.ultimo_angulo = None
        self.angulo_acumulado = 0.0
        self.get_logger().info(f"🟢 Prompt recibido: {self.prompt}")

        if not self.plot_inicializado:
            self.inicializar_plot()
            self.plot_inicializado = True

    
    def inicializar_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.im_plot = self.ax.imshow(np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8))
        self.ax.axis('off')
        self.fig.canvas.manager.set_window_title("Grounding DINO - Detección en vivo")


    def lidar_callback(self, msg):
        self.distancia_critica = msg.point.x < 0.7

    def brujula_callback(self, msg):
        nuevo_angulo = msg.data
        if self.recibido:
            if self.ultimo_angulo is None:
                self.ultimo_angulo = nuevo_angulo
                return

            delta = nuevo_angulo - self.ultimo_angulo
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360

            self.angulo_acumulado += abs(delta)
            self.ultimo_angulo = nuevo_angulo

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame = cv2.resize(img, (FRAME_WIDTH, FRAME_HEIGHT))
        except Exception as e:
            self.get_logger().error(f"❌ Error al convertir imagen: {e}")

    def procesar_comportamiento(self):
        if self.last_frame is None:
            return

        frame = self.last_frame.copy()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if self.recibido and self.prompt:
            if not self.avanzando and self.angulo_acumulado >= 460.0:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("❌ Objeto no encontrado tras 1 vuelta.")
                self.reset_estado()
                return

            if self.en_espera_para_avanzar:
                self.frames_espera += 1
                if self.frames_espera >= ESPERA_CENTRADO_FRAMES:
                    self.get_logger().info("🎯 Objeto centrado y espera cumplida. Avanzando.")
                    self.avanzando = True
                    self.en_espera_para_avanzar = False
                else:
                    self.girar()
                return

            if not self.avanzando:
                self.girar()

                if self.frame_count % DETECTION_INTERVAL == 0:
                    try:
                        boxes, logits, phrases = predict(
                            model=model,
                            image=rgb,
                            caption=self.prompt,
                            box_threshold=BOX_THRESHOLD,
                            text_threshold=TEXT_THRESHOLD,
                            device=device
                        )
                        self.last_annotated = annotate(rgb, boxes, logits, phrases)
                        print(f"🔍 Detectados: {len(boxes)}")

                        for box, phrase in zip(boxes, phrases):
                            if self.prompt.lower() in phrase.lower():
                                cx = (box[0] + box[2]) / 2
                                if 0.4 < cx < 0.6:
                                    self.detecciones_consecutivas += 1
                                    print(f"✅ Objeto centrado (cx={cx:.2f}) -> detecciones: {self.detecciones_consecutivas}")
                                    if self.detecciones_consecutivas >= UMBRAL_DETECCIONES:
                                        self.en_espera_para_avanzar = True
                                        self.frames_espera = 0
                                        self.get_logger().info("⏳ Esperando centrado estable antes de avanzar...")
                                else:
                                    print(f"⚠️ Objeto detectado pero descentrado (cx={cx:.2f})")
                                break

                    except Exception as e:
                        self.get_logger().error(f"Error en predicción: {e}")
            else:
                self.avanzar()
        else:
            self.last_annotated = frame

        self.update_plot(self.last_annotated)
        self.frame_count += 1

    def girar(self):
        twist = Twist()
        twist.angular.z = -0.25
        twist.linear.x = 0.0
        #self.cmd_pub.publish(twist)

    def avanzar(self):
        twist = Twist()
        if not self.distancia_critica:
            twist.linear.x = 0.25
            twist.angular.z = -0.05
            #self.cmd_pub.publish(twist)
        else:
            #self.cmd_pub.publish(Twist())
            self.get_logger().info("🛑 Objeto alcanzado. Parando.")
            self.reset_estado()

    def reset_estado(self):
        self.recibido = False
        self.prompt = None
        self.avanzando = False
        self.en_espera_para_avanzar = False
        self.frames_espera = 0
        self.detecciones_consecutivas = 0
        self.ultimo_angulo = None
        self.angulo_acumulado = 0.0

    def update_plot(self, img_bgr):
        if not self.plot_inicializado:
            return  # No dibujar nada si aún no se ha inicializado

        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        self.im_plot.set_data(rgb)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



def main():
    rclpy.init()
    node = DinoBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🔴 Interrumpido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.close()


if __name__ == '__main__':
    main()

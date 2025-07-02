import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Abre la webcam USB (ajusta el índice si es necesario, ej. 0, 1, 2...)
        self.cap = cv2.VideoCapture(0)  # Usa 0 si es la única cámara

        if not self.cap.isOpened():
            self.get_logger().error("❌ No se pudo abrir la cámara USB")
            exit()

        # Opcional: configura resolución deseada
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)  # 30 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("⚠️ No se pudo leer un frame")
            return

        # Convertir a mensaje ROS
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

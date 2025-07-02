import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib

# Usa el backend más estable con GUI
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

class CameraSubscriberTkAgg(Node):
    def __init__(self):
        super().__init__('camera_subscriber_tkagg')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.image = None

        # Inicializar ventana de matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        dummy_img = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.im_plot = self.ax.imshow(dummy_img)
        self.ax.axis('off')
        self.fig.canvas.manager.set_window_title("Cámara USB (TkAgg)")

    def listener_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.update_plot()
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')

    def update_plot(self):
        if self.image is not None:
            rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            self.im_plot.set_data(rgb_image)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberTkAgg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.close()

if __name__ == '__main__':
    main()
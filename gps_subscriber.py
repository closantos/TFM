import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps',
            self.gps_callback,
            10)
        self.subscription  # Evita que se elimine la suscripción

    def gps_callback(self, msg):
        print(msg.data)  # Imprime directamente las coordenadas


def main(args=None):
    rclpy.init(args=args)
    gps_subscriber = GPSSubscriber()
    rclpy.spin(gps_subscriber)
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

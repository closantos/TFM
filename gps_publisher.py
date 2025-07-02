import serial
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(String, 'gps', 10)

        try:
            self.gps = serial.Serial('/dev/ttyUSB0', baudrate=4800, timeout=0.1)
            self.get_logger().info("✅ Conectado al GPS en /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"❌ Error al conectar con el GPS: {e}")
            return

        self.timer = self.create_timer(0.05, self.read_gps_data)  # Más frecuente (20 Hz aprox)

    def parse_gngga(self, nmea):
        match = re.match(r'\$GNGGA,\d+\.\d+,(\d{2})(\d{2}\.\d+),([NS]),(\d{3})(\d{2}\.\d+),([EW])', nmea)
        if match:
            lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir = match.groups()

            lat = float(lat_deg) + float(lat_min) / 60.0
            lon = float(lon_deg) + float(lon_min) / 60.0
            if lat_dir == 'S':
                lat = -lat
            if lon_dir == 'W':
                lon = -lon
            return lat, lon
        return None

    def read_gps_data(self):
        try:
            line = self.gps.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$GNGGA'):
                self.get_logger().info(f"NMEA: {line}")  # Muestra línea completa del GPS
                coords = self.parse_gngga(line)
                if coords:
                    lat, lon = coords
                    msg = f"{lat:.8f},{lon:.8f}"
                    self.publisher_.publish(String(data=msg))
                    self.get_logger().info(f"📍 Lat: {lat:.6f}, Lon: {lon:.6f}")
        except Exception as e:
            self.get_logger().warn(f"Error al leer GPS: {e}")

    def destroy_node(self):
        if hasattr(self, 'gps') and self.gps.is_open:
            self.gps.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

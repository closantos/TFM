import rclpy
import json
import base64
import time
import wave
from rclpy.node import Node
from unitree_api.msg import Request

class AudioPublisherLite(Node):
    def __init__(self, wav_path):
        super().__init__('audio_publisher_lite')
        self.publisher = self.create_publisher(Request, '/api/audiohub/request', 10)
        self.get_logger().info(f"🔊 Enviando archivo: {wav_path}")
        self.send_audio(wav_path)

    def send_audio(self, wav_path):
        with open(wav_path, "rb") as f:
            wav_data = f.read()

        # Codificar en base64
        b64_audio = base64.b64encode(wav_data).decode("utf-8")
        chunk_size = 16 * 1024
        chunks = [b64_audio[i:i + chunk_size] for i in range(0, len(b64_audio), chunk_size)]
        total_chunks = len(chunks)

        def send(api_id, param=""):
            msg = Request()
            msg.header.identity.api_id = api_id
            msg.parameter = param
            self.publisher.publish(msg)

        # Inicio
        send(4001)
        self.get_logger().info("🟢 Inicio enviado")
        time.sleep(0.1)

        # Chunks
        for i, chunk in enumerate(chunks, 1):
            block = {
                "current_block_index": i,
                "total_block_number": total_chunks,
                "block_content": chunk
            }
            send(4003, json.dumps(block))
            self.get_logger().info(f"📤 Chunk {i}/{total_chunks}")
            time.sleep(0.15)

        # Calcular duración aprox desde cabecera WAV
        try:
            with wave.open(wav_path, 'rb') as wf:
                frames = wf.getnframes()
                rate = wf.getframerate()
                duration = frames / float(rate)
        except:
            duration = 8.0  # Por defecto

        self.get_logger().info(f"⏳ Esperando {duration:.2f}s de reproducción...")
        time.sleep(duration + 1)

        # Fin
        send(4002)
        self.get_logger().info("✅ Reproducción finalizada")

def main():
    rclpy.init()
    wav_path = "/home/unitree/ros2_ws_cyclon/go2_robot_sdk/go2_robot_sdk/descripcion.wav"
    node = AudioPublisherLite(wav_path)
    rclpy.spin_once(node, timeout_sec=1)

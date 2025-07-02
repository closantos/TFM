import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import base64
import io
import os
import json
import time
import wave
import matplotlib
matplotlib.use('TkAgg')  # 👈 Forzar uso de backend compatible

import matplotlib.pyplot as plt  # Luego de definir el backend
from openai import OpenAI
from pydub import AudioSegment
from unitree_api.msg import Request
from std_msgs.msg import String

AUDIO_DIR = "/home/carmen/ros2_ws/src/go2_robot_sdk/go2_robot_sdk"
MP3_PATH = os.path.join(AUDIO_DIR, "descripcion.mp3")
WAV_PATH = os.path.join(AUDIO_DIR, "descripcion.wav")

class ChatGPTDescriptor:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)
        self.model = "gpt-4o"
        self.system_prompt = "You are an AI assistant that provides one sentence, objective image descriptions."

    def encode_image_base64(self, image):
        _, buffer = cv2.imencode('.jpg', image)
        return base64.b64encode(buffer).decode("utf-8")

    def ask_from_img(self, image):
        img_b64 = self.encode_image_base64(image)
        messages = [
            {"role": "system", "content": self.system_prompt},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "Describe the image in detail."},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}}
                ]
            }
        ]
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.2,
                max_tokens=300
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            print(f"[ERROR] Descripción fallida: {str(e)}")
            return None

    def translate_to_spanish(self, english_text):
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "Eres un traductor profesional. Devuelve únicamente la traducción al español."},
                    {"role": "user", "content": f"Traduce al español: {english_text}"}
                ],
                temperature=0.2,
                max_tokens=300
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            print(f"[ERROR] Traducción fallida: {str(e)}")
            return None

    def generate_audio(self, text):
        try:
            response = self.client.audio.speech.create(
                model="tts-1",
                voice="nova",
                input=text
            )
            with open(MP3_PATH, "wb") as f:
                f.write(response.content)
            print(f"🔊 Guardado como MP3: {MP3_PATH}")

            audio = AudioSegment.from_mp3(io.BytesIO(response.content))
            audio.export(WAV_PATH, format="wav")
            print(f"🔊 Convertido a WAV: {WAV_PATH}")

            time.sleep(1)

        except Exception as e:
            print(f"[ERROR] Generación de audio fallida: {str(e)}")


class ImageAudioNode(Node):
    def __init__(self, api_key):
        super().__init__('image_audio_node')
        self.bridge = CvBridge()
        self.descriptor = ChatGPTDescriptor(api_key)
        self.audio_pub = self.create_publisher(Request, '/api/audiohub/request', 10)

        # NUEVO: Para controlar activación/desactivación
        self.activado = False
        self.image_received = False

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)

        self.activador_sub = self.create_subscription(
            String, '/describir_entorno', self.activador_callback, 10)

        self.activador_pub = self.create_publisher(String, '/describir_entorno', 10)

    def activador_callback(self, msg):
        if msg.data.lower() == "true" and not self.activado:
            self.activado = True
            self.image_received = False
            self.get_logger().info("🟢 Activado por interfaz para describir entorno.")

    def send_audio(self):
        with open(WAV_PATH, "rb") as f:
            wav_data = f.read()

        b64_audio = base64.b64encode(wav_data).decode("utf-8")
        chunk_size = 16 * 1024
        chunks = [b64_audio[i:i + chunk_size] for i in range(0, len(b64_audio), chunk_size)]

        def send(api_id, param=""):
            msg = Request()
            msg.header.identity.api_id = api_id
            msg.parameter = param
            self.audio_pub.publish(msg)

        # Inicio
        send(4001)
        self.get_logger().info("🟢 Inicio enviado")
        time.sleep(0.1)

        # Chunks
        for i, chunk in enumerate(chunks, 1):
            block = {
                "current_block_index": i,
                "total_block_number": len(chunks),
                "block_content": chunk
            }
            send(4003, json.dumps(block))
            self.get_logger().info(f"📤 Chunk {i}/{len(chunks)}")
            time.sleep(0.15)

        # Duración
        try:
            with wave.open(WAV_PATH, 'rb') as wf:
                frames = wf.getnframes()
                rate = wf.getframerate()
                duration = frames / float(rate)
        except:
            duration = 8.0

        self.get_logger().info(f"⏳ Esperando {duration:.2f}s de reproducción...")
        time.sleep(duration + 1)

        send(4002)
        self.get_logger().info("✅ Reproducción finalizada")
        # Cerrar la ventana matplotlib si existe
        try:
            plt.close(self.fig)
        except Exception:
            pass

    def listener_callback(self, msg):
        if not self.activado or self.image_received:
            return

        try:
            self.image_received = True
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Mostrar imagen hasta que termine la reproducción
            plt.ion()
            self.fig, self.ax = plt.subplots()
            self.ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            self.ax.axis("off")
            self.fig.canvas.manager.set_window_title("🖼️ Imagen capturada para descripción")
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.1)  # forzamos render inicial

            self.get_logger().info("📸 Imagen recibida del topic")

            descripcion_en = self.descriptor.ask_from_img(img)
            self.get_logger().info(f"📝 Descripción (inglés): {descripcion_en}")

            descripcion_es = self.descriptor.translate_to_spanish(descripcion_en)
            self.get_logger().info(f"🌐 Traducción al español: {descripcion_es}")

            self.descriptor.generate_audio(descripcion_es)

            self.send_audio()

            # NUEVO: Resetear para permitir nuevas ejecuciones
            self.activado = False
            self.image_received = False  # permite nueva imagen en siguiente ciclo

            reset_msg = String()
            reset_msg.data = "false"
            self.activador_pub.publish(reset_msg)
            self.get_logger().info("🔁 Reset enviado a /describir_entorno")
            self.get_logger().info("✅ Proceso completo. Esperando próxima activación.")


        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")

def main():
    API_KEY = "" 
    rclpy.init()
    node = ImageAudioNode(API_KEY)
    rclpy.spin(node)

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import tkinter as tk
import threading
import googlemaps
import folium
import webbrowser
import math
import json
import subprocess
from tkinter import messagebox, simpledialog

ARCHIVO_FAVORITOS = "ubicaciones_favoritas.json"
rutas_favoritas = {}
datos_ruta = []
origen_actual = None

class InterfazRobot(Node):
    def __init__(self):
        super().__init__('interfaz_robot')
        self.publisher_describir = self.create_publisher(String, '/describir_entorno', 10)
        self.publisher_buscar = self.create_publisher(String, '/buscar_objeto', 10)
        self.publisher_hablar = self.create_publisher(String, '/reproducir_texto', 10)
        self.publisher_evitacion = self.create_publisher(String, '/activar_evitacion', 10)
        #self.publisher_webcam = self.create_publisher(String, '/activar_webcam', 10)
        self.subscription_gps = self.create_subscription(String, 'gps', self.gps_callback, 10)
        self.subscription_brujula = self.create_subscription(Float64, 'brujula_arduino', self.brujula_callback, 10)

        self.root = tk.Tk()
        self.root.title("Interfaz Robot")
        self.root.geometry("420x640")
        self.root.configure(bg="#f0f0f0")

        self.main_frame = tk.Frame(self.root, bg="#f0f0f0")
        self.search_frame = tk.Frame(self.root, bg="#f0f0f0")
        self.voice_frame = tk.Frame(self.root, bg="#f0f0f0")
        self.gps_label = tk.Label(self.main_frame, text="GPS: --", font=("Helvetica", 12), bg="#f0f0f0")
        self.brujula_label = tk.Label(self.main_frame, text="Orientación: --", font=("Helvetica", 12), bg="#f0f0f0")        
        self.crear_pantalla_principal()
        self.crear_pantalla_busqueda()
        self.crear_pantalla_voz()
        self.crear_pantalla_navegacion()

        self.evitacion_activada = False
        self.proceso_dino = None

        self.main_frame.pack(expand=True, fill="both")

    def crear_pantalla_principal(self):
        tk.Label(self.main_frame, text="Robot", font=("Helvetica", 24, "bold"), bg="#f0f0f0").pack(pady=30)

        tk.Button(self.main_frame, text="Descríbeme el entorno", font=("Helvetica", 14),
                  command=self.enviar_descripcion, width=25).pack(pady=10)

        tk.Button(self.main_frame, text="Búscame ...", font=("Helvetica", 14),
                  command=self.ir_a_busqueda, width=25).pack(pady=10)

        tk.Button(self.main_frame, text="Habla", font=("Helvetica", 14),
                  command=self.ir_a_voz, width=25).pack(pady=10)

        self.boton_evitacion = tk.Button(self.main_frame, text="Activar evitación", font=("Helvetica", 14),
                                 command=self.toggle_evitacion, width=25)
        self.boton_evitacion.pack(pady=10)


        tk.Button(self.main_frame, text="Ver cámara", font=("Helvetica", 14),
                  command=self.activar_webcam, width=25).pack(pady=10)

        tk.Button(self.main_frame, text="Navegación exterior", font=("Helvetica", 14),
                  command=self.ir_a_navegacion, width=25).pack(pady=10)

        self.gps_label.pack(pady=5)
        self.brujula_label.pack(pady=5)

    def brujula_callback(self, msg):
        orientacion = msg.data
        #self.get_logger().info(f"🧭 Orientación recibida: {orientacion}")
        self.brujula_label.config(text=f"Orientación: {orientacion}")


    def gps_callback(self, msg):
        gps_text = msg.data
        lat = lon = None
        try:
            parts = gps_text.split(',')
            for part in parts:
                if 'Lat' in part:
                    lat = float(part.split(':')[1].strip())
                elif 'Lon' in part:
                    lon = float(part.split(':')[1].strip())
            if lat and lon:
                global origen_actual
                origen_actual = (lat, lon)
                self.gps_label.config(text=f"GPS: Lat: {lat}, Lon: {lon}")
        except Exception as e:
            self.get_logger().error(f"Error procesando GPS: {e}")

    def activar_webcam(self):
        subprocess.Popen(["ros2", "run", "go2_robot_sdk", "webcam"])

    def toggle_evitacion(self):
        msg = String()
        if not self.evitacion_activada:
            msg.data = "true"
            self.evitacion_activada = True
            self.boton_evitacion.config(text="Desactivar evitación")
            self.get_logger().info("🟢 Evitación de obstáculos ACTIVADA")
        else:
            msg.data = "false"
            self.evitacion_activada = False
            self.boton_evitacion.config(text="Activar evitación")
            self.get_logger().info("🔴 Evitación de obstáculos DESACTIVADA")

        self.publisher_evitacion.publish(msg)


    def crear_pantalla_busqueda(self):
        tk.Label(self.search_frame, text="¿Qué deseas que busque?", font=("Helvetica", 16), bg="#f0f0f0").pack(pady=20)
        self.entry_prompt = tk.Entry(self.search_frame, font=("Helvetica", 14), width=30)
        self.entry_prompt.pack(pady=10)
        self.entry_prompt.bind("<Return>", self.enviar_busqueda)
        tk.Button(self.search_frame, text="Back", font=("Helvetica", 12),
                  command=self.volver_a_principal).pack(side="bottom", anchor="se", padx=10, pady=10)

    def crear_pantalla_voz(self):
        tk.Label(self.voice_frame, text="¿Qué quieres que diga?", font=("Helvetica", 16), bg="#f0f0f0").pack(pady=20)
        self.entry_voz = tk.Entry(self.voice_frame, font=("Helvetica", 14), width=30)
        self.entry_voz.pack(pady=10)
        self.entry_voz.bind("<Return>", self.enviar_texto_voz)
        tk.Button(self.voice_frame, text="Back", font=("Helvetica", 12),
                  command=self.volver_a_principal).pack(side="bottom", anchor="se", padx=10, pady=10)

    def crear_pantalla_navegacion(self):
        self.navegacion_frame = tk.Frame(self.root, bg="#f0f0f0")

        tk.Label(self.navegacion_frame, text="Destino:", bg="#f0f0f0", font=("Arial", 12)).grid(row=0, column=0, padx=10, pady=5)
        self.destino_entry = tk.Entry(self.navegacion_frame, font=("Arial", 12), width=30)
        self.destino_entry.grid(row=0, column=1, padx=10, pady=5)

        tk.Button(self.navegacion_frame, text="Calcular Ruta", command=self.calcular_ruta, bg="#4CAF50", fg="white").grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="ew")
        tk.Button(self.navegacion_frame, text="Favoritos", command=self.mostrar_favoritos, bg="#FFD700").grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        tk.Button(self.navegacion_frame, text="Guardar Favorito", command=self.guardar_favorito, bg="#87CEEB").grid(row=2, column=1, padx=10, pady=5, sticky="ew")
        tk.Button(self.navegacion_frame, text="Borrar Favorito", command=self.borrar_favorito, bg="#FF6347").grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky="ew")
        tk.Button(self.navegacion_frame, text="Volver", command=self.volver_a_principal, bg="#f44336", fg="white").grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

    def enviar_descripcion(self):
        msg = String()
        msg.data = "true"
        self.publisher_describir.publish(msg)

    def enviar_busqueda(self, event=None):
        texto = self.entry_prompt.get()
        if texto.strip():
            msg = String()
            msg.data = texto.strip()
            self.publisher_buscar.publish(msg)
            self.entry_prompt.delete(0, tk.END)

    # def lanzar_descripcion_chatgpt(self):
    #     # Lanza el nodo de descripción solo si no está ya lanzado
    #     subprocess.Popen(["ros2", "run", "go2_robot_sdk", "descripcion_chatgpt"])

    def lanzar_dino(self):
        if self.proceso_dino is None or self.proceso_dino.poll() is not None:
            self.proceso_dino = subprocess.Popen(["ros2", "run", "go2_robot_sdk", "dino"])
            self.get_logger().info("🟢 Nodo de Grounding DINO lanzado")


    def enviar_texto_voz(self, event=None):
        texto = self.entry_voz.get()
        if texto.strip():
            msg = String()
            msg.data = texto.strip()
            self.publisher_hablar.publish(msg)
            self.entry_voz.delete(0, tk.END)

    def ir_a_busqueda(self):
        self.lanzar_dino()  
        self._ocultar_todos()
        self.search_frame.pack(expand=True, fill="both")

    def ir_a_voz(self):
        #self.lanzar_descripcion_chatgpt()
        self._ocultar_todos()
        self.voice_frame.pack(expand=True, fill="both")

    def ir_a_navegacion(self):
        self._ocultar_todos()
        self.navegacion_frame.pack(expand=True, fill="both")

    def volver_a_principal(self):
        self._ocultar_todos()
        self.main_frame.pack(expand=True, fill="both")

        # 💥 Cierra Grounding DINO si está activo
        if self.proceso_dino and self.proceso_dino.poll() is None:
            self.proceso_dino.terminate()
            self.proceso_dino.wait()
            self.get_logger().info("🔴 Nodo de Grounding DINO detenido")
            self.proceso_dino = None


    def _ocultar_todos(self):
        self.main_frame.pack_forget()
        self.search_frame.pack_forget()
        self.voice_frame.pack_forget()
        if hasattr(self, 'navegacion_frame'):
            self.navegacion_frame.pack_forget()

    def obtener_coordenadas_desde_texto(self, texto):
        api_key = "TU_API_KEY"
        gmaps = googlemaps.Client(key=api_key)
        geocode_result = gmaps.geocode(texto)
        if geocode_result:
            ubicacion = geocode_result[0]['geometry']['location']
            return ubicacion['lat'], ubicacion['lng']
        return None

    def calcular_ruta(self):
        global datos_ruta
        destino_texto = self.destino_entry.get()
        destino = self.obtener_coordenadas_desde_texto(destino_texto)
        if destino and origen_actual:
            gmaps = googlemaps.Client(key="TU_API_KEY")
            directions_result = gmaps.directions(origen_actual, destino, mode="walking")
            if directions_result:
                datos_ruta = []
                ruta = directions_result
                mapa = folium.Map(location=[origen_actual[0], origen_actual[1]], zoom_start=15)
                primer_checkpoint = True
                for i, step in enumerate(ruta[0]['legs'][0]['steps']):
                    folium.Marker(location=[step['start_location']['lat'], step['start_location']['lng']]).add_to(mapa)
                    folium.PolyLine(locations=[(step['start_location']['lat'], step['start_location']['lng']),
                                               (step['end_location']['lat'], step['end_location']['lng'])], color='blue').add_to(mapa)
                    checkpoint_data = {
                        "checkpoint": i+1,
                        "latitud": step['start_location']['lat'],
                        "longitud": step['start_location']['lng']
                    }
                    datos_ruta.append(checkpoint_data)
                    if i < len(ruta[0]['legs'][0]['steps']) - 1:
                        next_step = ruta[0]['legs'][0]['steps'][i+1]
                        current_direction = self.obtener_angulo((step['start_location']['lat'], step['start_location']['lng']),
                                                               (step['end_location']['lat'], step['end_location']['lng']))
                        next_direction = self.obtener_angulo((next_step['start_location']['lat'], next_step['start_location']['lng']),
                                                             (next_step['end_location']['lat'], next_step['end_location']['lng']))
                        degrees = next_direction - current_direction
                        if degrees > 180:
                            degrees -= 360
                        elif degrees < -180:
                            degrees += 360
                        if not primer_checkpoint:
                            datos_ruta.append({"checkpoint": i+1, "giro": degrees})
                    primer_checkpoint = False
                with open("datos_ruta.json", "w") as f:
                    json.dump(datos_ruta, f)
                mapa.save("ruta.html")
                webbrowser.open("ruta.html")

    def obtener_angulo(self, p1, p2):
        lat1, lon1 = p1
        lat2, lon2 = p2
        dLon = lon2 - lon1
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        return (bearing + 360) % 360

    def guardar_favorito(self):
        nombre = simpledialog.askstring("Guardar Favorito", "Ingresa un nombre para esta ubicación:")
        if nombre:
            destino = self.obtener_coordenadas_desde_texto(self.destino_entry.get())
            if destino:
                rutas_favoritas[nombre] = destino
                with open(ARCHIVO_FAVORITOS, "w") as f:
                    json.dump(rutas_favoritas, f)
                messagebox.showinfo("Guardar Favorito", "Ubicación guardada como favorita.")

    def mostrar_favoritos(self):
        try:
            with open(ARCHIVO_FAVORITOS, "r") as f:
                favoritos = json.load(f)
                nombres = list(favoritos.keys())
                seleccion = simpledialog.askstring("Favoritos", "\n".join(nombres))
                if seleccion and seleccion in favoritos:
                    coords = favoritos[seleccion]
                    self.destino_entry.delete(0, tk.END)
                    self.destino_entry.insert(0, f"{coords[0]}, {coords[1]}")
        except:
            messagebox.showinfo("Favoritos", "No hay ubicaciones favoritas.")

    def borrar_favorito(self):
        try:
            with open(ARCHIVO_FAVORITOS, "r") as f:
                favoritos = json.load(f)
                nombres = list(favoritos.keys())
                seleccion = simpledialog.askstring("Borrar Favorito", "\n".join(nombres))
                if seleccion and seleccion in favoritos:
                    del favoritos[seleccion]
                    with open(ARCHIVO_FAVORITOS, "w") as f:
                        json.dump(favoritos, f)
                    messagebox.showinfo("Borrar Favorito", f"Ubicación '{seleccion}' borrada correctamente.")
        except:
            messagebox.showinfo("Favoritos", "No hay ubicaciones favoritas guardadas.")

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    interfaz = InterfazRobot()
    ros_thread = threading.Thread(target=rclpy.spin, args=(interfaz,), daemon=True)
    ros_thread.start()
    interfaz.run()
    interfaz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

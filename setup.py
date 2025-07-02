# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'go2_robot_sdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'dae'), glob(os.path.join('dae', '*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'calibration'), glob(os.path.join('calibration', '*'))),
        (os.path.join('share', package_name, 'external_lib'), ['external_lib/libvoxel.wasm']),
        (os.path.join('share', package_name, 'external_lib/aioice'), glob(os.path.join('external_lib/aioice/src/aioice', '*'))),
        
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brimo',
    maintainer_email='abizov94@gmail.com',
    description='Go2 ROS2 SDK for Unitree Go2 Edu/Pro/Air models',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_driver_node = go2_robot_sdk.go2_driver_node:main',
            'lidar_to_pointcloud = go2_robot_sdk.lidar_to_point:main',
            'sportmodestate = go2_robot_sdk.sportmodestate_listener:main',

            # ------ ALTAVOZ ------
            'altavoz = go2_robot_sdk.altavoz_offline:main', # Reproducir un texto en el robot
            'altavoz_webrtc = go2_robot_sdk.altavoz_offline_webrtc:main', # No funciona

            # ------ ARDUINO ------
            'arduino = go2_robot_sdk.arduino:main', # Publica el topic de la brújula de Arduino
            
            # ------ BRÚJULA ------
            'brujula = go2_robot_sdk.brujula:main', # Establece la IMU del robot en 0 al ejecutar
            'brujula_dron = go2_robot_sdk.brujula_dron_publisher:main',
            'brujula_correccion = go2_robot_sdk.brujula_correccion:main', # Ajustar el offset que necesita la brújula para que coincida con el móvil

            # ------ CÁMARAS ------
	        'camara = go2_robot_sdk.camara_robot_def:main', # No funciona
            'webcam = go2_robot_sdk.webcam_subscriber:main', # Se suscribe al topic de la webcam que está conectada al robot

            # ------ DESCRIPCIÓN DEL ENTORNO ------
            'descripcion_chatgpt = go2_robot_sdk.descripcion_img_chatgpt:main',
            
            # ------ EVITACIÓN DE OBSTÁCULOS ------
            'caminar = go2_robot_sdk.caminar:main',
            
            # ------ GPS ------
            'gps_subscriber = go2_robot_sdk.gps_subscriber:main', # Se suscribe al topic /gps
            'gps_publisher = go2_robot_sdk.gps_publisher:main',

            # ------ GROUNDING DINO ------
            #'dino = go2_robot_sdk.interfaz_dino:main'
            'dino = go2_robot_sdk.grounding_dino:main', 

            # ------ INTERFAZ ------
            'interfaz = go2_robot_sdk.interfaz:main',

            # ------ NAVEGACIÓN EXTERIORES ------
            'guardar_gps = go2_robot_sdk.comparacion_gps_guardar_datos_v2:main',
            'destino_clasico = go2_robot_sdk.destino:main',
            'destino_pca = go2_robot_sdk.destino2:main',
            'destino_proporcional = go2_robot_sdk.controlador_proporcional:main',
            'destino_stanley = go2_robot_sdk.controlador_stanley:main',
            'simulacion_clasica = go2_robot_sdk.simulacion_orientacion:main', # Simulación de la navegación de exteriores escribiendo a mano los valores de brújula y gps           
            'simulacion_controladores = go2_robot_sdk.simulacion_orientacion:main' # Simulación de la navegación de exteriores para ajustar las ganancias de los controladores
            ],
    },
)

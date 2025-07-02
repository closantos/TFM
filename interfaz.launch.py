from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de la interfaz gráfica
        Node(
            package='go2_robot_sdk',
            executable='interfaz',
            name='interfaz',
            output='screen'
        ),

        # Nodo del altavoz (reproduce texto en el robot)
        Node(
            package='go2_robot_sdk',
            executable='altavoz',
            name='altavoz_offline',
            output='screen'
        ),

        # # Nodo de la cámara (webcam)
        # Node(
        #     package='go2_robot_sdk',
        #     executable='webcam',
        #     name='webcam_subscriber',
        #     output='screen'
        # ),

        # Nodo de descripción del entorno con ChatGPT
        Node(
            package='go2_robot_sdk',
            executable='descripcion_chatgpt',
            name='descripcion_entorno',
            output='screen'
        ),

        # Nodo de evitación de obstáculos
        Node(
            package='go2_robot_sdk',
            executable='caminar',
            name='evitacion_obstaculos',
            output='screen'
        ),

        # Nodo de GPS
        Node(
            package='go2_robot_sdk',
            executable='gps_subscriber',
            name='gps_listener',
            output='screen'
        ),

        # # Nodo de Grounding DINO (búsqueda de objetos)
        # Node(
        #     package='go2_robot_sdk',
        #     executable='dino',
        #     name='grounding_dino',
        #     output='screen'
        # ),
    ])

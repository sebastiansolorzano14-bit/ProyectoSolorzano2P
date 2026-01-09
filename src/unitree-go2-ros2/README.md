# Proyecto 2P: SLAM y Planificación Global - Unitree Go2 (Office)

Este repositorio contiene la configuración y el desarrollo para el robot Unitree Go2 utilizando el controlador CHAMP en ROS 2 Humble. El proyecto se enfoca en el mapeo de un entorno de oficina y la implementación de un planificador de trayectoria global.

**Autor:** Sebastian Solorzano

**Algoritmo Asignado:** Theta*

**Mapa Asignado:** Office (Oficina)


**Video Demostrativo:** [INSERTE LINK DE YOUTUBE AQUÍ]

## Descripción General
En este proyecto se implementa un planificador de trayectorias globales para el robot cuadrúpedo Unitree Go2 en simulación, incluyendo:
* Mapeo del entorno (SLAM) con SLAM Toolbox.
* Planificación global de trayectoria con el algoritmo Theta*.
* Visualización de resultados en RViz y Gazebo.

## 1. Instalación y Dependencias

### 1.0 Instalar dependencias de ROS 2:
```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-velodyne-gazebo-plugins
sudo apt-get install ros-humble-velodyne-description
sudo apt install ros-humble-slam-toolbox

###1.1 Clonar y preparar el Workspace:
Bash

mkdir -p ~/go2_solorzano/src
cd ~/go2_solorzano/src
# Clona tu repositorio aquí
git clone [https://github.com/sebastiansolorzano14-bit/ProyectoSolorzano2P.git](https://github.com/sebastiansolorzano14-bit/ProyectoSolorzano2P.git) .
cd ~/go2_solorzano
rosdep install --from-paths src --ignore-src -r -y

1.2 Compilar:
Bash

colcon build
source install/setup.bash

2. Guía de Ejecución
2.1 Lanzar Simulación (Mundo Office)
Bash

ros2 launch go2_config gazebo_velodyne.launch.py world:=office

2.2 Ejecución de SLAM (Parte A)

Para generar el mapa de la oficina:

    Iniciar SLAM:

Bash

ros2 launch go2_config slam.launch.py use_sim_time:=true

    Mover el robot con el teclado:

Bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard

    Guardar el mapa (Genera .pgm y .yaml):

Bash

ros2 run nav2_map_server map_saver_cli -f ~/go2_solorzano/src/maps/go2_solorzano

2.3 Planificación Global (Parte B)

Para ejecutar el planificador Theta* desarrollado:
Bash

ros2 run go2_planner theta_star_node

    Use el botón "2D Goal Pose" en RViz para definir el objetivo. El nodo se suscribe a /map, /odom y /goal_pose para publicar el /path.

3. Algoritmo Theta*

El algoritmo implementado es Theta*, el cual es una variante de A* que optimiza la trayectoria permitiendo ángulos arbitrarios.

    Lógica: A diferencia de A*, Theta* verifica si existe línea de visión (Line-of-Sight) entre el nodo padre y el sucesor actual. Si existe, conecta los nodos directamente, eliminando el movimiento en "zigzag" típico de las rejillas de celdas.

    Variables: Utiliza una lista abierta/cerrada, costo acumulado g(n) y heurística euclidiana h(n).

4. Estructura del Proyecto
Plaintext

go2_solorzano/
├── src/
│   ├── unitree-go2-ros2/      # Paquete base y configuraciones de CHAMP
│   ├── maps/                  # Entregables Parte A (go2_solorzano.pgm/yaml)
│   └── go2_planner/           # Nodo de planificación Theta* (Parte B)
├── waypoints/                 # Waypoints generados por el planificador
└── README.md

5. Resultados y Evidencias
Parte A: Mapeo

    Archivos disponibles en la carpeta /src

    El mapa fue generado manualmente recorriendo el entorno office evitando colisiones para mantener la integridad de la odometría.

Parte B: Planificación

    El nodo publica un mensaje de tipo nav_msgs/Path.

    La trayectoria se actualiza dinámicamente cada vez que se envía un nuevo goal en RViz.



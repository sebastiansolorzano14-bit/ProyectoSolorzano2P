# Proyecto 2P: SLAM y Planificación Global - Unitree Go2 (Office)

Este repositorio contiene la configuración y el desarrollo para el robot Unitree Go2 utilizando el controlador CHAMP en ROS 2 Humble. El proyecto se enfoca en el mapeo de un entorno de oficina y la implementación de un planificador de trayectoria global.

**Autor:** Sebastian Solorzano

**Algoritmo Asignado:** Theta*

**Mapa Asignado:** Office (Oficina)


**Video Demostrativo:** https://youtu.be/HbYUXGEA0TM

## Descripción General
En este proyecto se implementa un planificador de trayectorias globales para el robot cuadrúpedo Unitree Go2 en simulación, incluyendo:
* Mapeo del entorno (SLAM) con SLAM Toolbox.
* Planificación global de trayectoria con el algoritmo Theta*.
* Visualización de resultados en RViz y Gazebo.

## 1. Instalación y Dependencias

### 1.0 Instalar dependencias de ROS 2:

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

mkdir -p ~/go2_solorzano/src

cd ~/go2_solorzano/src

# Clona tu repositorio aquí

git clone [https://github.com/sebastiansolorzano14-bit/ProyectoSolorzano2P.git](https://github.com/sebastiansolorzano14-bit/ProyectoSolorzano2P.git) .
cd ~/go2_solorzano

rosdep install --from-paths src --ignore-src -r -y


###1.2 Compilar:
colcon build

source install/setup.bash


## 2. Guía de Ejecución


###2.1 Lanzar Simulación (Mundo Office)

ros2 launch go2_config gazebo_velodyne.launch.py world:=office


###2.2 Ejecución de SLAM 

Para generar el mapa de la oficina:


ros2 launch go2_config slam.launch.py use_sim_time:=true


Mover el robot con el teclado:


ros2 run teleop_twist_keyboard teleop_twist_keyboard


Guardar el mapa (Genera .pgm y .yaml):

ros2 run nav2_map_server map_saver_cli -f ~/go2_solorzano/src

## 2. Archivos:
El mapa genereado se encuentra en el src/unitree-go2-ros2 con el nombre:
go2_solorzano.pgm para la imagen  
Su archivo yaml :  go2_solorzano.yaml


## 3. Planificación global de trayectoria

Este proyecto implementa un sistema completo de planificación global de trayectorias en ROS 2 utilizando el algoritmo Theta*, integrado al framework Navigation2 (Nav2) y visualizado en RViz sobre un mapa generado con SLAM en Gazebo.

Flujo del sistema

El sistema opera bajo el siguiente flujo:

El robot TurtleBot3 simulado en Gazebo publica su odometría en el tópico:

/odom

El mapa del entorno (oficina) es cargado por Nav2 desde un archivo YAML:
/map

El usuario define la posición inicial del robot en RViz usando:

2D Pose Estimate

El usuario define un objetivo usando:

2D Nav Goal

El planificador global Theta* se ejecuta dentro del servidor de planificación de Nav2. Este nodo:

Se suscribe a:

Pose actual del robot (/odom → TF)

Mapa (/map)

Goal del usuario

Ejecuta:

Algoritmo Theta* para planificación global

Publica:

nav_msgs/Path en el tópico:

/plan

La trayectoria calculada se visualiza en RViz como una línea que conecta la posición del robot con el objetivo, respetando los obstáculos del mapa.

Cada vez que el usuario define un nuevo “2D Nav Goal”, el sistema recalcula automáticamente la trayectoria y actualiza el Path publicado.


## Registro de puntos de referencia 

El planificador global (Theta*) publica la trayectoria planificada como `nav_msgs/Path` en el tema `/plan`.

Un nodo ROS 2 personalizado (`path_logger`) se suscribe a este tema y guarda automáticamente los puntos de referencia generados en el disco cada vez que se envía un nuevo objetivo 2D desde RViz.

### Cómo funciona
- Tema de entrada: `/plan` (nav_msgs/Path)
- Para cada nuevo plan:
- Se extrae cada pose (x, y, guiñada)
- Se crea un archivo CSV

Los archivos se guardan en:

~/go2_solorzano/waypoints/

Cada archivo contiene:

x, y, guiñada

que representan la trayectoria global completa generada por el planificador Theta*.

Esto cumple con el requisito de entregar los archivos de puntos de referencia generados por el planificador global.

## 4.Guia de uso del planificador
Lanzar Gazebo con el mundo Office

En una terminal nueva:

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch gazebo_ros gazebo.launch.py \world:=/home/erick/go2_solorzano/src/unitree-go2-ros2/robots/configs/go2_config/worlds/office/office.worl

Esto inicia Gazebo con el mapa de oficina.

Spawn del TurtleBot3
En otra terminal:

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py \x_pose:=2.0 y_pose:=1.5 z_pose:=0.01 yaw:=1.57

Publicar TF del LiDAR (base_link → base_scan)

El TurtleBot no publica correctamente el TF del láser en este mundo, así que se crea manualmente:

source /opt/ros/humble/setup.bash

ros2 run tf2_ros static_transform_publisher \0 0 0.15 0 0 0 \base_link base_scan

Esto conecta el sensor LiDAR al robot.

Publicar TF del robot (base_footprint → base_link)

Nav2 necesita la cadena:

source /opt/ros/humble/setup.bash

ros2 run tf2_ros static_transform_publisher \
0 0 0 0 0 0 \
base_footprint base_link


Esto arregla el árbol TF para Nav2 y RViz.

Lanzar Navegación + Theta*

source /opt/ros/humble/setup.bash

source ~/go2_solorzano/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \params_file:=/home/erick/go2_solorzano/src/unitree-go2-ros2/champ/champ_config/config/autonomy/navigation.yaml

Este archivo ya tiene registrado el plugin:

theta_star_planner::ThetaStarPlanner, como planificador global de Nav2.

Abrir RViz

source /opt/ros/humble/setup.bash

rviz2

En RViz agregar:

Map

RobotModel

TF

Path

Y usar:

2D Pose Estimate

2D Nav Goal

Registro de Waypoints (Path Logger)

El paquete path_logger cumple con el requerimiento de la tarea.

Este nodo:

Se suscribe a: /plan
Cada vez que se envía un nuevo 2D Nav Goal

Extrae todos los puntos del Path

Guarda un archivo CSV con los waypoints

📂 Los archivos se guardan en: ~/go2_solorzano/waypoints/
en forma x, y, yaw






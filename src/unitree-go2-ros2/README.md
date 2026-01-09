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
El mapa genereado se encuentra en el src con el nombre go2_solorzano.pgm y su archivo go2_solorzano.yaml





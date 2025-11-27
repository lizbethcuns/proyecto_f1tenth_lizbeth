Descripción del Proyecto

Este proyecto implementa un controlador reactivo tipo Follow The Gap (FTG) para un vehículo autónomo F1Tenth utilizando ROS 2 Humble.
El controlador analiza los datos del sensor LIDAR para identificar el mayor espacio despejado (gap) y dirigir el vehículo hacia ese punto manteniendo una navegación segura.

El proyecto incluye:

Controlador Follow The Gap

Contador automático de vueltas

Cronómetro por vuelta

Integración con el simulador oficial F1Tenth

Compatibilidad con el mapa Budapest

Estructura del Código
liz_controlador_v2/
 ├── liz_controlador_v2/
 │     ├── seguidor_gap_node.py          # Controlador Follow the Gap
 │     ├── contador_vueltas_node.py      # Contador + cronómetro
 │     └── __init__.py
 ├── setup.py
 ├── package.xml

Instrucciones de Ejecución
1. Construir el workspace
cd ~/f1tenth_liz_ws
colcon build
source install/setup.bash

2. Lanzar el simulador con el mapa Budapest
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:=Budapest

3. Ejecutar el controlador Follow The Gap
ros2 run liz_controlador_v2 seguidor_gap

4. Ejecutar el contador de vueltas
ros2 run liz_controlador_v2 contador_vueltas


Resultados:
Aun que no se tuvo los resultados esperados que fue que el vehiculo complete todo el circuito y de las 10 vueltas en el menor tiempo,uno de los logros que se obtuvo es lograr cargar el simulador con el mapa requerido, y aparte que el auto logre estar centrado en el punto que yo lo defini, aparte tambien se logro 

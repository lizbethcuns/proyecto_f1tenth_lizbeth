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

## Enfoque Técnico: Follow the Gap (FTG)

El controlador usa el algoritmo **Follow the Gap** para navegación reactiva:
- **Entrada**: Datos del LiDAR (/scan) con 1080 rayos (ángulos -135° a 135°).
- **Preprocesamiento**: Limpia NaN/Inf a 30m, aplica disparity extender para evitar "paredes falsas" en curvas cerradas.
- **Burbuja de seguridad**: Crea un área prohibida de 160 rayos alrededor del obstáculo más cercano (min(ranges)).
- **Detección de gaps**: Encuentra segmentos continuos donde ranges > 0.2m; selecciona el más largo (mínimo 80 rayos para seguridad).
- **Punto objetivo**: En el gap más grande, elige el punto más lejano pero sesgado al centro (np.argsort[-15:] y argmin a centro) para curvas suaves.
- **Salida**: Comando AckermannDriveStamped (/drive) con velocidad adaptativa (5.8m/s en recta, 2.5m/s en curvas >28°) y steering limitado a ±34°.

## Estructura del Código

- `seguidor_gap_node.py`: Nodo principal FTG (callback_lidar procesa scan, publica drive).
  - `__init__`: Suscribe /scan, publica /drive.
  - `callback_lidar`: Preprocesa, burbuja, gap, punto objetivo, velocidad, publica.
- `contador_vueltas_node.py`: Registra meta inicial (x=57.21, y=74.95 para Budapest), cuenta vueltas al cruzar (distancia < 0.5m), mide tiempo por vuelta con self.get_clock().
  - Muestra: "Vuelta X completada en YY.YY s".
  - Envía /stop_flag después de 10 vueltas.
- Dependencias: sensor_msgs, ackermann_msgs, std_msgs, numpy, math.


## Instrucciones de Ejecución Detalladas

1. Clona el repo: `git clone https://github.com/lizbethcuns/proyecto_f1tenth_lizbeth.git`
2. Entra al workspace: `cd proyecto_f1tenth_lizbeth`
3. Instala dependencias: `rosdep install --from-paths src --ignore-src -r -y`
4. Compila: `colcon build --packages-select liz_controlador_v2`
5. Source: `source install/setup.bash`
6. Lanza simulador (Budapest): `ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:=Budapest`
7. Lanza FTG: `ros2 run liz_controlador_v2 seguidor_gap`
8. Lanza contador: `ros2 run liz_controlador_v2 contador_vueltas`

Resultados:
Aun que no se tuvo los resultados esperados que fue que el vehiculo complete todo el circuito y de las 10 vueltas en el menor tiempo,uno de los logros que se obtuvo es lograr cargar el simulador con el mapa requerido, y aparte que el auto logre estar centrado en el punto que yo lo defini, aparte tambien se logro

## Resultados y Evidencias
- Video:https://youtu.be/K_hcmxP4gAg

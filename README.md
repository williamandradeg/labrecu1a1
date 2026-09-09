# Laboratorio de Robótica
## Universidad Europea de Madrid
### Recuperación Práctica de laboratorio Actividad 1
#### William Andrade González - Profesor: Lisandro Puglisi

## Celda robótica UR3 transportador UR5 MiR100

El sistema consiste en una celda robotica que consta de un brazo robótico UR3 que se encuentra sobre una mesa, toma una caja con una ventosa cono efector y la pone sobre una banda transportdora. La caja es transportada y al llegar al final la toma un brazo robótico UR5 y la pone sobre un MIR, que se desplaza en línea recta hasta una mesa.

Para su desarrollo, ha sido utilizado virtual box y allí se ha creado un Workspace local para Ubuntu 24.04, ROS 2 Jazzy y Gazebo Harmonic. Los brazos proceden de ur_description de Universal Robots. La malla del MiR100 procede de DFKI, proyecto comunitario independiente de Mobile Industrial Robots.

## Inicio rápido

En una terminal de Ubuntu:

~~~bash
cd ~/lrrecupera
./iniciar.sh
~~~

Esperar a que ambos controladores estén activos (normalmente 15–30 segundos). En otra terminal:

~~~bash
cd ~/lrrecupera
./ejecutar_ciclo.sh
~~~

El proceso recoge una caja con UR3, la deposita sobre la banda, la transporta hasta UR5 y la carga sobre MiR100. El móvil avanza en línea recta y se detiene frente a la mesa final. Al terminar conserva la escena. Para repetir desde el estado inicial, ejecutar de nuevo ./iniciar.sh y después ./ejecutar_ciclo.sh.

./detener.sh cierra exclusivamente procesos identificados con la partición de esta celda. No borra archivos. 

## Compilación

~~~bash
cd ~/lrrecupera
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select celda_robotica
~~~

En una instalación nueva se necesitan los paquetes Jazzy de UR description, UR simulation GZ, gz_ros2_control, controller_manager, joint_state_broadcaster, joint_trajectory_controller, ros_gz_bridge, robot_state_publisher, xacro, los paquetes de desarrollo de Gazebo Harmonic, python3-scipy y python3-pil. 

generate_scene.py regenera las descripciones URDF, el mundo y los puntos articulares mediante cinemática inversa numérica. Se ejecuta desde el entorno Jazzy. 

## Arquitectura

- src/celda_robotica/worlds/celda.sdf: escena completa.
- urdf/ur3.urdf, urdf/ur5.urdf: modelos oficiales parametrizados, con ventosa agregada.
- config/*controllers.yaml: control de posición mediante ROS 2 control.
- config/waypoints.json: soluciones de cinemática inversa, pose objetivo y error numérico.
- scripts/cycle.py: coordinador secuencial y comprobaciones de proceso.
- src/CellEnvironment.cc: accionamiento ideal de banda, unión de ventosa y telemetría.
- entorno.sh: selección de Jazzy, biblioteca de control, dominio ROS 81 y partición exclusiva.

## Interfaces

Acciones:
- /ur3/joint_trajectory_controller/follow_joint_trajectory
- /ur5/joint_trajectory_controller/follow_joint_trajectory

ROS: /celda/estado, /clock, estados articulares bajo /ur3 y /ur5.
Gazebo Transport: /celda/poses, /celda/belt/run, /celda/mir/cmd_vel, /celda/camera.
Ventosas: /celda/{ur3,ur5}/attach, /detach, /attached.
MiR: /celda/mir/attach representa una retención ideal de la carga durante el desplazamiento.

## Fuentes

- https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
- https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation
- https://gazebosim.org/api/sim/8/detachablejoints.html
- https://github.com/DFKI-NI/mir_robot
- https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html

Licencias y procedencia en licenses/.

## Vídeo

El video de la ejecución de la celda robótica se encuentra en: 
https://youtu.be/zt8813a18H8
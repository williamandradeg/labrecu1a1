#!/usr/bin/env bash
set -e
cd "$HOME/lrrecupera"
source /opt/ros/jazzy/setup.bash
for pkg in ur_description ur_simulation_gz gz_ros2_control controller_manager joint_trajectory_controller joint_state_broadcaster robot_state_publisher ros_gz_bridge; do
 ros2 pkg prefix "$pkg" > /dev/null || { echo "Falta el paquete ROS: $pkg"; exit 1; }
done
python3 -c 'import scipy; from gz.transport13 import Node; from PIL import Image'
colcon build --symlink-install --packages-select celda_robotica
echo 'La Compilacion ha finalizado. Ejecutar en este orden ./iniciar.sh esperamos unos 30 segundos o un minuto y cuando veamos que gazebo ya cargó completo ejecutamos en otra terminal ./ejecutar_ciclo.sh'

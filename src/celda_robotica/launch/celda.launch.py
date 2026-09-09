from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction
from launch_ros.actions import Node
from pathlib import Path
import os

def generate_launch_description():
    root=Path.home()/'lrrecupera'
    p=root/'src/celda_robotica'
    actions=[ExecuteProcess(cmd=['gz','sim','-s','-r',os.environ.get('CELDA_WORLD',str(p/'worlds/celda.sdf'))],output='screen')]
    for typ in ['ur3','ur5']:
        actions.append(Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=typ,parameters=[{'robot_description':(p/'urdf'/f'{typ}.urdf').read_text(),'use_sim_time':True}],output='screen'))
        actions.append(TimerAction(period=5.,actions=[Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','joint_trajectory_controller','-c',f'/{typ}/controller_manager','--controller-manager-timeout','120','--service-call-timeout','120','--switch-timeout','120'],output='screen')]))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],output='screen'))
    return LaunchDescription(actions)

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Forzamos la carga de cada archivo de configuración
    moveit_config = (
        MoveItConfigsBuilder("cr20_robot", package_name="cr20_moveit")
        .robot_description(file_path="config/cr20_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/cr20_robot.srdf") # <--- Asegurar SRDF
        .robot_description_kinematics(file_path="config/kinematics.yaml") # <--- Asegurar Cinemática
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 1. Nodo Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    # 2. Rviz2
    rviz_config = os.path.join(get_package_share_directory("cr20_moveit"), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    # 3. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}]
    )

    return LaunchDescription([rsp, move_group_node, rviz_node])
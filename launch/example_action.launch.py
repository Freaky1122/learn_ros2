from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数,由ros2 launch 扫描调用"""
    action_robot_01 = Node(
        package="learn_ros2",
        executable="action_robot_node"
    )
    action_control_01 = Node(
        package="learn_ros2",
        executable="action_controller_node"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [action_robot_01, action_control_01])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
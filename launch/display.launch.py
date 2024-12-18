from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Find the package share directory
    pkg_share = FindPackageShare("baxter_ros2_simulation").find("baxter_ros2_simulation")
    urdf_dir = os.path.join(pkg_share, "urdf")
    
    # Use baxter.rviz config
    rviz_config_file = os.path.join(pkg_share, "config", "baxter.rviz")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([urdf_dir, "baxter.urdf.xacro"]), " ",
            "gripper_enabled:=true"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Create a joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # Create an rviz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else None,
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add any actions
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    balltze_description = get_package_share_directory("agimus_demo_05_pick_and_place")
    xacro_file = PathJoinSubstitution(
        [
            "/home/gepetto/ros2_ws/src/agimus-demos/agimus_demo_05_pick_and_place/agimus_demo_05_pick_and_place/urdf/demo.urdf.xacro"
        ]
    )
    rviz_config = PathJoinSubstitution(
        [balltze_description, "rviz", "model_preveiw.rviz"]
    )

    robot_description = {
        "robot_description": Command(
            ["xacro --verbosity 0 ", xacro_file, " use_sim:=true", " use_gpu:=false"]
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=rviz_config,
                description="Absolute path to rviz config file",
            ),
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )

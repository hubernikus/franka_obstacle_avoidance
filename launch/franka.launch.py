from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_argument = DeclareLaunchArgument(
        "model",
        description="URDF/XACRO description file with the robot.",
        default_value="panda_arm",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_panda_description"),
                    "robots/",
                    LaunchConfiguration("model"),
                ]
            ),
            ".urdf.xacro",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_obstacle_avoidance"),
                    "config/franka_obstacle.rviz",
                ]
            ),
        ],
        output="log",
    )

    nodes = [
        robot_state_pub_node,
        joint_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription([launch_argument] + nodes)

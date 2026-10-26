from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    gripper_port = LaunchConfiguration("gripper_port")
    tcp_port = LaunchConfiguration("tcp_port")
    twist_command_topic = LaunchConfiguration("twist_command_topic")
    twist_command_safe_topic = LaunchConfiguration("twist_command_safe_topic")
    gripper_command_topic = LaunchConfiguration("gripper_command_topic")
    enable_state_feedback = LaunchConfiguration("enable_state_feedback")
    enable_gripper = LaunchConfiguration("enable_gripper")
    send_hz = LaunchConfiguration("send_hz")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")
    acceleration = LaunchConfiguration("acceleration")
    command_duration = LaunchConfiguration("command_duration")
    stale_timeout_sec = LaunchConfiguration("stale_timeout_sec")
    connect_timeout_sec = LaunchConfiguration("connect_timeout_sec")
    deadband = LaunchConfiguration("deadband")
    linear_x_sign = LaunchConfiguration("linear_x_sign")
    linear_y_sign = LaunchConfiguration("linear_y_sign")
    linear_z_sign = LaunchConfiguration("linear_z_sign")
    angular_x_sign = LaunchConfiguration("angular_x_sign")
    angular_y_sign = LaunchConfiguration("angular_y_sign")
    angular_z_sign = LaunchConfiguration("angular_z_sign")
    gripper_open_position = LaunchConfiguration("gripper_open_position")
    gripper_close_position = LaunchConfiguration("gripper_close_position")
    gripper_speed = LaunchConfiguration("gripper_speed")
    gripper_force = LaunchConfiguration("gripper_force")

    ur_driver_launch = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("robot_port", default_value="30002"),
            DeclareLaunchArgument("gripper_port", default_value="63352"),
            DeclareLaunchArgument("enable_state_feedback", default_value="true"),
            DeclareLaunchArgument("enable_gripper", default_value="true"),
            DeclareLaunchArgument("tcp_port", default_value="10000"),
            DeclareLaunchArgument("send_hz", default_value="8.0"),
            DeclareLaunchArgument("max_linear_speed", default_value="0.020"),
            DeclareLaunchArgument("max_angular_speed", default_value="0.0"),
            DeclareLaunchArgument("acceleration", default_value="0.10"),
            DeclareLaunchArgument("command_duration", default_value="0.14"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="0.18"),
            DeclareLaunchArgument("connect_timeout_sec", default_value="0.06"),
            DeclareLaunchArgument("deadband", default_value="0.002"),
            DeclareLaunchArgument("linear_x_sign", default_value="-1.0"),
            DeclareLaunchArgument("linear_y_sign", default_value="-1.0"),
            DeclareLaunchArgument("linear_z_sign", default_value="1.0"),
            DeclareLaunchArgument("angular_x_sign", default_value="1.0"),
            DeclareLaunchArgument("angular_y_sign", default_value="1.0"),
            DeclareLaunchArgument("angular_z_sign", default_value="1.0"),
            DeclareLaunchArgument("gripper_open_position", default_value="0"),
            DeclareLaunchArgument("gripper_close_position", default_value="255"),
            DeclareLaunchArgument("gripper_speed", default_value="180"),
            DeclareLaunchArgument("gripper_force", default_value="20"),
            DeclareLaunchArgument(
                "gripper_command_topic",
                default_value="/team_1/gripper_command",
            ),
            DeclareLaunchArgument("twist_command_topic", default_value="/team_1/twist_command"),
            DeclareLaunchArgument(
                "twist_command_safe_topic",
                default_value="/team_1/twist_command_safe",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_driver_launch),
                condition=IfCondition(enable_state_feedback),
                launch_arguments={
                    "ur_type": "ur3",
                    "robot_ip": robot_ip,
                    "headless_mode": "true",
                    "launch_dashboard_client": "false",
                    "launch_rviz": "false",
                    "activate_joint_controller": "false",
                }.items(),
            ),
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="unity_tcp_endpoint",
                output="screen",
                parameters=[
                    {"ROS_IP": "0.0.0.0"},
                    {"ROS_TCP_PORT": tcp_port},
                ],
            ),
            Node(
                package="teleop_core",
                executable="twist_command_monitor",
                name="twist_command_monitor",
                output="screen",
                parameters=[
                    {"input_topic": twist_command_topic},
                    {"output_topic": twist_command_safe_topic},
                    {"safety_status_topic": "/team_1/safety_status"},
                    {"max_linear_speed": max_linear_speed},
                    {"max_angular_speed": max_angular_speed},
                    {"log_every_n": 10},
                ],
            ),
            Node(
                package="teleop_core",
                executable="ur_speedl_wifi_bridge",
                name="ur_speedl_wifi_bridge",
                output="screen",
                parameters=[
                    {"robot_ip": robot_ip},
                    {"robot_port": robot_port},
                    {"input_topic": twist_command_safe_topic},
                    {"send_hz": send_hz},
                    {"max_linear_speed": max_linear_speed},
                    {"max_angular_speed": max_angular_speed},
                    {"acceleration": acceleration},
                    {"command_duration": command_duration},
                    {"stale_timeout_sec": stale_timeout_sec},
                    {"connect_timeout_sec": connect_timeout_sec},
                    {"deadband": deadband},
                    {"linear_x_sign": linear_x_sign},
                    {"linear_y_sign": linear_y_sign},
                    {"linear_z_sign": linear_z_sign},
                    {"angular_x_sign": angular_x_sign},
                    {"angular_y_sign": angular_y_sign},
                    {"angular_z_sign": angular_z_sign},
                    {"log_every_n": 10},
                ],
            ),
            Node(
                package="teleop_core",
                executable="robotiq_gripper_bridge",
                name="robotiq_gripper_bridge",
                output="screen",
                condition=IfCondition(enable_gripper),
                parameters=[
                    {"robot_ip": robot_ip},
                    {"robot_port": gripper_port},
                    {"input_topic": gripper_command_topic},
                    {"open_position": gripper_open_position},
                    {"close_position": gripper_close_position},
                    {"speed": gripper_speed},
                    {"force": gripper_force},
                ],
            ),
        ]
    )

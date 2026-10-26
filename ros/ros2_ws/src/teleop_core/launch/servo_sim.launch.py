from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _load_yaml(path: Path) -> dict:
    import yaml

    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description() -> LaunchDescription:
    tcp_port = LaunchConfiguration("tcp_port")
    twist_command_topic = LaunchConfiguration("twist_command_topic")
    twist_command_safe_topic = LaunchConfiguration("twist_command_safe_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    trajectory_topic = LaunchConfiguration("trajectory_topic")

    teleop_share = Path(get_package_share_directory("teleop_core"))
    srdf_path = teleop_share / "config" / "ur3_workshop.srdf"
    kinematics_path = teleop_share / "config" / "kinematics.yaml"
    joint_limits_path = teleop_share / "config" / "joint_limits.yaml"
    servo_path = teleop_share / "config" / "ur3_servo.yaml"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "ur_type:=ur3",
            " ",
            "name:=ur3",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    robot_description_semantic = {
        "robot_description_semantic": _load_text(srdf_path)
    }
    kinematics_yaml = _load_yaml(kinematics_path)
    joint_limits_yaml = _load_yaml(joint_limits_path)
    servo_yaml = _load_yaml(servo_path)
    servo_yaml["moveit_servo"]["scale"]["rotational"] = 0.2

    return LaunchDescription(
        [
            DeclareLaunchArgument("tcp_port", default_value="10000"),
            DeclareLaunchArgument("twist_command_topic", default_value="/team_1/twist_command"),
            DeclareLaunchArgument(
                "twist_command_safe_topic",
                default_value="/team_1/twist_command_safe",
            ),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument(
                "trajectory_topic",
                default_value="/team_1/servo_joint_trajectory",
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
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="teleop_core",
                executable="servo_joint_state_adapter",
                name="servo_joint_state_adapter",
                output="screen",
                parameters=[
                    {"trajectory_topic": trajectory_topic},
                    {"joint_states_topic": joint_states_topic},
                    {"publish_hz": 50.0},
                    {"initial_positions": [-0.5, -1.6, 1.2, -1.2, -1.57, 0.0]},
                ],
            ),
            Node(
                package="teleop_core",
                executable="tcp_actual_publisher",
                name="tcp_actual_publisher",
                output="screen",
                parameters=[
                    {"base_frame": "base_link"},
                    {"tcp_frame": "tool0"},
                    {"output_topic": "/team_1/tcp_actual"},
                    {"publish_hz": 30.0},
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
                    {"max_linear_speed": 0.12},
                    {"max_angular_speed": 0.3},
                    {"log_every_n": 30},
                ],
            ),
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                output="screen",
                parameters=[
                    servo_yaml,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    joint_limits_yaml,
                ],
            ),
            Node(
                package="teleop_core",
                executable="servo_twist_enable",
                name="servo_twist_enable",
                output="screen",
                parameters=[
                    {"start_service": "/servo_node/start_servo"},
                    {"unpause_service": "/servo_node/unpause_servo"},
                    {"retry_period_sec": 0.5},
                ],
            ),
        ]
    )

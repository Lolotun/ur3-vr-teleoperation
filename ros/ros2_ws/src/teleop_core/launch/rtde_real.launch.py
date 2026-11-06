"""Relative pose -> pose feedback -> MoveIt Servo -> RTDE servoJ."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

from teleop_core.ur3_model import UR3_ARM_JOINTS


def robot_parameters(calibration_file='', tcp_xyz=(0., 0., 0.), tcp_rpy=(0., 0., 0.)):
    """Use one model for Servo and TF, including the configured active TCP."""
    description_share = Path(get_package_share_directory('ur_description'))
    calibration = calibration_file or str(description_share / 'config/ur3/default_kinematics.yaml')
    document = xacro.process_file(str(description_share / 'urdf/ur.urdf.xacro'),
                                  mappings={'ur_type': 'ur3', 'name': 'ur3',
                                            'kinematics_params': calibration})
    root = ET.fromstring(document.toxml())
    ET.SubElement(root, 'link', name='teleop_tcp')
    joint = ET.SubElement(root, 'joint', name='teleop_tcp_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='tool0')
    ET.SubElement(joint, 'child', link='teleop_tcp')
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, tcp_xyz)), rpy=' '.join(map(str, tcp_rpy)))
    share = Path(get_package_share_directory('teleop_core'))
    semantic = ET.fromstring((share / 'config/ur3_workshop.srdf').read_text())
    semantic.find('group/chain').set('tip_link', 'teleop_tcp')
    semantic.remove(semantic.find('end_effector'))  # The arm itself is not a separate gripper group.
    limits = yaml.safe_load((share / 'config/joint_limits.yaml').read_text())
    lower, upper, velocity, continuous = [], [], [], []
    for name in UR3_ARM_JOINTS:
        joint = root.find(f"joint[@name='{name}']")
        bound = joint.find('limit')
        continuous.append(joint.get('type') == 'continuous')
        # Continuous joints have no URDF lower/upper bounds; the executor skips
        # these placeholders and unwraps their targets around actual feedback.
        lower.append(float(bound.get('lower', '-3.141592653589793')))
        upper.append(float(bound.get('upper', '3.141592653589793')))
        velocity.append(min(float(bound.get('velocity')), limits['joint_limits'][name]['max_velocity']))
        limits['joint_limits'][name]['max_velocity'] = velocity[-1]
    model = {'robot_description': ET.tostring(root, encoding='unicode'),
             'robot_description_semantic': ET.tostring(semantic, encoding='unicode'),
             'robot_description_kinematics': yaml.safe_load((share / 'config/kinematics.yaml').read_text()),
             'robot_description_planning': limits}
    return model, {'joint_lower_limits': lower, 'joint_upper_limits': upper,
                   'joint_velocity_limits': velocity, 'joint_continuous': continuous}


def launch_nodes(context):
    def value(name, kind=float):
        return kind(LaunchConfiguration(name).perform(context))

    frequency = value('frequency')
    if not math.isfinite(frequency) or not 1 <= frequency <= 125:
        raise ValueError('UR3 CB3 frequency must be in 1..125 Hz')
    def vector(name):
        result = yaml.safe_load(value(name, str))
        if not isinstance(result, list) or len(result) != 3 or not all(
                isinstance(x, (float, int)) and math.isfinite(x) for x in result):
            raise ValueError(f'{name} must contain three finite numbers')
        return [float(x) for x in result]

    model, execution_limits = robot_parameters(value('kinematics_params_file', str),
                                               vector('tcp_xyz'), vector('tcp_rpy'))
    share = Path(get_package_share_directory('teleop_core'))
    servo = yaml.safe_load((share / 'config/ur3_rtde_servo.yaml').read_text())
    servo['moveit_servo']['publish_period'] = 1 / frequency
    servo['moveit_servo']['incoming_command_timeout'] = value('trajectory_timeout')
    servo['moveit_servo']['lower_singularity_threshold'] = value('lower_singularity_threshold')
    servo['moveit_servo']['hard_stop_singularity_threshold'] = value('hard_stop_singularity_threshold')
    common = {name: value(name) for name in ['frequency', 'command_timeout', 'feedback_timeout']}
    launch_model = {**model,
                    'robot_description': ParameterValue(model['robot_description'], value_type=str),
                    'robot_description_semantic': ParameterValue(model['robot_description_semantic'], value_type=str)}
    return [
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint', name='unity_tcp_endpoint',
             condition=IfCondition(LaunchConfiguration('start_endpoint')),
             parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': value('tcp_port', int)}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': launch_model['robot_description'], 'publish_frequency': frequency}],
             output='screen'),
        Node(package='teleop_core', executable='relative_pose_controller', output='screen', parameters=[{
            name: value(name) for name in ['position_scale', 'mapping_yaw_deg', 'max_displacement',
                                          'max_rotation', 'feedback_timeout']}]),
        Node(package='teleop_core', executable='pose_tracking_controller', output='screen', parameters=[{
            **common, **{name: value(name) for name in ['max_linear_speed', 'max_angular_speed',
                                                       'position_gain', 'rotation_gain']}}]),
        Node(package='moveit_servo', executable='servo_node_main', name='servo_node',
             output='screen', parameters=[launch_model, servo]),
        Node(package='teleop_core', executable='servo_twist_enable', output='screen', parameters=[{
            'start_service': '/servo_node/start_servo', 'unpause_service': ''}]),
        Node(package='teleop_core', executable='ur_rtde_bridge', output='screen', parameters=[{
            **common, **execution_limits, 'robot_ip': value('robot_ip', str),
            'enable_motion': value('enable_motion', str).lower() == 'true',
            **{name: value(name) for name in ['trajectory_timeout', 'max_joint_tracking_error',
                                             'lead_soft_start', 'lead_soft_limit',
                                             'lookahead_time', 'gain']}}]),
        Node(package='teleop_core', executable='robotiq_gripper_bridge',
             condition=IfCondition(LaunchConfiguration('enable_gripper')), output='screen', parameters=[{
                 'robot_ip': value('robot_ip', str), 'robot_port': value('gripper_port', int),
                 'force': value('gripper_force', int)}]),
    ]


def generate_launch_description():
    defaults = {'robot_ip': '', 'tcp_port': '10000', 'start_endpoint': 'true',
                'enable_motion': 'false', 'enable_gripper': 'false', 'gripper_port': '63352',
                'gripper_force': '20', 'frequency': '125.0', 'position_scale': '1.0',
                'mapping_yaw_deg': '180.0', 'max_displacement': '0.25', 'max_rotation': '1.0',
                'max_linear_speed': '0.020', 'max_angular_speed': '0.12',
                'position_gain': '2.0', 'rotation_gain': '2.0',
                'command_timeout': '0.25', 'feedback_timeout': '0.25', 'trajectory_timeout': '0.1',
                'max_joint_tracking_error': '0.15', 'lookahead_time': '0.1', 'gain': '300.0',
                'lead_soft_start': '0.06', 'lead_soft_limit': '0.10',
                'kinematics_params_file': '', 'tcp_xyz': '[0.0, 0.0, 0.0]', 'tcp_rpy': '[0.0, 0.0, 0.0]',
                'lower_singularity_threshold': '17.0', 'hard_stop_singularity_threshold': '30.0'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=launch_nodes)])

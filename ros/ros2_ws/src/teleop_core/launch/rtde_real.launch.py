"""RTDE pose teleoperation. Read-only preview unless enable_motion is explicit."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults={'robot_ip':'','tcp_port':'10000','enable_motion':'false','enable_gripper':'false',
              'gripper_port':'63352','gripper_force':'20','frequency':'125.0',
              'position_scale':'1.0','mapping_yaw_deg':'180.0','max_displacement':'0.25',
              'max_rotation':'1.0','max_linear_speed':'0.020','max_angular_speed':'0.12',
              'command_timeout':'0.25','feedback_timeout':'0.25','lookahead_time':'0.1','gain':'300.0'}
    def value(name,kind): return ParameterValue(LaunchConfiguration(name),value_type=kind)
    actions=[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
    actions += [
        Node(package='ros_tcp_endpoint',executable='default_server_endpoint',name='unity_tcp_endpoint',
             parameters=[{'ROS_IP':'0.0.0.0','ROS_TCP_PORT':value('tcp_port',int)}],output='screen'),
        Node(package='teleop_core',executable='relative_pose_controller',output='screen',parameters=[{
            k:value(k,float) for k in ['position_scale','mapping_yaw_deg','max_displacement',
                                     'max_rotation','feedback_timeout']}]),
        Node(package='teleop_core',executable='ur_rtde_bridge',output='screen',parameters=[{
            'robot_ip':value('robot_ip',str),'enable_motion':value('enable_motion',bool),
            **{k:value(k,float) for k in ['frequency','max_linear_speed','max_angular_speed',
               'command_timeout','feedback_timeout','lookahead_time','gain']}}]),
        Node(package='teleop_core',executable='robotiq_gripper_bridge',
             condition=IfCondition(LaunchConfiguration('enable_gripper')),output='screen',parameters=[{
                 'robot_ip':value('robot_ip',str),'robot_port':value('gripper_port',int),
                 'force':value('gripper_force',int)}]),
    ]
    return LaunchDescription(actions)

#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

case "${1:-wifi_speedl}" in
  servo_sim)
    exec ros2 launch teleop_core servo_sim.launch.py \
      tcp_port:=${ROS_TCP_PORT:-10000} \
      twist_command_topic:=${TWIST_COMMAND_TOPIC:-/team_1/twist_command} \
      twist_command_safe_topic:=${TWIST_COMMAND_SAFE_TOPIC:-/team_1/twist_command_safe} \
      joint_states_topic:=${JOINT_STATES_TOPIC:-/joint_states} \
      trajectory_topic:=${SERVO_TRAJECTORY_TOPIC:-/team_1/servo_joint_trajectory}
    ;;
  rtde_real)
    exec ros2 launch teleop_core rtde_real.launch.py \
      robot_ip:=${ROBOT_IP:?Set ROBOT_IP} \
      tcp_port:=${ROS_TCP_PORT:-10000} \
      enable_motion:=${RTDE_ENABLE_MOTION:-false} \
      pose_check_mode:=${RTDE_POSE_CHECK_MODE:-robot} \
      enable_gripper:=${RTDE_ENABLE_GRIPPER:-false} \
      gripper_port:=${GRIPPER_PORT:-63352} \
      gripper_force:=${GRIPPER_FORCE:-20} \
      frequency:=${RTDE_FREQUENCY:-125.0} \
      position_scale:=${RTDE_POSITION_SCALE:-1.0} \
      mapping_yaw_deg:=${RTDE_MAPPING_YAW_DEG:-180.0} \
      max_displacement:=${RTDE_MAX_DISPLACEMENT:-0.25} \
      max_rotation:=${RTDE_MAX_ROTATION:-1.0} \
      max_linear_speed:=${RTDE_MAX_LINEAR_SPEED:-0.020} \
      max_angular_speed:=${RTDE_MAX_ANGULAR_SPEED:-0.12} \
      command_timeout:=${RTDE_COMMAND_TIMEOUT:-0.25} \
      feedback_timeout:=${RTDE_FEEDBACK_TIMEOUT:-0.25} \
      lookahead_time:=${RTDE_LOOKAHEAD_TIME:-0.1} \
      gain:=${RTDE_GAIN:-300.0}
    ;;
  wifi_speedl)
    exec ros2 launch teleop_core wifi_speedl.launch.py \
      robot_ip:=${ROBOT_IP:?Set ROBOT_IP to the UR controller IP address} \
      robot_port:=${ROBOT_PORT:-30002} \
      gripper_port:=${GRIPPER_PORT:-63352} \
      enable_gripper:=${ENABLE_GRIPPER:-true} \
      enable_state_feedback:=${SPEEDL_ENABLE_STATE_FEEDBACK:-true} \
      tcp_port:=${ROS_TCP_PORT:-10000} \
      send_hz:=${SPEEDL_SEND_HZ:-8.0} \
      max_linear_speed:=${SPEEDL_MAX_LINEAR_SPEED:-0.020} \
      max_angular_speed:=${SPEEDL_MAX_ANGULAR_SPEED:-0.0} \
      acceleration:=${SPEEDL_ACCELERATION:-0.10} \
      command_duration:=${SPEEDL_COMMAND_DURATION:-0.14} \
      stale_timeout_sec:=${SPEEDL_STALE_TIMEOUT_SEC:-0.18} \
      connect_timeout_sec:=${SPEEDL_CONNECT_TIMEOUT_SEC:-0.06} \
      deadband:=${SPEEDL_DEADBAND:-0.002} \
      linear_x_sign:=${SPEEDL_LINEAR_X_SIGN:--1.0} \
      linear_y_sign:=${SPEEDL_LINEAR_Y_SIGN:--1.0} \
      linear_z_sign:=${SPEEDL_LINEAR_Z_SIGN:-1.0} \
      angular_x_sign:=${SPEEDL_ANGULAR_X_SIGN:-1.0} \
      angular_y_sign:=${SPEEDL_ANGULAR_Y_SIGN:-1.0} \
      angular_z_sign:=${SPEEDL_ANGULAR_Z_SIGN:-1.0} \
      gripper_open_position:=${GRIPPER_OPEN_POSITION:-0} \
      gripper_close_position:=${GRIPPER_CLOSE_POSITION:-255} \
      gripper_speed:=${GRIPPER_SPEED:-180} \
      gripper_force:=${GRIPPER_FORCE:-20} \
      gripper_command_topic:=${GRIPPER_COMMAND_TOPIC:-/team_1/gripper_command} \
      twist_command_topic:=${TWIST_COMMAND_TOPIC:-/team_1/twist_command} \
      twist_command_safe_topic:=${TWIST_COMMAND_SAFE_TOPIC:-/team_1/twist_command_safe}
    ;;
  bash) exec /bin/bash ;;
  *) echo "Usage: entrypoint.sh [servo_sim|wifi_speedl|rtde_real|bash]" >&2; exit 1 ;;
esac

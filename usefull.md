 ./scripts/run.sh shell
 source ../opt/ros/humble/setup.bash 

 ros2 topic echo /joint_states --once

 ./scripts/run.sh build
RTDE_ENABLE_MOTION=false RTDE_ENABLE_GRIPPER=false ./scripts/run.sh rtde
./scripts/run.sh logs


 ros2 topic echo /teleop/tcp_actual


ros2 param get /pose_tracking_controller max_linear_speed
ros2 param get /pose_tracking_controller position_gain
ros2 topic echo /teleop/servo/twist --field twist.linear
ros2 topic echo /teleop/servo/twist --field twist.angular


22267010721084
---
^Croot@isr-lab-4:/ws# ^C
root@isr-lab-4:/ws# ros2 topic echo /joint_states --once
header:
  stamp:
    sec: 1793965406
    nanosec: 417023621
  frame_id: ''
name:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
position:
- 0.005314629524946213
- -1.5409448782550257
- 1.555553913116455
- -0.9605520407306116
- 1.3303171396255493
- 4.340577125549316
velocity:
- 0.0
- 0.0
- -0.0
- 0.0
- 0.0
- 0.0
effort: []
---
root@isr-lab-4:/ws# 
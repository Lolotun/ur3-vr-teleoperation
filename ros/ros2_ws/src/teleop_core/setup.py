from setuptools import setup
from glob import glob

package_name = "teleop_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UR3 VR Workshop",
    maintainer_email="workshop@example.com",
    description="UR3 VR teleoperation with virtual Servo and real socket bridges",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "twist_command_monitor = teleop_core.twist_command_monitor:main",
            "ur_speedl_wifi_bridge = teleop_core.ur_speedl_wifi_bridge:main",
            "robotiq_gripper_bridge = teleop_core.robotiq_gripper_bridge:main",
            "servo_joint_state_adapter = teleop_core.servo_joint_state_adapter:main",
            "servo_twist_enable = teleop_core.servo_twist_enable:main",
            "relative_pose_controller = teleop_core.relative_pose_controller:main",
            "ur_rtde_bridge = teleop_core.ur_rtde_bridge:main",
            "tcp_actual_publisher = teleop_core.tcp_actual_publisher:main",
        ],
    },
)

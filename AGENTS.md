# Repository guidance

- This repository contains Unity/Quest teleoperation and a ROS 2 Humble backend for UR3. Maintained backend code is in `ros/ros2_ws/src/teleop_core` and `teleop_collision`; project-specific Unity code is in `unity/Assets/Scripts`.
- Read `how_to_use.md` and the relevant documents in `docs/` before changing launch behavior. Preserve unrelated working-tree changes.
- `scripts/run.sh` wraps Docker Compose. `./scripts/run.sh build` builds the backend; runtime commands may connect to hardware. Do not launch hardware control merely to validate edits.
- Offline Python unit tests: `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=ros/ros2_ws/src/teleop_core python3 -m unittest discover -s tests -p 'test_*.py'`. Feedback tests use localhost sockets. ROS smoke tests require ROS dependencies and an isolated ROS domain without robot drivers.
- RTDE control selects STOP, VR, or POLICY through `control_mux`. Keep launch files, `setup.py`, Compose, entrypoint, and `.env.example` consistent when adding nodes or parameters. Preserve measured versus requested gripper state and mode-transition/watchdog behavior.
- `tests/policy_ros_smoke.py` also depends on policy bridge and plugin sources from the sibling `vr-dagger` repository. Do not edit that repository automatically.
- Preserve Unity `.meta` files and serialized references. Vendor assets can have indirect or runtime dependencies; do not remove them based only on text searches.
- `tmp/` is local scratch space. Keep caches, recordings, credentials, and generated builds out of Git. Commit useful experiment documentation outside `tmp/`.

# CMU's Zoë2 Rover
## Requirements
- [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
- [ROS2 Iron Irwini](https://docs.ros.org/en/iron/index.html)
- [ROS2 Control (Iron Branch)](https://control.ros.org/iron/index.html)
- [Gazebo Classic (11.0.0)](https://classic.gazebosim.org/)
## Setup
1. Install ROS2, ROS2 Control, and Gazebo classic, following their documentation.
2. Create a ROS workspace.
3. Clone this repo into the `src/` directory of your ROS workspace.
4. At the root of your workspace, run `colcon build`.
## Operation
1. Source your package using `source install/setup.bash` at the root of your workspace. This needs to be done in any new terminal instance.
2. Launch the robot in an empty world using `ros2 launch zoe2_bringup zoe2.launch.py`.
3. Alternatively, launch the robot in a custom world using `ros2 launch zoe2_bringup zoe2.launch.py world:="moon.world z:=1"`, or any other world stored in `zoe2_bringup/worlds`.
4. To control the robot, open an additional terminal and `source install/setup.bash`.
5. Then send a drive arc command: `ros2 service call /zoe_drive zoe2_interfaces/srv/DriveCommand '{drive_arc: {radius: 10, speed: 1, time: 1000, sender: "example"}}'`

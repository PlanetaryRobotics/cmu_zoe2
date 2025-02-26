# CMU's Zoë2 Rover
## Requirements
- [Ubuntu 24.04](https://releases.ubuntu.com/jammy/)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
- [ROS2 Control (Jazzy Branch)](https://control.ros.org/jazzy/index.html)
- [Gazebo Harmonic (8.8.0)](https://gazebosim.org/docs/harmonic/getstarted/)
## Setup
1. Install ROS2, ROS2 Control, and Gazebo, following their documentation. See [INSTALL.md](INSTALL.md) for more details
2. Create a ROS workspace.
3. Clone this repo into the `src/` directory of your ROS workspace.
4. At the root of your workspace, run `colcon build`.
## Operation
1. Source your package using `source install/setup.bash` at the root of your workspace. This needs to be done in any new terminal instance.
2. Launch the robot in an empty world using `ros2 launch zoe2_bringup zoe2.launch.py`.
3. Alternatively, launch the robot in a custom world using `ros2 launch zoe2_bringup zoe2.launch.py world:=moon/model.sdf z:=8`, or any other world stored in `zoe2_bringup/worlds`.
4. To control the robot, open an additional terminal and `source install/setup.bash`.
5. Launch rqt: `rqt` and open the Robot Steering plugin (Plugins -> Robot Tools -> Robot Steering)
7. Set the message topic to `/cmd_vel_unstamped`
8. Use the sliders to steer the robot!
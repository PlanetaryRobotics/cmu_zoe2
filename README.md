# CMU's Zoë2 Rover
## Requirements
- [Ubuntu 24.04](https://releases.ubuntu.com/jammy/)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
- [ROS2 Control (Jazzy Branch)](https://control.ros.org/jazzy/index.html)
- [Gazebo Harmonic (8.8.0)](https://gazebosim.org/docs/harmonic/getstarted/)
## Setup
1. Install ROS2, ROS2 Control, and Gazebo, following their documentation. See [INSTALL.md](INSTALL.md) for more details
2. If you don't already have one, create a [ROS workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). 
3. Clone this repo into the `src/` directory of your ROS workspace.
4. Initialize the submodules with `git submodule update --init`
    1. If you run into an error, you may need to [generate an SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux) and retry.
6. Run [rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html#rosdep-operation) to install all missing packages.
```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
``` 
5. At the root of your workspace, run `colcon build`.

## Launch
### Simulation Mode
1. Open a new terminal and source your package using `source install/setup.bash` at the root of your workspace. This needs to be done in any new terminal when running the program.
2. Launch the simulation in an empty world using `ros2 launch zoe2_bringup sim.launch.py`.
3. Alternatively, launch the robot in a custom world using `ros2 launch zoe2_bringup zoe2.launch.py world:=moon/model.sdf z:=8`, or any other world stored in `zoe2_bringup/worlds`.
### Hardware Mode
1. Open a new terminal and source your package using `source install/setup.bash` at the root of your workspace. This needs to be done in any new terminal when running the program.
2. Plug the Kvaser Leaf Light v2 USB into your computer. Run `lsusb` to see if it is recognized.
3. In your terminal, run `sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0` to establish communication
4. Run `source zoe2_hardware/config/mapMotors.sh` to map the motor's PDO commands. This only needs to be done once per power cycle.
5. Launch the hardware using `ros2 launch zoe2_bringup hw.launch.py`.

## Operation
The robot can be steered by publishing a twist to `/cmd_vel_unstamped`. This can currently be done with the RQT Robot Steering plugin, or by a physical gamepad configured in `zoe2_joystick`.
### RQT Steering
1. Open a new terminal and `source install/setup.bash` at the root of your workspace.
2. Launch rqt: `rqt` and open the Robot Steering plugin (Plugins -> Robot Tools -> Robot Steering)
3. Set the message topic to `/cmd_vel_unstamped`
4. Use the sliders to steer the robot!
### Joystick Steering
1. Open a new terminal and `source install/setup.bash` at the root of your workspace.
2. Launch the joystick node: `ros2 launch zoe2_joystick joystick.launch.py`
3. Tilt the left stick front/back to command forward velocity, right stick side/side for turn radius, and use the upper right trigger to trigger motion


## Simulation Demo
[![Zoe Rover Demo](http://img.youtube.com/vi/RqhPxBom7Jg/0.jpg)](http://www.youtube.com/watch?v=RqhPxBom7Jg)

## Troubleshooting
- Gazebo is loading, but the window is black
    - There is likely an issue with your graphics driver, so Gazebo might have defaulted to headless mode. Run `export LIBGL_ALWAYS_SOFTWARE=1` before launching gazebo to render 3D graphics on CPU
- I'm getting build errors!
    - Make sure you are on the correct version of Ubuntu, ROS2, and Gazebo. This will not work on previous versions.
    - Make sure you have installed the correct version of ROS2 control (Jazzy branch, not rolling).
    - Make sure you properly sourced your ROS2 installation

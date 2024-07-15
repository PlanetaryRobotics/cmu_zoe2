# Running DiffDrive Simulation
1. Go to the root of your workspace folder (there where `src`, `build`, `install` and `log` files are).
2. Install the package by calling `colcon build --packages-select zoe2_bringup`
3. Open a new terminal and source environment `source install/setup.bash`
4. Launch the robot using `ros2 launch zoe2_bringup launch_sim.launch.py`
5. Alternatively, launch the robot in a custom world using `ros2 launch zoe2_bringup launch_sim.launch.py world:="moon.world"`
6. Open a new terminal and source environment `source install/setup.bash`
7. Control the robot using `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped`
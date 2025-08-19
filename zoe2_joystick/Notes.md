To make sure the joystick is working, use the command:
```
jstest /dev/input/js0
```

To find out the joysticks that we are connected to, run the command:

```
ros2 run joy joy_enumerate_devices
```

To install joystick packages
```
sudo apt-get install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```


To launch the joystick
```
ros2 launch zoe2_joystick joystick_launch.py
```
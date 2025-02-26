# CMU_Zoe2 Install Instructions
## Install Ubuntu 24.04
1. This can be done on a virtual machine or dual boot
2. Download the ISO of the latest long-term-stable (LTS) version here: https://ubuntu.com/download/desktop
3. If you choose a virtual machine, you can use [Vbox](https://www.virtualbox.org/) or [VMWare Workstation](https://blogs.vmware.com/workstation/2024/05/vmware-workstation-pro-now-available-free-for-personal-use.html). Between the two, VMWare has better performance, but the install process is more convoluted.
4. If you choose to install a dual boot, or a full boot, you will need to flash the ISO to a USB drive. You can use [Rufus](https://rufus.ie/en/), [Balena Etcher](https://etcher.balena.io/), or any other flasher.

## Install ROS2 Jazzy
Once you have installed and opened Ubuntu, [install ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). At the time of writing, the steps are as follows:
```bash
# SET LOCALE
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# ENABLE REQUIRED REPOSITORIES
# ensure that Ubuntu Universe repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add the ROS2 GPG key with apt
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# INSTALL DEVELOPMENT TOOLS
sudo apt update && sudo apt install ros-dev-tools

# INSTALL ROS2
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

# SOURCE THE ENVIRONMENT
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash

# SET UP AUTO-SOURCING
echo "# ROS2 Setup" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
After install, I recommend you close out of your terminal and relaunch it

## Optional: Install RQT add-ons
When you install Jazzy, you will get the base version of RQT. There are some useful plugins that are nice to install.
```bash
sudo apt update
sudo apt install '~nros-jazzy-rqt*'
```
If you run `rqt` and don't see **Robot Tools**, you can reload by closing and running `rqt --force-discover`.

## Install ROS2 Control Jazzy
Follow the [installation instructions](https://control.ros.org/jazzy/doc/getting_started/getting_started.html#installation) on the wiki. At the time of writing, these are:
```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

## Install Gazebo Harmonic
```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
sudo apt-get install ros-${ROS_DISTRO}-gz-ros2-control
```
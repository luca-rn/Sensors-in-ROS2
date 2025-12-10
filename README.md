# Sensors-in-ROS2
A directory for a project at Seaonics, connecting to many sensors on a Jetson Orin NX with ROS2 Humble.\
\
System: NVIDIA Jetson Orin NX 8gb ([datasheet](https://developer.download.nvidia.com/assets/embedded/secure/jetson/orin_nx/docs/Jetson_Orin_NX_DS-10712-001_v0.5.pdf?__token__=exp=1764669901~hmac=39f47f53ef546b6d9e80313ec546f1aec0aa05bd6a7896753ac6a1ff9c9ccae2&t=eyJscyI6IndlYnNpdGUiLCJsc2QiOiJkZXZlbG9wZXIubnZpZGlhLmNvbS9idXktamV0c29uP3Byb2R1Y3Q9YWxsXHUwMDI2bG9jYXRpb249Tk8ifQ==))\
OS: [Ubuntu 22.04 Jammy Jellyfish](https://releases.ubuntu.com/jammy/)\
ROS Version: [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)

## ROS 2
### Installation
Here we detail how to install ROS 2 Humble, the original instructions from the ROS 2 website can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
1) Ensure that the Ubuntu Universe repository is enabled.
  ```bash
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  ```
2) Add the ROS 2 apt repository to the system and sources list
  ```bash
  sudo apt update && sudo apt install curl -y
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  ```
3) Update apt repository caches and ensure the system is up to date
  ```bash
  sudo apt update
  sudo apt upgrade
  ```
4) Install ROS 2 Humble Hawksbill (or your preferred ROS 2 version)
  ```bash
  sudo apt install ros-humble-desktop
  ```
__If you have no experience with ROS, it is recommended that you follow the tutorial on the [ROS 2 website](https://docs.ros.org/en/humble/Tutorials.html)__
### Environment Setup
A ROS 2 environment can be set up with the following command 
```bash
source /opt/ros/humble/setup.bash
```
To ensure that this file is sourced every time a terminal is opened, this setup can be made persistent with the following command.
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
To confirm this was successful, use the command 
```bash
tail -1 .bashrc
```
To ensure the environment is properly set up, use the command
```bash
printenv | grep -i ROS
```
This should return the following.
```bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```
### Initialising rosdep
If rosdep is not installed, run:
```bash
sudo apt-get install python3-rosdep
```
The rosdep init command will create a file of dependencies in /etc/ros/rosdep/sources.list.d that hold some basic distro dependencies.
```bash
sudo rosdep init
rosdep update
```
_Note_: Running update with sudo can later result in permission errors, so this is __not__ recommended.
### Colcon Build
Colcon is a tool that automates the process of building packages in their topological order and
handles the workflow of environment setup. It must be installed before working with workspaces.\
\
To install Colcon, run:
```bash
sudo apt install python3-colcon-common-extensions
```
When using colcon build, a specific package can be selected with the modifier `--packages-select XXXX` to make the build faster.
### Creating a Workspace
To create a workspace, use the command
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws
```
Follow the [ROS 2 Humble guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) if you would like to learn more about using a workspace.
### Install Tools
There are some known issues that require the xterm terminal emulator installation and usage so that the stdin user interaction is possible, so it is best to install this. Run:
```bash
sudo apt update
sudo apt install xterm
```
## Basler Cameras - Ace & Blaze
A separate repository has been created for these cameras found [here](https://github.com/luca-rn/pylon-ros-camera.git). It contains a slightly modified version Basler's [pylon-ros-camera](https://github.com/basler/pylon-ros-camera.git). The repository contains a readme file with detailed instructions for installation and use.
```bash
cd ~/dev_ws/src/ && git clone https://github.com/luca-rn/pylon-ros-camera.git pylon_ros2_camera
```

## Cobo Sensors (CANbus/CANopen)
A separate repository has been created for these devices found [here](https://github.com/luca-rn/ros2can_bridge.git). It contains a slightly modified version ROS4Space's [ros2can_bridge](https://github.com/ROS4SPACE/ros2can_bridge.git). The repository contains a readme file with detailed instructions for installation and use.
```bash
cd ~/dev_ws/src/ && git clone https://github.com/luca-rn/ros2can_bridge.git
```

## Luxonis (OAK-D Pro PoE)
### Installing from ROS Binaries (apt)
This is the simplest and fastest way to install depthai ros. Use the following command:
```bash
sudo apt install ros-<distro>-depthai-ros
```
### Installing from source
Follow [this guide](https://docs.luxonis.com/software/ros/depthai-ros/build/) to install depthai ros from source or with docker. We found that this method took a long time and was ultimately problematic in the installation to the point where we could not get it working. Follow this method at your own risk.
### IP configuration
If the camera is not on the same LAN, you can manually specify the IP address of the device. If DHCP server is not available, the camera will fallback to a static IP `169.254.1.222`.\
\
Our recommendation, and what we did in our project, is to let the camera fallback to its static ip and add an ipv4 address `169.254.X.XX` to your ethernet network connection with a subnet mask  `255.255.0.0`.
### Using the launch files
The run the basic camera launch file in the depthai ros driver, use the following command:
```bash
ros2 launch depthai_ros_driver camera.launch.py
```
For more examples, see the launch directory or [Luxonis' ROS page](https://docs.luxonis.com/software/ros/depthai-ros/driver/).

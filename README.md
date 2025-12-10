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
## Basler Cameras
### Installing pylon (Notes from [_Interfacing Basler Cameras with ROS 2_](https://rjwilson.com/wp-content/uploads/Interfacing-Basler-Cameras-with-ROS-2-RJ-Wilson-Inc.pdf))
The [pylon-ros2-camera driver package](https://github.com/basler/pylon-ros-camera/tree/humble) requires that the library of pylon version 6.2 or newer is
installed. If you need to install a suitable pylon version, continue with the following steps. Otherwise, continue with [Setting up the Driver in ROS2](#setting-up-the-pylon-camera-driver-in-ros-2).

1. Visit the [Basler software downloads](https://www.baslerweb.com/en/downloads/software/) page.\
2. Download the appropriate pylon version package for your OS. The install notes (downloadable) on the downloads page for each pylon version are useful for this step.\
  __For use with the supplementary package for Basler Blaze, it may be necessary to downloaded an outdated Pylon version. Check the requirements before installing.__\
  \
  For our system we downloaded pylon 7.3.0 with the .tar.gz for a Debian based ARM 64-bit system. This was used alongside the pylon Supplementary Package for blaze 1.7.3.\
  \
  If you're using a Debian-based Linux distribution (e.g., Ubuntu) you can choose one of the corresponding Debian packages provided with this pylon release. Alternatively, you can always use the tar.gz files, which will also work for Linux distributions not based on Debian.\
  \
  __If you downloaded a debian/.deb package:__\
  On many Debian-based Linux distributions, you can install the Debian package by double-clicking the file or with the command `sudo dpkg -i route\to\deb\install\pylon_X.X.X.XXXXX-deb0_arm64.deb`. Check the pylon root location environment variable and make sure it exists (using `echo $PYLON_ROOT`). If not, type the following `echo “export PYLON_ROOT=/opt/pylon” >> ~/.bashrc`. Check again with `echo $PYLON_ROOT`and the output should be `opt/pylon`.\
  \
  Alternatively, follow these steps:\
    a. Change to the directory that contains the pylon Debian package.\
    b. Install the Debian packages: `sudo apt-get install ./pylon_*.deb ./codemeter*.deb`\
  \
  During the installation, an environment variable required for pylon GenTL producers and a permission file for Basler USB cameras are installed automatically. For this to take effect, you need to log out and log in again toyour Linux system as well as unplug and replug all USB cameras.\
  \
  __If you downloaded a .tar.gz package:__\
  Details about installation and configuration are available from the included INSTALL and README files.

### Setting up the pylon camera driver in ROS 2
__ALTERNATIVE 1__ (Recommended): Using this repository
1. Clone this repository into your src folder
  ```bash
  cd ~/dev_ws/src/ && git clone https://github.com/luca-rn/Sensors-in-ROS2.git
  ```
  or, for the specific package
  ```bash
  cd ~/dev_ws/src/ && git clone https://github.com/luca-rn/Sensors-in-ROS2/pylon_ros2_camera.git
  ```
2. (Not Tested) Clone any necessary additional packages. For example packages from ros-perception. Example given by basler is image_common.git.
  ```bash
  cd ~/dev_ws/src/pylon_ros2_camera && git clone –b humble https://github.com/rosperception/image_common.git image_common
  ```
3. Install mandatory dependencies
  ```bash
  cd ~/dev_ws && sudo rosdep install --frompaths src --ignore-src –r -y
  ```
4. Build the workspace using colcon build
  ```bash
  cd ~/dev_ws && colcon build
  ```
5. Permanent setup of environment settings
  ```bash
  echo “source ~/dev_ws/install/setup.bash” >> ~/.bashrc
  ```
  Then, open a new terminal or run
  ```bash
  source ~/.bashrc
  ```
6. Running the package
  ```bash
  ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py
  ```
  This automatically uses the first camera model that is found by underlaying pylon API. If no camera can be found, it will create an error.

__Alternative 2__: Install from Basler's github

Follow the same steps, but replace step 1 with the following:

1. Clone the driver packages to the relevant src folder
```bash
cd ~/dev_ws/src/ && git clone –b humble
https://github.com/basler/pylon-ros-camera pylon_ros2_camera
```

### Camera configuration
The Basler cameras must be configured with a suitable IP for connection with the ROS nodes. This can be easily achieved with the pylon IP configurator.
1) Connect the camera via ethernet, ensure it is receiving power
2) Open the pylon ip configurator
3) The connected camera should show up in the list of devices connected via ethernet. Select this device and give it an appropriate IP and name.\
  For Basler cameras, an IPV4 address `192.168.5.XX` is recommended, with the subnet mask `255.255.255.0` and ensure that the changes are saved.\
  For our Basler Ace2 camera used the ip `192.168.5.10` and the device name BaslerAce1. An address and name of the same format was used for our Blaze camera as well.
4) In terminal, use the command `ip addr` to check the your network connections. If an inet with an address  `192.168.5.XX` is listed under your ethernet connections, you may be able to connect to your camera in ROS immediately. Otherwise, you must add such an address manually.
5) Go to network settings on your device and click to edit a network connection. For our project, we created a new connection as we had to add ip addresses for multiple devices.
6) Add in IPV4 address `192.168.5.XX` (we used `192.168.5.2`) and the subnet mask `255.255.255.0`.
7) Ensure that the changes are persisted.\
\
This should be enough to enable that the camera is able to connect in ROS 2, it is recommended that you now test this in your ROS 2 workspace with the [camera launch command](#using-the-pylon-ros2-camera-wrapper).\
\
If the camera can still not be found, it is recommended to edit or create a new `.yaml` file for the camera configuration. this can be found in `/pylon_ros2_camera/pylon_ros2_camera_wrapper/config`. Set the device name to that which you assigned to your device in the .yaml file. If you are using an RGB-enabled camera, you may also want to set the image encoding to RGB-8 or another coloured encoding.\
\
In the pylon suite, it is possible to make a custom configuration for the camera settings. This can be used to set your desired configuration. In our experience, it was necessary to change the image encoding from greyscale to RGB in order to obtain a colour output from our basler ace camera. The use this configuration in ROS2, follow these steps:
1) Save the custom configuration in the pylon suite and remember the name.
2) Open the file, pylon_ros2_camera.launch.py in the launch directory of the pylon_ros2_camera_wrapper.
3) Find the function DeclareLaunchArgument.
4) Change the default_value parameter to the configuration that matches the name of your custom configuration (UserSet1, UserSet2 or UserSet3).
5) Save the file and use colcon to build your workspace again
```bash
colcon build --packages-select pylon_ros2_camera
```

### Using the pylon ros2 camera wrapper
For the basler ace:
```bash
ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py
```
For the basler blaze:
```bash
ros2 launch pylon_ros2_camera_wrapper my_blaze.launch.py
```
Use `ros2 topic list` in another terminal to check if nodes run correctly and `rviz2` to view the camera streams.

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

## CANBus/CANopen Sensors (Cobo)
We interfaced with the following sensors in this project:
- FTI2 Incline/IMU.
- FTM2 Speed, direction, distance and angle radar sensor.
- FTU1 Ultrasonic sensor for distance measurements.
- FTP2 IMU

These sensors work on [CANopen](https://en.wikipedia.org/wiki/CANopen), a high-level protocol based on the CAN bus standard, designed for modular and real-time control in industrial automation systems.\
\
For interfacing with these sensors, we used a [Lawicel CANUSB adapter](https://www.canusb.com/products/canusb/).\
\
The setup for a CANUSB seems quite specific to the USB and the system. We originally used [this guide](https://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html) to set up. On one Jetson this went perfectly, when attempting to replicate the set up on another Jetson we encountered the problem that the Serial CAN module (slcan) could not be found when running `sudo lsmod | grep slcan` and `sudo modprobe slcan`. This seemed to be a problem with the linux installation and was not solveable without significant effort.


### ROS 2 Integration
__ALTERNATIVE 1__ (Recommended): Using this repository\
For integration with ROS2, we used a GitHub repository from ROS4SPACE, [ros2can_bridge](https://github.com/ROS4SPACE/ros2can_bridge.git). Some changes had to be made to get the code working, mostly renaming attributes which did not match what they were supposed to.
1. Clone this repository to your src folder
  ```bash
  cd ~/dev_ws/src/ && git clone https://github.com/luca-rn/Sensors-in-ROS2.git
  ```
  or, for the specific packages
  ```bash
  cd ~/dev_ws/src/ && git clone https://github.com/luca-rn/Sensors-in-ROS2/ros2can_bridge
  ```
2. Install necessary dependencies and build the workspace
  ```bash
  cd ~/dev_ws && sudo rosdep install --frompaths src --ignore-src –r -y
  sudo apt install ros-humble-can-msgs
  colcon build --packages-select ros2socketcan_bridge can_msgs
  ```

__Alternative 2__: Clone from ROS4SPACE
Follow the same steps as alternative 1 but replace step 1 with the following.
1. Clone it into your workspace with the following instructions
  ```bash
  cd ~/dev_ws/src/
  git clone https://github.com/ROS4SPACE/ros2can_bridge.git
  ```
  There are some issues with the ROS4SPACE than need changing, which is why using the code from this GitHub is recommended. One such change is the ros2_socketcan.cpp, where the name of the can socket must be changed (to slcan0 for whatever you have named your interface).

### Using the CANBus with ROS 2
Assuming you have been able to follow the instruction from [Pascal Walter's guide](https://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html), using the CANBus with ROS 2 should be relatively simple.

1. First, check than the CANBus is working after plugging the the USB:\
  Note: you do not need to bring up the slcan interface again if you succesfully made it permanent earlier. Also, if ttyUSBX is different then it needs to be changed, it can be checked by running "ls /dev" in terminal
  ```bash
  sudo slcand -o -c -f -s4 /dev/ttyUSB0 slcan0
  sudo ifconfig slcan0 up
  candump slcan0
  ```
  The COBO sensors must be given a start command, this can be done by sending the following with another terminal:
  ```bash
  cansend slcan0 000#0100
  ```
  Confirm that the candump is publishing messages. A stop command can then be sent with
  ```bash
  cansend slcan0 000#0200
  ```
  See the [CANopen wikipedia page](https://en.wikipedia.org/wiki/CANopen) to learn more about the various CANopen protocol command messages.
2. Test the sensors in ROS 2
  To start the ROS 2 node, use the command:
  ```bash
  ros2 run ros2socketcan_bridge ros2socketcan slcan0
  ```
  You can check what topics are published using
  ```bash
  ros2 topic list
  ```
  The topics `CAN/slcan0/transmit` and `CAN/slcan0/receive` should appear.\
  \
  Again, to get the sensors to begin transmitting you must send a start command. Use the following command to test this.
  ```bash
  cd ~/dev_ws
  ros2 topic pub /CAN/slcan0/transmit can_msgs/msg/Frame "{id: 0x000, dlc: 2, data: [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], err: 0, rtr: 0, eff:0}" --once
  ```
  You can then check if data is being sent using:
  ```bash
  ros2 topic echo CAN/slcan0/receive
  ```
  To stop transmitting, use the command
  ```bash
  ros2 topic pub /CAN/slcan0/transmit can_msgs/msg/Frame "{id: 0x000, dlc: 2, data: [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], err: 0, rtr: 0, eff:0}" --once
  ```
# Resources Used
[_Interfacing Basler Cameras with ROS 2_](https://rjwilson.com/wp-content/uploads/Interfacing-Basler-Cameras-with-ROS-2-RJ-Wilson-Inc.pdf)\
[Basler Software Downloads](https://www.baslerweb.com/en/downloads/software/)\
[Guide to Setting up a Lawicel CANUsb](https://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html)

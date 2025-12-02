# Sensors-in-ROS2
A directory for a project at Seaonics where I connected to many sensors on a Jetson Orin NX with ROS2 Humble.\
\
System: NVIDIA Jetson Orin NX 8gb ([datasheet](https://developer.download.nvidia.com/assets/embedded/secure/jetson/orin_nx/docs/Jetson_Orin_NX_DS-10712-001_v0.5.pdf?__token__=exp=1764669901~hmac=39f47f53ef546b6d9e80313ec546f1aec0aa05bd6a7896753ac6a1ff9c9ccae2&t=eyJscyI6IndlYnNpdGUiLCJsc2QiOiJkZXZlbG9wZXIubnZpZGlhLmNvbS9idXktamV0c29uP3Byb2R1Y3Q9YWxsXHUwMDI2bG9jYXRpb249Tk8ifQ==))\
OS: [Ubuntu 22.04 Jammy Jellyfish](https://releases.ubuntu.com/jammy/)\
ROS Version: [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)

## Basler Cameras
### Setting up pylon (Notes from [_Interfacing Basler Cameras with ROS 2_](https://rjwilson.com/wp-content/uploads/Interfacing-Basler-Cameras-with-ROS-2-RJ-Wilson-Inc.pdf))
The [pylon-ros2-camera driver package](https://github.com/basler/pylon-ros-camera/tree/humble) requires that the library of pylon version 6.2 or newer is
installed. If you need to install a suitable pylon version, continue with the following steps. Otherwise,
continue with [Setting up the Driver in ROS2](#setting-up-the-driver-in-ros-2).

1. Visit the [Basler software downloads](https://www.baslerweb.com/en/downloads/software/) page.\
2. Download the appropriate pylon version package for your OS. The install notes (downloadable) on the downloads page for each python version are useful for this step.\
For our system we downloaded Pylon 7.3.0 with the .tar.gz for a Debian based ARM 64-bit system.\
If you're using a Debian-based Linux distribution (e.g., Ubuntu) you can choose one of the corresponding Debian packages provided with this pylon release. Alternatively, you can always use the tar.gz files, which will also work for Linux distributions not based on Debian.\
\
__If you downloaded a debian/.deb package:__\
On many Debian-based Linux distributions, you can install the Debian package by double-clicking the file or with the command `sudo dpkg -i route\to\deb\install\pylon_X.X.X.XXXXX-deb0_arm64.deb`. Check the pylon root location environment variable and make sure it exists (using `echo $PYLON_ROOT`). If not, type the following `echo “export PYLON_ROOT=/opt/pylon” >> ~/.bashrc`. Check again with `echo $PYLON_ROOT`and the output should be `opt/pylon`.\
\
Alternatively, follow these steps:\
  a. Change to the directory that contains the pylon Debian package.\
  b. Install the Debian packages:
```bash
sudo apt-get install ./pylon_*.deb ./codemeter*.deb
```
During the installation, an environment variable required for pylon GenTL producers and a permission file for Basler USB cameras are installed automatically. For this to take effect, you need to log out and log in again toyour Linux system as well as unplug and replug all USB cameras.\
__If you downloaded a .tar.gz package:__\
  Details about installation and configuration are available from the included INSTALL and README files.

### Setting up the Driver in ROS 2

1. Clone the driver packages to the relevant src folder

```bash
cd ~/dev_ws/src/ && git clone –b humble
https://github.com/basler/pylon-ros-camera pylon_ros2_camera
```
2. Clone any necessary additional packages. For example packages from ros-perception. Example given by basler is image_common.git (untested).
```bash
cd ~/pylon_ws/src/pylon_ros2_camera && git clone –b humble https://github.com/rosperception/image_common.git image_common
```
4. Install mandatory dependencies
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

# Resources Used
[_Interfacing Basler Cameras with ROS 2_](https://rjwilson.com/wp-content/uploads/Interfacing-Basler-Cameras-with-ROS-2-RJ-Wilson-Inc.pdf)\
[Basler Software Downloads](https://www.baslerweb.com/en/downloads/software/)

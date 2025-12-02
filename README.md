# Sensors-in-ROS2
A directory for a project at Seaonics where I connected to many sensors on a Jetson Orin NX with ROS2 Humble

## Basler Cameras
### Setting up pylon
The pylon-ros2-camera driver package requires that the library of pylon version 6.2 or newer is
installed. If you need to install a suitable pylon version, continue with the following steps. Otherwise,
continue with Setting up the Driver in ROS2(#setting-up-the-driver-in-ros2).


### Setting up the Driver in ROS2

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
This automatically uses the first camera model that is found by underlaying pylon API.
If no camera can be found, it will create an error.

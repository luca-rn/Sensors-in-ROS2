Overview

This package provides an experimental ROS 2 node for interfacing with Lucid event-based cameras using the Arena SDK.
It has been tested with the Lucid TRT0095-EC camera on ROS 2 Humble, running Ubuntu 22.04 on an ARM CPU.

Note: This node is still under active development.

Setup

Clone the package into your ROS 2 workspace:

cd ~/ros2_ws/src
git clone <your-repo-url> lucid_event_camera


Download the Arena SDK (ARM64 Linux version) from Lucid:

https://thinklucid.com/downloads-hub/

Update the CMake configuration
Edit line 12 of CMakeLists.txt to point to the root directory of your Arena SDK installation.
For example:

set(ARENA_SDK_ROOT <your_path>/ArenaSDK_v0.1.78_Linux_ARM64/ArenaSDK_Linux_ARM64)


Build the workspace:

cd ~/ros2_ws
colcon build --packages-select lucid_event_camera

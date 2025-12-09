# ROS 2 Lucid Event Camera Interface

## Overview

This package provides an **experimental ROS 2 node** designed for interfacing with **Lucid Vision event-based cameras** using the **Arena SDK**. The current node is able to identify the camera. This package can be used as a starting point to develop a ROS2 node using Arena SDK that can gather data from event cameras.

* **Tested Hardware:** Lucid TRT0095-EC camera.
* **Tested Environment:**
    * **ROS 2 Distribution:** Humble Hawksbill
    * **Operating System:** Ubuntu 22.04 (Jammy Jellyfish)
    * **Architecture:** ARM CPU

> **Note:** This node is currently under **active development** and should be considered experimental.

---

## Setup

### 1. Clone the Package

Clone this repository into the `src` directory of your ROS 2 workspace (e.g., `~/ros2_ws`):

```bash
cd ~/ros2_ws/src
git clone https://github.com/luca-rn/Sensors-in-ROS2.git lucid_event_camera
```

### 2. Download the Arena SDK
Download the appropriate version of the Arena SDK from the official Lucid Vision website. Ensure you get the ARM64 Linux version to match the tested environment.
* **Download Link:** https://thinklucid.com/downloads-hub/

### 3. Update CMake Configuration
You need to tell the build system where to find the Arena SDK files.

* Open the CMakeLists.txt file located in the lucid_event_camera directory.
* Edit line 12 to set the correct path for the ARENA_SDK_ROOT variable. This path should point to the root directory of your Arena SDK installation (where the main library files are located).

Example:

```CMake
set(ARENA_SDK_ROOT <your_path>/ArenaSDK_<version>_Linux_ARM64/ArenaSDK_Linux_ARM64)
```
Replace `<your_path>/ArenaSDK_<version>_Linux_ARM64/ArenaSDK_Linux_ARM64` with the actual path on your system.


### 4. Build the Workspace
After updating the path, navigate back to your main workspace directory and build the package using colcon:

```Bash
cd ~/ros2_ws
colcon build --packages-select lucid_event_camera
```

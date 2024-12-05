# moondawg-ros

## Overview

`Soybean-Jetson` is a ROS2 package designed to interface with Soybean Data Collection Robot, providing functionalities for Xbox controller input, serial communication, image capture, and GPS tagging. The package includes several nodes and scripts to facilitate these operations.

## Prerequisites
- ROS2 Humble
This package is made for ROS2 humble, ensure it is installed and sourced on Ubuntu 22.04. (see https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- ROS2 packages
```bash
sudo apt install ros-humble-cv-bridge ros-humble-rosbridge-suite ros-humble-image-tools ros-humble-ublox-gps
```
- Python modules
```bash
sudo apt install python3-opencv python3-serial
```

## Running the package
1. Clone the repository:
```bash
git clone https://github.com/Senior-Design-Soybean-Bot/Soybean-Jetson.git
cd Soybean-Jetson
```

2. Build the package:
```bash
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

4. Run:
```bash
ros2 launch moondawg moondawg.launch.py
```

## Nodes
### xbox_translator

Translates Xbox controller inputs to Arduino-parsable messages.

### serial_bridge
Sends strings via serial to configured port (default: /dev/ttyACM0).

### diagnostics
Provides diagnostic information about the package's status.

## Configuration

### Parameters

- **rate**: Baud rate for serial communication (default: 9600).
- **port**: Serial port (default: `/dev/ttyACM0`).
- **wheel_full_speed**: The fastest forward speed of the wheel motors (default: 130).
- **wheel_full_stopped**: The value for a stopped wheel motor (default: 90).

## Web Interface

A simple web interface is provided to display Xbox controller inputs and diagnostics.

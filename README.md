# Soybean-Jetson ROS2 Package

## Overview

`Soybean-Jetson` is a ROS2 package for controlling and managing a Soybean Data Collection Robot. This package is built on an Nvidia Jetson Nano and provides integrated functionality for:
- Xbox controller input processing
- Serial communication with Arduino
- Image capture
- GPS data integration and tagging
- Web-based monitoring interface

## System Requirements

### Operating System
- Ubuntu 22.04 LTS
  
Alternatively you can use the pre-installed Nvidia Jetson Nano OS which is a fork of Ubuntu 22.04

### Core Dependencies
- ROS2 Humble
- Python 3.x

### ROS2 Packages
```bash
sudo apt install \
    ros-humble-cv-bridge \
    ros-humble-rosbridge-suite \
    ros-humble-image-tools \
    ros-humble-ublox-gps
```

### Python Dependencies
```bash
sudo apt install \
    python3-opencv \
    python3-serial
```

#### Installation
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

## Usage

### Launching the System
```bash
ros2 launch moondawg moondawg.launch.py
```

### Controlling
- Connect to the Network being hosted on the Nvidia Jetson `Jetson-ssid`
- Open the `Script.js` and ensure the IP is the same as the Jetson
- Open the HTML and press F12 and navigate to the `Network` tab to see when you are connected
- Grab you Xbox Controller that's connected via bluetooth or cable

## Architecture

### Core Nodes

#### xbox_translator
- Processes Xbox controller inputs
- Converts controller signals to Arduino-compatible commands
- Handles motion control and special functions

#### serial_bridge
- Manages serial communication with Arduino
- Handles message queuing and transmission
- Default port: `/dev/ttyACM0`

#### image_capture
- Manages GPS coordinates
- Saves images from all four cameras with the following: Time & Date, Camera #, Long, Lat, and Alt


## Configuration

### System Parameters
| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| rate | Serial communication baud rate | 9600 |
| port | Serial communication port | /dev/ttyACM0 |
| port | GPS | /dev/ttyACM1 |
| wheel_full_speed | Maximum wheel motor speed | 130 |
| wheel_full_stopped | Wheel motor stop value | 90 |

## Web Interface

This is currently not implemented correctly but has the ability to do the following:
- Real-time Xbox controller input visualization
- System diagnostics display
- Status monitoring
- Error reporting
- etc.

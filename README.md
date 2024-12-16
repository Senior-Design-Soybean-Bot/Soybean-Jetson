# Soybean-Jetson

ROS2 package for controlling the robot remotely, taking pictures, and geo tagging.

## Requirements

- Jetson Nano Developer Kit with JetPakc 4.2 or newer
- Ubuntu 18.04
- Minimum 4GB RAM

  ## Dependencies

  ### System Dependencies

  - ROS2 Humble
  - Python 3.8+
  - NetworkManager
  - hostapd
  - dnsmasq
 
  ### ROS Dependencies

  - OpenCV
  - gpsd
  - gpsd-clients
  - python3-gps
 
  ### Python Dependencies

  ```bash
  pip3 install -r requirements.txt
  ```

  ## ROS2 Humble Installation

  For detailed installation instructions and guides, visit:
  [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

  ## Network Configuration

  The 'ros-network-service' consists of two parts:

  ### 1. Network Setup Script ('ros_network_setup.sh')

  This script:
  - Sources the ROS2 Humble environment
  - Configures the wireless interface 'wlP1p1s0':
    - Brings up the interface
    - Assigns a static IP address (10.0.0.4/24)

  ### 2. Systemd Service ('ros-network-setup.service')

  This service:
  - Runs after NetworkManager starts
  - Executes as the 'soybean' user
  - Automatically starts the network setup script at boot
 
  ### Connecting to the WiFi

  When you want to connect to the WiFi you want to start the NetworkManager using the following command:
  ```bash
  sudo systemctl start NetworkManager
  ```

  ## Controlling the Robot

  ### Connecting

  When you connect to the hotspot hosted by the robot you'll need to set your IP to be ```10.0.0.x```. It can't be ```10.0.0.4``` as that is the Jetsons static IP.

  ### Using the Website
  
  You'll need to download the website folder onto the computer you will connect to the robot with. Once downloaded make sure the 2nd line ```JavaScript var ip = "";``` is set to ```10.0.0.4```. Once that is set and you are connected to the robot you can open the .html file in your browser and connect the controller to your computer.

  ## Recommendations

  ### ROS Nodes

  - ```image_capture.py```
    - Change how often the GPS publishes
      - Taken same time when pictures are taken in the time domain
      - Taken at set intevals from data you can gather from the Arduino Uno and Encoder
  - Overall
    - Change the package name and update comments on maintainer
    - Optimize the packages so it runs smoother
 
  ### ros_network_setup.sh/.service

  - Have this auto start the ROS Nodes so you don't need a monitor

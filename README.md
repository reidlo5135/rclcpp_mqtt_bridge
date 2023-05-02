# rclcpp-MQTT bridge client

## Document
  - [Environment](#environment)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Install Paho MQTT C Library](#install-paho-mqtt-c-library)
    - [Install Paho MQTT C++ Library](#install-paho-mqtt-c-library-1)
    - [Colcon Build](#clone--colcon-build)
    - [Run Test](#run-test)

## Environment
* <img src="https://img.shields.io/badge/cpp-magenta?style=for-the-badge&logo=cplusplus&logoColor=white">
* <img src="https://img.shields.io/badge/cmake-064F8C?style=for-the-badge&logo=cmake&logoColor=white">
* <img src="https://img.shields.io/badge/mqtt-660066?style=for-the-badge&logo=mqtt&logoColor=white">
* <img src="https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">

## Installation

### Prerequisites
- [ROS2 setup](https://index.ros.org/doc/ros2/Installation/) for use rclcpp -
  **INSTALL [ROS2 Foxy-Fitzroy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)**

### Install Paho MQTT C Library
```bash
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
sudo cmake --build build/ --target install
sudo ldconfig
cd build
make
sudo make install
```

### Install Paho MQTT C++ Library
```bash
git clone https://github.com/eclipse/paho.mqtt.cpp.git
cd paho.mqtt.cpp
cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
sudo cmake --build build/ --target install
sudo ldconfig
cd build
make
sudo make install
```

### Clone & Colcon Build
```bash
source /opt/ros/foxy/setup.bash
git clone https://github.com/reidlo5135/rclcpp_mqtt_client.git
cd rclcpp_mqtt_client
colcon build --symlink-install
source install/setup.bash
```

### Run Test
```bash
source rclcpp_mqtt_client/install/setup.bash
ros2 run rclcpp_mqtt_client ros_mqtt_bridge
```
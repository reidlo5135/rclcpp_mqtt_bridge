# rclcpp-mqtt bridge client

## Document
  - [Environment](#environment)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Install Paho MQTT C Library](#install-paho-mqtt-c-library)
    - [Install Paho MQTT C++ Library](#install-paho-mqtt-c-library-1)
    - [Install jsoncpp](#install-jsoncpp)
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

### Install jsoncpp
```bash
git clone https://github.com/open-source-parsers/jsoncpp.git
cd jsoncpp
mkdir build && cd build
cmake ..
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
Apply rclcpp_mqtt_client into your workspace
```bash
cd ${your ros2 workspace}
colcon build --packages-select rclcpp_mqtt_client
source install/setup.bash
```

Run ros_connection bridge first
```bash
ros2 run rclcpp_mqtt_client ros_connection_bridge
```

Then run ros_mqtt_bridge
```bash
ros2 run rclcpp_mqtt_client ros_mqtt_bridge
```

Check ros2 node
```bash
ros2 node list
ros2 node info /ros_connection_bridge
ros2 node info /ros_mqtt_bridge
```
Run ros2 demo_node_cpp talker
```bash
ros2 run demo_node_cpp talker
```

Try ros2 /chatter topic subscription
```bash
ros2 topic echo /ros_connection_bridge/chatter

Hello World: 1
Hello World: 2
Hello World: 3
Hello World: 4
.
.
.
.
```

Try MQTT /chatter topic subscription
```bash
mosquitto_sub -h localhost -t "/chatter"

Hello World: 1
Hello World: 2
Hello World: 3
Hello World: 4
.
.
.
.
```
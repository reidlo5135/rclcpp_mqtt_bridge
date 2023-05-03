#include "ros_mqtt_bridge.hpp"

int main(int argc, char** argv) {
    MqttMgr * mqtt_ptr = new MqttMgr(MQTT_ADDRESS, MQTT_CLIENT_ID);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RclMqttBridge>(mqtt_ptr);
    while(rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    
    delete mqtt_ptr;
    return 0;
}
#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

int main(int argc, char** argv) {
    Mqtt * mqtt_ptr = new Mqtt(MQTT_ADDRESS, MQTT_CLIENT_ID);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>(mqtt_ptr);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    delete mqtt_ptr;
    return 0;
}
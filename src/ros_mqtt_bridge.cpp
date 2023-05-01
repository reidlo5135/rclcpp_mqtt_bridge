#include "mqtt/mqtt.hpp"
#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

void MinimalPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello World! [" + std::to_string(count++) + "]";
    RCLCPP_INFO(this->get_logger(), "%s Publishing : %s", LOG_RCL, message.data.c_str());
    publisher->publish(message);
    std::cout << "Starting the MQTT client..." << std::endl;
    mqtt_client client;
    client.run();
}

int main(int argc, char** argv) {
    std::cout<<"Hello"<<std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
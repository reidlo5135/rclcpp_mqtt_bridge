#include "rcl/rcl.hpp"

void MinimalPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello World! [" + std::to_string(count++) + "]";
    RCLCPP_INFO(this->get_logger(), "Publishing : %s", message.data.c_str());
    publisher->publish(message);
}

int main(int argc, char** argv) {
    std::cout<<"Hello"<<std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
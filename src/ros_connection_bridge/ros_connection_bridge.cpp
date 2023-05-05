// Copyright [2023] [wavem-reidlo]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros_connection_bridge/ros_connection_bridge.hpp"

/**
 * @brief Constructor for initialize this class instance & ros_connection_bridge rclcpp::Node shared pointer & invoke this create_publishers()
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see std::shared_ptr
 * @see rclcpp::Node
 * @see create_publishers()
*/
RosConnectionPublisher::RosConnectionPublisher(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: ros_node_ptr_(ros_node_ptr) {
    this->create_publishers();
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
RosConnectionPublisher::~RosConnectionPublisher() {

}

/**
 * @brief Function for create & register publishers to ros_connection_bridge rclcpp::Node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @return void
 * @see ros_connections
*/
void RosConnectionPublisher::create_publishers() {
    ros_connections::publisher::ros_std_publisher_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(ros_topics::publisher::chatter, 10);
    ros_connections::publisher::ros_odom_publisher_ = ros_node_ptr_->create_publisher<nav_msgs::msg::Odometry>(ros_topics::publisher::odometry, 10);
}

/**
 * @brief Constructor for initialize this class instance & ros_connection_bridge rclcpp::Node shared pointer & invoke this create_subscriptions()
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see std::shared_ptr
 * @see rclcpp::Node
 * @see create_subscriptions()
*/
RosConnectionSubscription::RosConnectionSubscription(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: ros_node_ptr_(ros_node_ptr) {
    this->create_subscriptions();
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
RosConnectionSubscription::~RosConnectionSubscription() {

}

/**
 * @brief Function for create & register subscriptions to ros_connection_bridge rclcpp::Node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @return void
 * @see ros_connections
*/
void RosConnectionSubscription::create_subscriptions() {
    ros_connections::subscription::ros_std_subscription_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        ros_topics::subscription::chatter,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr callback_std_msgs) {
            auto callback_data = callback_std_msgs->data.c_str();
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "std_msgs callback : '%s'", callback_data);

            auto std_message = std_msgs::msg::String();
            std_message.data = callback_data;
            ros_connections::publisher::ros_std_publisher_->publish(std_message);
        }
    );
    ros_connections::subscription::ros_odom_subscription_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_topics::subscription::odometry,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_msgs) {
            double position_x = callback_odom_msgs->pose.pose.position.x;
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "odom_msgs callback : '%e'", position_x);

            auto odom_message = nav_msgs::msg::Odometry();
            odom_message.pose.pose.position.x = position_x;
            ros_connections::publisher::ros_odom_publisher_->publish(odom_message);
        }
    );
}

/**
 * @brief Constructor for initialize this class instance & create rclcpp::Node named with ros_connection_bridge & invoke ros_connections classes' constructors
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @see rclcpp::Node
 * @see RosConnectionPublisher
 * @see RosConnectionSubscription
*/
RosConnectionBridge::RosConnectionBridge()
: Node("ros_connection_bridge"),
log_ros_(LOG_ROS_CONNECTION_BRIDGE) {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_connection_publisher_ptr_ = new RosConnectionPublisher(ros_node_ptr_);
    ros_connection_subscription_ptr_ = new RosConnectionSubscription(ros_node_ptr_);
    this->check_current_topics_and_types();
}

/**
 * @brief Virtual Destructor for this class & delete RosConnection classes' pointer
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @see ros_connection_publisher_ptr_
 * @see ros_connection_subscription_ptr_
*/
RosConnectionBridge::~RosConnectionBridge() {
    delete ros_connection_publisher_ptr_;
    delete ros_connection_subscription_ptr_;
}

void RosConnectionBridge::check_current_topics_and_types() {
    auto topic_names_and_types = ros_node_ptr_->get_topic_names_and_types();

    for (const auto& topic_name_and_type : topic_names_and_types) {
        const std::string& topic_name = topic_name_and_type.first;
        const std::vector<std::string>& message_types = topic_name_and_type.second;

        std::cout << log_ros_ << " topic registered '" << topic_name << "' with type '";
        for (const auto& message_type : message_types) {
            std::cout << message_type << "'" << '\n';
        }
    }
}

/**
 * @brief Function for check rclcpp status & init logs
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @return void
 * @see rclcpp::ok()
*/
void check_rclcpp_status() {
    if(rclcpp::ok()) {
        std::cout << R"(
  _____   ____   _____    _____ ____  _   _ _   _ ______ _____ _______ _____ ____  _   _   ____  _____  _____ _____   _____ ______ 
 |  __ \ / __ \ / ____|  / ____/ __ \| \ | | \ | |  ____/ ____|__   __|_   _/ __ \| \ | | |  _ \|  __ \|_   _|  __ \ / ____|  ____|
 | |__) | |  | | (___   | |   | |  | |  \| |  \| | |__ | |       | |    | || |  | |  \| | | |_) | |__) | | | | |  | | |  __| |__   
 |  _  /| |  | |\___ \  | |   | |  | | . ` | . ` |  __|| |       | |    | || |  | | . ` | |  _ <|  _  /  | | | |  | | | |_ |  __|  
 | | \ \| |__| |____) | | |___| |__| | |\  | |\  | |___| |____   | |   _| || |__| | |\  | | |_) | | \ \ _| |_| |__| | |__| | |____ 
 |_|  \_\\____/|_____/   \_____\____/|_| \_|_| \_|______\_____|  |_|  |_____\____/|_| \_| |____/|_|  \_\_____|_____/ \_____|______|
                                                                                                                                   
                                                                                                                                   
        )" << '\n';
    } else {
        std::cerr << "[ros_connection_bridge] rclcpp is not ok" << '\n';
    }
}

/**
 * @brief Function for initialize rclcpp & spin ros_connection_bridge rclcpp::Node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param argc int
 * @param argv char**
 * @return int
 * @see rclcpp
 * @see RosConnectionBridge
 * @see check_rclcpp_status()
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    check_rclcpp_status();
    rclcpp::spin(std::make_shared<RosConnectionBridge>());
    rclcpp::shutdown();

    return 0;
}
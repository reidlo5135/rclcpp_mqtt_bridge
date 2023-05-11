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
 * @brief Constructor for initialize this class instance & invoke initialize_bridge()
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see rclcpp
*/
ros_connections::ros_connections_to_mqtt::Bridge::Bridge(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: ros_node_ptr_(ros_node_ptr),
ros_default_qos_(ROS_DEFAULT_QOS) {
    this->initialize_bridge();
}

/**
 * @brief Virtual Destructor for this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
ros_connections::ros_connections_to_mqtt::Bridge::~Bridge() {

}

/**
 * @brief Function for initialize ros publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see rclcpp
*/
void ros_connections::ros_connections_to_mqtt::Bridge::initialize_publishers() {
    ros_chatter_publisher_ptr_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(
        ros_topics::to_mqtt::bridge::chatter,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_robot_pose_publisher_ptr_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::Pose>(
        ros_topics::to_mqtt::bridge::robot_pose,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_scan_publisher_ptr_ = ros_node_ptr_->create_publisher<sensor_msgs::msg::LaserScan>(
        ros_topics::to_mqtt::bridge::scan,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_tf_publisher_ptr_ = ros_node_ptr_->create_publisher<tf2_msgs::msg::TFMessage>(
        ros_topics::to_mqtt::bridge::tf,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_tf_static_publisher_ptr_ = ros_node_ptr_->create_publisher<tf2_msgs::msg::TFMessage>(
        ros_topics::to_mqtt::bridge::tf_static,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_odom_publisher_ptr_ = ros_node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
        ros_topics::to_mqtt::bridge::odom,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_global_plan_publisher_ptr_ = ros_node_ptr_->create_publisher<nav_msgs::msg::Path>(
        ros_topics::to_mqtt::bridge::global_plan,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_local_plan_publisher_ptr_ = ros_node_ptr_->create_publisher<nav_msgs::msg::Path>(
        ros_topics::to_mqtt::bridge::local_plan,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
}

/**
 * @brief Function for initialize ros subscriptions
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see rclcpp
*/
void ros_connections::ros_connections_to_mqtt::Bridge::initialize_subscriptions() {
    ros_chatter_subscription_ptr_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        ros_topics::to_mqtt::origin::chatter,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const std_msgs::msg::String::SharedPtr callback_chatter_data) {
            ros_chatter_publisher_ptr_->publish(*callback_chatter_data);
        }
    );
    ros_robot_pose_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Pose>(
        ros_topics::to_mqtt::origin::robot_pose,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const geometry_msgs::msg::Pose::SharedPtr callback_robot_pose_data) {
            ros_robot_pose_publisher_ptr_->publish(*callback_robot_pose_data);
        }
    );
    ros_scan_subscription_ptr_ = ros_node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
        ros_topics::to_mqtt::origin::scan,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr callback_scan_data) {
            ros_scan_publisher_ptr_->publish(*callback_scan_data);
        }
    );
    ros_tf_subscription_ptr_ = ros_node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
        ros_topics::to_mqtt::origin::tf,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr callback_tf_data) {
            ros_tf_publisher_ptr_->publish(*callback_tf_data);
        }
    );
    ros_tf_static_subscription_ptr_ = ros_node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
        ros_topics::to_mqtt::origin::tf_static,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr callback_tf_static_data) {
            ros_tf_static_publisher_ptr_->publish(*callback_tf_static_data);
        }
    );
    ros_odom_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_topics::to_mqtt::origin::odom,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_data) {
            ros_odom_publisher_ptr_->publish(*callback_odom_data);
        }
    );
    ros_global_plan_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Path>(
        ros_topics::to_mqtt::origin::global_plan,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const nav_msgs::msg::Path::SharedPtr callback_global_plan_data) {
            ros_global_plan_publisher_ptr_->publish(*callback_global_plan_data);
        }
    );
    ros_local_plan_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Path>(
        ros_topics::to_mqtt::origin::local_plan,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const nav_msgs::msg::Path::SharedPtr callback_local_plan_data) {
            ros_local_plan_publisher_ptr_->publish(*callback_local_plan_data);
        }
    );
}

/**
 * @brief Function for invoke initialize_publishers(), initialize_subscriptions()
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see rclcpp
*/
void ros_connections::ros_connections_to_mqtt::Bridge::initialize_bridge() {
    this->initialize_publishers();
    this->initialize_subscriptions();
}

/**
 * @brief Constructor for initialize this class instance & invoke initialize_bridge()
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see rclcpp
*/
ros_connections::ros_connections_from_mqtt::Bridge::Bridge(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: ros_node_ptr_(ros_node_ptr),
ros_default_qos_(ROS_DEFAULT_QOS) {
    this->initialize_bridge();
}

/**
 * @brief Virtual Destructor for this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
ros_connections::ros_connections_from_mqtt::Bridge::~Bridge() {

}

/**
 * @brief Function for initialize ros publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see rclcpp
*/
void ros_connections::ros_connections_from_mqtt::Bridge::initialize_publishers() {
    ros_chatter_publisher_ptr_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(
        ros_topics::from_mqtt::origin::chatter,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_cmd_vel_publisher_ptr_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
        ros_topics::from_mqtt::origin::cmd_vel,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_initial_pose_publisher_ptr_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ros_topics::from_mqtt::origin::initial_pose,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
}

/**
 * @brief Function for initialize ros subscriptions
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see rclcpp
*/
void ros_connections::ros_connections_from_mqtt::Bridge::initialize_subscriptions() {
    ros_chatter_subscription_ptr_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        ros_topics::from_mqtt::bridge::chatter,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const std_msgs::msg::String::SharedPtr callback_chatter_data) {
            ros_chatter_publisher_ptr_->publish(*callback_chatter_data);
        }
    );
    ros_cmd_vel_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
        ros_topics::from_mqtt::bridge::cmd_vel,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const geometry_msgs::msg::Twist::SharedPtr callback_cmd_vel_data) {
            ros_cmd_vel_publisher_ptr_->publish(*callback_cmd_vel_data);
        }
    );
    ros_initial_pose_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ros_topics::from_mqtt::bridge::initial_pose,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr callback_initial_pose_data) {
            ros_initial_pose_publisher_ptr_->publish(*callback_initial_pose_data);
        }
    );
}

/**
 * @brief Function for invoke initialize_publishers(), initialize_subscriptions()
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see rclcpp
*/
void ros_connections::ros_connections_from_mqtt::Bridge::initialize_bridge() {
    this->initialize_publishers();
    this->initialize_subscriptions();
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
    ros_connections_to_mqtt_bridge_ptr_ = new ros_connections::ros_connections_to_mqtt::Bridge(ros_node_ptr_);
    ros_connections_from_mqtt_bridge_ptr_ = new ros_connections::ros_connections_from_mqtt::Bridge(ros_node_ptr_);

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
    delete ros_connections_to_mqtt_bridge_ptr_;
    delete ros_connections_from_mqtt_bridge_ptr_;
}

/**
 * @brief Function for check rclcpp status & init logs
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @return void
 * @see rclcpp::ok()
*/
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
  _____   ____   _____ ___     _____ ____  _   _ _   _ ______ _____ _______ _____ ____  _   _   ____  _____  _____ _____   _____ ______ 
 |  __ \ / __ \ / ____|__ \   / ____/ __ \| \ | | \ | |  ____/ ____|__   __|_   _/ __ \| \ | | |  _ \|  __ \|_   _|  __ \ / ____|  ____|
 | |__) | |  | | (___    ) | | |   | |  | |  \| |  \| | |__ | |       | |    | || |  | |  \| | | |_) | |__) | | | | |  | | |  __| |__   
 |  _  /| |  | |\___ \  / /  | |   | |  | | . ` | . ` |  __|| |       | |    | || |  | | . ` | |  _ <|  _  /  | | | |  | | | |_ |  __|  
 | | \ \| |__| |____) |/ /_  | |___| |__| | |\  | |\  | |___| |____   | |   _| || |__| | |\  | | |_) | | \ \ _| |_| |__| | |__| | |____ 
 |_|  \_\\____/|_____/|____|  \_____\____/|_| \_|_| \_|______\_____|  |_|  |_____\____/|_| \_| |____/|_|  \_\_____|_____/ \_____|______|
                                                                                                                                        
                                                                                                                                        
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
    auto node = std::make_shared<RosConnectionBridge>();
    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    while(rclcpp::ok()) {
        ros_executor.spin();
    }

    return 0;
}
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

#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

/**
 * @brief Constructor for initialize this class instance & ros publishers & message coverters
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see rclcpp
*/
ros_mqtt_connections::to_ros::Bridge::Bridge(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: log_ros_mqtt_connections_to_ros(LOG_ROS_MQTT_CONNECTION_TO_ROS),
ros_node_ptr_(ros_node_ptr),
ros_default_qos_(ROS_DEFAULT_QOS) {
    std_msgs_converter_ptr_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
    geometry_msgs_converter_ptr_ = new ros_message_converter::ros_geometry_msgs::GeometryMessageConverter();

    ros_chatter_publisher_ptr_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(
        ros_topics::to_ros::chatter,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_cmd_vel_publisher_ptr_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
        ros_topics::to_ros::cmd_vel,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
    ros_initial_pose_publisher_ptr_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ros_topics::to_ros::initial_pose,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
    );
}

/**
 * @brief Virtual Destructor for this class instance & delete message coverters' pointers' instances
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
ros_mqtt_connections::to_ros::Bridge::~Bridge() {
    delete std_msgs_converter_ptr_;
    delete geometry_msgs_converter_ptr_;
}

/**
 * @brief Function for publish to ros with mqtt subscription callback data that parsed from JSON String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param mqtt_topic std::string&
 * @param mqtt_payload std::string&
 * @return void
*/
void ros_mqtt_connections::to_ros::Bridge::bridge(std::string& mqtt_topic, std::string& mqtt_payload) {
    std::cout << log_ros_mqtt_connections_to_ros << " message arrived" << '\n';
    std::cout << "\ttopic: '" << mqtt_topic << "'" << '\n';
    std::cout << "\tpayload: '" << mqtt_payload << "'" << '\n';

    if(mqtt_topic == mqtt_topics::from_rcs::chatter) {
        try {
            std::cout << log_ros_mqtt_connections_to_ros << " publish to " << mqtt_topic << '\n';
            std_msgs::msg::String std_message = std_msgs_converter_ptr_->convert_json_to_chatter(mqtt_payload);
            ros_chatter_publisher_ptr_->publish(std_message);
        } catch(const std::exception& expn) {
            std::cerr << log_ros_mqtt_connections_to_ros << " publish chatter error : " << expn.what() << '\n';
        }
    } else if(mqtt_topic == mqtt_topics::from_rcs::cmd_vel) {
        try {
            std::cout << log_ros_mqtt_connections_to_ros << " publish to " << mqtt_topic << '\n';
            geometry_msgs::msg::Twist twist_message = geometry_msgs_converter_ptr_->convert_json_to_twist(mqtt_payload);
            ros_cmd_vel_publisher_ptr_->publish(twist_message);
        } catch(const std::exception& expn) {
            std::cerr << log_ros_mqtt_connections_to_ros << " publish cmd_vel error : " << expn.what() << '\n';
        }
    } else if(mqtt_topic == mqtt_topics::from_rcs::initial_pose) {
        try {
            std::cout << log_ros_mqtt_connections_to_ros << " publish to " << mqtt_topic << '\n';
            geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped_message = geometry_msgs_converter_ptr_->convert_json_to_pose_with_covariance_stamped(mqtt_payload);
            ros_initial_pose_publisher_ptr_->publish(pose_with_covariance_stamped_message);
        } catch(const std::exception& expn) {
            std::cerr << log_ros_mqtt_connections_to_ros << " publish initial_pose error : " << expn.what() << '\n';
        }
    }
}

/**
 * @brief Constructor for initialize this class instance & message coverters
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @param ros_mqtt_connections_publihser_ptr_ ros_mqtt_connections::to_ros::Bridge
 * @see rclcpp
 * @see ros_mqtt_connections::to_ros::Bridge
 * @see mqtt::callback
*/
ros_mqtt_connections::to_mqtt::Bridge::Bridge(std::shared_ptr<rclcpp::Node> ros_node_ptr, ros_mqtt_connections::to_ros::Bridge * ros_mqtt_connections_publisher_ptr)
: log_ros_mqtt_connections_to_mqtt(LOG_ROS_MQTT_CONNECTION_TO_MQTT),
ros_node_ptr_(ros_node_ptr),
ros_default_qos_(ROS_DEFAULT_QOS),
ros_mqtt_connections_publisher_ptr_(ros_mqtt_connections_publisher_ptr),
mqtt_async_client_(MQTT_ADDRESS, MQTT_CLIENT_ID),
mqtt_qos_(MQTT_QOS),
mqtt_is_success_(mqtt::SUCCESS) {
    this->mqtt_connect();
    this->grant_mqtt_subscriptions();
    this->bridge();

    std_msgs_converter_ptr_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
    geometry_msgs_converter_ptr_ = new ros_message_converter::ros_geometry_msgs::GeometryMessageConverter();
    sensor_msgs_converter_ptr_ = new ros_message_converter::ros_sensor_msgs::SensorMessageConverter();
    nav_msgs_converter_ptr_ = new ros_message_converter::ros_nav_msgs::NavMessageConverter();
    tf2_msgs_converter_ptr_ = new ros_message_converter::ros_tf2_msgs::Tf2MessageConverter();
}

/**
 * @brief Virtual Destructor for this class & delete message coverters' pointers' instances
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
ros_mqtt_connections::to_mqtt::Bridge::~Bridge() {
    delete std_msgs_converter_ptr_;
    delete geometry_msgs_converter_ptr_;
    delete sensor_msgs_converter_ptr_;
    delete nav_msgs_converter_ptr_;
    delete tf2_msgs_converter_ptr_;
}

/**
 * @brief Function for connect to mqtt by mqtt::async_client
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see mqtt::async_client
 * @see mqtt::connect_options
 * @see mqtt::exception
*/
void ros_mqtt_connections::to_mqtt::Bridge::mqtt_connect() {
    try {
        mqtt::connect_options mqtt_connect_opts;
        mqtt_connect_opts.set_clean_session(true);
        mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(60));
        if(mqtt_async_client_.is_connected()) {
            std::cout << log_ros_mqtt_connections_to_mqtt << " MQTT connection success" << '\n';
            mqtt_async_client_.set_callback(*this);
        } else {
            std::cout << log_ros_mqtt_connections_to_mqtt << " MQTT connection failed... trying to reconnect" << '\n';
            mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(30));
        }
    } catch (const mqtt::exception& mqtt_expn) {
        std::cerr << log_ros_mqtt_connections_to_mqtt << " connection error : " << mqtt_expn.what() << '\n';
    }
}

/**
 * @brief Function for synthesize after grant mqtt subscriptions
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see mqtt_subscribe
*/
void ros_mqtt_connections::to_mqtt::Bridge::grant_mqtt_subscriptions() {
    this->mqtt_subscribe(mqtt_topics::from_rcs::chatter);
    this->mqtt_subscribe(mqtt_topics::from_rcs::cmd_vel);
    this->mqtt_subscribe(mqtt_topics::from_rcs::initial_pose);
    this->mqtt_subscribe(mqtt_topics::from_rcs::navigate_to_pose);
    this->mqtt_subscribe(mqtt_topics::from_rcs::map_server_map);
}

/**
 * @brief Overrided function for handle cause when mqtt connection lost
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param mqtt_connection_lost_cause const std::string&
 * @return void
 * @see mqtt::callback
*/
void ros_mqtt_connections::to_mqtt::Bridge::connection_lost(const std::string& mqtt_connection_lost_cause) {
    std::cerr << log_ros_mqtt_connections_to_mqtt << " connection lost : " << mqtt_connection_lost_cause << '\n';
}

/**
 * @brief Overrided function for handle message when mqtt subscription get callback mqtt message
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param mqtt_message mqtt::const_message_ptr
 * @return void
 * @see mqtt::callback
 * @see mqtt::const_message_ptr
 * @see ros_mqtt_connections::publisher::ros_chatter_publisher_ptr
*/
void ros_mqtt_connections::to_mqtt::Bridge::message_arrived(mqtt::const_message_ptr mqtt_message) {
    std::string mqtt_topic = mqtt_message->get_topic();
    std::string mqtt_payload = mqtt_message->to_string();
    ros_mqtt_connections_publisher_ptr_->bridge(mqtt_topic, mqtt_payload);
}

/**
 * @brief Overrided function for handle delivered token
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param mqtt_delivered_token mqtt::delivery_token_ptr
 * @return void
 * @see mqtt::callback
*/
void ros_mqtt_connections::to_mqtt::Bridge::delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) {
	std::cout << log_ros_mqtt_connections_to_mqtt << " delivery complete with [" << mqtt_delivered_token <<  "] \n";
}

/**
 * @brief Function for mqtt publish into mqtt Broker
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param topic char *
 * @param payload std::string
 * @return void
 * @see mqtt::message_ptr
 * @see mqtt::exception
*/
void ros_mqtt_connections::to_mqtt::Bridge::mqtt_publish(const char * mqtt_topic, std::string mqtt_payload) {
	try {
		mqtt::message_ptr mqtt_publish_msg = mqtt::make_message(mqtt_topic, mqtt_payload);
		mqtt_publish_msg->set_qos(mqtt_qos_);
		auto delivery_token = mqtt_async_client_.publish(mqtt_publish_msg);
        delivery_token->wait();
        if (delivery_token->get_return_code() != mqtt_is_success_) {
            std::cerr << log_ros_mqtt_connections_to_mqtt << " publishing error : " << delivery_token->get_return_code() << '\n';
        }
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << log_ros_mqtt_connections_to_mqtt << " publishing error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Function for create mqtt subscription from mqtt broker
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param topic char *
 * @see mqtt::exception
*/
void ros_mqtt_connections::to_mqtt::Bridge::mqtt_subscribe(const char * mqtt_topic) {
	try {
		std::cout << log_ros_mqtt_connections_to_mqtt << " grant subscription with '" << mqtt_topic << "' " << '\n';
		mqtt_async_client_.subscribe(mqtt_topic, mqtt_qos_);
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << log_ros_mqtt_connections_to_mqtt << " grant subscription error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Function for create ros subscription with mqtt publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see rclcpp
 * @see ros_mqtt_connections
 * @see ros_mqtt_topics
*/
void ros_mqtt_connections::to_mqtt::Bridge::bridge() {
    ros_chatter_subscription_ptr_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        ros_topics::from_ros::chatter,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const std_msgs::msg::String::SharedPtr callback_chatter_data) {
            std::string chatter_json_str = std_msgs_converter_ptr_->convert_chatter_to_json(callback_chatter_data);
            mqtt_publish(mqtt_topics::to_rcs::chatter, chatter_json_str);
        }
    );
    ros_robot_pose_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Pose>(
        ros_topics::from_ros::robot_pose,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const geometry_msgs::msg::Pose::SharedPtr callback_robot_pose_data) {
            std::string robot_pose_json_str = geometry_msgs_converter_ptr_->convert_pose_to_json(callback_robot_pose_data);
            mqtt_publish(mqtt_topics::to_rcs::robot_pose, robot_pose_json_str);
        }
    );
    ros_cmd_vel_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
        ros_topics::from_ros::cmd_vel,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const geometry_msgs::msg::Twist::SharedPtr callback_twist_data) {
            std::string twist_json_str = geometry_msgs_converter_ptr_->convert_twist_to_json(callback_twist_data);
            mqtt_publish(mqtt_topics::to_rcs::cmd_vel, twist_json_str);
        }
    );
    ros_scan_subscription_ptr_ = ros_node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
        ros_topics::from_ros::scan,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr callback_scan_data) {
            std::string scan_json_str = sensor_msgs_converter_ptr_->convert_scan_to_json(callback_scan_data);
            mqtt_publish(mqtt_topics::to_rcs::scan, scan_json_str);
        }
    );
    ros_tf_subscription_ptr_ = ros_node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
        ros_topics::from_ros::tf,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr callback_tf_data) {
            std::string tf_json_str = tf2_msgs_converter_ptr_->convert_tf_to_json(callback_tf_data);
            mqtt_publish(mqtt_topics::to_rcs::tf, tf_json_str);
        }
    );
    ros_tf_static_subscription_ptr_ = ros_node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
        ros_topics::from_ros::tf_static,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr callback_tf_static_data) {
            std::string tf_static_json_str = tf2_msgs_converter_ptr_->convert_tf_to_json(callback_tf_static_data);
            mqtt_publish(mqtt_topics::to_rcs::tf_static, tf_static_json_str);
        }
    );
    ros_odom_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_topics::from_ros::odom,
        rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
        [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_data) {
            std::string odom_json_str = nav_msgs_converter_ptr_->convert_odom_to_json(callback_odom_data);
            mqtt_publish(mqtt_topics::to_rcs::odom, odom_json_str);
        }
    );
}

/**
 * @brief Constructor for initialize this class instance & create rclcpp::Node named with ros_mqtt_bridge
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @see rclcpp::Node
 * @see ros_mqtt_connections
*/
RosMqttBridge::RosMqttBridge()
: Node("ros_mqtt_bridge"),
log_ros_mqtt_bridge_(LOG_ROS_MQTT_BRIDGE) {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_mqtt_connections_to_ros_bridge_ptr_ = new ros_mqtt_connections::to_ros::Bridge(ros_node_ptr_);
    ros_mqtt_conenction_to_mqtt_bridge_ptr_ = new ros_mqtt_connections::to_mqtt::Bridge(ros_node_ptr_, ros_mqtt_connections_to_ros_bridge_ptr_);
}

/**
 * @brief Virtual Destructor for this class & delete ros_mqtt_connections bridge classes' pointers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
RosMqttBridge::~RosMqttBridge() {
    delete ros_mqtt_connections_to_ros_bridge_ptr_;
    delete ros_mqtt_connections_to_ros_bridge_ptr_;
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
     _____   ____   _____ ___    __  __  ____ _______ _______   ____  _____  _____ _____   _____ ______ 
    |  __ \ / __ \ / ____|__ \  |  \/  |/ __ \__   __|__   __| |  _ \|  __ \|_   _|  __ \ / ____|  ____|
    | |__) | |  | | (___    ) | | \  / | |  | | | |     | |    | |_) | |__) | | | | |  | | |  __| |__   
    |  _  /| |  | |\___ \  / /  | |\/| | |  | | | |     | |    |  _ <|  _  /  | | | |  | | | |_ |  __|  
    | | \ \| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |_) | | \ \ _| |_| |__| | |__| | |____ 
    |_|  \_\\____/|_____/|____| |_|  |_|\___\_\ |_|     |_|    |____/|_|  \_\_____|_____/ \_____|______|                                                                                                     
                                                                                                     
        )" << '\n';
    } else {
        std::cerr << "[ros_mqtt_bridge] rclcpp is not ok" << '\n';
    }
}

/**
 * @brief Function for initialize this class instance & initialize rclcpp & spin ros_mqtt_bridge rclcpp::Node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param argc int
 * @param argv char**
 * @return int
 * @see rclcpp
 * @see RosMqttBridge
 * @see check_rclcpp_status()
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    check_rclcpp_status();
    auto node = std::make_shared<RosMqttBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
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
 * @brief Constructor for initialize this class instance & message coverters
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @param ros_mqtt_connections_publihser_ptr_ ros_mqtt_connections::to_ros::Bridge
 * @see rclcpp
 * @see ros_mqtt_connections::to_ros::Bridge
 * @see mqtt::callback
*/
ros_mqtt_connections::manager::Bridge::Bridge(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: log_ros_mqtt_bridge_(LOG_ROS_MQTT_BRIDGE),
log_ros_mqtt_connections_to_mqtt_(LOG_ROS_MQTT_CONNECTION_TO_MQTT),
log_ros_mqtt_connections_to_ros_(LOG_ROS_MQTT_CONNECTION_TO_ROS),
ros_node_ptr_(ros_node_ptr),
ros_default_qos_(ROS_DEFAULT_QOS),
mqtt_async_client_(MQTT_ADDRESS, MQTT_CLIENT_ID),
mqtt_qos_(MQTT_QOS),
mqtt_is_success_(mqtt::SUCCESS) {
    this->mqtt_connect();
    this->grant_mqtt_subscriptions();
    this->bridge_ros_to_mqtt();
    this->bridge_mqtt_to_ros();

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
ros_mqtt_connections::manager::Bridge::~Bridge() {
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
void ros_mqtt_connections::manager::Bridge::mqtt_connect() {
    try {
        mqtt::connect_options mqtt_connect_opts;
        mqtt_connect_opts.set_clean_session(true);
        mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(60));
        if(mqtt_async_client_.is_connected()) {
            std::cout << log_ros_mqtt_bridge_ << " MQTT connection success" << '\n';
            mqtt_async_client_.set_callback(*this);
        } else {
            std::cout << log_ros_mqtt_bridge_ << " MQTT connection failed... trying to reconnect" << '\n';
            mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(30));
        }
    } catch (const mqtt::exception& mqtt_expn) {
        std::cerr << log_ros_mqtt_bridge_ << " connection error : " << mqtt_expn.what() << '\n';
    }
}

/**
 * @brief Function for synthesize after grant mqtt subscriptions
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @return void
 * @see mqtt_subscribe
*/
void ros_mqtt_connections::manager::Bridge::grant_mqtt_subscriptions() {
    this->mqtt_subscribe(mqtt_topics::from_rcs::chatter);
    this->mqtt_subscribe(mqtt_topics::from_rcs::cmd_vel);
    this->mqtt_subscribe(mqtt_topics::from_rcs::initial_pose);
    this->mqtt_subscribe(mqtt_topics::from_rcs::add_two_ints);
    this->mqtt_subscribe(mqtt_topics::from_rcs::map_server_map);
    // this->mqtt_subscribe(mqtt_topics::from_rcs::navigate_to_pose);
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
void ros_mqtt_connections::manager::Bridge::bridge_ros_to_mqtt() {
    try {
        ros_chatter_subscription_ptr_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
            ros_topics::from_ros::chatter,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const std_msgs::msg::String::SharedPtr callback_chatter_data) {
                if(callback_chatter_data == nullptr || callback_chatter_data == NULL) throw std::runtime_error("[ROS to MQTT] chatter callback is null");
                std::string chatter_json_str = std_msgs_converter_ptr_->convert_chatter_to_json(callback_chatter_data);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::chatter, chatter_json_str);
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /chatter mqtt response err : " << mqtt_expn.what() << '\n';
                }
            }
        );;
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /chatter bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_robot_pose_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Pose>(
            ros_topics::from_ros::robot_pose,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const geometry_msgs::msg::Pose::SharedPtr callback_robot_pose_data) {
                if(callback_robot_pose_data == nullptr || callback_robot_pose_data == NULL) throw std::runtime_error("[ROS to MQTT] robot_pose callback is null");
                std::string robot_pose_json_str = geometry_msgs_converter_ptr_->convert_pose_to_json(callback_robot_pose_data);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::robot_pose, robot_pose_json_str);   
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /robot_pose mqtt response err : " << mqtt_expn.what() << '\n';
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /robot_pose bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_cmd_vel_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
            ros_topics::from_ros::cmd_vel,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const geometry_msgs::msg::Twist::SharedPtr callback_twist_data) {
                if(callback_twist_data == nullptr || callback_twist_data == NULL) throw std::runtime_error("[ROS to MQTT] twist callback is null");
                std::string twist_json_str = geometry_msgs_converter_ptr_->convert_twist_to_json(callback_twist_data);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::cmd_vel, twist_json_str);
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /cmd_vel mqtt response err : "  << '\n';
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /cmd_vel bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_scan_subscription_ptr_ = ros_node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
            ros_topics::from_ros::scan,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr callback_scan_data) {
                if(callback_scan_data == nullptr || callback_scan_data == NULL) throw std::runtime_error("[ROS to MQTT] scan callback is null");
                std::string scan_json_str = sensor_msgs_converter_ptr_->convert_scan_to_json(callback_scan_data);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::scan, scan_json_str);
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /scan mqtt response err : "  << '\n';
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /scan bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_tf_subscription_ptr_ = ros_node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
            ros_topics::from_ros::tf,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr callback_tf_data) {
                if(callback_tf_data == nullptr || callback_tf_data == NULL) throw std::runtime_error("[ROS to MQTT] tf callback is null");
                std::string tf_json_str = tf2_msgs_converter_ptr_->convert_tf_to_json(callback_tf_data);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::tf, tf_json_str);
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /tf mqtt response err : "  << '\n';
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /tf bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_tf_static_subscription_ptr_ = ros_node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
            ros_topics::from_ros::tf_static,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr callback_tf_static_data) {
                if(callback_tf_static_data == nullptr || callback_tf_static_data == NULL) throw std::runtime_error("[ROS to MQTT] tf_static callback is null");
                std::string tf_static_json_str = tf2_msgs_converter_ptr_->convert_tf_to_json(callback_tf_static_data);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::tf_static, tf_static_json_str);
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /tf_static mqtt response err : "  << '\n';
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /tf bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_odom_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
            ros_topics::from_ros::odom,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_data) {
                if(callback_odom_data == nullptr || callback_odom_data == NULL) throw std::runtime_error("[ROS to MQTT] odom callback is null");
                std::string odom_json_str = nav_msgs_converter_ptr_->convert_odom_to_json(callback_odom_data);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::odom, odom_json_str);
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /odom mqtt response err : "  << '\n';
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /odom bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_add_two_ints_subscription_ptr_ = ros_node_ptr_->create_subscription<example_interfaces::srv::AddTwoInts_Response>(
            ros_topics::from_ros::add_two_ints,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const example_interfaces::srv::AddTwoInts_Response::SharedPtr callback_add_two_ints_data) {
                if(callback_add_two_ints_data == nullptr || callback_add_two_ints_data == NULL) throw std::runtime_error("[ROS to MQTT] add two ints callback is null");
                std::string add_two_ints_response = std::to_string(callback_add_two_ints_data->sum);
                try {
                    mqtt_publish(mqtt_topics::to_rcs::add_two_ints, add_two_ints_response);
                } catch(const mqtt::exception& mqtt_expn) {
                    std::cerr << "[ROS to MQTT] /add_two_ints/response mqtt response err : "  << '\n';
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /add_two_ints bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_map_server_map_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::srv::GetMap_Response>(
            ros_topics::from_ros::map_server_map,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_)),
            [this](const nav_msgs::srv::GetMap_Response::SharedPtr callback_map_server_map_data) {
                if(callback_map_server_map_data == nullptr || callback_map_server_map_data == NULL) {
                    throw std::runtime_error("[ROS to MQTT] map server map callback is null");
                } else if(callback_map_server_map_data->map.header.frame_id == ros_services::exceptions::map_server_map_timed_out) {
                    std::cerr << "[ROS to MQTT] /map_server/map service timed out"  << '\n';
                    try {
                        mqtt_publish(mqtt_topics::to_rcs::map_server_map, ros_services::exceptions::map_server_map_timed_out);
                    } catch(const mqtt::exception& mqtt_expn) {
                        std::cerr << "[ROS to MQTT] /map_server/map/response mqtt response err : " << mqtt_expn.what()  << '\n';
                    }
                } else {
                    std::cout << "[ROS to MQTT] /map_server/map/response callback : " << callback_map_server_map_data->map.info.width << '\n';
                    std::string parsed_map_server_map_response = nav_msgs_converter_ptr_->convert_map_response_to_json(callback_map_server_map_data);
                    std::cout << "[MQTT to ROS] parsed map result : " << parsed_map_server_map_response << '\n';
                    try {
                        mqtt_publish(mqtt_topics::to_rcs::map_server_map, parsed_map_server_map_response);
                    } catch(const mqtt::exception& mqtt_expn) {
                        std::cerr << "[ROS to MQTT] /map_server/map/response mqtt response err : " << mqtt_expn.what()  << '\n';
                    }
                }
            }
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[ROS to MQTT] /map_server_map bridge err : " << rcl_expn.what() << '\n';
    }
}

/**
 * @brief Function for initialize ros publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @return void
*/
void ros_mqtt_connections::manager::Bridge::bridge_mqtt_to_ros() {
    try {
        ros_chatter_publisher_ptr_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(
            ros_topics::to_ros::chatter,
            rclcpp::QoS(ros_default_qos_)
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[MQTT to ROS] /chatter bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_cmd_vel_publisher_ptr_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
            ros_topics::to_ros::cmd_vel,
            rclcpp::QoS(ros_default_qos_)
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[MQTT to ROS] /cmd_vel bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_initial_pose_publisher_ptr_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            ros_topics::to_ros::initial_pose,
            rclcpp::QoS(ros_default_qos_)
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[MQTT to ROS] /initialpose bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_add_two_ints_service_client_ptr_ = ros_node_ptr_->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_service");
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[MQTT to ROS] /add_two_ints bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_add_two_ints_publisher_ptr_ = ros_node_ptr_->create_publisher<example_interfaces::srv::AddTwoInts_Response>(
            ros_topics::to_ros::add_two_ints,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[MQTT to ROS] /add_two_ints publish bridge err : " << rcl_expn.what() << '\n';
    }

    try {
        ros_map_server_map_publisher_ptr_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(
            ros_topics::to_ros::map_server_map,
            rclcpp::QoS(rclcpp::KeepLast(ros_default_qos_))
        );
    } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
        std::cerr << "[MQTT to ROS] /map_server/map publish bridge err : " << rcl_expn.what() << '\n';
    }
}

/**
 * @brief Function for publish to ros with mqtt subscription callback data that parsed from JSON String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param mqtt_topic std::string&
 * @param mqtt_payload std::string&
 * @return void
*/
void ros_mqtt_connections::manager::Bridge::bridge_mqtt_to_ros(std::string& mqtt_topic, std::string& mqtt_payload) {
    std::cout << "[MQTT to ROS] message arrived" << '\n';
    std::cout << "\ttopic: '" << mqtt_topic << "'" << '\n';
    std::cout << "\tpayload: '" << mqtt_payload << "'" << '\n';

    if(mqtt_topic == mqtt_topics::from_rcs::chatter) {
        try {
            std::cout << "[MQTT to ROS] publish to " << mqtt_topic << '\n';
            std_msgs::msg::String std_message = std_msgs_converter_ptr_->convert_json_to_chatter(mqtt_payload);
            ros_chatter_publisher_ptr_->publish(std_message);
        } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
            std::cerr << "[MQTT to ROS] publish chatter error : " << rcl_expn.what() << '\n';
        }
    } else if(mqtt_topic == mqtt_topics::from_rcs::cmd_vel) {
        try {
            std::cout << "[MQTT to ROS] publish to " << mqtt_topic << '\n';
            geometry_msgs::msg::Twist twist_message = geometry_msgs_converter_ptr_->convert_json_to_twist(mqtt_payload);
            ros_cmd_vel_publisher_ptr_->publish(twist_message);
        } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
            std::cerr << "[MQTT to ROS] publish cmd_vel error : " << rcl_expn.what() << '\n';
        }
    } else if(mqtt_topic == mqtt_topics::from_rcs::initial_pose) {
        try {
            std::cout << "[MQTT to ROS] publish to " << mqtt_topic << '\n';
            geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped_message = geometry_msgs_converter_ptr_->convert_json_to_pose_with_covariance_stamped(mqtt_payload);
            ros_initial_pose_publisher_ptr_->publish(pose_with_covariance_stamped_message);
        } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
            std::cerr << "[MQTT to ROS] publish initial_pose error : " << rcl_expn.what() << '\n';
        }
    } else if(mqtt_topic == mqtt_topics::from_rcs::add_two_ints) {
        try {
            std::cout << "[MQTT to ROS] service call to /add_two_ints_service " << '\n';
            bool is_add_two_ints_service_ready = ros_add_two_ints_service_client_ptr_->wait_for_service(std::chrono::seconds(1));
            while (!is_add_two_ints_service_ready) {
                if (!rclcpp::ok()) {
                    std::cout << "[MQTT to ROS] interrupted while waiting /add_two_ints service..." << '\n';
                    break;
                } else if(is_add_two_ints_service_ready) {
                    std::cout << "[MQTT to ROS] /add_two_ints service is ready!" << '\n';
                    break;
                }
            }
            std::shared_ptr<example_interfaces::srv::AddTwoInts_Request> add_two_ints_request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            add_two_ints_request->a = 40;
            add_two_ints_request->b = 60;
            std::shared_future<std::shared_ptr<example_interfaces::srv::AddTwoInts_Response>> add_two_ints_result_future = ros_add_two_ints_service_client_ptr_->async_send_request(add_two_ints_request);
            const std::shared_ptr<example_interfaces::srv::AddTwoInts_Response> add_two_ints_service_call_result = add_two_ints_result_future.get();
            std::cout << "[MQTT to ROS] /add_two_ints result of " << add_two_ints_request->a << " + " << add_two_ints_request->b << " = " << add_two_ints_service_call_result->sum << '\n';
            ros_add_two_ints_publisher_ptr_->publish(*add_two_ints_service_call_result);
        } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
            std::cerr << "[MQTT to ROS] call /add_two_ints error : " << rcl_expn.what() << '\n';
        }
    } else if(mqtt_topic == mqtt_topics::from_rcs::map_server_map) {
        try {
            const std_msgs::msg::String::SharedPtr empty_request = std::make_shared<std_msgs::msg::String>();
            ros_map_server_map_publisher_ptr_->publish(*empty_request);
        } catch(const rclcpp::exceptions::RCLError& rcl_expn) {
            std::cerr << "[MQTT to ROS] call /map_server/map error : " << rcl_expn.what() << '\n';
        }
    } else {
        return;
    }
}

/**
 * @brief Overrided function for handle cause when mqtt connection lost
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param mqtt_connection_lost_cause const std::string&
 * @return void
 * @see mqtt::callback
*/
void ros_mqtt_connections::manager::Bridge::connection_lost(const std::string& mqtt_connection_lost_cause) {
    std::cerr << log_ros_mqtt_bridge_ << " connection lost : " << mqtt_connection_lost_cause << '\n';
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
void ros_mqtt_connections::manager::Bridge::message_arrived(mqtt::const_message_ptr mqtt_message) {
    std::string mqtt_topic = mqtt_message->get_topic();
    std::string mqtt_payload = mqtt_message->to_string();
    this->bridge_mqtt_to_ros(mqtt_topic, mqtt_payload);
}

/**
 * @brief Overrided function for handle delivered token
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param mqtt_delivered_token mqtt::delivery_token_ptr
 * @return void
 * @see mqtt::callback
*/
void ros_mqtt_connections::manager::Bridge::delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) {
	std::cout << log_ros_mqtt_bridge_ << " delivery complete with [" << mqtt_delivered_token <<  "] \n";
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
void ros_mqtt_connections::manager::Bridge::mqtt_publish(const char * mqtt_topic, std::string mqtt_payload) {
	try {
		mqtt::message_ptr mqtt_publish_msg = mqtt::make_message(mqtt_topic, mqtt_payload);
		mqtt_publish_msg->set_qos(mqtt_qos_);
		auto delivery_token = mqtt_async_client_.publish(mqtt_publish_msg);
        delivery_token->wait();
        if (delivery_token->get_return_code() != mqtt_is_success_) {
            std::cerr << log_ros_mqtt_connections_to_mqtt_ << " publishing error : " << delivery_token->get_return_code() << '\n';
        }
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << log_ros_mqtt_connections_to_mqtt_ << " publishing error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Function for create mqtt subscription from mqtt broker
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @param topic char *
 * @see mqtt::exception
*/
void ros_mqtt_connections::manager::Bridge::mqtt_subscribe(const char * mqtt_topic) {
	try {
		std::cout << log_ros_mqtt_connections_to_ros_ << " grant subscription with '" << mqtt_topic << "' " << '\n';
		mqtt_async_client_.subscribe(mqtt_topic, mqtt_qos_);
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << log_ros_mqtt_connections_to_ros_ << " grant subscription error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Constructor for initialize this class instance & create rclcpp::Node named with ros_mqtt_bridge
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @see rclcpp::Node
 * @see ros_mqtt_connections
*/
RosMqttBridge::RosMqttBridge()
: Node("ros_mqtt_bridge") {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_mqtt_conenctions_to_mqtt_bridge_ptr_ = new ros_mqtt_connections::manager::Bridge(ros_node_ptr_);
}

/**
 * @brief Virtual Destructor for this class & delete ros_mqtt_connections bridge classes' pointers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
RosMqttBridge::~RosMqttBridge() {
    delete ros_mqtt_conenctions_to_mqtt_bridge_ptr_;
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
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

#ifndef ROS_MQTT_MESSAGE_CONVERTER
#define ROS_MQTT_MESSAGE_CONVERTER

/**
 * include cpp header files
 * @see iostream
 * @see math.h
 * @see unistd.h
 * @see signal.h
 * @see functional
*/
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>
#include <jsoncpp/json/json.h>

/**
 * include rclcpp header files
 * @see rclcpp/rclcpp.hpp
*/
#include "rclcpp/rclcpp.hpp"

/**
 * include std_msgs::msg::String header file
 * @see std_msgs::msg::String
*/
#include "std_msgs/msg/string.hpp"

/**
 * include geometry_msgs' header files
 * @see geometry_msgs::msg::Twsit
 * @see geometry_msgs::msg::PoseWithCovarianceStamped
*/
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

/**
 * include sensor_msgs' header files
 * @see sensor_msgs::msg::LaserScan
*/
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * include nav_msgs' header files
 * @see nav_msgs::msg::path
 * @see nav_msgs::srv::GetMap
 * @see nav_msgs::msg::Odometry
 * @see nav2_msgs::action::Navigate_To_Pose
*/
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

/**
 * include tf2_msgs' header files
 * @see tf2_msgs::msg::TFMessage
*/
#include "tf2_msgs/msg/tf_message.hpp"

/**
 * @brief namespace for declare Converter Classes for each message types
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.06
*/
namespace ros_message_converter {
    namespace ros_std_msgs {
        class StdMessageConverter {                
            public :
                StdMessageConverter();
                virtual ~StdMessageConverter();
                Json::Value convert_header_to_json(const std_msgs::msg::Header header_msgs);
                std_msgs::msg::Header convert_json_to_header(Json::Value raw_header_data);
                std::string convert_chatter_to_json(const std_msgs::msg::String::SharedPtr chatter_msgs_ptr);
                std_msgs::msg::String convert_json_to_chatter(std::string& raw_std_string_data);
        };
    }
    namespace ros_geometry_msgs {
        class GeometryMessageConverter {
            private :
                ros_message_converter::ros_std_msgs::StdMessageConverter * std_message_converter_;
            public :
                GeometryMessageConverter();
                virtual ~GeometryMessageConverter();
                Json::Value convert_point_to_json(const geometry_msgs::msg::Point point_msgs);
                Json::Value convert_quaternion_to_json(const geometry_msgs::msg::Quaternion quaternion_msgs);
                std::string convert_pose_to_json(const geometry_msgs::msg::Pose::SharedPtr pose_msgs_ptr);
                Json::Value convert_pose_to_json(const geometry_msgs::msg::Pose pose_msgs);
                Json::Value convert_vector_to_json(const geometry_msgs::msg::Vector3 vector_msgs);
                std::string convert_twist_to_json(const geometry_msgs::msg::Twist::SharedPtr twistmsgs_ptr);
                geometry_msgs::msg::Vector3 convert_json_to_vector(Json::Value raw_vector_data);
                geometry_msgs::msg::Twist convert_json_to_twist(std::string& raw_twist_data);
                geometry_msgs::msg::Point convert_json_to_point(Json::Value raw_point_data);
                geometry_msgs::msg::Quaternion convert_json_to_quaternion(Json::Value raw_quaternion_data);
                std::array<double, 36UL> convert_json_to_pose_covariance(Json::Value raw_pose_data);
                geometry_msgs::msg::PoseWithCovarianceStamped convert_json_to_pose_with_covariance_stamped(std::string& raw_pose_with_covariance_stamped_data);
        };
    }
    namespace ros_sensor_msgs {
        class SensorMessageConverter {
            private :
                ros_message_converter::ros_std_msgs::StdMessageConverter * std_message_converter_;
            public :
                SensorMessageConverter();
                virtual ~SensorMessageConverter();
                std::string convert_scan_to_json(const sensor_msgs::msg::LaserScan::SharedPtr scan_msgs_ptr);
        };
    }
    namespace ros_nav_msgs {
        class NavMessageConverter {
            private :
                ros_message_converter::ros_std_msgs::StdMessageConverter * std_message_converter_;
                ros_message_converter::ros_geometry_msgs::GeometryMessageConverter * geometry_message_converter_;
            public:
                NavMessageConverter();
                virtual ~NavMessageConverter();
                std::string convert_odom_to_json(const nav_msgs::msg::Odometry::SharedPtr odom_msgs_ptr);
                std::string convert_path_to_json(const nav_msgs::msg::Path::SharedPtr path_msgs_ptr);
                Json::Value convert_meta_data_to_json(const nav_msgs::msg::MapMetaData map_meta_data_msgs);
                std::string convert_map_response_to_json(const nav_msgs::srv::GetMap_Response::SharedPtr map_response_msgs_ptr);
        };
    }
    namespace ros_tf2_msgs {
        class Tf2MessageConverter {
            private :
                ros_message_converter::ros_std_msgs::StdMessageConverter * std_message_converter_;
                ros_message_converter::ros_geometry_msgs::GeometryMessageConverter * geometry_message_converter_;
            public :
                Tf2MessageConverter();
                virtual ~Tf2MessageConverter();
                std::string convert_tf_to_json(const tf2_msgs::msg::TFMessage::SharedPtr tf_msgs_ptr);
        };
    }
}

#endif
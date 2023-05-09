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

#ifndef ROS_MESSAGE_CONVERTER
#define ROS_MESSAGE_CONVERTER

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
 * @see std_msgs/msgs/string.hpp
 * @see nav_msgs/msg/odometry.hpp
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
                std::string convert_chatter_to_json(const std_msgs::msg::String::SharedPtr chatter_msgs_ptr);
                std_msgs::msg::String::SharedPtr convert_json_to_chatter(Json::Value received_json);
        };
    }
    namespace ros_geometry_msgs {
        class GeometryMessageConverter {
            public :
                GeometryMessageConverter();
                virtual ~GeometryMessageConverter();
        };
    }
    namespace ros_sensor_msgs {
        class SensorMessageConverter {
            public :
                SensorMessageConverter();
                virtual ~SensorMessageConverter();
                std::string convert_scan_to_json(const sensor_msgs::msg::LaserScan::SharedPtr scan_msgs_ptr);
        };
    }
    namespace ros_nav_msgs {
        class NavMessageConverter {
            public:
                NavMessageConverter();
                virtual ~NavMessageConverter();
                std::string convert_odom_to_json(const nav_msgs::msg::Odometry::SharedPtr odom_msgs_ptr);
        };
    }
    namespace ros_tf2_msgs {
        class Tf2MessageConverter {
            public :
                Tf2MessageConverter();
                virtual ~Tf2MessageConverter();
                std::string convert_tf2_to_json(const tf2_msgs::msg::TFMessage::SharedPtr tf2_msgs_ptr);
        };
    }
}

#endif
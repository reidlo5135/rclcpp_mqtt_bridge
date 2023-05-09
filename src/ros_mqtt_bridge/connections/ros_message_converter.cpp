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

#include "ros_mqtt_bridge/connections/ros_message_converter.hpp"


/**
 * @brief Constructor for initialize this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.06
*/
ros_message_converter::ros_std_msgs::StdMessageConverter::StdMessageConverter() {

}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.06
*/
ros_message_converter::ros_std_msgs::StdMessageConverter::~StdMessageConverter() {

}

/**
 * @brief Function for convert std_msgs::msg::String data into Json String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.06
 * @param chatter_msg_ptr std_msgs::msg::String::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_std_msgs::StdMessageConverter::convert_chatter_to_json(const std_msgs::msg::String::SharedPtr chatter_msgs_ptr) {
    Json::Value chatter_json;
    chatter_json["data"] = chatter_msgs_ptr->data;

    std::string chatter_json_str = Json::StyledWriter().write(chatter_json);
    return chatter_json_str;
}

/**
 * @brief Constructor for initialize this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
ros_message_converter::ros_sensor_msgs::SensorMessageConverter::SensorMessageConverter() {

}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
ros_message_converter::ros_sensor_msgs::SensorMessageConverter::~SensorMessageConverter() {

}

/**
 * @brief Function for convert sensor_msgs::msg::LaserScan data into Json String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param scan_msgs_ptr sensor_msgs::msg::LaserScan::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_sensor_msgs::SensorMessageConverter::convert_scan_to_json(const sensor_msgs::msg::LaserScan::SharedPtr scan_msgs_ptr) {
    Json::Value scan_json;
    scan_json["header"]["frame_id"] = scan_msgs_ptr->header.frame_id;
    scan_json["header"]["seq"] = scan_msgs_ptr->header.stamp.sec;
    scan_json["header"]["stamp"] = scan_msgs_ptr->header.stamp.sec + scan_msgs_ptr->header.stamp.nanosec * 1e-9;

    scan_json["angle_min"] = scan_msgs_ptr->angle_min;
    scan_json["angle_max"] = scan_msgs_ptr->angle_max;
    scan_json["angle_increment"] = scan_msgs_ptr->angle_increment;

    scan_json["time_increment"] = scan_msgs_ptr->time_increment;
    scan_json["scan_time"] = scan_msgs_ptr->scan_time;

    scan_json["range_min"] = scan_msgs_ptr->range_min;
    scan_json["range_max"] = scan_msgs_ptr->range_max;
    for (const auto& range : scan_msgs_ptr->ranges) {
        scan_json["ranges"].append(range);
    }

    for (const auto& intense : scan_msgs_ptr->intensities) {
        scan_json["intensities"].append(intense);
    }

    std::string scan_json_str = Json::StyledWriter().write(scan_json);
    return scan_json_str;
}

/**
 * @brief Constructor for initialize this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.06
*/
ros_message_converter::ros_nav_msgs::NavMessageConverter::NavMessageConverter() {

}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.06
*/
ros_message_converter::ros_nav_msgs::NavMessageConverter::~NavMessageConverter() {

}

/**
 * @brief Function for convert nav_msgs::msg::Odometry data into Json String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.06
 * @param odom_msgs_ptr nav_msgs::msg::Odometry
 * @return std::string
*/
std::string ros_message_converter::ros_nav_msgs::NavMessageConverter::convert_odom_to_json(const nav_msgs::msg::Odometry::SharedPtr odom_msgs_ptr) {
    Json::Value odom_json;
    odom_json["header"]["frame_id"] = odom_msgs_ptr->header.frame_id;
    odom_json["header"]["seq"] = odom_msgs_ptr->header.stamp.sec;
    odom_json["header"]["stamp"] = odom_msgs_ptr->header.stamp.sec + odom_msgs_ptr->header.stamp.nanosec * 1e-9;
    odom_json["child_frame_id"] = odom_msgs_ptr->child_frame_id;

    odom_json["pose"]["pose"]["position"]["x"] = odom_msgs_ptr->pose.pose.position.x;
    odom_json["pose"]["pose"]["position"]["y"] = odom_msgs_ptr->pose.pose.position.y;
    odom_json["pose"]["pose"]["position"]["z"] = odom_msgs_ptr->pose.pose.position.z;
    odom_json["pose"]["pose"]["orientation"]["x"] = odom_msgs_ptr->pose.pose.orientation.x;
    odom_json["pose"]["pose"]["orientation"]["y"] = odom_msgs_ptr->pose.pose.orientation.y;
    odom_json["pose"]["pose"]["orientation"]["z"] = odom_msgs_ptr->pose.pose.orientation.z;
    odom_json["pose"]["pose"]["orientation"]["w"] = odom_msgs_ptr->pose.pose.orientation.w;
    odom_json["pose"]["covariance"] = Json::arrayValue;
    for (const auto& cov : odom_msgs_ptr->pose.covariance) {
        odom_json["pose"]["covariance"].append(cov);
    }

    odom_json["twist"]["twist"]["linear"]["x"] = odom_msgs_ptr->twist.twist.linear.x;
    odom_json["twist"]["twist"]["linear"]["y"] = odom_msgs_ptr->twist.twist.linear.y;
    odom_json["twist"]["twist"]["linear"]["z"] = odom_msgs_ptr->twist.twist.linear.z;
    odom_json["twist"]["twist"]["angular"]["x"] = odom_msgs_ptr->twist.twist.angular.x;
    odom_json["twist"]["twist"]["angular"]["y"] = odom_msgs_ptr->twist.twist.angular.y;
    odom_json["twist"]["twist"]["angular"]["z"] = odom_msgs_ptr->twist.twist.angular.z;
    odom_json["twist"]["covariance"] = Json::arrayValue;
    for (const auto& cov : odom_msgs_ptr->twist.covariance) {
        odom_json["twist"]["covariance"].append(cov);
    }

    std::string odom_json_str = Json::StyledWriter().write(odom_json);
    return odom_json_str;
}

ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::Tf2MessageConverter() {

}

ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::~Tf2MessageConverter() {

}

std::string ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::convert_tf2_to_json(const tf2_msgs::msg::TFMessage::SharedPtr tf2_msgs_ptr) {
    Json::Value tf2_json;

    for(const auto& transform : tf2_msgs_ptr->transforms) {
        tf2_json["header"]["frame_id"] = transform.header.frame_id;
        tf2_json["header"]["seq"] = transform.header.stamp.sec;
        tf2_json["header"]["stamp"] = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9;
        tf2_json["child_frame_id"] = transform.child_frame_id;
        tf2_json["transform"]["translation"]["x"] = transform.transform.translation.x;
        tf2_json["transform"]["translation"]["y"] = transform.transform.translation.y;
        tf2_json["transform"]["translation"]["z"] = transform.transform.translation.z;
        tf2_json["transform"]["rotation"]["x"] = transform.transform.rotation.x;
        tf2_json["transform"]["rotation"]["x"] = transform.transform.rotation.y;
        tf2_json["transform"]["rotation"]["x"] = transform.transform.rotation.z;
        tf2_json["transform"]["rotation"]["x"] = transform.transform.rotation.w;
    }
    
    std::string tf2_json_str = Json::StyledWriter().write(tf2_json);
    return tf2_json_str;
}
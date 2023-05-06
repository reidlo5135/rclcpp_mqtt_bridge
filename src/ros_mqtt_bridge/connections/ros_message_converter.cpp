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

ros_message_converter::ros_std_msgs::StdMessageConverter::StdMessageConverter() {

}

ros_message_converter::ros_std_msgs::StdMessageConverter::~StdMessageConverter() {

}

std::string ros_message_converter::ros_std_msgs::StdMessageConverter::convert_chatter_to_json(const std_msgs::msg::String::SharedPtr chatter_msgs_ptr) {
    Json::Value json_chatter;
    json_chatter["data"] = chatter_msgs_ptr->data;

    std::string chatter_json_str = Json::StyledWriter().write(json_chatter);
    return chatter_json_str;
}

ros_message_converter::ros_nav_msgs::NavMessageConverter::NavMessageConverter() {

}

ros_message_converter::ros_nav_msgs::NavMessageConverter::~NavMessageConverter() {

}

std::string ros_message_converter::ros_nav_msgs::NavMessageConverter::convert_odom_to_json(const nav_msgs::msg::Odometry::SharedPtr odom_msgs_ptr) {
    Json::Value json_odom;
    json_odom["header"]["frame_id"] = odom_msgs_ptr->header.frame_id;
    json_odom["header"]["seq"] = odom_msgs_ptr->header.stamp.sec;
    json_odom["header"]["stamp"] = odom_msgs_ptr->header.stamp.sec + odom_msgs_ptr->header.stamp.nanosec * 1e-9;
    json_odom["child_frame_id"] = odom_msgs_ptr->child_frame_id;
    json_odom["pose"]["pose"]["position"]["x"] = odom_msgs_ptr->pose.pose.position.x;
    json_odom["pose"]["pose"]["position"]["y"] = odom_msgs_ptr->pose.pose.position.y;
    json_odom["pose"]["pose"]["position"]["z"] = odom_msgs_ptr->pose.pose.position.z;
    json_odom["pose"]["pose"]["orientation"]["x"] = odom_msgs_ptr->pose.pose.orientation.x;
    json_odom["pose"]["pose"]["orientation"]["y"] = odom_msgs_ptr->pose.pose.orientation.y;
    json_odom["pose"]["pose"]["orientation"]["z"] = odom_msgs_ptr->pose.pose.orientation.z;
    json_odom["pose"]["pose"]["orientation"]["w"] = odom_msgs_ptr->pose.pose.orientation.w;
    json_odom["pose"]["covariance"] = Json::arrayValue;
    for (const auto& cov : odom_msgs_ptr->pose.covariance) {
        json_odom["pose"]["covariance"].append(cov);
    }

    json_odom["twist"]["twist"]["linear"]["x"] = odom_msgs_ptr->twist.twist.linear.x;
    json_odom["twist"]["twist"]["linear"]["y"] = odom_msgs_ptr->twist.twist.linear.y;
    json_odom["twist"]["twist"]["linear"]["z"] = odom_msgs_ptr->twist.twist.linear.z;
    json_odom["twist"]["twist"]["angular"]["x"] = odom_msgs_ptr->twist.twist.angular.x;
    json_odom["twist"]["twist"]["angular"]["y"] = odom_msgs_ptr->twist.twist.angular.y;
    json_odom["twist"]["twist"]["angular"]["z"] = odom_msgs_ptr->twist.twist.angular.z;
    json_odom["twist"]["covariance"] = Json::arrayValue;
    for (const auto& cov : odom_msgs_ptr->twist.covariance) {
        json_odom["twist"]["covariance"].append(cov);
    }

    std::string odom_json_str = Json::StyledWriter().write(json_odom);
    return odom_json_str;
}
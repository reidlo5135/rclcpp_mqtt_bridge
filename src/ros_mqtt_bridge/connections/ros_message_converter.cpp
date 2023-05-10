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

std_msgs::msg::String ros_message_converter::ros_std_msgs::StdMessageConverter::convert_json_to_chatter(std::string& raw_std_string_data) {
    std::cout << "[RosMessageConverter] json to std_msgs raw data : " << raw_std_string_data << '\n';

    Json::Value std_string_json;
    Json::Reader json_reader;
    std_msgs::msg::String std_string_message = std_msgs::msg::String();
    
    try {
        bool is_parsing_success = json_reader.parse(raw_std_string_data, std_string_json);
        if(is_parsing_success) {
            std::cout << "[RosMessageConverter] parsing std_string into json completed : " << std_string_json << '\n';
            std_string_message.data = std_string_json.get("data", "nullstr").asString();
        } else {
            std::cerr << "[RosMessageConverter] parsing std_string into json err : "  << json_reader.getFormatedErrorMessages() << '\n';
        }
    } catch(const std::exception& expn) {
        std::cerr << "[RosMessageConverter] parsing std std_string into json err: " << expn.what() << '\n';
    }

    return std_string_message;
}

/**
 * @brief Constructor for initialize this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::GeometryMessageConverter() {

}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::~GeometryMessageConverter() {

}

/**
 * @brief Function for convert geometry_msg::msg::Pose data into Json String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param pose_msgs_ptr geometry_msgs::msg::Pose::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_pose_to_json(const geometry_msgs::msg::Pose::SharedPtr pose_msgs_ptr) {
    Json::Value pose_json;

    pose_json["position"]["x"] = pose_msgs_ptr->position.x;
    pose_json["position"]["y"] = pose_msgs_ptr->position.y;
    pose_json["position"]["z"] = pose_msgs_ptr->position.z;

    pose_json["orientation"]["x"] = pose_msgs_ptr->orientation.x;
    pose_json["orientation"]["y"] = pose_msgs_ptr->orientation.y;
    pose_json["orientation"]["z"] = pose_msgs_ptr->orientation.z;
    pose_json["orientation"]["w"] = pose_msgs_ptr->orientation.w;

    std::string pose_json_str = Json::StyledWriter().write(pose_json);
    return pose_json_str;
}

std::string ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_twist_to_json(const geometry_msgs::msg::Twist::SharedPtr twist_msgs_ptr) {
    Json::Value twist_json;

    twist_json["linear"]["x"] = twist_msgs_ptr->linear.x;
    twist_json["linear"]["y"] = twist_msgs_ptr->linear.y;
    twist_json["linear"]["z"] = twist_msgs_ptr->linear.z;

    twist_json["angular"]["x"] = twist_msgs_ptr->angular.x;
    twist_json["angular"]["y"] = twist_msgs_ptr->angular.y;
    twist_json["angular"]["z"] = twist_msgs_ptr->angular.z;

    std::string twist_json_str = Json::StyledWriter().write(twist_json);
    return twist_json_str;
}

geometry_msgs::msg::Twist ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_json_to_twist(std::string& raw_twist_data) {
    std::cout << "[RosMessageConverter] json to std_msgs raw data : " << raw_twist_data << '\n';

    Json::Value twist_json;   
    Json::Reader json_reader;
    geometry_msgs::msg::Twist twist_message = geometry_msgs::msg::Twist();

    try {
        bool is_twist_parsing_success = json_reader.parse(raw_twist_data, twist_json);
        if(is_twist_parsing_success) {
            std::cout << "[RosMessageConverter] twist parsing completed : " << twist_json << '\n';
            
            Json::Value linear_json = twist_json.get("linear", Json::Value::null);
            if(!linear_json.isNull()) {
                std::cout << "[RosMessageConverter] linear parsing completed : " << linear_json << '\n';
                twist_message.linear.x = linear_json.get("x", 0.0).asDouble();
                twist_message.linear.y = linear_json.get("y", 0.0).asDouble();
                twist_message.linear.z = linear_json.get("z", 0.0).asDouble();
            } else {
                std::cerr << "[RosMessageConverter] parsing twist linear json is null " << '\n';
            }

            Json::Value angular_json = twist_json.get("angular", Json::Value::null);
            if(!angular_json.isNull()) {
                std::cout << "[RosMessageConverter] angular parsing completed : " << angular_json << '\n';
                twist_message.angular.x = angular_json.get("x", 0.0).asDouble();
                twist_message.angular.y = angular_json.get("y", 0.0).asDouble();
                twist_message.angular.z = angular_json.get("z", 0.0).asDouble();
            } else {
                std::cerr << "[RosMessageConverter] parsing twist angular json is null " << '\n';
            }
        } else {
            std::cerr << "[RosMessageConverter] parsing twist json err : "  << json_reader.getFormatedErrorMessages() << '\n';
        }
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing twist json err : " << json_expn.what() << '\n';
    }
    return twist_message;
}

geometry_msgs::msg::PoseWithCovarianceStamped ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_json_to_pose_with_covariance_stamped(std::string& raw_pose_with_covariance_stamped_data) {
    std::cout << "[RosMessageConverter] pose with covaraince stamped raw data : " << raw_pose_with_covariance_stamped_data << '\n';

    Json::Value pose_with_covariance_stamped_json;
    Json::Reader json_reader;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped_message = geometry_msgs::msg::PoseWithCovarianceStamped();

    try {
        bool is_pose_with_covariance_stamped_parsing_success = json_reader.parse(raw_pose_with_covariance_stamped_data, pose_with_covariance_stamped_json);
        if(is_pose_with_covariance_stamped_parsing_success) {
            std::cout << "[RosMessageConverter] pose with covariance stamped parsing completed : " << pose_with_covariance_stamped_json << '\n';
            if(is_pose_with_covariance_stamped_parsing_success) {
                Json::Value header_json = pose_with_covariance_stamped_json.get("header", Json::Value::null);
                if(!header_json.isNull()) {
                    pose_with_covariance_stamped_message.header.frame_id = header_json.get("frame_id", "nullstr").asString();
                    pose_with_covariance_stamped_message.header.stamp.sec = header_json.get("sec", 0.0).asDouble();
                    pose_with_covariance_stamped_message.header.stamp.nanosec = header_json.get("nanosec", 0.0).asDouble();
                } else {
                    std::cerr << "[RosMessageConverter] parsing pose with covariance stamped json is null " << '\n';
                }
                Json::Value pose_json = pose_with_covariance_stamped_json.get("pose", Json::Value::null);
                if(!pose_json.isNull()) {
                    Json::Value pose_covariance_from_json = pose_json["covariance"];
                    std::array<double, 36> pose_covariance_array;

                    if(pose_covariance_from_json.isArray() && pose_covariance_from_json.size() == 36) {
                        for(int i=0;i<36;i++) {
                            pose_covariance_array[i] = pose_covariance_from_json[i].asDouble();
                        }
                    } else {
                        std::cerr << "[RosMessageCoverter] parsing pose with covariance stamped pose.covariance is not array or out of range" << '\n';
                    }

                    Json::Value pose_pose_json = pose_json.get("pose", Json::Value::null);
                    if(!pose_pose_json.isNull()) {
                        Json::Value pose_pose_position_json = pose_pose_json.get("position", Json::Value::null);
                        if(!pose_pose_position_json.isNull()) {
                            pose_with_covariance_stamped_message.pose.pose.position.x = pose_pose_position_json.get("x", 0.0).asDouble();
                            pose_with_covariance_stamped_message.pose.pose.position.y = pose_pose_position_json.get("y", 0.0).asDouble();
                            pose_with_covariance_stamped_message.pose.pose.position.z = pose_pose_position_json.get("z", 0.0).asDouble();
                        } else {
                            std::cerr << "[RosMessageConverter] parsing with covariance stamped pose.pose.position json is null " << '\n';
                        }

                        Json::Value pose_pose_orientation_json = pose_pose_json.get("orientation", Json::Value::null);
                        if(!pose_pose_orientation_json.isNull()) {
                            pose_with_covariance_stamped_message.pose.pose.orientation.x = pose_pose_orientation_json.get("x", 0.0).asDouble();
                            pose_with_covariance_stamped_message.pose.pose.orientation.y = pose_pose_orientation_json.get("y", 0.0).asDouble();
                            pose_with_covariance_stamped_message.pose.pose.orientation.z = pose_pose_orientation_json.get("z", 0.0).asDouble();
                        } else {
                            std::cerr << "[RosMessageConverter] parsing with covariance stamped pose.pose.orientation json is null " << '\n';
                        }
                    } else {
                        std::cerr << "[RosMessageConverter] parsing with covariance stamped pose.position json is null " << '\n';
                    }
                } else {
                    std::cerr << "[RosMessageConverter] parsing pose with covariance stamped pose json is null " << '\n';
                }
            } else {
                std::cerr << "[RosMessageConverter] parsing pose with covariance stamped json err : "  << json_reader.getFormatedErrorMessages() << '\n';
            }
        } else {
            std::cerr << "[RosMessageConverter] parsing twist json err : "  << json_reader.getFormatedErrorMessages() << '\n';
        }
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing pose with covariance stamped json err : " << json_expn.what() << '\n';
    }
    
    return pose_with_covariance_stamped_message;
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
    
    for (const float& range : scan_msgs_ptr->ranges) {
        scan_json["ranges"].append(range);
    }

    for (const float& intense : scan_msgs_ptr->intensities) {
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
    for (const double& cov : odom_msgs_ptr->pose.covariance) {
        odom_json["pose"]["covariance"].append(cov);
    }

    odom_json["twist"]["twist"]["linear"]["x"] = odom_msgs_ptr->twist.twist.linear.x;
    odom_json["twist"]["twist"]["linear"]["y"] = odom_msgs_ptr->twist.twist.linear.y;
    odom_json["twist"]["twist"]["linear"]["z"] = odom_msgs_ptr->twist.twist.linear.z;

    odom_json["twist"]["twist"]["angular"]["x"] = odom_msgs_ptr->twist.twist.angular.x;
    odom_json["twist"]["twist"]["angular"]["y"] = odom_msgs_ptr->twist.twist.angular.y;
    odom_json["twist"]["twist"]["angular"]["z"] = odom_msgs_ptr->twist.twist.angular.z;

    odom_json["twist"]["covariance"] = Json::arrayValue;
    for (const double& cov : odom_msgs_ptr->twist.covariance) {
        odom_json["twist"]["covariance"].append(cov);
    }

    std::string odom_json_str = Json::StyledWriter().write(odom_json);
    return odom_json_str;
}

/**
 * @brief Function for convert nav_msgs::msg::Path data into Json String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param nav_msgs_ptr nav_msgs::msg::Path
 * @return std::string
*/
std::string ros_message_converter::ros_nav_msgs::NavMessageConverter::convert_path_to_json(const nav_msgs::msg::Path::SharedPtr path_msgs_ptr) {
    Json::Value path_json;

    path_json["header"]["frame_id"] = path_msgs_ptr->header.frame_id;
    path_json["header"]["seq"] = path_msgs_ptr->header.stamp.sec;
    path_json["header"]["stamp"] = path_msgs_ptr->header.stamp.sec + path_msgs_ptr->header.stamp.nanosec * 1e-9;
    
    for(const geometry_msgs::msg::PoseStamped& pose : path_msgs_ptr->poses) {
        path_json["pose"]["header"]["frame_id"] = pose.header.frame_id;
        path_json["pose"]["header"]["seq"] = pose.header.stamp.sec;
        path_json["pose"]["header"]["stamp"] = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9;

        path_json["pose"]["pose"]["position"]["x"] = pose.pose.position.x;
        path_json["pose"]["pose"]["position"]["y"] = pose.pose.position.y;
        path_json["pose"]["pose"]["position"]["z"] = pose.pose.position.z;
        
        path_json["pose"]["pose"]["orientation"]["x"] = pose.pose.orientation.x;
        path_json["pose"]["pose"]["orientation"]["y"] = pose.pose.orientation.y;
        path_json["pose"]["pose"]["orientation"]["z"] = pose.pose.orientation.z;
        path_json["pose"]["pose"]["orientation"]["w"] = pose.pose.orientation.w;
    }

    std::string path_json_str = Json::StyledWriter().write(path_json);
    return path_json_str;
}

ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::Tf2MessageConverter() {

}

ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::~Tf2MessageConverter() {

}

std::string ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::convert_tf_to_json(const tf2_msgs::msg::TFMessage::SharedPtr tf_msgs_ptr) {
    Json::Value tf_json;

    for(const geometry_msgs::msg::TransformStamped& transform : tf_msgs_ptr->transforms) {
        tf_json["header"]["frame_id"] = transform.header.frame_id;
        tf_json["header"]["seq"] = transform.header.stamp.sec;
        tf_json["header"]["stamp"] = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9;
        tf_json["child_frame_id"] = transform.child_frame_id;

        tf_json["transform"]["translation"]["x"] = transform.transform.translation.x;
        tf_json["transform"]["translation"]["y"] = transform.transform.translation.y;
        tf_json["transform"]["translation"]["z"] = transform.transform.translation.z;

        tf_json["transform"]["rotation"]["x"] = transform.transform.rotation.x;
        tf_json["transform"]["rotation"]["y"] = transform.transform.rotation.y;
        tf_json["transform"]["rotation"]["z"] = transform.transform.rotation.z;
        tf_json["transform"]["rotation"]["w"] = transform.transform.rotation.w;
    }
    
    std::string tf2_json_str = Json::StyledWriter().write(tf_json);
    return tf2_json_str;
}
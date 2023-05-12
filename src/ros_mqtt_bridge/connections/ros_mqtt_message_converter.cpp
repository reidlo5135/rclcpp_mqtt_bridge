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

#include "ros_mqtt_bridge/connections/ros_mqtt_message_converter.hpp"


/**
 * @brief Constructor for initialize this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_std_msgs::StdMessageConverter::StdMessageConverter() {

}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_std_msgs::StdMessageConverter::~StdMessageConverter() {

}

/**
 * @brief Function for convert ros message std_msgs::msg::Header data into Json::Value
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param header_msgs const std_msgs::msg::Header
 * @return Json::Value
*/
Json::Value ros_message_converter::ros_std_msgs::StdMessageConverter::convert_header_to_json(const std_msgs::msg::Header header_msgs) {
    Json::Value header_json;

    try {
        header_json["frame_id"] = header_msgs.frame_id;
        header_json["seq"] = header_msgs.stamp.sec;
        header_json["stamp"] = header_msgs.stamp.sec + header_msgs.stamp.nanosec * 1e-9;
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing std std_string to std_msg::msg::String  err: " << json_expn.what() << '\n';
    }

    return header_json;
}

/**
 * @brief Function for convert ros message std_msgs::msg::String data into std::string(JSON style)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param chatter_msg_ptr const std_msgs::msg::String::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_std_msgs::StdMessageConverter::convert_chatter_to_json(const std_msgs::msg::String::SharedPtr chatter_msgs_ptr) {
    Json::Value chatter_json;

    try {
        chatter_json["data"] = chatter_msgs_ptr->data;
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing std chatter to json err: " << json_expn.what() << '\n';
    }

    std::string chatter_json_str = Json::StyledWriter().write(chatter_json);
    return chatter_json_str;
}

/**
 * @brief Function for convert std::string& into ros message std_msgs::msg::String
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_std_string_data std::string&
 * @return std_msgs::msg::String
*/
std_msgs::msg::String ros_message_converter::ros_std_msgs::StdMessageConverter::convert_json_to_chatter(std::string& raw_std_string_data) {
    std::cout << "[RosMessageConverter] json to std_msgs raw data : " << raw_std_string_data << '\n';

    Json::Value std_string_json;
    Json::Reader json_reader;
    std_msgs::msg::String std_string_message = std_msgs::msg::String();
    
    try {
        bool is_parsing_success = json_reader.parse(raw_std_string_data, std_string_json);
        if(is_parsing_success) {
            std::cout << "[RosMessageConverter] parsing JSON string to json completed : " << std_string_json << '\n';
            std_string_message.data = std_string_json.get("data", "nullstr").asString();
        } else {
            std::cerr << "[RosMessageConverter] parsing JSON string to json err : "  << json_reader.getFormatedErrorMessages() << '\n';
        }
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing JSON string to std_msg::msg::String  err: " << json_expn.what() << '\n';
    }

    return std_string_message;
}

/**
 * @brief Function for convert Json::Value into ros std_msgs::msgs::Header
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_header_data Json::Value
 * @return std_msgs::msg::Header
*/
std_msgs::msg::Header ros_message_converter::ros_std_msgs::StdMessageConverter::convert_json_to_header(Json::Value raw_header_data) {
    std_msgs::msg::Header header_message = std_msgs::msg::Header();

    try {
        header_message.frame_id = raw_header_data.get("frame_id", "nullstr").asString();
        header_message.stamp.sec = raw_header_data.get("sec", 0.0).asDouble();
        header_message.stamp.nanosec = raw_header_data.get("nanosec", 0.0).asDouble();
    } catch(Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing json to std_msgs::msg::Header err : " << json_expn.what() << '\n';
    }

    return header_message;
}

/**
 * @brief Constructor for initialize this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::GeometryMessageConverter() {
    std_message_converter_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::~GeometryMessageConverter() {
    delete std_message_converter_;
}


/**
 * @brief Function for convert ros geometry_msgs::msg::Point into Json::Value
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param point_msgs const geometry_msgs::msg::Point
 * @return Json::Value
*/
Json::Value ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_point_to_json(const geometry_msgs::msg::Point point_msgs) {
    Json::Value point_json;

    try {
        point_json["x"] = point_msgs.x;
        point_json["y"] = point_msgs.y;
        point_json["z"] = point_msgs.z;
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing json to geometry_msgs::msg::Point err : " << json_expn.what() << '\n';
    }

    return point_json;
}

/**
 * @brief Function for convert ros geometry_msgs::msg::Quaternion into Json::Value
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param quaternion_msgs const geometry_msgs::msg::Quaternion
 * @return Json::Value
*/
Json::Value ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_quaternion_to_json(const geometry_msgs::msg::Quaternion quaternion_msgs) {
    Json::Value quaternion_json;

    try {
        quaternion_json["x"] = quaternion_msgs.x;
        quaternion_json["y"] = quaternion_msgs.y;
        quaternion_json["z"] = quaternion_msgs.z;
        quaternion_json["w"] = quaternion_msgs.w;
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing json to geometry_msgs::msg::Quaternion err : " << json_expn.what() << '\n';
    }

    return quaternion_json;
}

/**
 * @brief Function for convert ros message geometry_msg::msg::Pose data into std::string(JSON value)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param pose_msgs_ptr const geometry_msgs::msg::Pose::SharedPtr
 * @return std::string
 * @see convert_point_to_json
 * @see convert_quaternion_to_json
*/
std::string ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_pose_to_json(const geometry_msgs::msg::Pose::SharedPtr pose_msgs_ptr) {
    Json::Value pose_json;

    try {
        pose_json["position"] = convert_point_to_json(pose_msgs_ptr->position);
        pose_json["orientation"] = convert_quaternion_to_json(pose_msgs_ptr->orientation);
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing geometry_msgs::msg::Pose::SharedPtr to json err : " << json_expn.what() << '\n';
    }

    std::string pose_json_str = Json::StyledWriter().write(pose_json);
    return pose_json_str;
}

Json::Value ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_pose_to_json(const geometry_msgs::msg::Pose pose_msgs) {
    Json::Value pose_json;

    try {
        pose_json["position"] = convert_point_to_json(pose_msgs.position);
        pose_json["orientation"] = convert_quaternion_to_json(pose_msgs.orientation);
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing geometry_msgs::msg::Pose::SharedPtr to json err : " << json_expn.what() << '\n';
    }

    return pose_json;
}

/**
 * @brief Function for convert ros geometry_msgs::msg::Vector3 into Json::Value
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param vector_msgs const geometry_msgs::msg::Vector3
 * @return Json::Value
*/
Json::Value ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_vector_to_json(const geometry_msgs::msg::Vector3 vector_msgs) {
    Json::Value vector_json;

    try {
        vector_json["x"] = vector_msgs.x;
        vector_json["y"] = vector_msgs.y;
        vector_json["z"] = vector_msgs.z;
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing geometry_msgs::msg::Vector3 to json err : " << json_expn.what() << '\n';
    }

    return vector_json;
}

/**
 * @brief Function for convert ros geometry_msgs::msg::Twist into std::string(JSON style)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param twist_msgs_ptr const geometry_msgs::msg::Twist::SharedPtr
 * @return std::string
 * @see convert_vector_to_json
*/
std::string ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_twist_to_json(const geometry_msgs::msg::Twist::SharedPtr twist_msgs_ptr) {
    Json::Value twist_json;

    try {
        twist_json["linear"] = convert_vector_to_json(twist_msgs_ptr->linear);
        twist_json["angular"] = convert_vector_to_json(twist_msgs_ptr->angular);
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing geometry_msgs::msg::Twist::SharedPtr to json err : " << json_expn.what() << '\n';
    }

    std::string twist_json_str = Json::StyledWriter().write(twist_json);
    return twist_json_str;
}

/**
 * @brief Function for convert Json::Value into ros geometry_msgs::msg::Vector3
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_vector_data Json::Value
 * @return geometry_msgs::msg::Vector3
*/
geometry_msgs::msg::Vector3 ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_json_to_vector(Json::Value raw_vector_data) {
    geometry_msgs::msg::Vector3 vector_message = geometry_msgs::msg::Vector3();

    try {
        vector_message.x = raw_vector_data.get("x", 0.0).asDouble();
        vector_message.y = raw_vector_data.get("y", 0.0).asDouble();
        vector_message.z = raw_vector_data.get("z", 0.0).asDouble();
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing geometry_msgs::msg::Vector3 to json err : " << json_expn.what() << '\n';
    }

    return vector_message;
}

/**
 * @brief Function for convert Json::Value into ros message geometry_msgs::msg::Twist
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_twist_data std::string&
 * @return geometry_msgs::msg::Twist
 * @see convert_json_to_vector
*/
geometry_msgs::msg::Twist ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_json_to_twist(std::string& raw_twist_data) {
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
                twist_message.linear = convert_json_to_vector(linear_json);
            } else {
                std::cerr << "[RosMessageConverter] parsing twist linear json is null " << '\n';
            }

            Json::Value angular_json = twist_json.get("angular", Json::Value::null);
            if(!angular_json.isNull()) {
                std::cout << "[RosMessageConverter] angular parsing completed : " << angular_json << '\n';
                twist_message.angular = convert_json_to_vector(angular_json);
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

/**
 * @brief Function for convert Json::Value into ros message geometry_msgs::msg::Point
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_point_data Json::Value
 * @return geometry_msgs::msg::Point
*/
geometry_msgs::msg::Point ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_json_to_point(Json::Value raw_point_data) {
    geometry_msgs::msg::Point point_message = geometry_msgs::msg::Point();

    try {
        point_message.x = raw_point_data.get("x", 0.0).asDouble();
        point_message.y = raw_point_data.get("y", 0.0).asDouble();
        point_message.z = raw_point_data.get("z", 0.0).asDouble();
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing json to geometry_msgs::msg::Point err : " << json_expn.what() << '\n';
    }

    return point_message;
}

/**
 * @brief Function for convert Json::Value into ros message geometry_msgs::msg::Quaternion
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_quaternion_data Json::Value
 * @return geometry_msgs::msg::Quaternion
*/
geometry_msgs::msg::Quaternion ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_json_to_quaternion(Json::Value raw_quaternion_data) {
    geometry_msgs::msg::Quaternion quaternion_message = geometry_msgs::msg::Quaternion();

    try {
        quaternion_message.x = raw_quaternion_data.get("x", 0.0).asDouble();
        quaternion_message.y = raw_quaternion_data.get("y", 0.0).asDouble();
        quaternion_message.z = raw_quaternion_data.get("z", 0.0).asDouble();
        quaternion_message.w = raw_quaternion_data.get("w", 0.0).asDouble();
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing json to geometry_msgs::msg::Point err : " << json_expn.what() << '\n';
    }

    return quaternion_message;
}

/**
 * @brief Function for convert Json::Value into std::array<double, 36UL>
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_pose_data Json::value
 * @return std::array<double, 36UL>
*/
std::array<double, 36UL> ros_message_converter::ros_geometry_msgs::GeometryMessageConverter::convert_json_to_pose_covariance(Json::Value raw_pose_data) {
    Json::Value pose_covariance_from_json = raw_pose_data["covariance"];
    std::array<double, 36UL> pose_covariance_array;

    try {
        if(pose_covariance_from_json.isArray() && pose_covariance_from_json.size() == 36) {
            for(int i=0;i<36;i++) {
                pose_covariance_array[i] = pose_covariance_from_json[i].asDouble();
            }
        } else {
            std::cerr << "[RosMessageCoverter] parsing pose with covariance stamped pose.covariance is not array or out of range" << '\n';
        }
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageCoverter] parsing geometry_msgs::msg::PoseWithCovarianceStamped::covariance into array err " << json_expn.what() << '\n';
    }

    return pose_covariance_array;
}

/**
 * @brief Function for convert std::string(JSON style) into ros message geometry_msgs::msg::PoseWithCovarianceStamped
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param raw_pose_with_covariance_stamped_data std::string&
 * @return geometry_msgs::msg::PoseWithCovarianceStamped
*/
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
                    pose_with_covariance_stamped_message.header = std_message_converter_->convert_json_to_header(header_json);
                } else {
                    std::cerr << "[RosMessageConverter] parsing pose with covariance stamped header json is null " << '\n';
                }
                Json::Value pose_json = pose_with_covariance_stamped_json.get("pose", Json::Value::null);
                if(!pose_json.isNull()) {
                    Json::Value pose_pose_json = pose_json.get("pose", Json::Value::null);
                    if(!pose_pose_json.isNull()) {
                        Json::Value pose_pose_position_json = pose_pose_json.get("position", Json::Value::null);
                        if(!pose_pose_position_json.isNull()) {
                            pose_with_covariance_stamped_message.pose.pose.position = convert_json_to_point(pose_pose_position_json);
                        } else {
                            std::cerr << "[RosMessageConverter] parsing with covariance stamped pose.pose.position json is null " << '\n';
                        }
                        Json::Value pose_pose_orientation_json = pose_pose_json.get("orientation", Json::Value::null);
                        if(!pose_pose_orientation_json.isNull()) {
                            pose_with_covariance_stamped_message.pose.pose.orientation = convert_json_to_quaternion(pose_pose_orientation_json);
                        } else {
                            std::cerr << "[RosMessageConverter] parsing with covariance stamped pose.pose.orientation json is null " << '\n';
                        }
                    } else {
                        std::cerr << "[RosMessageConverter] parsing with covariance stamped pose.position json is null " << '\n';
                    }
                    pose_with_covariance_stamped_message.pose.covariance = convert_json_to_pose_covariance(pose_json);
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
 * @date 23.05.12
*/
ros_message_converter::ros_sensor_msgs::SensorMessageConverter::SensorMessageConverter() {
    std_message_converter_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_sensor_msgs::SensorMessageConverter::~SensorMessageConverter() {
    delete std_message_converter_;
}

/**
 * @brief Function for convert ros message sensor_msgs::msg::LaserScan data into std::string(JSON style)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param scan_msgs_ptr const sensor_msgs::msg::LaserScan::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_sensor_msgs::SensorMessageConverter::convert_scan_to_json(const sensor_msgs::msg::LaserScan::SharedPtr scan_msgs_ptr) {
    Json::Value scan_json;

    scan_json["header"] = std_message_converter_->convert_header_to_json(scan_msgs_ptr->header);
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
 * @date 23.05.12
*/
ros_message_converter::ros_nav_msgs::NavMessageConverter::NavMessageConverter() {
    std_message_converter_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
    geometry_message_converter_ = new ros_message_converter::ros_geometry_msgs::GeometryMessageConverter();
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_nav_msgs::NavMessageConverter::~NavMessageConverter() {
    delete std_message_converter_;
    delete geometry_message_converter_;
}

/**
 * @brief Function for convert ros message nav_msgs::msg::Odometry data into std::string(JSON style)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param odom_msgs_ptr const nav_msgs::msg::Odometry::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_nav_msgs::NavMessageConverter::convert_odom_to_json(const nav_msgs::msg::Odometry::SharedPtr odom_msgs_ptr) {
    Json::Value odom_json;

    try {
        odom_json["header"] = std_message_converter_->convert_header_to_json(odom_msgs_ptr->header);
        odom_json["child_frame_id"] = odom_msgs_ptr->child_frame_id;
        odom_json["pose"]["pose"]["position"] = geometry_message_converter_->convert_point_to_json(odom_msgs_ptr->pose.pose.position);
        odom_json["pose"]["pose"]["orientation"] = geometry_message_converter_->convert_quaternion_to_json(odom_msgs_ptr->pose.pose.orientation);
        odom_json["pose"]["covariance"] = Json::arrayValue;
        for (const double& cov : odom_msgs_ptr->pose.covariance) {
            odom_json["pose"]["covariance"].append(cov);
        }

        odom_json["twist"]["twist"]["linear"] = geometry_message_converter_->convert_vector_to_json(odom_msgs_ptr->twist.twist.linear);
        odom_json["twist"]["twist"]["angular"] = geometry_message_converter_->convert_vector_to_json(odom_msgs_ptr->twist.twist.angular);
        odom_json["twist"]["covariance"] = Json::arrayValue;
        for (const double& cov : odom_msgs_ptr->twist.covariance) {
            odom_json["twist"]["covariance"].append(cov);
        }
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing odom to json err : " << json_expn.what() << '\n';
    }

    std::string odom_json_str = Json::StyledWriter().write(odom_json);
    return odom_json_str;
}

/**
 * @brief Function for convert ros message nav_msgs::msg::Path data into std::string(JSON style)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param nav_msgs_ptr const nav_msgs::msg::Path::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_nav_msgs::NavMessageConverter::convert_path_to_json(const nav_msgs::msg::Path::SharedPtr path_msgs_ptr) {
    Json::Value path_json;

    try {
        path_json["header"] = std_message_converter_->convert_header_to_json(path_msgs_ptr->header);

        for(const geometry_msgs::msg::PoseStamped& pose : path_msgs_ptr->poses) {
            path_json["pose"]["header"] = std_message_converter_->convert_header_to_json(pose.header);
            path_json["pose"]["pose"]["position"] = geometry_message_converter_->convert_point_to_json(pose.pose.position);
            path_json["pose"]["pose"]["orientation"] = geometry_message_converter_->convert_quaternion_to_json(pose.pose.orientation);
        }
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing path to json err : " << json_expn.what() << '\n';
    }

    std::string path_json_str = Json::StyledWriter().write(path_json);
    return path_json_str;
}

/**
 * @brief Function for convert ros message nav_msgs::msg::MapMetaData data into Json::Value
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param map_meta_data_msgs const nav_msgs::msg::MapMetaData
 * @return Json::Value
*/
Json::Value ros_message_converter::ros_nav_msgs::NavMessageConverter::convert_meta_data_to_json(const nav_msgs::msg::MapMetaData map_meta_data_msgs) {
    Json::Value map_meta_data_json;

    try {
        map_meta_data_json["width"] = map_meta_data_msgs.width;
        map_meta_data_json["height"] = map_meta_data_msgs.height;
        map_meta_data_json["origin"] = geometry_message_converter_->convert_pose_to_json(map_meta_data_msgs.origin);
        map_meta_data_json["resolution"] = map_meta_data_msgs.resolution;
        map_meta_data_json["map_load_time"]["sec"] = map_meta_data_msgs.map_load_time.sec;
        map_meta_data_json["map_load_time"]["nanosec"] = map_meta_data_msgs.map_load_time.nanosec;
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing occupancy grid to json err : " << json_expn.what() << '\n';
    }

    return map_meta_data_json;
}

/**
 * @brief Function for convert ros message nav_msgs::srv::GetMap_Response data into std::string(JSON style)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param map_response_msgs_ptr const nav_msgs::srv::GetMap_Response::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_nav_msgs::NavMessageConverter::convert_map_response_to_json(const nav_msgs::srv::GetMap_Response::SharedPtr map_response_msgs_ptr) {
    Json::Value map_response_json;

    try {
        map_response_json["header"] = std_message_converter_->convert_header_to_json(map_response_msgs_ptr->map.header);
        map_response_json["info"] = convert_meta_data_to_json(map_response_msgs_ptr->map.info);
        
        std::vector<int8_t> map_data_vec = map_response_msgs_ptr->map.data;
        int8_t map_data_arr[map_data_vec.size()];
        std::copy(map_data_vec.begin(), map_data_vec.end(), map_data_arr);
        map_response_json["data"] = map_data_arr;
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing map response to json err : " << json_expn.what() << '\n';
    }
}

/**
 * @brief Constructor for initialize this class instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::Tf2MessageConverter() {
    std_message_converter_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
    geometry_message_converter_ = new ros_message_converter::ros_geometry_msgs::GeometryMessageConverter();
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
*/
ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::~Tf2MessageConverter() {
    delete std_message_converter_;
    delete geometry_message_converter_;
}

/**
 * @brief Function for convert ros message tf2_msgs::msg::TFMessage data into std::string(JSON style)
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.12
 * @param nav_msgs_ptr const nav_msgs::msg::Path::SharedPtr
 * @return std::string
*/
std::string ros_message_converter::ros_tf2_msgs::Tf2MessageConverter::convert_tf_to_json(const tf2_msgs::msg::TFMessage::SharedPtr tf_msgs_ptr) {
    Json::Value tf_json;

    try {
        for(const geometry_msgs::msg::TransformStamped& transform : tf_msgs_ptr->transforms) {
            tf_json["header"] = std_message_converter_->convert_header_to_json(transform.header);
            tf_json["child_frame_id"] = transform.child_frame_id;
            tf_json["transform"]["translation"] = geometry_message_converter_->convert_vector_to_json(transform.transform.translation);
            tf_json["transform"]["rotation"] = geometry_message_converter_->convert_quaternion_to_json(transform.transform.rotation);
        }   
    } catch(const Json::Exception& json_expn) {
        std::cerr << "[RosMessageConverter] parsing tf to json err : " << json_expn.what() << '\n';
    }
    
    std::string tf2_json_str = Json::StyledWriter().write(tf_json);
    return tf2_json_str;
}
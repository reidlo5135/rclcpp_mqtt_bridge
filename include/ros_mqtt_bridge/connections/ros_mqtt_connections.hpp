#ifndef ROS_MQTT_CONNECTIONS
#define ROS_MQTT_CONNECTIONS

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

/**
 * include mqtt header file
 * @see mqtt/mqtt.hpp
*/
#include "mqtt/mqtt.hpp"

/**
 * include rclcpp header files
 * @see rclcpp/rclcpp.hpp
 * @see std_msgs/msgs/string.hpp
 * @see nav_msgs/msg/odometry.hpp
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * @brief namespace for declare rclcpp shared pointers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @see rclcpp::Publisher
 * @see rclcpp::Subscription
*/
namespace ros_mqtt_connections {
    namespace publisher {

    }
    namespace subscription {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_std_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_;
    }
}

/**
 * @brief namespace for declare mqtt topics
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
namespace mqtt_topics {
    namespace publisher {
        const char * chatter_topic = "ros_connection_bridge/chatter";
        const char * odom_topic = "ros_connection_bridge/odom";
    }
    namespace subscription {
        
    }
}

#endif
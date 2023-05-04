#ifndef ROS_MQTT_CONNECTIONS
#define ROS_MQTT_CONNECTIONS

#include <iostream>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "mqtt/mqtt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace ros_mqtt_connections {
    namespace publisher {

    }
    namespace subscription {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_std_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_;
    }
}

namespace mqtt_topics {
    namespace publisher {
        const char * chatter_topic = "ros_connection_bridge/chatter";
        const char * odom_topic = "ros_connetion_bridge/odom";
    }
    namespace subscription {
        
    }
}

#endif
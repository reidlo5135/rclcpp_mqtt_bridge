#ifndef ROS_CONNECTIONS
#define ROS_CONNECTIONS

#include <math.h>
#include <typeinfo>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace ros_connections {
    namespace publisher {
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_std_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_publisher_;
    }
    namespace subscription {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_std_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_;
    }
}

namespace ros_topics {
    namespace publisher {
        const char * chatter = "ros_connection_bridge/chatter";
        const char * odometry = "ros_connection_bridge/odom";
    }
    namespace subscription {
        const char * chatter = "/chatter";
        const char * odometry = "/odom";
    }
}

#endif
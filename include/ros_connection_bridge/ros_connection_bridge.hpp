#ifndef ROS_CONNECTION_BRIDGE
#define ROS_CONNECTION_BRIDGE

#include <iostream>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_connection_bridge/connections/ros_connections.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Class for establish ros2 publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosConnectionPublisher {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
    public :
        RosConnectionPublisher(std::shared_ptr<rclcpp::Node> ros_node_ptr);
        virtual ~RosConnectionPublisher();
        void create_publishers();
};

/**
 * @brief Class for establish ros2 subscriptions
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosConnectionSubscription {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
    public :
        RosConnectionSubscription(std::shared_ptr<rclcpp::Node> ros_node_ptr);
        virtual ~RosConnectionSubscription();
        void create_connection_bridge();
};

/**
 * @brief Class for initialize rclcpp::Node & register ros2 connections into node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosConnectionBridge : public rclcpp::Node {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosConnectionPublisher * ros_connection_publisher_ptr_;
        RosConnectionSubscription * ros_connection_subscription_ptr_;
    public :
        RosConnectionBridge();
        virtual ~RosConnectionBridge();
};

void check_rclcpp();

#endif
#ifndef ROS_CONNECTION_BRIDGE
#define ROS_CONNECTION_BRIDGE

#include <iostream>
#include "ros_connection_bridge/connections/ros_connections.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosConnectionPublisher {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
    public :
        RosConnectionPublisher(std::shared_ptr<rclcpp::Node> ros_node_ptr);
        virtual ~RosConnectionPublisher();
        void create_publishers();
};

class RosConnectionSubscription {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
    public :
        RosConnectionSubscription(std::shared_ptr<rclcpp::Node> ros_node_ptr);
        virtual ~RosConnectionSubscription();
        void create_connection_bridge();
};

class RosConnectionBridge : public rclcpp::Node {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosConnectionPublisher * ros_connection_publisher_ptr_;
        RosConnectionSubscription * ros_connection_subscription_ptr_;
    public :
        RosConnectionBridge();
        virtual ~RosConnectionBridge();
};

#endif
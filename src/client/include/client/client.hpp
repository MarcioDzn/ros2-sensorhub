#ifndef CLIENT_HPP
#define CLIENT_HPP

#include "rclcpp/rclcpp.hpp"

class ClientNode : public rclcpp::Node
{
    public:
        explicit ClientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ClientNode();
        
};

#endif // CLIENT_HPP

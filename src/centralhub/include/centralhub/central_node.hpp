#ifndef CENTRAL_NODE_HPP
#define CENTRAL_NODE_HPP

#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CentralNode : public rclcpp::Node
{
    public:
        explicit CentralNode();
        virtual ~CentralNode();

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        void timer_callback();
};

#endif // CENTRAL_NODE_HPP
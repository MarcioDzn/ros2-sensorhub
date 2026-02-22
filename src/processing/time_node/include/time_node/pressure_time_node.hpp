#ifndef PRESSURE_TIME_NODE_HPP
#define PRESSURE_TIME_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/time.hpp"

class PressureTimeNode : public rclcpp::Node
{
    public:
        PressureTimeNode();

    private:
        void time_callback(const interfaces::msg::Time::SharedPtr msg);
        rclcpp::Subscription<interfaces::msg::Time>::SharedPtr sub_;
};

#endif //PRESSURE_TIME_NODE_HPP
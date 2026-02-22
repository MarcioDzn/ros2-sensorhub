#ifndef ACTUATOR_TIME_NODE_HPP
#define ACTUATOR_TIME_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/time.hpp"

class ActuatorTimeNode : public rclcpp::Node
{
    public:
        ActuatorTimeNode();

    private:
        void time_callback(const interfaces::msg::Time::SharedPtr msg);
        rclcpp::Subscription<interfaces::msg::Time>::SharedPtr sub_;
};

#endif //ACTUATOR_TIME_NODE_HPP
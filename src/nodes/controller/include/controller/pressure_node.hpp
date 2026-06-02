#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/controller_in.hpp"
#include "interfaces/msg/controller_out.hpp"


class ControllerNode : public rclcpp::Node
{
    public:
        explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ControllerNode();
        
    private:
        rclcpp::Subscription<interfaces::msg::ControllerIn>::SharedPtr subscriber_;
        rclcpp::Publisher<interfaces::msg::ControllerOut>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // CONTROLLER_HPP

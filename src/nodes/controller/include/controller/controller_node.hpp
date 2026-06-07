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
        void controller_callback(const interfaces::msg::ControllerIn::SharedPtr msg);

        rclcpp::Subscription<interfaces::msg::ControllerIn>::SharedPtr subscriber_;
        rclcpp::Publisher<interfaces::msg::ControllerOut>::SharedPtr publisher_;
        interfaces::msg::ControllerIn last_msg_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // CONTROLLER_HPP

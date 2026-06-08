#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/controller_in.hpp"
#include "interfaces/msg/controller_out.hpp"

#include "interfaces/msg/mg8008_e_command.hpp"
#include "interfaces/msg/mg8008_e_state.hpp"


class ControllerNode : public rclcpp::Node
{
    public:
        explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ControllerNode();
        
    private:
        void send_angle(std::string name, int32_t angle, int32_t speed);
        void get_angle(const interfaces::msg::MG8008EState::SharedPtr msg);
        void controller_callback(const interfaces::msg::ControllerIn::SharedPtr msg);
        void publish_state();

        rclcpp::Publisher<interfaces::msg::MG8008ECommand>::SharedPtr actuator_publisher_;
        rclcpp::Subscription<interfaces::msg::MG8008EState>::SharedPtr actuator_subscriber_;

        rclcpp::Subscription<interfaces::msg::ControllerIn>::SharedPtr subscriber_;
        rclcpp::Publisher<interfaces::msg::ControllerOut>::SharedPtr publisher_;

        interfaces::msg::ControllerIn last_msg_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string name_;
        std::atomic<int32_t> real_angle_{0};
        std::atomic<int32_t> ref_angle_{0};
        uint32_t current_period_us_{10000}; // 10ms
        std::chrono::microseconds feedback_period_{10000}; // 10ms
};

#endif // CONTROLLER_HPP

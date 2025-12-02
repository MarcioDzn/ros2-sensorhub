#ifndef MANIPULATOR_NODE_HPP
#define MANIPULATOR_NODE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "common_serial/serial_handler.hpp"
#include "interfaces/srv/set_motor_config.hpp"
#include "interfaces/msg/actuator_goal_position.hpp"
#include "manipulator_manager.hpp"

using SetMotorConfig = interfaces::srv::SetMotorConfig;
using ActuatorGoalPosition = interfaces::msg::ActuatorGoalPosition;

class ManipulatorNode : public rclcpp::Node
{
    public:
        explicit ManipulatorNode();
        virtual ~ManipulatorNode();

        bool init_serial(const char* device, int baudrate);
        void send_packet();
        
        std::shared_ptr<ManipulatorManager> get_manipulator_manager() { return manipulator_manager_; }
        std::shared_ptr<SerialHandler> get_serial_handler() { return serial_handler_; }
        void set_serial_handler();
        void set_manipulator_manager();
        
    private:
        void timer_callback();
        void goal_position_callback(const ActuatorGoalPosition& msg);
        void load_parameters();
        void set_parameters();
        void motor_service_callback(
                const std::shared_ptr<SetMotorConfig::Request> request,
                std::shared_ptr<SetMotorConfig::Response> response);
    
        rclcpp::Subscription<ActuatorGoalPosition>::SharedPtr actuator_subscriber_;
        rclcpp::Service<SetMotorConfig>::SharedPtr motor_service_;
        
        std::shared_ptr<ManipulatorManager> manipulator_manager_;
        
        std::shared_ptr<SerialHandler> serial_handler_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MANIPULATOR_NODE_HPP

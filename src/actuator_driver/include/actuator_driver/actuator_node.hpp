#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "common_serial/serial_handler.hpp"
#include "interfaces/srv/set_motor_config.hpp"
#include "interfaces/msg/actuator_goal_position.hpp"
#include "actuator_manager.hpp"

using SetMotorConfig = interfaces::srv::SetMotorConfig;
using ActuatorGoalPosition = interfaces::msg::ActuatorGoalPosition;

class ActuatorNode : public rclcpp::Node
{
    public:
        explicit ActuatorNode();
        virtual ~ActuatorNode();

        bool init_serial(const char* device, int baudrate);
        void send_packet();
        
        std::shared_ptr<ActuatorManager> get_actuator_manager() { return actuator_manager_; }
        std::shared_ptr<SerialHandler> get_serial_handler() { return serial_handler_; }
        void set_serial_handler();
        void set_actuator_manager();
        
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
        
        std::shared_ptr<ActuatorManager> actuator_manager_;
        
        std::shared_ptr<SerialHandler> serial_handler_;
        rclcpp::TimerBase::SharedPtr timer_;

        int update_rate_ms_;
};

#endif // ACTUATOR_NODE_HPP

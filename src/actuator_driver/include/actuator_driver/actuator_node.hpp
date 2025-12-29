#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <optional>

#include "actuator_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/set_motor_config.hpp"
#include "interfaces/msg/actuator_goal_position.hpp"

using SetMotorConfig = interfaces::srv::SetMotorConfig;
using ActuatorGoalPosition = interfaces::msg::ActuatorGoalPosition;

class ActuatorNode : public rclcpp::Node
{
    public:
        explicit ActuatorNode();
        virtual ~ActuatorNode();

    private:
        void goal_position_callback(const ActuatorGoalPosition::SharedPtr msg);
        void motor_service_callback(
            const std::shared_ptr<SetMotorConfig::Request> request,
            std::shared_ptr<SetMotorConfig::Response> response);

        std::shared_ptr<ActuatorManager> manager_;
        rclcpp::Subscription<ActuatorGoalPosition>::SharedPtr actuator_subscriber_;
        rclcpp::Service<SetMotorConfig>::SharedPtr motor_service_;
        
        int update_rate_ms_;
};

#endif // ACTUATOR_NODE_HPP
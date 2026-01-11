#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <string>
#include <optional>

#include "actuator_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/set_motor_config.hpp"
#include "interfaces/msg/actuator_goal_position.hpp"
#include "interfaces/msg/actuator_current_position.hpp"

using SetMotorConfig = interfaces::srv::SetMotorConfig;
using ActuatorGoalPosition = interfaces::msg::ActuatorGoalPosition;
using ActuatorCurrentPosition = interfaces::msg::ActuatorCurrentPosition;

class ActuatorNode : public rclcpp::Node
{
    public:
        explicit ActuatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ActuatorNode();

    private:
        void goal_position_callback(const ActuatorGoalPosition::SharedPtr msg);
        void motor_service_callback(
            const std::shared_ptr<SetMotorConfig::Request> request,
            std::shared_ptr<SetMotorConfig::Response> response);
        void publish_position_callback();

        std::shared_ptr<ActuatorManager> manager_;
        
        rclcpp::Subscription<ActuatorGoalPosition>::SharedPtr actuator_subscriber_;
        std::unordered_map<uint8_t,
            rclcpp::Publisher<ActuatorCurrentPosition>::SharedPtr> publishers_;
        rclcpp::Service<SetMotorConfig>::SharedPtr motor_service_;
        
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ACTUATOR_NODE_HPP
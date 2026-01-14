#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <string>
#include <optional>

#include "node_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include "interfaces/srv/set_torque.hpp"

#include "interfaces/msg/command.hpp"
#include "interfaces/msg/state.hpp"

using SetTorque = interfaces::srv::SetTorque;
using Command = interfaces::msg::Command;
using State = interfaces::msg::State;

class ActuatorNode : public rclcpp::Node
{
    public:
        explicit ActuatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ActuatorNode();

    private:
        void goal_position_callback(const Command::SharedPtr msg);
        void state_callback();

        void set_torque_service_callback(
            const std::shared_ptr<SetTorque::Request> request,
            std::shared_ptr<SetTorque::Response> response);

        std::shared_ptr<NodeManager> node_manager_;
        std::shared_ptr<ParameterManager> parameter_manager_;
        
        rclcpp::Subscription<Command>::SharedPtr actuator_subscriber_;
        rclcpp::Publisher<State>::SharedPtr state_publisher_;

        rclcpp::Service<SetTorque>::SharedPtr set_torque_service_;
        
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ACTUATOR_NODE_HPP
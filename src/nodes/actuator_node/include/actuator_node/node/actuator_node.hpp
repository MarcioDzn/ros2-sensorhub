#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <mutex>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "driver/actuator_factory.hpp"

#include "interfaces/srv/set_torque.hpp"
#include "interfaces/msg/command.hpp"
#include "interfaces/msg/actuator_state.hpp"

class NodeManager;
class ParameterManager;

using SetTorque = interfaces::srv::SetTorque;
using Command = interfaces::msg::Command;
using State = interfaces::msg::ActuatorState;

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

        std::mutex driver_mutex_;

        std::shared_ptr<IActuatorDriver> actuator_driver_;
        std::shared_ptr<ParameterManager> parameter_manager_;
        
        rclcpp::Subscription<Command>::SharedPtr actuator_subscriber_;
        rclcpp::Publisher<State>::SharedPtr state_publisher_;
        rclcpp::Service<SetTorque>::SharedPtr set_torque_service_;
        
        rclcpp::TimerBase::SharedPtr timer_;

        std::ofstream timing_log_;
};

#endif // ACTUATOR_NODE_HPP
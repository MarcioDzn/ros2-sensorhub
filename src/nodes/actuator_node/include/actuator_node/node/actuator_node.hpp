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
#include "interfaces/msg/time.hpp"

class NodeManager;
class ParameterManager;

using SetTorque = interfaces::srv::SetTorque;
using ActuatorCommand = interfaces::msg::Command;
using ActuatorState = interfaces::msg::ActuatorState;
using Time = interfaces::msg::Time;

struct ActuatorData {
    int8_t id;
    std::string name;
    int16_t position;
}

class ActuatorNode : public rclcpp::Node
{
    public:
        explicit ActuatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ActuatorNode();

    private:
        void set_goal_position(const ActuatorCommand::SharedPtr msg);

        ActuatorState read_actuator_data(Time& time_data);
        void publish_actuator_state();

        void set_torque(
            const std::shared_ptr<SetTorque::Request> request,
            std::shared_ptr<SetTorque::Response> response);

        template <typename Func>
        inline double measure_micros(Func&& func) {
            auto start = std::chrono::high_resolution_clock::now();
            func();
            auto end = std::chrono::high_resolution_clock::now();
            return static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
            );
        }

        std::mutex driver_mutex_;

        std::shared_ptr<IActuatorDriver> actuator_driver_;
        std::shared_ptr<ParameterManager> parameter_manager_;
        
        rclcpp::Subscription<ActuatorCommand>::SharedPtr actuator_subscriber_;
        rclcpp::Publisher<ActuatorState>::SharedPtr state_publisher_;
        rclcpp::Publisher<Time>::SharedPtr time_publisher_;
        rclcpp::Service<SetTorque>::SharedPtr set_torque_service_;
        
        rclcpp::TimerBase::SharedPtr timer_;

        std::ofstream timing_log_;
};

#endif // ACTUATOR_NODE_HPP

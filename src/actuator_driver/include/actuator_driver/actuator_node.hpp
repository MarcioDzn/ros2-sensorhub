#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "common_serial/serial_handler.hpp"
#include "interfaces/srv/set_motor_config.hpp"
#include "interfaces/msg/actuator_goal_position.hpp"
#include "actuator_manager.hpp"
#include "actuator_comm/controller/dynamixel_controller.hpp"
#include "actuator_comm/controller/actuator_controller.hpp"

using SetMotorConfig = interfaces::srv::SetMotorConfig;
using ActuatorGoalPosition = interfaces::msg::ActuatorGoalPosition;

struct Actuator
{
    int id;
    int min_deg; 
    int max_deg;
    std::string device; 
}; 

struct ActuatorType
{
    double angular_resolution;
    std::map<int, Actuator> actuators;
}; 

class ActuatorNode : public rclcpp::Node
{
    public:
        explicit ActuatorNode(const rclcpp::NodeOptions & options);
        virtual ~ActuatorNode();

        SerialHandler* init_serial(const char* device, int baudrate);

    private:
        void load_actuators_config();
        void load_hardware_config();

        std::optional<ActuatorType> get_actuator_type(
            uint8_t id, std::string& type);
        std::optional<std::shared_ptr<ActuatorController>> get_controller(
            uint8_t id, std::string device);

        void goal_position_callback(const ActuatorGoalPosition::SharedPtr msg);
        void motor_service_callback(
            const std::shared_ptr<SetMotorConfig::Request> request,
            std::shared_ptr<SetMotorConfig::Response> response);

        std::map<std::string, std::shared_ptr<ActuatorController>> controller_map_;
        std::map<std::string, ActuatorType> actuators_config_;
            
        rclcpp::Subscription<ActuatorGoalPosition>::SharedPtr actuator_subscriber_;
        rclcpp::Service<SetMotorConfig>::SharedPtr motor_service_;
        
        int update_rate_ms_;
};

#endif // ACTUATOR_NODE_HPP
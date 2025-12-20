#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "common_serial/serial_handler.hpp"
#include "interfaces/srv/set_motor_config.hpp"
#include "interfaces/msg/actuator_goal_position.hpp"
#include "actuator_manager.hpp"

using SetMotorConfig = interfaces::srv::SetMotorConfig;
using ActuatorGoalPosition = interfaces::msg::ActuatorGoalPosition;

struct DeviceInterface
{
    std::shared_ptr<SerialHandler> serial;
    std::shared_ptr<ActuatorManager> manager;
};

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
        
        void send_packet();
        void set_actuator_manager();
        
        std::unique_ptr<ActuatorManager>& get_actuator_manager() { return actuator_manager_; }
        
        std::map<std::string, std::shared_ptr<SerialHandler>>& get_serial_handlers() { return serial_handlers_; }

    private:
        void load_actuators_config();
        void load_hardware_config();

        void goal_position_callback(const ActuatorGoalPosition::SharedPtr msg);
        void load_parameters();
        void motor_service_callback(
            const std::shared_ptr<SetMotorConfig::Request> request,
            std::shared_ptr<SetMotorConfig::Response> response);

        std::map<std::string, DeviceInterface> hardware_map_;
        std::map<std::string, ActuatorType> actuators_config_;
            
        rclcpp::Subscription<ActuatorGoalPosition>::SharedPtr actuator_subscriber_;
        rclcpp::Service<SetMotorConfig>::SharedPtr motor_service_;
        
        int update_rate_ms_;
};

#endif // ACTUATOR_NODE_HPP
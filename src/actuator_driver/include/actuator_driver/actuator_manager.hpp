#ifndef ACTUATOR_MANAGER_HPP
#define ACTUATOR_MANAGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include "actuator_comm/controller/actuator_controller.hpp"
#include "actuator_comm/core/actuator_factory.hpp"
#include "rclcpp/rclcpp.hpp"

struct ActuatorParams {
    std::string base_name;
    int update_rate_ms;
    std::string usb_port;
    uint32_t baudrate;
    std::vector<uint8_t> actuator_ids;
};

enum class CommandType {
    SetGoalPosition,
    SetTorque
};

class ActuatorManager
{
    public:
        explicit ActuatorManager();

        void init_node(rclcpp::Node* node);
        std::shared_ptr<ActuatorController> get_controller() { return controller_; }
        int init_comm();
        int execute_command( uint8_t id, const std::string& command, 
            const std::vector<int16_t>& params);
        int set_goal_position(uint8_t id, uint16_t goal);
        int get_current_position(uint8_t id, uint16_t& curr_pos);

        ActuatorParams get_parameters() { return parameters_; }
        
    private:
        void declare_parameters(rclcpp::Node* node);
        void load_parameters(rclcpp::Node* node);
        bool is_valid_id(uint8_t id) const;

        ActuatorParams parameters_;
        std::shared_ptr<ActuatorController> controller_;

};

#endif // ACTUATOR_MANAGER_HPP
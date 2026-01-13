#ifndef NODE_MANAGER_HPP
#define NODE_MANAGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "actuator_driver/controller/actuator_controller.hpp"
#include "actuator_driver/core/actuator_factory.hpp"
#include "control/model/actuator_manager.hpp"
#include "control/node/parameter_manager.hpp"
#include "rclcpp/rclcpp.hpp"

struct ActuatorParams {
    std::string base_name;
    int update_rate_ms;
    std::string usb_port;
    uint32_t baudrate;
    std::vector<uint8_t> actuator_ids;
};

enum class ActuatorError {
    OK = 0,
    NotInitialized,
    InvalidID,
    InvalidParameter,
    CommunicationError,
    UnsupportedCommand
};

class NodeManager
{
    public:
        explicit NodeManager();

        void init_node(rclcpp::Node* node);
        std::shared_ptr<ActuatorController> get_controller() { return controller_; }
        ActuatorError init_serial();

        ActuatorError set_torque(uint8_t id, uint8_t status);
        ActuatorError set_goal_position(uint8_t id, uint16_t goal);
        ActuatorError get_current_position(uint8_t id, uint16_t& curr_pos);

        const ActuatorParams& get_parameters() const { return parameters_; }
        
    private:
        void declare_parameters(rclcpp::Node* node);
        void load_parameters(rclcpp::Node* node);
        bool is_valid_id(uint8_t id) const;

        std::shared_ptr<ActuatorManager> actuator_manager_;
        std::shared_ptr<ParameterManager> parameter_manager_;
        ActuatorParams parameters_;
        std::shared_ptr<ActuatorController> controller_;

};

#endif // ACTUATOR_MANAGER_HPP
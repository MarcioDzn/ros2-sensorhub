#ifndef NODE_MANAGER_HPP
#define NODE_MANAGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

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
        explicit NodeManager(
            std::shared_ptr<ActuatorManager> actuator_manager, 
            std::shared_ptr<ParameterManager> parameter_manager);

        void init_node(rclcpp::Node* node);
        std::shared_ptr<ActuatorController> get_controller() { return controller_; }
        ActuatorError init_serial();
        ActuatorError set_torque(const std::vector<uint8_t>& ids, bool status);
        ActuatorError set_goal_position(    
            const std::vector<uint8_t>& ids, 
            const std::vector<uint16_t>& positions);
        ActuatorError get_current_position(    
            const std::vector<uint8_t>& ids, 
            std::vector<uint16_t>& positions);

    private:
        std::shared_ptr<ActuatorManager> actuator_manager_;
        std::shared_ptr<ParameterManager> parameter_manager_;
        std::shared_ptr<ActuatorController> controller_;

        std::mutex bus_mutex_;
};

#endif // NODE_MANAGER_HPP
#ifndef PRESSURE_MANAGER_HPP
#define PRESSURE_MANAGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "driver/common/pressure_controller.hpp"
#include "driver/pressure_factory.hpp"
#include "control/node/parameter_manager.hpp"
#include "models/pressure.hpp"

#include "rclcpp/rclcpp.hpp"

struct PressureParams {
    std::string base_name;
    int update_rate_ms;
    std::vector<std::string> usb_ports;
    uint32_t baudrate;
    std::vector<uint8_t> ids;
};

enum class PressureError {
    OK = 0,
    NotInitialized,
    InvalidID,
    InvalidParameter,
    CommunicationError,
    UnsupportedCommand
};

class PressureManager
{
    public:
        explicit PressureManager(
            std::shared_ptr<ParameterManager> parameter_manager);

        void init_node(rclcpp::Node* node);
        std::map<uint8_t, std::shared_ptr<PressureController>> 
        get_controllers() { return controllers_; }
        PressureError init_comm();
        PressureError get_data(uint8_t id, std::vector<uint16_t>& data);
        
    private:
        bool is_valid_id(uint8_t id) const;

        std::shared_ptr<ParameterManager> parameter_manager_;
        std::map<uint8_t, std::shared_ptr<PressureController>> controllers_;

};

#endif // PRESSURE_MANAGER_HPP
#ifndef PRESSURE_MANAGER_HPP
#define PRESSURE_MANAGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "pressure_comm/pressure_controller.hpp"
#include "pressure_comm/core/pressure_factory.hpp"
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
        explicit PressureManager();

        void init_node(rclcpp::Node* node);
        std::map<uint8_t, std::shared_ptr<PressureController>> 
        get_controllers() { return controllers_; }
        PressureError init_comm();
        PressureError get_data(uint8_t id, uint16_t& data);

        const PressureParams& get_parameters() const { return parameters_; }
        
    private:
        void declare_parameters(rclcpp::Node* node);
        void load_parameters(rclcpp::Node* node);
        bool is_valid_id(uint8_t id) const;

        PressureParams parameters_;
        std::map<uint8_t, std::shared_ptr<PressureController>> controllers_;

};

#endif // PRESSURE_MANAGER_HPP
#ifndef PRESSURE_NODE_HPP
#define PRESSURE_NODE_HPP

#include <memory>
#include <map>
#include <vector>

#include "common_serial/serial_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/pressure_state.hpp"
#include "interfaces/msg/pressure_data.hpp"
#include "common_serial/serial_handler.hpp"
#include "pressure_manager.hpp"

using PressureState = interfaces::msg::PressureState;
using PressureData = interfaces::msg::PressureData;

struct DeviceInterface
{
    std::shared_ptr<SerialHandler> serial;
};

struct PressureSensor
{
    int id;
    std::string device; 
};

class PressureNode : public rclcpp::Node
{
    public:
        explicit PressureNode();
        virtual ~PressureNode();

        bool init_serial(const char* device, int baudrate);
        
    private:
        void state_callback();
        std::shared_ptr<PressureManager> manager_;

        std::map<uint8_t, PressureSensor> pressure_sensors_;
        rclcpp::Publisher<PressureState>::SharedPtr publisher_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        int update_rate_ms_;
};

#endif // PRESSURE_NODE_HPP

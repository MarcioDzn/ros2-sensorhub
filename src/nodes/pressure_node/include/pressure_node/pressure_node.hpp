#ifndef PRESSURE_NODE_HPP
#define PRESSURE_NODE_HPP

#include <memory>
#include <map>
#include <vector>

#include "common_serial/serial_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/pressure_data.hpp"
#include "common_serial/serial_handler.hpp"
#include "pressure_manager.hpp"

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
        void timer_callback();
        void publish_sensor_data(
            int sensor_id, const std::vector<uint16_t>& values);

        void load_parameters();

        std::shared_ptr<PressureManager> manager_;

        std::map<uint8_t, PressureSensor> pressure_sensors_;
        std::map<uint8_t, rclcpp::Publisher<PressureData>::SharedPtr> publishers_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        int update_rate_ms_;
};

#endif // PRESSURE_NODE_HPP

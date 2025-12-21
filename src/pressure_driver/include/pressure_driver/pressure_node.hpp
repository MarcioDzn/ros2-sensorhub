#ifndef PRESSURE_NODE_HPP
#define PRESSURE_NODE_HPP

#include <memory>
#include <map>
#include <vector>

#include "common_serial/serial_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/insole_data.hpp"
#include "common_serial/serial_handler.hpp"

using InsoleData = interfaces::msg::InsoleData;

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
        explicit PressureNode(const rclcpp::NodeOptions & options);
        virtual ~PressureNode();

        bool init_serial(const char* device, int baudrate);
        
    private:
        void load_hardware_config();
        void load_pressure_sensors_config();
        void create_publishers();
        bool setup_serial_port(const std::string &path, const int baudrate);

        void timer_callback();
        void load_parameters();
        void set_parameters();
        bool get_pressure_data(std::shared_ptr<SerialHandler> handler, char* buffer, size_t max_size);

        std::map<std::string, DeviceInterface> hardware_map_;
        std::map<int, PressureSensor> pressure_sensors_;

        std::map<int, rclcpp::Publisher<InsoleData>::SharedPtr> publishers_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        int update_rate_ms_;
};

#endif // PRESSURE_NODE_HPP

#ifndef PRESSURE_NODE_HPP
#define PRESSURE_NODE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/insole_data.hpp"
#include "common_serial/serial_handler.hpp"

using InsoleData = interfaces::msg::InsoleData;

class PressureNode : public rclcpp::Node
{
    public:
        explicit PressureNode();
        virtual ~PressureNode();

        bool init_serial(const char* device, int baudrate);
        
    private:
        void timer_callback();
        void load_parameters();
        void set_parameters();
        
        rclcpp::Publisher<InsoleData>::SharedPtr publisher_;
        
        std::unique_ptr<SerialHandler> serial_handler_;
        rclcpp::TimerBase::SharedPtr timer_;
        int update_rate_ms_;
};

#endif // PRESSURE_NODE_HPP

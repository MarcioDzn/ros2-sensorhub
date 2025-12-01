#ifndef MANIPULATOR_NODE_HPP
#define MANIPULATOR_NODE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "common_serial/serial_handler.hpp"

class ManipulatorNode : public rclcpp::Node
{
    public:
        explicit ManipulatorNode();
        virtual ~ManipulatorNode();

        bool init_serial(const char* device, int baudrate);
        void send_packet();
        
    private:
        void timer_callback();
        void load_parameters();
        void set_parameters();
        
        std::unique_ptr<SerialHandler> serial_handler_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MANIPULATOR_NODE_HPP

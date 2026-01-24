#ifndef PRESSURE_NODE_HPP
#define PRESSURE_NODE_HPP

#include <memory>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/pressure_state.hpp"
#include "interfaces/msg/pressure_data.hpp"

#include "control/node/parameter_manager.hpp"
#include "driver/pressure_driver.hpp"

using PressureState = interfaces::msg::PressureState;
using PressureData = interfaces::msg::PressureData;

struct DeviceInterface
{
    std::shared_ptr<SerialHandler> serial;
};

struct PressureSensorInfo
{
    int id;
    std::string device; 
};

class PressureNode : public rclcpp::Node
{
    public:
        explicit PressureNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~PressureNode();

        bool init_serial(const char* device, int baudrate);
        
    private:
        void state_callback();

        std::vector<std::shared_ptr<IPressureDriver>> pressure_drivers_;
        std::shared_ptr<ParameterManager> parameter_manager_;

        rclcpp::Publisher<PressureState>::SharedPtr publisher_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        int update_rate_ms_;
};

#endif // PRESSURE_NODE_HPP

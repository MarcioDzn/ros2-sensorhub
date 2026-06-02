#include <vector>
#include <sstream>

#include "pressure_node.hpp"
#include "driver/pressure_factory.hpp"

using namespace std::chrono_literals;

PressureNode::PressureNode(const rclcpp::NodeOptions& options) 
    : Node("pressure_node", options)
{
    init_driver();
    setup_node();
    
    subscriber_ = this->create_subscription<interfaces:msg:ControllerOut>(
        "pressure/state", qos);
}

PressureNode::~PressureNode() = default;

#include <vector>
#include <sstream>

#include "controller_node.hpp"

using namespace std::chrono_literals;

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options) 
    : Node("controller_node", options)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();

    subscriber_ = this->create_subscription<interfaces::msg::ControllerIn>(
        "controller/command",
        qos,
        std::bind(
            &ControllerNode::controller_callback,
            this,
            std::placeholders::_1
        )
    );

    publisher_ = this->create_publisher<interfaces::msg::ControllerOut>(
        "controller/state", qos);
}

void ControllerNode::controller_callback(
    const interfaces::msg::ControllerIn::SharedPtr msg)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Recebi mensagem!"
    );

    last_msg_ = *msg;
}

ControllerNode::~ControllerNode() = default;

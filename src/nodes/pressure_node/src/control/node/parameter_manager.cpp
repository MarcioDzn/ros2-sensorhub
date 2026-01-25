#include "control/node/parameter_manager.hpp"

#include "rclcpp/rclcpp.hpp"

ParameterManager::ParameterManager(
    rclcpp::Node* node) : node_(node)
{
    declare_parameters();
    load_parameters();
}

void ParameterManager::declare_parameters()
{
    node_->declare_parameter("base_name", "pressure");
    node_->declare_parameter("usb_ports", std::vector<std::string>{"/dev/ACM0", "/dev/ACM1"});
    node_->declare_parameter("baudrate", 115200);
    node_->declare_parameter("ids", std::vector<int64_t>{1, 2});
    node_->declare_parameter("names", std::vector<std::string>{"right_insole", "left_insole"});
    node_->declare_parameter("update_rate_ms", 15);
}

void ParameterManager::load_parameters()
{
    base_name_ = node_->get_parameter("base_name").as_string();
    baudrate_ = static_cast<uint32_t>(node_->get_parameter("baudrate").as_int());
    update_rate_ = node_->get_parameter("update_rate_ms").as_int();
    usb_ports_ = node_->get_parameter("usb_ports").as_string_array();
    names_ = node_->get_parameter("names").as_string_array();
    std::vector<long> raw_ids = node_->get_parameter("ids").as_integer_array();
    
    // converte de long pra uint8_t
    ids_.clear();
    ids_.reserve(raw_ids.size()); 
    for (long id : raw_ids)
        if (id >= 0 && id <= 253) // valores suportados por uint8_t
            ids_.push_back(static_cast<uint8_t>(id)); 
}
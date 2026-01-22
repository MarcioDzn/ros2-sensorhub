#include "control/node/parameter_manager.hpp"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>

ParameterManager::ParameterManager(
    rclcpp::Node* node) : node_(node)
{
    declare_parameters();
    load_parameters();
}

void ParameterManager::declare_parameters()
{
    node_->declare_parameter("base_name", "imu");
    node_->declare_parameter("ids", std::vector<int64_t>{1, 2, 3});
    node_->declare_parameter("multiplexer", std::vector<int64_t>{0, 1, 0});
    node_->declare_parameter("addresses", std::vector<int64_t>{40, 41, 40});
    node_->declare_parameter("update_rate_ms", 15);
}

void ParameterManager::load_parameters()
{
    base_name_ = node_->get_parameter("base_name").as_string();
    update_rate_ = node_->get_parameter("update_rate_ms").as_int();
    std::vector<long> raw_ids = node_->get_parameter("ids").as_integer_array();
    std::vector<long> raw_multiplexer = node_->get_parameter("multiplexer").as_integer_array();
    std::vector<long> raw_addresses = node_->get_parameter("addresses").as_integer_array();
    
    // evita segfault
    size_t min_size = std::min(
        raw_ids.size(), std::min(raw_addresses.size(), raw_multiplexer.size()));

    ids_.clear();
    ids_.reserve(min_size);
    multiplexer_.clear();
    multiplexer_.reserve(min_size);
    addresses_.clear();
    addresses_.reserve(min_size);

    // converte de long pra uint8_t
    for (size_t i = 0; i < min_size; i++)
    {
        for (long id : raw_ids)
            if (id >= 0 && id <= 253) 
                ids_.push_back(static_cast<uint8_t>(id)); 

        for (long multiplexer : raw_multiplexer)
            if (multiplexer >= 0 && multiplexer <= 253) 
                multiplexer_.push_back(static_cast<uint8_t>(multiplexer)); 

        for (long address : raw_addresses)
            if (address >= 0 && address <= 253) 
                addresses_.push_back(static_cast<uint8_t>(address)); 
    }
}
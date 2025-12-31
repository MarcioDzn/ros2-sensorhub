#include "imu_manager.hpp"

#include <thread>

IMUManager::IMUManager() {}


int IMUManager::init(rclcpp::Node* node)
{
    declare_parameters(node);
    load_parameters(node);

    if (create_imus() != 0) return -1;
    if (setup_imus() != 0) return -1;

    return 0;
}

void IMUManager::declare_parameters(rclcpp::Node* node)
{
    node->declare_parameter("base_name", "imu");
    node->declare_parameter("ids", std::vector<int64_t>{1, 2, 3});
    node->declare_parameter("multiplexer", std::vector<int64_t>{0, 1, 0});
    node->declare_parameter("addresses", std::vector<int64_t>{40, 41, 40});
    node->declare_parameter("update_rate_ms", 15);
}

void IMUManager::load_parameters(rclcpp::Node* node)
{
    parameters_.base_name = node->get_parameter("base_name").as_string();
    parameters_.update_rate_ms = node->get_parameter("update_rate_ms").as_int();

    std::vector<int64_t> raw_ids = node->get_parameter("ids").as_integer_array();
    std::vector<int64_t> raw_multiplexer = node->get_parameter("multiplexer").as_integer_array();
    std::vector<int64_t> raw_addresses = node->get_parameter("addresses").as_integer_array();

    // encontrando menor quantidade de itens
    size_t size_ids = raw_ids.size();
    size_t size_multiplexer = raw_multiplexer.size();
    size_t size_addresses = raw_addresses.size();

    size_t max_size = std::min({size_ids, size_multiplexer, size_addresses});

    // adicionando os valores nos arrays
    parameters_.ids.clear();
    parameters_.multiplexer.clear();
    parameters_.addresses.clear();
    for (size_t i = 0; i < max_size; i++)
    {
        parameters_.ids.push_back(static_cast<int>(raw_ids[i]));
        parameters_.multiplexer.push_back(static_cast<int64_t>(raw_multiplexer[i]));
        parameters_.addresses.push_back(static_cast<uint8_t>(raw_addresses[i]));
    }
}

int IMUManager::create_imus()
{
    size_t max_size = parameters_.ids.size();
    for (size_t i = 0; i < max_size; i++)
    {
        auto imu = std::make_shared<BNO055IMU>(
            parameters_.multiplexer[i], 
            parameters_.ids[i], 
            parameters_.addresses[i]);
        
        if (imus_.find(parameters_.ids[i]) != imus_.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("IMUManager"), 
            "Id %d duplicado. Sobrescrevendo!", parameters_.ids[i]);
        }
        imus_[parameters_.ids[i]] = imu;
    }
    return 0;
}

int IMUManager::setup_imus()
{
    if (imus_.empty())
        return -1;

    BNO055IMU::setup_wiringpi();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms

    for (const auto& [_, imu] : imus_) imu->setup();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms

    for (const auto& [_, imu] : imus_) imu->calibrate();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1s;

    return 0;
}
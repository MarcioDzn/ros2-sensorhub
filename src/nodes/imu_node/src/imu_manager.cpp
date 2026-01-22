#include "imu_manager.hpp"

#include <thread>

IMUManager::IMUManager() {}

int IMUManager::init(rclcpp::Node* node)
{
    if (create_imus() != 0) return -1;
    if (setup_imus() != 0) return -1;

    return 0;
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

    for (const auto& [_, imu] : imus_) imu->setup();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms

    for (const auto& [_, imu] : imus_) imu->calibrate_euler();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1s;

    return 0;
}
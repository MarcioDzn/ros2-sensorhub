#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <vector>
#include <string>
#include <thread>
#include <chrono>

#include "interfaces/msg/imu_data.hpp"
#include "managers/interface_sensor_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "imu_lib.hpp"

using IMUData = interfaces::msg::IMUData;

class IMUManager : public ISensorManager
{
    public:
        IMUManager(rclcpp::Node* node);

        void loadParameters() override;
        void createSensors() override; 
        void initialize();
        void createPublishers() override; 
        void publishAll() override;

    private:
        rclcpp::Node* node_;

        std::vector<std::shared_ptr<BNO055IMU>> imus_;
        std::vector<rclcpp::Publisher<IMUData>::SharedPtr> publishers_;

        // parâmetros
        std::vector<int64_t> imu_ids_;
        std::vector<int64_t> imu_addresses_;
        std::vector<int64_t> multiplex_ids_;
        std::vector<std::vector<int>> euler_orders_;
        std::vector<std::string> imu_names_;

        void setParameters();
        std::vector<std::vector<int>> chunkVector(
            const std::vector<int64_t>& flat, 
            size_t group_size
        );
};

#endif // IMU_MANAGER_HPP
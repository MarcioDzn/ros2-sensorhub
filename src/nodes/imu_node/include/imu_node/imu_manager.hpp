#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <map>
#include <memory>

#include "driver/bno055imu.hpp"

#include "rclcpp/rclcpp.hpp"

struct IMUParams {
    std::string base_name;
    int update_rate_ms;
    std::vector<int64_t> multiplexer;
    std::vector<uint8_t> addresses;
    std::vector<int> ids;
};

class IMUManager
{
    public:
        IMUManager();

        int init(rclcpp::Node* node);

        const std::map<int, std::shared_ptr<BNO055IMU>>&
        get_imus() const { return imus_; }

        const IMUParams& get_parameters() const { return parameters_; }
        
    private:
        void declare_parameters(rclcpp::Node* node);
        void load_parameters(rclcpp::Node* node);
        int create_imus();
        int setup_imus();

        std::map<int, std::shared_ptr<BNO055IMU>> imus_;
        IMUParams parameters_;

};

#endif // IMU_MANAGER_HPP
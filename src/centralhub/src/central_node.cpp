#include "central_node.hpp"

using namespace std::chrono_literals;

CentralNode::CentralNode() : Node("central_node"), count_(0)
{
    publisher_ = this->create_publisher<IMUData>("/imu", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&CentralNode::timer_callback, this));
}

void CentralNode::timer_callback()
{
    auto message = IMUData();

    std::vector<double> imu_data;
    get_imu_data(imu_data);

    message.roll = imu_data[0];
    message.pitch = imu_data[1];
    message.yaw = imu_data[2];

    RCLCPP_INFO(
        this->get_logger(), 
        "PUBLICANDO\nROLL: %f\nPITCH: %f\nYAW: %f", 
        message.roll, message.pitch, message.yaw);

    publisher_->publish(message);
}

// mock de dados retornados por um IMU
void CentralNode::get_imu_data(std::vector<double>& imu_data)
{
    imu_data.resize(3);
    imu_data[0] = 1.43;
    imu_data[1] = 280.12;
    imu_data[2] = 14.54;
}

CentralNode::~CentralNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}
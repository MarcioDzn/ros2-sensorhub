#include "central_node.hpp"

using namespace std::chrono_literals;

CentralNode::CentralNode() : Node("central_node"), count_(0), imu_ids_({1, 2, 3})
{
    // criando um publisher pra cada IMU
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        auto publisher = this->create_publisher<IMUData>("/sensor" + std::to_string(imu_ids_[id]) + "/imu", 10);
        publishers_.push_back(publisher);
    }

    timer_ = this->create_wall_timer(500ms, std::bind(&CentralNode::timer_callback, this));
}

void CentralNode::timer_callback()
{
    for (size_t id = 0; id < imu_ids_.size(); id++)
    {
        auto message = IMUData();
        std::vector<double> imu_data;

        get_imu_data(id, imu_data);

        message.roll = imu_data[0];
        message.pitch = imu_data[1];
        message.yaw = imu_data[2];

        RCLCPP_INFO(
            this->get_logger(), 
            "[ID %zu] PUBLICANDO\nROLL: %f\nPITCH: %f\nYAW: %f", 
            id, message.roll, message.pitch, message.yaw);

        publishers_[id]->publish(message);
    }
}

// mock de dados retornados por um IMU
void CentralNode::get_imu_data(int id, std::vector<double>& imu_data)
{
    imu_data.resize(3);
    imu_data[0] = id*15.43;
    imu_data[1] = id*28.12;
    imu_data[2] = id*1.54;
}

CentralNode::~CentralNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}
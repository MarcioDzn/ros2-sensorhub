#include "central_node.hpp"
#include "managers/imu_manager.hpp"

using namespace std::chrono_literals;

CentralNode::CentralNode() : Node("central_node"), count_(0)
{
    imu_manager_ = std::make_unique<IMUManager>(this);
    imu_manager_->loadParameters();
    imu_manager_->createSensors();
    imu_manager_->createPublishers();

    int rate = imu_manager_->getUpdateRateMs();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(rate), 
        std::bind(&CentralNode::timer_callback, this));
}

// callback que envia os dados dos 3 sensores
void CentralNode::timer_callback()
{
    imu_manager_->publishAll();
}

CentralNode::~CentralNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}
#include "imu_node.hpp"

using namespace std::chrono_literals;

IMUNode::IMUNode(const rclcpp::NodeOptions & options) : Node("imu_node", options)
{
    manager_ = std::make_unique<IMUManager>();

    if (manager_->init(this) < 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware");
        throw std::runtime_error("");
    }
    
    const auto& parameters = manager_->get_parameters();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    auto publisher_ = this->create_publisher<IMUState>(
        parameters.base_name + "/state", qos);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters.update_rate_ms), 
        std::bind(&IMUNode::state_callback, this));
}

void IMUNode::state_callback()
{
    const auto& imus = manager_->get_imus();
    auto msg = IMUState();

    for (const auto& [id, imu] : imus)
    {
        std::vector<float> imu_data;
        imu->get_euler_data(imu_data);

        IMUData data;
        data.roll = imu_data[0];
        data.pitch = imu_data[1];
        data.yaw = imu_data[2];
        
        msg.imus.push_back(data);
    }

    msg.stamp = this->get_clock()->now();
    publisher_->publish(msg);
}

IMUNode::~IMUNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}

#include "imu_node.hpp"

using namespace std::chrono_literals;

IMUNode::IMUNode(const rclcpp::NodeOptions & options) : Node("imu_node", options)
{
    parameter_manager_ = std::make_shared<ParameterManager>(this);
    manager_ = std::make_unique<IMUManager>(parameter_manager_);

    if (manager_->init(this) < 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware");
        throw std::runtime_error("");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    publisher_ = this->create_publisher<IMUState>(
        parameter_manager_->get_base_name() + "/state", qos);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        std::bind(&IMUNode::state_callback, this));
}

void IMUNode::state_callback()
{
    const auto& imus = manager_->get_imus();
    auto msg = IMUState();

    for (const auto& [id, imu] : imus)
    {
        std::vector<float> imu_euler_data;
        std::vector<float> imu_quaternions_data;
        
        imu->get_euler_data(imu_euler_data);
        imu->get_quaternions_data(imu_quaternions_data);

        IMUData data;
        data.id = id;

        // euler angles
        data.roll = imu_euler_data[0];
        data.pitch = imu_euler_data[1];
        data.yaw = imu_euler_data[2];

        // quaternions
        data.q_w = imu_quaternions_data[0];
        data.q_x = imu_quaternions_data[1];
        data.q_y = imu_quaternions_data[2];
        data.q_z = imu_quaternions_data[3];
        
        msg.imus.push_back(data);
    }

    msg.header.stamp = this->get_clock()->now();
    publisher_->publish(msg);
}

IMUNode::~IMUNode() = default;

#include "imu_node.hpp"

using namespace std::chrono_literals;

IMUNode::IMUNode() : Node("imu_node")
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
    for (const auto &id: parameters.ids)
    {
        // cria um publisher pra cada IMU
        RCLCPP_INFO(this->get_logger(), 
            "Criando publisher para o sensor %d", id);

        auto topic = parameters.base_name + "/data/id_" + 
            std::to_string(static_cast<int>(id));
        auto publisher = this->create_publisher<IMUData>(topic, qos);
        publishers_[id] = publisher;
    }
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters.update_rate_ms), 
        std::bind(&IMUNode::timer_callback, this));
}

// callback que envia os dados dos 3 sensores
void IMUNode::timer_callback()
{
    const auto& imus = manager_->get_imus();
    for (const auto& [id, imu] : imus)
    {
        auto message = IMUData();
        std::vector<double> imu_data;
        
        imu->get_data(imu_data);
        message.roll = imu_data[0];
        message.pitch = imu_data[1];
        message.yaw = imu_data[2];
        
        message.stamp = this->get_clock()->now();

        publishers_[id]->publish(message);
    }
}

IMUNode::~IMUNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}

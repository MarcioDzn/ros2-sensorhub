#include "convergence_node.hpp"

ConvergenceNode::ConvergenceNode() : Node("convergence_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
          .best_effort()
          .durability_volatile();
    sub_ = this->create_subscription<interfaces::msg::SyncedSensorData>(
        "/synced_data", qos,
        std::bind(&ConvergenceNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "ConvergenceNode iniciado");
}

void ConvergenceNode::time_callback(const interfaces::msg::SyncedSensorData::SharedPtr msg)
{
    // Calcula tempo relativo
    uint64_t current_time_ms = rclcpp::Time(msg->header.stamp).nanoseconds() / 1000000ULL;
    if (start_time_ms_ == 0) start_time_ms_ = current_time_ms;
    double relative_time_ms = static_cast<double>(current_time_ms - start_time_ms_);

    // Janela 1: Juntas (usando o índice 1)
    plotter_.add_data(1, "joint_1", relative_time_ms, static_cast<double>(msg->actuator_data.positions[0]));
    plotter_.add_data(1, "joint_2", relative_time_ms, static_cast<double>(msg->actuator_data.positions[1]));
    plotter_.add_data(1, "joint_3", relative_time_ms, static_cast<double>(msg->actuator_data.positions[2]));
    
    // Janela 2: IMU (usando o índice 2)
    plotter_.add_data(2, "q_x", relative_time_ms, static_cast<double>(msg->imu_data.imus[0].q_x));
    plotter_.add_data(2, "q_y", relative_time_ms, static_cast<double>(msg->imu_data.imus[0].q_y));
    plotter_.add_data(2, "q_z", relative_time_ms, static_cast<double>(msg->imu_data.imus[0].q_z));
    plotter_.add_data(2, "q_w", relative_time_ms, static_cast<double>(msg->imu_data.imus[0].q_w));

    // Rate limiting: plota a cada 20 mensagens para evitar crash no X11
    plotter_.plot();
}

ConvergenceNode::~ConvergenceNode()
{
    plotter_.show(); // exibe o gráfico após o fim da execução
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConvergenceNode>());
    rclcpp::shutdown();
    return 0;
}

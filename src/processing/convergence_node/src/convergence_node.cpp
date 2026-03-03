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
    // tempo total
    plotter_.add_data("joint_1", msg_counter_, static_cast<double>(msg->actuator_data.positions[0]));
    plotter_.add_data("joint_2", msg_counter_, static_cast<double>(msg->actuator_data.positions[1]));
    plotter_.add_data("joint_3", msg_counter_, static_cast<double>(msg->actuator_data.positions[2]));

    // tempos individuais
    //for (size_t idx = 0; idx < msg->times.size(); idx++) 
        //plotter_.add_data("imu_time_" + msg->names[idx], msg_counter_, static_cast<double>(msg->times[idx]));

    // tempo esperado (update_rate)
    // TODO: enviar pela msg
    //plotter_.add_data("imu_update_rate", msg_counter_, static_cast<double>(15000.0)); // 15ms

    plotter_.plot();

    // --- CSV ---
    /*
    CsvRow row;
    for (double t : msg->times)
        row.values.push_back(static_cast<double>(t));
    row.values.push_back(static_cast<double>(msg->total_time)); // última coluna = total
    csv_writer_.add_row(row);

    // cabeçalho dinâmico
    std::vector<std::string> header;
    for (size_t idx = 0; idx < msg->times.size(); idx++)
        header.push_back(msg->names[idx]);
    header.push_back("TotalTime");

    csv_writer_.save("imu_times.csv", header);
    */
    msg_counter_++;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConvergenceNode>());
    rclcpp::shutdown();
    return 0;
}

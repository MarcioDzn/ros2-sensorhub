#include "client.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options)
    : Node("client_node", options)
{
    actuator_publisher_ = this->create_publisher<interfaces::msg::ActuatorState>("/actuator/state", 10);
    pressure_publisher_ = this->create_publisher<interfaces::msg::PressureState>("/pressure/state", 10);
    imu_publisher_ = this->create_publisher<interfaces::msg::IMUState>("/imu/state", 10);

    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    execute_path();
}

// Auxiliar para printar timestamp
std::string ts_to_string(const rclcpp::Time& t) {
    auto ns = t.nanoseconds();
    std::ostringstream oss;
    oss << ns / 1000000 << " ms";
    return oss.str();
}

// Função principal de execução
void ClientNode::execute_path()
{
    // Configura delays entre tópicos (em milissegundos)
    std::vector<int> delays_ms = {0, 1, 1}; // Actuator -> Pressure -> IMU

    for (size_t idx = 0; idx < 3; idx++) // 3 loops só pra teste
    {
        auto start_time = this->get_clock()->now();

        // --- Actuator ---
        interfaces::msg::ActuatorState actuator_msg;
        actuator_msg.header.stamp = start_time;
        actuator_msg.names.push_back("joint_1");
        actuator_msg.positions.push_back(500);
        actuator_publisher_->publish(actuator_msg);

        rclcpp::sleep_for(std::chrono::milliseconds(delays_ms[0]));

        // --- Pressure ---
        // interfaces::msg::PressureUnitSensor pressure_unit;
        // pressure_unit.id = 1;
        // pressure_unit.pressure = 1000;

        // interfaces::msg::PressureData pressure_data;
        // pressure_data.pressures.push_back(pressure_unit);

        // interfaces::msg::PressureState pressure_msg;
        // pressure_msg.header.stamp = this->get_clock()->now();
        // pressure_msg.names.push_back("insole_1");
        // pressure_msg.pressures.push_back(pressure_data);
        // pressure_publisher_->publish(pressure_msg);

        // rclcpp::sleep_for(std::chrono::milliseconds(delays_ms[1]));

        // --- IMU ---
        interfaces::msg::IMUData imu_data;
        imu_data.name = "imu_1";
        imu_data.q_x = 0.8;
        imu_data.q_y = 0.8;
        imu_data.q_z = 0.8;
        imu_data.q_w = 0.8;

        interfaces::msg::IMUState imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.imus.push_back(imu_data);
        imu_publisher_->publish(imu_msg);

        rclcpp::sleep_for(std::chrono::milliseconds(delays_ms[2]));

        // --- Print de debug ---
        RCLCPP_INFO(this->get_logger(),
            "Loop %zu | Actuator: %s | IMU: %s",
            idx,
            ts_to_string(actuator_msg.header.stamp).c_str(),
            //ts_to_string(pressure_msg.header.stamp).c_str(),
            ts_to_string(imu_msg.header.stamp).c_str());
    }
}

ClientNode::~ClientNode() {}
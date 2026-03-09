#include "client.hpp"

#include <vector>
#include <string>

using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options) 
    : Node("client_node", options)
{   
    
    synced_data_publisher_ = this->create_publisher<interfaces::msg::SyncedSensorData>("/synced_data", 10);
    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    execute_path();
}

void ClientNode::execute_path() 
{
    interfaces::msg::ActuatorState actuator_state_msg;
    actuator_state_msg.header.stamp = this->get_clock()->now();
    actuator_state_msg.names.push_back("joint_1");
    actuator_state_msg.positions.push_back(500);

    interfaces::msg::PressureUnitSensor pressure_unit_sensor_data;
    pressure_unit_sensor_data.id = 1;
    pressure_unit_sensor_data.pressure = 1000;

    interfaces::msg::PressureData pressure_data;
    pressure_data.pressures.push_back(pressure_unit_sensor_data);

    interfaces::msg::PressureState pressure_state_msg;
    pressure_state_msg.header.stamp = this->get_clock()->now();
    pressure_state_msg.names.push_back("insole_1");
    pressure_state_msg.pressures.push_back(pressure_data);

    interfaces::msg::IMUData imu_data;
    imu_data.name = "imu_1";
    imu_data.q_x = 0.8;
    imu_data.q_y = 0.8;
    imu_data.q_z = 0.8;
    imu_data.q_w = 0.8;

    interfaces::msg::IMUState imu_state_msg;
    imu_state_msg.header.stamp  = this->get_clock()->now();
    imu_state_msg.imus.push_back(imu_data);


    interfaces::msg::SyncedSensorData synced_msg;
    synced_msg.header.stamp     = this->get_clock()->now();
    synced_msg.actuator_data    = actuator_state_msg;
    synced_msg.pressure_data    = pressure_state_msg;
    synced_msg.imu_data         = imu_state_msg;


    for (size_t idx = 0; idx < 100; idx++) 
    {
        synced_data_publisher_->publish(synced_msg);
        rclcpp::sleep_for(15ms);
    }
}

ClientNode::~ClientNode() {};

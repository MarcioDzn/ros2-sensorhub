#include "actuator_time_node.hpp"

ActuatorTimeNode::ActuatorTimeNode() : Node("actuator_time_node")
{
    sub_ = this->create_subscription<interfaces::msg::Time>(
        "/actuator/time", 10,
        std::bind(&ActuatorTimeNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "ActuatorTimeNode iniciado");
}

void ActuatorTimeNode::time_callback(const interfaces::msg::Time::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
                "[Actuator] Total time: %ld us | Names: %lu",
                msg->total_time, msg->names.size());

    // tempo total
    plotter_.add_data("actuator_total_time", msg_counter_, static_cast<double>(msg->total_time));

    // tempos individuais
    for (size_t idx = 0; idx < msg->times.size(); idx++) 
        plotter_.add_data("actuator_time_" + msg->names[idx], msg_counter_, static_cast<double>(msg->times[idx]));

    // tempo esperado (update_rate)
    // TODO: enviar pela msg
    plotter_.add_data("actuator_update_rate", msg_counter_, static_cast<double>(15000.0)); // 15ms

    msg_counter_++;

    plotter_.plot();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorTimeNode>());
    rclcpp::shutdown();
    return 0;
}
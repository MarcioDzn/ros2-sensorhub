#ifndef PRESSURE_TIME_NODE_HPP
#define PRESSURE_TIME_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/time.hpp"
#include "plotter.hpp"

class PressureTimeNode : public rclcpp::Node
{
    public:
        PressureTimeNode();

    private:
        void time_callback(const interfaces::msg::Time::SharedPtr msg);
        rclcpp::Subscription<interfaces::msg::Time>::SharedPtr sub_;
        
        Plotter plotter_;
        int msg_counter_ = 0; // eixo X do gráfico
};

#endif //PRESSURE_TIME_NODE_HPP
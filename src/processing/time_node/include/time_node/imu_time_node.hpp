#ifndef IMU_TIME_NODE_HPP
#define IMU_TIME_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/time.hpp"
#include "plotter.hpp"

class IMUTimeNode : public rclcpp::Node
{
    public:
        IMUTimeNode();

    private:
        void time_callback(const interfaces::msg::Time::SharedPtr msg);
        rclcpp::Subscription<interfaces::msg::Time>::SharedPtr sub_;

        Plotter plotter_;
        int msg_counter_ = 0; // eixo X do gráfico
};

#endif //IMU_TIME_NODE_HPP
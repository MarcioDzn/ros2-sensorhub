#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/actuator_goal_position.hpp"
#include <chrono>

#define START_POS           1300
#define MAX_POS_LIMIT       2900
#define MIN_POS_LIMIT       1400
#define STEP                20
#define FREQUENCY           100


using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("actuator_publisher");
    auto publisher = node->create_publisher<interfaces::msg::ActuatorGoalPosition>("/goal_position", 10);

    rclcpp::WallRate loop_rate(FREQUENCY);
    
    uint16_t pos = START_POS;
    int direction = 1; // 1 pra cima / -1 pra baixo 
    while (rclcpp::ok())
    {
        auto msg = interfaces::msg::ActuatorGoalPosition();
        msg.id = 1;
        msg.goal = pos;
        publisher->publish(msg);
        
        msg.id = 2;
        msg.goal = pos;
        publisher->publish(msg);
        
        RCLCPP_INFO(node->get_logger(), "Publicando posicao: %d", msg.goal);

        rclcpp::spin_some(node);
        loop_rate.sleep();
        
        if (pos > MAX_POS_LIMIT)
        {
            direction = -1;
        } else if (pos < MIN_POS_LIMIT)
        {
            direction = 1;
        }
        
        pos += STEP * direction;
    }

    rclcpp::shutdown();
    return 0;
}

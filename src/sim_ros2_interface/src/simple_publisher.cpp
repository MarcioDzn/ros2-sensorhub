// simple_publisher.cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher()
  : Node("simple_publisher"), count_(0) // Inicializa o Node com o nome
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("ola_mundo", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimplePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Node Publisher iniciado e publicando a cada 0.5s.");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Olá, ROS 2 C++! Count: " + std::to_string(count_++);
    
    publisher_->publish(message);
    
    RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", message.data.c_str());
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}

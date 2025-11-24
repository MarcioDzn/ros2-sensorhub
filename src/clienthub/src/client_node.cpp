#include <cstdio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/set_motor_config.hpp"

using SetMotorConfig = interfaces::srv::SetMotorConfig;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motor_config_client");

  rclcpp::Client<SetMotorConfig>::SharedPtr client =
    node->create_client<SetMotorConfig>("motor_config");

  auto request = std::make_shared<SetMotorConfig::Request>();

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }


  std::string command = argv[1];
  request->command = command;
  
    uint16_t motor_id = static_cast<uint16_t>(std::stoul(argv[2]));  
    request->motor_id = motor_id;
  
  for (int i = 3; i < argc; i++)
  {
    request->params.push_back(std::stoll(argv[i]));
  }
  
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Funcionou");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Não funcionou");
  }

  rclcpp::shutdown();
  return 0;
}

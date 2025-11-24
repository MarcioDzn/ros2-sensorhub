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


  uint16_t command = static_cast<uint16_t>(std::stoul(argv[1])); 
  std::string command_str;  
  
  switch(command)
  {
      case 1:
        command_str = "enable_torque";
        break;
      case 2:
        command_str = "disable_torque";
        break;
      case 3:
        command_str = "set_goal_position";
        break;
      case 4:
        command_str = "get_present_position";
        break;
      default:
        command_str = "get_present_position";
  }
  

  request->command = command_str;
  
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

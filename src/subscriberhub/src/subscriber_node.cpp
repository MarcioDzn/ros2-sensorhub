#include <memory>

#include "subscriber_node.hpp"

using std::placeholders::_1;

SubscriberNode::SubscriberNode() : Node("subscriber_node")
{
    subscription_ = this->create_subscription<IMUData>(
    "/sensor_1/imu", 10, std::bind(&SubscriberNode::topic_callback, this, _1));
}
    
void SubscriberNode::topic_callback(const IMUData & msg) const
{
	RCLCPP_INFO(
		this->get_logger(), "OUVINDO - ROLL: %f | PITCH: %f | YAW: %f", 
		msg.roll, msg.pitch, msg.yaw);
}

SubscriberNode::~SubscriberNode() = default;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

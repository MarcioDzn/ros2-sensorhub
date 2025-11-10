#include <memory>

#include "subscriber_node.hpp"

using std::placeholders::_1;

SubscriberNode::SubscriberNode() : Node("subscriber_node")
{
    this->declare_parameter<std::vector<std::string>>("imu_names", {"sensor_1", "sensor_2", "sensor_3"});
    auto imu_names = this->get_parameter("imu_names").as_string_array();
    
    for (size_t id = 0; id < imu_names.size(); id++)
    {
        auto imu_name = imu_names[id];
        auto subscription = this->create_subscription<IMUData>(
          "/" + imu_name + "/imu", 
          10, 
          [this, imu_name](const IMUData & msg) {
            topic_callback(msg, imu_name);
          }
        );
        
        imu_subscriptions_.push_back(subscription);
    }
}
    
void SubscriberNode::topic_callback(const IMUData & msg, const std::string & imu_name) const
{
	RCLCPP_INFO(
		this->get_logger(), "[%s] OUVINDO - ROLL: %f | PITCH: %f | YAW: %f", 
		imu_name.c_str(), msg.roll, msg.pitch, msg.yaw);
}

SubscriberNode::~SubscriberNode() = default;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

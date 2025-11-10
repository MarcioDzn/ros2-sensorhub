#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/imu_data.hpp"

using IMUData = interfaces::msg::IMUData;

class SubscriberNode : public rclcpp::Node
{
  public:
    explicit SubscriberNode();
    virtual ~SubscriberNode();

  private:
  
    std::vector<rclcpp::Subscription<IMUData>::SharedPtr> imu_subscriptions_;
    void topic_callback(const IMUData & msg, const std::string & imu_name) const;	
};

#endif // SUBSCRIBER_NODE_HPP

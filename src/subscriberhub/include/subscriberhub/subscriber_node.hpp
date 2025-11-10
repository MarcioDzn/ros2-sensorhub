#ifndef SUBSCRIBER_NODE_HPP
#define SUBSCRIBER_NODE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/imu_data.hpp"

using IMUData = interfaces::msg::IMUData;

class SubscriberNode : public rclcpp::Node
{
  public:
    explicit SubscriberNode();
    virtual ~SubscriberNode();

  private:
    rclcpp::Subscription<IMUData>::SharedPtr subscription_;
    void topic_callback(const IMUData & msg) const;	
};

#endif // SUBSCRIBER_NODE_HPP

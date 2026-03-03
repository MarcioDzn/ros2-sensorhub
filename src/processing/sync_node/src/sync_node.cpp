#include "sync_node.hpp"


SyncNode::SyncNode() : Node("sync_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
        
	publisher_ = this->create_publisher<SyncedSensorData>("synced_data", qos);
	
	imu_sub_.subscribe(this, "imu/state", qos.get_rmw_qos_profile());
	pressure_sub_.subscribe(this, "pressure/state", qos.get_rmw_qos_profile());
	actuator_sub_.subscribe(this, "actuator/state", qos.get_rmw_qos_profile());

    uint32_t queue_size = 50;
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), imu_sub_, pressure_sub_, actuator_sub_);

    sync_->registerCallback(
        std::bind(&SyncNode::synced_callback, this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3));
}

void SyncNode::synced_callback(
    const IMUState::ConstSharedPtr& imu_msg, 
    const PressureState::ConstSharedPtr& pressure_msg,
    const ActuatorState::ConstSharedPtr& actuator_msg)
{
    auto synced_msg = std::make_unique<SyncedSensorData>();

    synced_msg->imu_data = *imu_msg;
    synced_msg->pressure_data = *pressure_msg;
    synced_msg->actuator_data = *actuator_msg;
    
    publisher_->publish(std::move(synced_msg));

    RCLCPP_DEBUG(this->get_logger(), "Par de dados sincronizados publicado.");
}

SyncNode::~SyncNode() = default;




#include "sync_node.hpp"


SyncNode::SyncNode() : Node("sync_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
        
	publisher_ = this->create_publisher<SyncedSensorData>("synced_data", qos);
	
	imu_sub_.subscribe(this, "sensor_1/imu", qos.get_rmw_qos_profile());
	pressure_sub_.subscribe(this, "pressure", qos.get_rmw_qos_profile());

    uint32_t queue_size = 50;
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), imu_sub_, pressure_sub_);
    sync_->registerCallback(
        std::bind(&SyncNode::synced_callback, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
}

void SyncNode::synced_callback(const IMUData::ConstSharedPtr& imu_msg, 
                             const InsoleData::ConstSharedPtr& pressure_msg)
{
    auto synced_msg = std::make_unique<SyncedSensorData>();

    synced_msg->imu_data.stamp = imu_msg->stamp;

    synced_msg->imu_data.roll = imu_msg->roll; 
    synced_msg->imu_data.pitch = imu_msg->pitch;
    synced_msg->imu_data.yaw = imu_msg->yaw;
    
    synced_msg->pressure_data.stamp = pressure_msg->stamp;
    synced_msg->pressure_data.pressures = pressure_msg->pressures;
    
    publisher_->publish(std::move(synced_msg));

    RCLCPP_DEBUG(this->get_logger(), "Par de dados sincronizados publicado.");
}

SyncNode::~SyncNode() = default;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

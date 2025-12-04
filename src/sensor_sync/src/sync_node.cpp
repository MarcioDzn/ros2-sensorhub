#include "sync_node.hpp"


SyncNode::SyncNode() : Node("sync_node")
{
	publisher_ = this->create_publisher<SyncedSensorData>("synced_data", 10);
	
    imu_sub_.reset(new message_filters::Subscriber<IMUData>(this, "sensor_1/imu"));
    pressure_sub_.reset(new message_filters::Subscriber<InsoleData>(this, "pressure"));

    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(10), *imu_sub_, *pressure_sub_));

    const std::chrono::milliseconds SLOP_MS = std::chrono::milliseconds(5);
    sync_->setInterMessageLowerBound(SLOP_MS);
    
    using std::placeholders::_1;
    using std::placeholders::_2;
    sync_->registerCallback(std::bind(&SyncNode::synced_callback, this, _1, _2));
}

void SyncNode::synced_callback(const IMUData::ConstSharedPtr& imu_msg, 
                             const InsoleData::ConstSharedPtr& pressure_msg)
{
    auto synced_msg = std::make_unique<SyncedSensorData>();

    synced_msg->imu_stamp = imu_msg->stamp;

    synced_msg->roll = imu_msg->roll; // Exemplo: Substitua pelos campos reais
    synced_msg->pitch = imu_msg->pitch;
    synced_msg->yaw = imu_msg->yaw;
    
    synced_msg->pressure_stamp = pressure_msg->stamp;
    synced_msg->pressures = pressure_msg->pressures;
    
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

#include "sync_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

SyncNode::SyncNode() : Node("sync_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    publisher_ = this->create_publisher<SyncedSensorData>("/sync/data", qos);

    imu_sub_.subscribe(this, "imu/state", qos.get_rmw_qos_profile());
    pressure_sub_.subscribe(this, "pressure/state", qos.get_rmw_qos_profile());
    actuator_sub_.subscribe(this, "actuator/state", qos.get_rmw_qos_profile());

    imu_sub_.registerCallback([this](const IMUState::ConstSharedPtr& msg) { 
        last_imu_time_ = this->now(); 
        last_imu_msg_ = msg;
    });
    pressure_sub_.registerCallback([this](const PressureState::ConstSharedPtr& msg) { 
        last_pressure_time_ = this->now(); 
        last_pressure_msg_ = msg;
    });
    actuator_sub_.registerCallback([this](const ActuatorState::ConstSharedPtr& msg) { 
        last_actuator_time_ = this->now(); 
        last_actuator_msg_ = msg;
    });

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), imu_sub_, pressure_sub_, actuator_sub_);

    sync_->registerCallback(std::bind(&SyncNode::synced_callback, this, _1, _2, _3));

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&SyncNode::watchdog_callback, this));

    RCLCPP_INFO(this->get_logger(), "SyncNode inicializado com cache de dados.");
}

void SyncNode::watchdog_callback()
{
    auto now = this->now();

    // IMU
    if ((now - last_imu_time_) > timeout_threshold_) {
        // copia a mensagem real (se tiver) ou cria uma nova
        auto dummy = last_imu_msg_ ? std::make_shared<IMUState>(*last_imu_msg_) 
                                   : std::make_shared<IMUState>();
        dummy->header.stamp = now;
        sync_->add<0>(dummy); 
    }

    // pressure
    if ((now - last_pressure_time_) > timeout_threshold_) {
        auto dummy = last_pressure_msg_ ? std::make_shared<PressureState>(*last_pressure_msg_) 
                                         : std::make_shared<PressureState>();
        dummy->header.stamp = now;
        sync_->add<1>(dummy);
    }

    // actuator
    if ((now - last_actuator_time_) > timeout_threshold_) {
        auto dummy = last_actuator_msg_ ? std::make_shared<ActuatorState>(*last_actuator_msg_) 
                                         : std::make_shared<ActuatorState>();
        dummy->header.stamp = now;
        sync_->add<2>(dummy);
    }
}

void SyncNode::synced_callback(
    const IMUState::ConstSharedPtr& imu_msg,
    const PressureState::ConstSharedPtr& pressure_msg,
    const ActuatorState::ConstSharedPtr& actuator_msg)
{
    auto synced_msg = std::make_unique<SyncedSensorData>();

    synced_msg->header.stamp = this->now();
    
    synced_msg->imu_data = *imu_msg;
    synced_msg->pressure_data = *pressure_msg;
    synced_msg->actuator_data = *actuator_msg;

    publisher_->publish(std::move(synced_msg));
}

SyncNode::~SyncNode() = default;
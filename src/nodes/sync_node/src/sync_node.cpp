#include "sync_node.hpp"

SyncNode::SyncNode() : Node("sync_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    publisher_ = this->create_publisher<SyncedSensorData>("/sync/data", qos);

    imu_sub_.subscribe(this, "imu/state", qos.get_rmw_qos_profile());
    pressure_sub_.subscribe(this, "pressure/state", qos.get_rmw_qos_profile());
    actuator_sub_.subscribe(this, "actuator/state", qos.get_rmw_qos_profile());

    imu_sub_.registerCallback([this](const IMUState::ConstSharedPtr&) { 
        last_imu_time_ = this->now(); 
    });
    pressure_sub_.registerCallback([this](const PressureState::ConstSharedPtr&) { 
        last_pressure_time_ = this->now(); 
    });
    actuator_sub_.registerCallback([this](const ActuatorState::ConstSharedPtr&) { 
        last_actuator_time_ = this->now(); 
    });

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), imu_sub_, pressure_sub_, actuator_sub_);

    sync_->registerCallback(std::bind(&SyncNode::synced_callback, this, _1, _2, _3));

    // timer de watchdog (roda a 20Hz para checar timeouts)
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&SyncNode::watchdog_callback, this));

    RCLCPP_INFO(this->get_logger(), "SyncNode inicializado");
}

void SyncNode::watchdog_callback()
{
    auto now = this->now();

    // se IMU falhar injeta dummy
    if ((now - last_imu_time_) > timeout_threshold_) {
        auto dummy = std::make_shared<IMUState>();
        dummy->header.stamp = now;
        sync_->add<0>(dummy); 
    }

    // se pressure falhar injeta dummy
    if ((now - last_pressure_time_) > timeout_threshold_) {
        auto dummy = std::make_shared<PressureState>();
        dummy->header.stamp = now;
        sync_->add<1>(dummy);
    }

    // se actuator falhar injeta dummy
    if ((now - last_actuator_time_) > timeout_threshold_) {
        auto dummy = std::make_shared<ActuatorState>();
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
    
    // se forem mensagens dummy, estarão com campos zerados/padrão.
    synced_msg->imu_data = *imu_msg;
    synced_msg->pressure_data = *pressure_msg;
    synced_msg->actuator_data = *actuator_msg;

    publisher_->publish(std::move(synced_msg));
    RCLCPP_DEBUG(this->get_logger(), "Publicado pacote sincronizado (Resiliente)");
}

SyncNode::~SyncNode() = default;
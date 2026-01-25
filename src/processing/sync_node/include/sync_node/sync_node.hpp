#ifndef SYNC_NODE_HPP
#define SYNC_NODE_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "interfaces/msg/imu_state.hpp"
#include "interfaces/msg/pressure_state.hpp"
#include "interfaces/msg/state.hpp"

#include "interfaces/msg/synced_sensor_data.hpp"


using IMUState = interfaces::msg::IMUState;
using PressureState = interfaces::msg::PressureState;
using ActuatorState = interfaces::msg::State; // actuator

using SyncedSensorData = interfaces::msg::SyncedSensorData;

using SyncPolicy = message_filters::sync_policies::ApproximateTime<IMUState, PressureState, ActuatorState>;

class SyncNode : public rclcpp::Node
{
	public:
		explicit SyncNode();
		virtual ~SyncNode();
		
	private:
	
		void synced_callback(
			const IMUData::ConstSharedPtr& imu_msg, 
			const PressureData::ConstSharedPtr& pressure_msg
			const PressureData::ConstSharedPtr& actuator_msg);

		message_filters::Subscriber<IMUState> imu_sub_;
		message_filters::Subscriber<PressureState> pressure_sub_;
		message_filters::Subscriber<ActuatorState> actuator_sub_;

		rclcpp::Publisher<SyncedSensorData>::SharedPtr publisher_;
		std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
		
};

#endif // SYNC_NODE_HPP

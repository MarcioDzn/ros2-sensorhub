#ifndef SYNC_NODE_HPP
#define SYNC_NODE_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "interfaces/msg/imu_data.hpp"
#include "interfaces/msg/pressure_data.hpp"
#include "interfaces/msg/synced_sensor_data.hpp"


using IMUData = interfaces::msg::IMUData;
using PressureData = interfaces::msg::PressureData;
using SyncedSensorData = interfaces::msg::SyncedSensorData;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<IMUData, PressureData>;

class SyncNode : public rclcpp::Node
{
	public:
		explicit SyncNode();
		virtual ~SyncNode();
		
	private:
	
		void synced_callback(const IMUData::ConstSharedPtr& imu_msg, 
                             const PressureData::ConstSharedPtr& pressure_msg);
		message_filters::Subscriber<IMUData> imu_sub_;
		message_filters::Subscriber<PressureData> pressure_sub_;
		rclcpp::Publisher<SyncedSensorData>::SharedPtr publisher_;
		
		std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
		
};

#endif // SYNC_NODE_HPP

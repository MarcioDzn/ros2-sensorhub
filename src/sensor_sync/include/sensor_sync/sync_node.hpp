#ifndef SYNC_NODE_HPP
#define SYNC_NODE_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "interfaces/msg/imu_data.hpp"
#include "interfaces/msg/insole_data.hpp"
#include "interfaces/msg/synced_sensor_data.hpp"


using IMUData = interfaces::msg::IMUData;
using InsoleData = interfaces::msg::InsoleData;
using SyncedSensorData = interfaces::msg::SyncedSensorData;

class SyncNode : public rclcpp::Node
{
	public:
		explicit SyncNode();
		virtual ~SyncNode();
		
	private:
	
		void synced_callback(const IMUData::ConstSharedPtr& imu_msg, 
                             const InsoleData::ConstSharedPtr& pressure_msg);
		std::shared_ptr<message_filters::Subscriber<IMUData>> imu_sub_;
		std::shared_ptr<message_filters::Subscriber<InsoleData>> pressure_sub_;
		rclcpp::Publisher<SyncedSensorData>::SharedPtr publisher_;
		
		std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
		
};

#endif // SYNC_NODE_HPP

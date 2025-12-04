#ifndef SYNC_NODE_HPP
#define SYNC_NODE_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
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
		void add_to_buffer(std::string topic);
		void sync_data();
		void publish_data();
		
		void get_imu_data(const IMUData& msg);
		void get_pressure_data(const InsoleData& msg);
		
		rclcpp::Publisher<SyncedSensorData>::SharedPtr publisher_;
		rclcpp::Subscription<InsoleData>::SharedPtr pressure_subscriptor_;
		rclcpp::Subscription<IMUData>::SharedPtr imu_subscriptor_;
				
		std::vector<InsoleData> pressure_buffer_;
		std::vector<IMUData> imu_buffer_;
		
};

#endif // SYNC_NODE_HPP

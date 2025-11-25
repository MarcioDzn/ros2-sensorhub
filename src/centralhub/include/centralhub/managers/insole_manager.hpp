#ifndef INSOLE_MANAGER_HPP
#define INSOLE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/insole_data.hpp"
#include "device_comm/device_comm.hpp"

using InsoleData = interfaces::msg::InsoleData;

class InsoleManager
{
    public:
        InsoleManager(rclcpp::Node* node);

        int initComm(const char *device, int baudrate);
        int receivePacket(char *buffer, int length);
        void createPublisher();
        void publishAll();
        
    private:
        rclcpp::Node* node_;
        DeviceComm device_;
        
        rclcpp::Publisher<InsoleData>::SharedPtr publisher_;
};

#endif

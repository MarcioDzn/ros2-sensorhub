#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>

#include "interfaces/srv/set_motor_config.hpp"
#include "device_comm/device_comm.hpp"

// TODO: definir outros valores
// e ver o que eles representam
// ex: NORMAL é int ou é uma string?
enum SpinDirection {
    NORMAL = 1, 
    REVERSE = -1
};

using SetMotorConfig = interfaces::srv::SetMotorConfig;

class MotorManager
{
    public:
        MotorManager(
            rclcpp::Node* node, 
            int motor_id
        );

        void loadParameters();
        void createServer();

        // controle
        bool setTorqueCurr(int torque);
        bool setSpeed(double speed);
        bool setAngle(double angle);

        int getTorqueCurr() { return torque_curr_; };
        double getSpeed() { return speed_; };
        double getAngle() { return angle_; };

        // configs basicas
        bool setRS485Baud(int baud);
        bool setSpinDir(SpinDirection dir);

        int getRS485Baud() { return rs485_baud_; };
        SpinDirection getSpinDir() { return spin_dir_; };

        // limites
        bool setMaxAngle(double angle); // graus
        bool setMaxSpeed(double speed); // dps
        bool setMaxAccel(int accel); // dps/s
        bool setMaxTorqueCurr(int torque);
        bool setTorqueCurrRamp(int ramp);

        double getMaxAngle() { return max_angle_; };
        double getMaxSpeed() { return max_speed_; };
        int  getMaxAccel() { return max_accel_; };
        int getMaxTorqueCurr() { return max_torque_curr_; };
        int getTorqueCurrRamp() { return torque_curr_ramp_; };

        int initComm(const char* device, int baudrate);
        int sendReceivePacket(
            const std::vector<uint8_t>& packet, 
            std::vector<uint8_t>& response,
            int timeout_ms
        );
        int sendPacket(const std::vector<uint8_t>& packet);
        int receivePacket(std::vector<uint8_t>& buffer, int timeout_ms);
        bool applySettings();
        bool start();
        bool stop();

    private:
        rclcpp::Node* node_;
        DeviceComm device_;
        int motor_id_;

        // controle
        int torque_curr_;
        double speed_;        
        double angle_;        

        // configs básicas
        int rs485_baud_;
        SpinDirection spin_dir_;

        // limites
        double max_angle_;    
        double max_speed_;    
        int max_accel_;       
        int max_torque_curr_;
        int torque_curr_ramp_;

        // constantes
        static constexpr unsigned char PACKAGE_HEADER = 0x3E;
        static constexpr unsigned char SET_TORQUE_COMMAND = 0xBB;

};

#endif

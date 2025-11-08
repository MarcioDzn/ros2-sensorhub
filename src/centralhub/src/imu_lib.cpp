#include "imu_lib.hpp"

#include <stdio.h>      
#include <stdlib.h>   
#include <stdexcept>      
#include <iostream>       
#include <thread>         
#include <chrono>         
#include "rclcpp/rclcpp.hpp"   

#include <wiringPi.h>


#define SEL_A 2
#define SEL_B 0

BNO055IMU::BNO055IMU(int32_t imu_id, int sensor_id, uint8_t address) : 
    bno_(BNO055(imu_id, address)), sensor_id_(sensor_id) {
    setup_states();
} 

void BNO055IMU::setup() {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    
    RCLCPP_INFO(rclcpp::get_logger("BNO055IMU"),
                "Inicializando BNO%d...", sensor_id_);
    if(!bno_.begin())
    {
        std::string msg = "[ERRO] BNO" + std::to_string(sensor_id_) + ": não inicializado";
        RCLCPP_ERROR(rclcpp::get_logger("BNO055IMU"), "%s", msg.c_str());
        throw std::runtime_error(msg);
    }
    RCLCPP_INFO(rclcpp::get_logger("BNO055IMU"),
                "BNO%d inicializado com sucesso", sensor_id_);
}

void BNO055IMU::get_data(std::vector<double>& out_data) {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    out_data.resize(3);
    bno_.readEuler(out_data.data());
    delay(1);
    
    // calibrando
    out_data[0] = out_data[0] - calibration_ref_[0]; // yaw
    out_data[1] = out_data[1] - calibration_ref_[1]; // pitch
    out_data[2] = out_data[2] - calibration_ref_[2]; // roll
}

void BNO055IMU::calibrate() {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    bno_.readEuler(calibration_ref_);
    delay(1);
}

void BNO055IMU::setup_wiringpi() {
    if (wiringPiSetup() < 0)
        exit(1);
        
	pinMode(SEL_A, OUTPUT);
	pinMode(SEL_B, OUTPUT);
	
    digitalWrite(SEL_A, LOW);
	digitalWrite(SEL_B, LOW);

	delay(1);
}

void BNO055IMU::setup_states() {
    switch (sensor_id_) {
        case 1:
        case 2:
            selA_state_ = LOW;
            selB_state_ = LOW;
            break;
            
        case 3:
        case 4:
            selA_state_ = LOW;
            selB_state_ = HIGH;
            break;
            
        default:
            break;
    }
}
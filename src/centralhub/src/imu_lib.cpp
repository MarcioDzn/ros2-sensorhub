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

BNO055IMU::BNO055IMU(int32_t bnoID, int sensorID, uint8_t address) : 
    bno_(BNO055(bnoID, address)), sensorID_(sensorID) {
    setupStates();
} 

void BNO055IMU::setup() {
    digitalWrite(SEL_A, selAState_);
    digitalWrite(SEL_B, selBState_);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    
    RCLCPP_INFO(rclcpp::get_logger("BNO055IMU"),
                "Inicializando BNO%d...", sensorID_);
    if(!bno_.begin())
    {
        std::string msg = "[ERRO] BNO" + std::to_string(sensorID_) + ": não inicializado";
        RCLCPP_ERROR(rclcpp::get_logger("BNO055IMU"), "%s", msg.c_str());
        throw std::runtime_error(msg);
    }
    RCLCPP_INFO(rclcpp::get_logger("BNO055IMU"),
                "BNO%d inicializado com sucesso", sensorID_);
}

void BNO055IMU::getData(std::vector<double>& outData) {
    digitalWrite(SEL_A, selAState_);
    digitalWrite(SEL_B, selBState_);
    
    delay(1);

    outData.resize(3);
    bno_.readEuler(outData.data());
    delay(1);
    
    // calibrando
    outData[0] = outData[0] - callibrationRef_[0]; // yaw
    outData[1] = outData[1] - callibrationRef_[1]; // pitch
    outData[2] = outData[2] - callibrationRef_[2]; // roll
}

void BNO055IMU::callibrate() {
    digitalWrite(SEL_A, selAState_);
    digitalWrite(SEL_B, selBState_);
    
    delay(1);

    bno_.readEuler(callibrationRef_);
    delay(1);
}

void BNO055IMU::setupWiringPi() {
    if (wiringPiSetup() < 0)
        exit(1);
        
	pinMode(SEL_A, OUTPUT);
	pinMode(SEL_B, OUTPUT);
	
    digitalWrite(SEL_A, LOW);
	digitalWrite(SEL_B, LOW);

	delay(1);
}

void BNO055IMU::setupStates() {
    switch (sensorID_) {
        case 1:
        case 2:
            selAState_ = LOW;
            selBState_ = LOW;
            break;
            
        case 3:
        case 4:
            selAState_ = LOW;
            selBState_ = HIGH;
            break;
            
        default:
            break;
    }
}
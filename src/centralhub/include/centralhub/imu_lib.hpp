#ifndef IMU_LIB_HPP_
#define IMU_LIB_HPP_

#include <vector>
#include <cstdint>

#include "bno055.h"

class BNO055IMU {
    public:
        BNO055IMU(int32_t bnoID, int sensorID, uint8_t address);
        void setup();
        void getData(std::vector<double>& outData);
        void callibrate();
        static void setupWiringPi();
      
    private:
      BNO055 bno_;
      int selAState_;
      int selBState_;
      int sensorID_;
      double callibrationRef_[3] = {0, 0, 0};
      
      void setupStates();
};

#endif // IMU_LIB_HPP_
#ifndef IMU_LIB_HPP_
#define IMU_LIB_HPP_

#include <vector>
#include <cstdint>

#include "bno055.h"

class BNO055IMU {
    public:
        BNO055IMU(int32_t bno_id, int sensor_id, uint8_t address);
        void setup();
        void get_data(std::vector<double>& out_data);
        void calibrate();
        static void setup_wiringpi();
      
    private:
        BNO055 bno_;
        int selA_state_;
        int selB_state_;
        int sensor_id_;
        double calibration_ref_[3] = {0, 0, 0};
        
        void setup_states();
};

#endif // IMU_LIB_HPP_
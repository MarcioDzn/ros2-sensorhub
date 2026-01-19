#ifndef IMU_LIB_HPP_
#define IMU_LIB_HPP_

#include <vector>
#include <cstdint>

#include "driver/common/imu.hpp"

class BNO055IMU {
    public:
        BNO055IMU(int32_t bno_id, int sensor_id, uint8_t address);
        void setup();
        void get_euler_data(std::vector<float>& out_data);
        void get_quaternions_euler_data(std::vector<float>& out_data);
        void calibrate_euler();
        void calibrate_quaternions_euler();
        static void setup_wiringpi();
      
    private:
        IMU bno_;
        int selA_state_;
        int selB_state_;
        int sensor_id_;
        float calibration_ref_[3] = {0, 0, 0};
        
        void setup_states();
};

#endif // IMU_LIB_HPP_
#include "models/imu.hpp"

IMU::IMU(uint8_t id)
    : id_(id),
      quaternion_{0.0f, 0.0f, 0.0f, 1.0f}, 
      euler_angles_{0.0f, 0.0f, 0.0f}
{}
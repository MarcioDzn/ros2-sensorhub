#include "models/imu.hpp"

IMU::IMU(uint8_t id)
    : id_(id),
      quaternion_{0.0f, 0.0f, 0.0f, 1.0f}, 
      euler_angles_{0.0f, 0.0f, 0.0f}
{}

void IMU::set_euler_angles(float roll, float pitch, float yaw) 
{
  euler_angles_.roll = roll;
  euler_angles_.pitch = pitch;
  euler_angles_.yaw = yaw;
}

void IMU::set_quaternion(float x, float y, float z, float w) 
{
  quaternion_.x = x;
  quaternion_.y = y;
  quaternion_.z = z;
  quaternion_.w = w;
}
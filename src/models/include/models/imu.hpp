#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>
#include <vector>
#include <memory>

struct Quaternion {
    float x;
    float y;
    float z;
    float w;
};

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

class IMU
{
    public:
        explicit IMU(uint8_t id);
        virtual ~IMU() = default;

        // normalizar no cpp
        void set_quaternion(float x, float y, float z, float w);
        void set_euler_angles(float roll, float pitch, float yaw);

        uint8_t get_id() const;
        Quaternion get_quaternion() const;
        EulerAngles get_euler_angles() const;

    private:
        uint8_t id_;
        Quaternion quaternion_;
        EulerAngles euler_angles_;
};


#endif // IMU_HPP
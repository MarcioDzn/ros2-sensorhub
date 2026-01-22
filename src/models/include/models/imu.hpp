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

        // TODO: limitar
        void set_quaternion(float x, float y, float z, float w);
        void set_euler_angles(float roll, float pitch, float yaw);

        uint8_t get_id() const { return id_; };
        Quaternion get_quaternion() const { return quaternion_; };
        EulerAngles get_euler_angles() const { return euler_angles_; };

    private:
        uint8_t id_;
        Quaternion quaternion_;
        EulerAngles euler_angles_;
};


#endif // IMU_HPP
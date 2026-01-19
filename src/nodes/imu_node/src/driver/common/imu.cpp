#include "driver/common/imu.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cmath>

#include <wiringPi.h>
#include <wiringPiI2C.h>

IMU::IMU(int32_t id, uint8_t address)
: id_(id), address_(address) {}

int IMU::setup_wiringpi()
{
    if (wiringPiSetup() == -1) return -1;

    dev_ = wiringPiI2CSetup(address_);
    if (dev_ < 0) {
        return -1;
    }
    return 0;
}

int IMU::init_bno055()
{
    // modo configuracao
    if (wiringPiI2CWriteReg8(dev_, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG) < 0)
        return -1;
    usleep(30 * 1000); // 30ms

    // reset
    // 0x20 -> comando reset
    if (wiringPiI2CWriteReg8(dev_, BNO055_SYS_TRIGGER_ADDR, 0x20) < 0) 
        return -1;

    // espera até o id do chip ser válido (reset termina)
    while (wiringPiI2CReadReg8(dev_, BNO055_CHIP_ID_ADDR) != BNO055_ID)
        usleep(10 * 1000); // 10ms
    usleep(50 * 1000); // 50ms

    // power mode normal
    if (wiringPiI2CWriteReg8(dev_, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL) < 0)
        return -1;

    // página 0
    if (wiringPiI2CWriteReg8(dev_, BNO055_PAGE_ID_ADDR, 0) < 0)
        return -1;

    // reset terminou
    if (wiringPiI2CWriteReg8(dev_, BNO055_SYS_TRIGGER_ADDR, 0x00) < 0)
        return -1;
    usleep(10 * 1000);

    // modo de operação NDOF
    if (wiringPiI2CWriteReg8(dev_, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF) < 0)
        return -1;
    usleep(50 * 1000);

    return 0;
}

void IMU::read_quaternions(float quaternion[4])
{
    if (dev_ < 0) return;

    int wl = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_W_LSB_ADDR);
    int wm = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_W_MSB_ADDR);
    int xl = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_X_LSB_ADDR);
    int xm = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_X_MSB_ADDR);
    int yl = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_Y_LSB_ADDR);
    int ym = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_Y_MSB_ADDR);
    int zl = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_Z_LSB_ADDR);
    int zm = (int) wiringPiI2CReadReg8(dev_, BNO055_QUATERNION_DATA_Z_MSB_ADDR);

    // imu manda de 0 a 65535 (unsigned int)
    // mas os valores (sem normalização) devem
    // ir de -32768 a 32768, por isso subtrai
    // a divisão é pra normalizar (-1 a 1)
    quaternion[0] = ((float)(wl | (wm << 8)) - 32768.0f) / 32768.0f;
    quaternion[1] = ((float)(xl | (xm << 8)) - 32768.0f) / 32768.0f;
    quaternion[2] = ((float)(yl | (ym << 8)) - 32768.0f) / 32768.0f;
    quaternion[3] = ((float)(zl | (zm << 8)) - 32768.0f) / 32768.0f;
}

// https://madecalculators.com/quaternion-to-euler-calculator/?utm_source=chatgpt.com
void IMU::read_quaternions_euler(float euler[3])
{
    if (dev_ < 0) return;

    float quaternion[4];
    read_quaternions(quaternion);
    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];

    float roll = atan2(2.0 * (w*x + z*y), 1.0 - 2.0 * (x*x + y*y));

    float sinp = 2.0 * (w*y - z*x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asin(sinp);

    float yaw = atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));

    euler[0] = roll;
    euler[1] = pitch;
    euler[2] = yaw;
}

void IMU::read_euler(float euler[3])
{
    int hl = (int) wiringPiI2CReadReg8(dev_, BNO055_EULER_H_LSB_ADDR);
    int hm = (int) wiringPiI2CReadReg8(dev_, BNO055_EULER_H_MSB_ADDR);
    int rl = (int) wiringPiI2CReadReg8(dev_, BNO055_EULER_R_LSB_ADDR);
    int rm = (int) wiringPiI2CReadReg8(dev_, BNO055_EULER_R_MSB_ADDR);
    int pl = (int) wiringPiI2CReadReg8(dev_, BNO055_EULER_P_LSB_ADDR);
    int pm = (int) wiringPiI2CReadReg8(dev_, BNO055_EULER_P_MSB_ADDR);

    int h = (hm << 8) | hl;
    int r = (rm << 8) | rl;
    int p = (pm << 8) | pl;

    // divide por 16 pra converter de LSB pra graus
    // 1 grau = 16 (LSB)
    euler[0] = r / 16.0f; // roll
    euler[1] = p / 16.0f; // pitch
    euler[2] = h / 16.0f; // yaw
}


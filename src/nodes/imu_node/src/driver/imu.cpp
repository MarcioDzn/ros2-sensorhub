#include "driver/imu.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cmath>

#include <wiringPi.h>
#include <wiringPiI2C.h>

IMU::IMU(int32_t id, uint8_t address, int shared_fd)
:  dev_(shared_fd), id_(id), address_(address) {}


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
    int retry = 0;
    while (wiringPiI2CReadReg8(dev_, BNO055_CHIP_ID_ADDR) != BNO055_ID)
    {
        usleep(10 * 1000); // 10ms
        if (++retry > 50) return -1; // Sai após 500ms se o sensor não responder
    }

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

    uint8_t reg = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    uint8_t buf[8];

    // estrutura de mensagens para o I2C
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;

    // informa qual reg será lido
    msgs[0].addr  = address_; 
    msgs[0].flags = 0;        // 0 -> escrita
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    // lê 8 bytes em sequência
    msgs[1].addr  = address_;
    msgs[1].flags = I2C_M_RD; // read flag
    msgs[1].len   = 8;
    msgs[1].buf   = buf;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(dev_, I2C_RDWR, &msgset) < 0) {
        return; 
    }

    quaternion[0] = (float)((int16_t)(buf[0] | (buf[1] << 8))) / 16384.0f;
    quaternion[1] = (float)((int16_t)(buf[2] | (buf[3] << 8))) / 16384.0f;
    quaternion[2] = (float)((int16_t)(buf[4] | (buf[5] << 8))) / 16384.0f;
    quaternion[3] = (float)((int16_t)(buf[6] | (buf[7] << 8))) / 16384.0f;

    // normalização
    float n = sqrt(quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] + 
                   quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]);
    if (n > 0.0001f) {
        quaternion[0] /= n; quaternion[1] /= n; quaternion[2] /= n; quaternion[3] /= n;
    }
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


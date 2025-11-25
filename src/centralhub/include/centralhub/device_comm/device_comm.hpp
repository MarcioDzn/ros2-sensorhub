#ifndef DEVICE_COMM_HPP 
#define DEVICE_COMM_HPP

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <string>

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>


// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
class DeviceComm
{
    public:
        DeviceComm();
        virtual ~DeviceComm();

        int init(const char* device, int baudrate);
        int setBaudRate(int speed);
        int writeData(const std::vector<uint8_t>& data);
        int readData(uint8_t* buffer, int length);
        int readStringData(char *buffer, int length);

    private:
        int fd_;
};

#endif // DEVICE_COMM_HPP

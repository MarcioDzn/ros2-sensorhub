#ifndef DEVICE_COMM_HPP 
#define DEVICE_COMM_HPP

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>


// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
class DeviceComm
{
    public:
        DeviceComm();
        virtual ~DeviceComm();

        int init(const char* device);
        int writeData(const std::vector<uint8_t>& data);
        int readData(
            std::vector<unsigned char>& read_buf,
            size_t bytes_to_read
        );

    private:
        int fd_;
        struct termios tty_;
};

#endif // DEVICE_COMM_HPP
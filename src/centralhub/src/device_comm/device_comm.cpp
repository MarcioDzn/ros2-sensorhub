#include "device_comm/device_comm.hpp"

#include <termios.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>

DeviceComm::DeviceComm() : fd_(-1) {}

int DeviceComm::init(const char* device, int baudrate)
{
    struct termios config;

    fd_ = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cout << "Error " << errno 
            << " from open: " << std::strerror(errno) 
            << std::endl;
        return -1;
    }

    // configurações
    if (tcgetattr(fd_, &config) != 0) {
        std::cerr << "Erro em tcgetattr(): " << strerror(errno) << std::endl;
        return -1;
    }

    config.c_cflag = CS8 | CLOCAL | CREAD;
    config.c_iflag = IGNPAR;
    config.c_oflag      = 0;
    config.c_lflag      = 0;
    config.c_cc[VTIME]  = 0;
    config.c_cc[VMIN]   = 0;

    // limpa o buffer
    tcflush(fd_, TCIFLUSH);

    // aplica as configs
    if (tcsetattr(fd_, TCSANOW, &config) != 0) {
        std::cerr << "Erro ao aplicar configuração: " 
            << strerror(errno) 
            << std::endl;
        return -1;
    }

    // aplica o baudrate
    if (setBaudRate(baudrate) != 0) {
        std::cerr << "Erro ao aplicar baudrate custom!" << std::endl;
        return -1;
    }
    
    return 0;
}

int DeviceComm::setBaudRate(int speed)
{
    struct termios2 options;
    if (ioctl(fd_, TCGETS2, &options) != -1)
    {
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = speed;
        options.c_ospeed = speed;

        if (ioctl(fd_, TCSETS2, &options) != -1)
            return 0;
        return -1; 
    }
    return -1; 
}

int DeviceComm::writeData(const std::vector<uint8_t>& data)
{
    // fd deve ser válido
    if (fd_ < 0) {
        std::cerr << "Dispositivo não inicializado. Chame init() primeiro.\n";
        return -1;
    }

    size_t total_written = 0;
    while (total_written < data.size()) {
        ssize_t n = write(fd_, data.data() + total_written, data.size() - total_written);
        if (n < 0) {
            std::cerr << "Erro ao escrever: " << std::strerror(errno) << std::endl;
            return -1;
        }
        total_written += n;
    }

    return static_cast<int>(total_written);
}

int DeviceComm::readData(
    std::vector<unsigned char>& read_buf,
    size_t bytes_to_read
)
{
    // fd deve ser válido
    if (fd_ < 0) {
        std::cerr << "Dispositivo não inicializado. Chame init() primeiro.\n";
        return -1;
    }

    read_buf.clear();
    read_buf.resize(bytes_to_read);

    size_t total_read = 0;

    auto start_time = std::chrono::steady_clock::now();
    int timeout_ms = 20;

    // lê até receber todos os dados
    while (total_read < bytes_to_read) {
        ssize_t n = read(fd_, read_buf.data() + total_read, bytes_to_read - total_read);
        if (n > 0) {
            total_read += n;
        } 
        else if (n < 0) {
            // se o erro for de "sem dados disponíveis" (EAGAIN)
            // então apenas continua o loop
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Erro fatal de leitura: " << strerror(errno) << std::endl;
                return -1;
            }
        }

        // verifica se deu timeout
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    
        if (elapsed >= timeout_ms) {
            break; 
        }

        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    read_buf.resize(total_read);
    return static_cast<int>(total_read);
}


DeviceComm::~DeviceComm()
{
    if (fd_ >= 0) close(fd_);
}
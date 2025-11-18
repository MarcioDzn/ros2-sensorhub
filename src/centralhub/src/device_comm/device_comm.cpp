#include "device_comm/device_comm.hpp"

DeviceComm::DeviceComm() : fd_(-1) {}

int DeviceComm::init(const char* device)
{
    fd_ = open(device, O_RDWR);
    if (fd_ < 0) {
        std::cout << "Error " << errno 
            << " from open: " << std::strerror(errno) 
            << std::endl;
        return -1;
    }

    // config
    if (tcgetattr(fd_, &tty_) != 0)
    {
        std::cout << "Error " << errno 
            << " from tcgetattr: " << std::strerror(errno) 
            << std::endl;
        return -1;
    }

    return 0;
}

int DeviceComm::writeData(const std::vector<unsigned char>& data)
{
    // fd deve ser válido
    if (fd_ < 0) {
        std::cerr << "Dispositivo não inicializado. Chame init() primeiro.\n";
        return -1;
    }

    ssize_t n = write(fd_, data.data(), data.size());
    if (n < 0) {
        std::cerr << "Erro ao escrever no dispositivo: " 
            << std::strerror(errno) 
            << std::endl;
        return -1;
    }

    // TODO: loop até escrever todos os dados
    if (static_cast<size_t>(n) != data.size()) {
        std::cerr << "Aviso: apenas " << n 
            << " de " << data.size() 
            << " bytes escritos." 
            << std::endl;
    }

    return static_cast<int>(n);
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

    read_buf.resize(bytes_to_read);
    ssize_t n = read(fd_, read_buf.data(), read_buf.size());
    if (n < 0) {
        std::cerr << "Erro ao ler do dispositivo: " 
            << std::strerror(errno) 
            << std::endl;
        return -1;
    }

    read_buf.resize(n);
    return static_cast<int>(n);
}

DeviceComm::~DeviceComm()
{
    if (fd_ >= 0) close(fd_);
}
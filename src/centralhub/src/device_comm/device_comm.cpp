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

    // configurações
    cfsetospeed(&tty_, B115200);
    cfsetispeed(&tty_, B115200);
    tty_.c_cflag = (tty_.c_cflag & ~CSIZE) | CS8;  // 8 bits
    tty_.c_cflag &= ~PARENB;                       // sem paridade
    tty_.c_cflag &= ~CSTOPB;                       // 1 stop bit
    tty_.c_cflag &= ~CRTSCTS;                      // sem flow control
    tty_.c_lflag &= ~(ICANON | ECHO | ISIG);       // raw mode
    tty_.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL); // desativa flow control e conversões
    tty_.c_oflag &= ~OPOST;                          // saída bruta
    tty_.c_cc[VMIN]  = 0;                            // mínimo de bytes para read()
    tty_.c_cc[VTIME] = 10;                           // timeout 1s (10 decisegundos)

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
    size_t total_read = 0;

    // TODO: verificar se existe algum delimitador
    // no RX do motor

    // lê até receber todos os dados
    while (total_read < bytes_to_read) {
        ssize_t n = read(fd_, read_buf.data() + total_read, bytes_to_read - total_read);
        if (n < 0) {
            std::cerr << "Erro ao ler do dispositivo: " << std::strerror(errno) << std::endl;
            return -1;
        } else if (n == 0) {
            break;
        }
        total_read += n;
    }

    read_buf.resize(total_read);
    return static_cast<int>(total_read);
}

DeviceComm::~DeviceComm()
{
    if (fd_ >= 0) close(fd_);
}
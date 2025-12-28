#include <iostream>
#include "actuator_comm/controller/dynamixel_controller.hpp"

int main()
{
    DynamixelController controller;

    const std::string porta = "/dev/pts/6";
    const int baudrate = 115200;

    std::cout << "Tentando abrir a porta: " << porta << std::endl;

    if (controller.init(porta, baudrate) < 0) {
        std::cerr << "ERRO ao inicializar o controller!" << std::endl;
        return -1;
    }

    std::cout << "Controller inicializado com sucesso!" << std::endl;

    uint8_t id_motor = 1;

    std::cout << "Habilitando torque..." << std::endl;
    if (controller.setTorque(id_motor, 1) < 0) {
        std::cerr << "Falha ao habilitar torque!" << std::endl;
        return -1;
    }

    std::cout << "Enviando goal position..." << std::endl;
    if (controller.setGoalPosition(id_motor, 512) < 0) {
        std::cerr << "Falha ao enviar goal position!" << std::endl;
        return -1;
    }

    uint16_t pos = 0;
    if (controller.getCurrentPosition(id_motor, pos) < 0) {
        std::cerr << "Falha ao ler posição!" << std::endl;
        return -1;
    }

    std::cout << "Posição atual lida: " << pos << std::endl;

    return 0;
}

#include "imu_comm/bno055imu.hpp"

#include <stdio.h>      
#include <stdlib.h>   
#include <stdexcept>      
#include <iostream>       
#include <thread>         
#include <chrono>         
#include <wiringPi.h>


#define SEL_A 2
#define SEL_B 0

/**
 * @brief Construtor da classe BNO055IMU.
 * 
 * Inicializa o objeto BNO055 e define o estado dos pinos de seleção
 * conforme o identificador do sensor.
 * 
 * @param imu_id Identificador do IMU no barramento.
 * @param sensor_id Identificador lógico do sensor
 * @param address Endereço I²C do sensor BNO055.
 */
BNO055IMU::BNO055IMU(int32_t imu_id, int sensor_id, uint8_t address) : 
    bno_(BNO055(imu_id, address)), sensor_id_(sensor_id) {
    setup_states();
} 

/**
 * @brief Inicializa o sensor BNO055 selecionado.
 * 
 * Configura os pinos de seleção do multiplexador, aguarda estabilização,
 * e tenta iniciar a comunicação I²C com o BNO055. Caso falhe, uma exceção
 * `std::runtime_error` é lançada.
 * 
 * @throws std::runtime_error Se o sensor não puder ser inicializado.
 */
void BNO055IMU::setup() {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(!bno_.begin())
    {
        std::string msg = "[ERRO] BNO" + std::to_string(sensor_id_) + ": não inicializado";
        throw std::runtime_error(msg);
    }
}

/**
 * @brief Lê os dados de Euler (yaw, pitch, roll) do sensor.
 * 
 * Atualiza o vetor de saída `out_data` com os valores de orientação do BNO055,
 * já compensados pela calibração atual.
 * 
 * @param[out] out_data Vetor com 3 elementos: {yaw, pitch, roll}.
 */
void BNO055IMU::get_data(std::vector<double>& out_data) {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    out_data.resize(3);
    bno_.readEuler(out_data.data());
    delay(1);
    
    // calibrando
    out_data[0] = out_data[0] - calibration_ref_[0]; // yaw
    out_data[1] = out_data[1] - calibration_ref_[1]; // pitch
    out_data[2] = out_data[2] - calibration_ref_[2]; // roll
}

/**
 * @brief Realiza a calibração inicial do sensor.
 * 
 * Lê os valores atuais de Euler e os armazena como referência de calibração,
 * para que leituras futuras sejam compensadas.
 */
void BNO055IMU::calibrate() {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    bno_.readEuler(calibration_ref_);
    delay(1);
}

/**
 * @brief Inicializa o sistema de GPIO da Raspberry Pi via WiringPi.
 * 
 * Configura os pinos de seleção SEL_A e SEL_B como saída
 * e define ambos inicialmente como LOW.
 * 
 * @note A função encerra o programa com `exit(1)` caso o `wiringPiSetup()` falhe.
 */
void BNO055IMU::setup_wiringpi() {
    if (wiringPiSetup() < 0) { exit(1); }
        
	pinMode(SEL_A, OUTPUT);
	pinMode(SEL_B, OUTPUT);
	
    digitalWrite(SEL_A, LOW);
	digitalWrite(SEL_B, LOW);

	delay(1);
}

/**
 * @brief Define os estados dos pinos SEL_A e SEL_B conforme o sensor selecionado.
 * 
 * Os pinos de seleção são usados para alternar entre sensores no multiplexador.
 * 
 * Outros IDs não são configurados.
 * 
 */
void BNO055IMU::setup_states() {
    switch (sensor_id_) {
        case 1:
        case 2:
            selA_state_ = LOW;
            selB_state_ = LOW;
            break;
            
        case 3:
        case 4:
            selA_state_ = LOW;
            selB_state_ = HIGH;
            break;
            
        default:
            break;
    }
}

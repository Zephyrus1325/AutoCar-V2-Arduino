#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "Arduino.h"
#include "sensors.h"

// TODO: Implementar classe responsável por toda navegação automatizada do veículo
class Navigation{
    private:
    IMUReading imuData; // Leituras do IMU

    public:
    int16_t posX = 0; // Posição X do veiculo
    int16_t posY = 0; // Posição Y do veiculo
    int16_t posZ = 0; // Posição Z do veiculo
    int16_t homeX = 0; // Posição X de home do veiculo
    int16_t homeY = 0; // Posição Y de home do veiculo
    int16_t homeZ = 0; // Posição Z de home do veiculo
    int16_t objectiveX = 0; // Posição X do objetivo do veiculo
    int16_t objectiveY = 0; // Posição Y do objetivo do veiculo
    int16_t objectiveZ = 0; // Posição Z do objetivo do veiculo

    

};

#endif
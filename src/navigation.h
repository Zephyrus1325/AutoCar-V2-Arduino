#pragma once

#include "Arduino.h"
#include "sensors.h"

/*--------------------------------------------------------------------------------------+
|   Classe Responsável pela navegação - Marco Aurélio (05/10/2024)                      |
|                                                                                       |
|   Classe responsável por ler dados de todos os sensores e definir dados como:         |
|   Posição em X,Y e Z, rotação em todos os eixos, alem de comandar os motores e ler    |
|   os comandos dados pelo algoritmo de navegação do ESP                                |
+--------------------------------------------------------------------------------------*/


class Navigation{
    private:
    /*
        Fator de escala do mapeamento (DEVE SER O MESMO QUE O DO ESP!!!)
        1 indica que 1 unidade de mapeamento é igual a 1 cm
        10 indica que 1 unidade de mapeamente é igual a 10 cm
        Isso afeta precisão e distância máxima alcançavel pelo robô
        !!!(Não usar ainda, transformação de unidade de mapeamento para cm ainda não implementado)!!!
    */
    const unsigned int scaling_factor = 1; // 
    IMUReading imuData; // Leituras do IMU
    int16_t posX = 0; // Posição X do veiculo
    int16_t posY = 0; // Posição Y do veiculo
    int16_t posZ = 0; // Posição Z do veiculo
    int16_t homeX = 0; // Posição X de home do veiculo
    int16_t homeY = 0; // Posição Y de home do veiculo
    int16_t homeZ = 0; // Posição Z de home do veiculo
    int16_t objectiveX = 0; // Posição X do objetivo do veiculo
    int16_t objectiveY = 0; // Posição Y do objetivo do veiculo
    int16_t objectiveZ = 0; // Posição Z do objetivo do veiculo
    float pitch, roll, heading; // Inclinações e direção do veiculo
    

};
#pragma once

#include "Arduino.h"
#include "sensors.h"
#include "motor.h"
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
    const float wheelDistance = 9.f; // Distancia entre o centro e as rodas (em cm)
    float angularSpeed = 0; // Velocidade angular do veiculo (variação de heading)
    float forwardSpeed = 0; // Velocidade para frente do veiculo
    timer updateTimer{0,50,true,true,true}; // Timer de atualização
    unsigned long lastTime = 0; // Momento da ultima atualização
    float posX = 0; // Posição X do veiculo
    float posY = 0; // Posição Y do veiculo
    float posZ = 0; // Posição Z do veiculo
    int16_t homeX = 0; // Posição X de home do veiculo
    int16_t homeY = 0; // Posição Y de home do veiculo
    int16_t homeZ = 0; // Posição Z de home do veiculo
    int16_t objectiveX = 0; // Posição X do objetivo do veiculo
    int16_t objectiveY = 0; // Posição Y do objetivo do veiculo
    int16_t objectiveZ = 0; // Posição Z do objetivo do veiculo
    float pitch = 0;    // Inclinações e direção do veiculo
    float roll = 0;     
    float heading = 0; 
    
    public:

    void begin(){}

    void update(IMUReading imu, float leftSpeed, float rightSpeed){
        if(updateTimer.CheckTime()){
            float deltaTime = (float)(millis() - lastTime)/1000.0f;
            //angularSpeed = (rightSpeed - leftSpeed)/wheelDistance;
            forwardSpeed = (rightSpeed + leftSpeed)/2.0f;
            angularSpeed = imu.gyroZ;
            heading += angularSpeed * deltaTime;
            heading = heading >= 360 ? heading - 360 : heading;
            heading = heading < 0 ? heading + 360 : heading;
            posX += cos(radians(heading)) * forwardSpeed * deltaTime;
            posY += sin(radians(heading)) * forwardSpeed * deltaTime;
            Serial.print(angularSpeed);
            Serial.print(" | ");
            Serial.print(deltaTime);
            Serial.print(" | ");
            Serial.println(heading);
            lastTime = millis();
        }
    }

    int16_t getPosX(){
        return posX;
    }

    int16_t getPosY(){
        return posY;
    }

    int16_t getPosZ(){
        return posZ;
    }

    float getHeading(){
        return heading;
    }

    void reset(){
        posX = 0;
        posY = 0;
        posZ = 0;
        heading = 0;
    }
};
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

//#define DEBUG

struct waypoint{
    int16_t x;
    int16_t y;
};



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
    float objectiveHeading = 0; // Heading de objetivo do veiculo
    float pitch = 0;    // Inclinações e direção do veiculo
    float roll = 0;     
    float heading = 0; 
    const float waypointThreshold = 20.f; // Região na qual um waypoint pode ser dito como atravessado
    unsigned int actualWaypoint = 0; // Contador do waypoint atual
    float forward = 0;
    int mode = 0; // Modo do Sistema de Navegação | 0 - Automatico  | 1 - Manual
    waypoint waypoints[7];
    PID headingControl;

    private:

    int getDelta(int head, int setPoint){
        int diff = ( setPoint - head + 180 ) % 360 - 180;
        diff = diff < -180 ? diff + 360 : diff;
        return -diff;
    }

    public:

    void begin(IMUReading imu){
        headingControl.setKp(0.8f);
        headingControl.setKi(0.01f);
        headingControl.setKd(0.0f);
        headingControl.setManualError(true);
        headingControl.update();
        reset(imu);
        waypoints[0] = waypoint{0, 150};
        waypoints[1] = waypoint{150, 150};
        waypoints[2] = waypoint{150, 0};
        waypoints[3] = waypoint{0, 0};
    }

    void update(IMUReading imu, float leftSpeed, float rightSpeed){
        if(updateTimer.CheckTime()){
            float deltaTime = (float)(millis() - lastTime)/1000.0f;
            //angularSpeed = (rightSpeed - leftSpeed)/wheelDistance;
            forwardSpeed = (rightSpeed + leftSpeed)/2.0f;
            angularSpeed = imu.gyroZ;
            heading += angularSpeed * deltaTime;
            heading += getDelta(imu.magHeading, heading) * 0.005f;
            heading = heading >= 359 ? heading - 359 : heading;
            heading = heading < 0 ? heading + 359 : heading;
            posX += sin(radians(heading)) * forwardSpeed * deltaTime;
            posY += cos(radians(heading)) * forwardSpeed * deltaTime;

            // Checa se a posição atual é proxima o bastante do waypoint
            if(distance(waypoints[actualWaypoint].x, waypoints[actualWaypoint].y) < waypointThreshold){
                actualWaypoint++;
                if(actualWaypoint >= 4){
                    actualWaypoint = 0;
                }
            }
            float waypointHeading = degrees(atan2((waypoints[actualWaypoint].x - posX ), (waypoints[actualWaypoint].y - posY)));
            
            waypointHeading = waypointHeading >= 360 ? waypointHeading - 360 : waypointHeading;
            waypointHeading = waypointHeading < 0 ? waypointHeading + 360 : waypointHeading;
            
            // Calculo de erro de mira
            float headingError = getDelta(heading, (int)waypointHeading);
            headingControl.setSetpoint((int)waypointHeading);
            headingControl.setError(headingError);
            headingControl.update();

            #ifdef DEBUG
                Serial.print("X: ");
                Serial.print(posX);
                Serial.print(" Y: ");
                Serial.print(posY);
                Serial.print(" H: ");
                Serial.print(heading);
                Serial.print(" destX: ");
                Serial.print(waypoints[actualWaypoint].x);
                Serial.print(" destY: ");
                Serial.print(waypoints[actualWaypoint].y);
                Serial.print(" Dist: ");
                Serial.print(distance(waypoints[actualWaypoint].x, waypoints[actualWaypoint].y));
                Serial.print(" setHeading: ");
                Serial.print(waypointHeading);
                Serial.print(" deltaHeading: ");
                Serial.print(getDelta(heading, waypointHeading));
                Serial.print(" magH: ");
                Serial.print(imu.magHeading);
                Serial.print(" deltaMag: ");
                Serial.println(getDelta(imu.magHeading, heading));
            #endif
            lastTime = millis();
        }
    }

    // Subrotinas internas
    private:
    // Retorna a distancia do robo a um ponto (x,y)
    float distance(int x, int y){
        return sqrt(sq(posX - x) + sq(posY - y));
    }

    // Métodos externos
    public:
    void setHeading(float value){
        headingControl.setSetpoint(value);
    }   

    void setMode(int newMode){
        mode = newMode;
    }

    float getSpeedLeft(){
        return -headingControl.getOutput() + forward;
    }

    float getSpeedRight(){
        return headingControl.getOutput() + forward;
    }

    float getDistance(){
        return distance(waypoints[actualWaypoint].x, waypoints[actualWaypoint].y);
    }

    float getObjectiveHeading(){
        return headingControl.getSetpoint();
    }

    void goForward(float speed){
        forward = speed;
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

    int getDestinationX(){
        return waypoints[actualWaypoint].x;
    }

    int getDestinationY(){
        return waypoints[actualWaypoint].y;
    }

    float getHeading(){
        return heading;
    }

    unsigned int getWaypoint(){
        return actualWaypoint;
    }

    void reset(IMUReading imu){
        posX = 0;
        posY = 0;
        posZ = 0;
        heading = imu.magHeading;
        headingControl.resetIntegral();
        actualWaypoint = 0;
    }
};

#ifdef DEBUG
#undef DEBUG
#endif
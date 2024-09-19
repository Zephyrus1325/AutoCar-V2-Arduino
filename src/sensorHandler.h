#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H
#include "sensors.h"
/*--------------------------------------------------------------------------------------+
|   Classe de Manipulador de Sensores - Marco Aurélio (08/09/2024)                      |
|                                                                                       |
|   Arquivo com a classe do Manipulador de Sensores                                     |
|   É possivel adicionar sensores e cuidar de todo o sensoriamento com uma classe só    |
+--------------------------------------------------------------------------------------*/

class SensorHandler{
    private:
    // Esse é um quick-fix que eu espero que não seja fixo
    // Vai existir um limite de 10 sensores de cada tipo, que eu acho que é razoável, espero
    // TODO: usar alocação dinâmica para os sensores, a fim de otimizar um pouco o uso de memoria, e organizar tudo no geral
    #define MAX_SENSORS 10
    unsigned int totalUltrassound = 0;              // Total atual de sensores Ultrassonicos
    unsigned int totalInfrared = 0;                 // Total atual de sensores Infravermelho
    Ultrassound ultrassoundSensors[MAX_SENSORS];    // Lista com todos os sensores Ultrassonicos Criados
    Infrared infraredSensors[MAX_SENSORS];          // Lista com todos os sensores Infravermelhos Criados   
    
    public:
    SensorHandler(){}

    // Função que atualiza todos os sensores
    // **Rodar essa função todo loop**
    void update(){
        for(unsigned int i = 0; i < totalUltrassound; i++){
            Ultrassound& sensor = ultrassoundSensors[i];
            sensor.update();
        }

        for(unsigned int i = 0; i < totalInfrared; i++){
            Infrared& sensor = infraredSensors[i];
            sensor.update();
        }
    }

    float getUltrassoundDistance(int id){
        Ultrassound& sensor = ultrassoundSensors[id];
        return sensor.getDistance();
    }

    /*-------------------------------------------------------------------------------------+
    | CUIDADO!! CUIDADO NOBRE PROGRAMADOR, CÓDIGO REPETITIVO A FRENTE                      |
    | CÓDIGO DE ADICIONAR CADA TIPO DE SENSOR                                              |
    --------------------------------------------------------------------------------------*/

    // Funções de adicionar Sensor Ultrassonico
    void addUltrassound(unsigned int triggerPin, unsigned int echoPin, unsigned long updateTime){
        if(totalUltrassound < MAX_SENSORS){
            Ultrassound newSensor = Ultrassound(triggerPin, echoPin);
            newSensor.begin();
            ultrassoundSensors[totalUltrassound] = newSensor;
            totalUltrassound++;
        }
    }

    void addUltrassound(unsigned int triggerPin, unsigned int echoPin, unsigned long maxTime, unsigned long updateTime){
        if(totalUltrassound < MAX_SENSORS){
            Ultrassound newSensor = Ultrassound(triggerPin, echoPin, maxTime);
            newSensor.begin();
            ultrassoundSensors[totalUltrassound] = newSensor;
            totalUltrassound++;
        }
    }

    //Funções de adicionar Sensor Infravermelho
    void addInfrared(unsigned int pin, unsigned long updateTime){
        if(totalInfrared < MAX_SENSORS){
            infraredSensors[totalInfrared] = Infrared(pin);
            infraredSensors[totalInfrared].begin();
            totalInfrared++;
        }
    }

    void addInfrared(unsigned int pin, bool mode,  unsigned long updateTime){
        if(totalInfrared < MAX_SENSORS){
            infraredSensors[totalInfrared] = Infrared(pin, mode);
            infraredSensors[totalInfrared].begin();
            totalInfrared++;
        }
    }
};
#endif
#ifndef SENSORS_H
#define SENSORS_H

// Descomente esta linha para ativar o debug
// (Mostrar todos os dados dos sensores no serial USB)
//#define DEBUG

#include <Arduino.h>
#include "timer.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP180.h"

/*------------------------------------------------------------+
|   Classes de Sensores - Marco Aurélio (08/09/2024)          |
|                                                             |
|   Arquivo com todas as classes de Sensores                  |
|   Adicione qualquer tipo de sensor aqui                     |
|                                                             |
|   Sensores adicionados até o momento:                       |
|   - Ultrassom                                               |
|   - Infravermelho                                           |
|                                                             |
+------------------------------------------------------------*/

// Classe mãe de todos os sensores usados
class Sensor{
    virtual void update();  // Função de atualização do Sensor
};

//
class Ultrassound : public Sensor {
    private:
    float distance;                     // Distancia lida pelo Sensor Ultrassonico em cm
    unsigned int triggerPin;      // Pino Trigger do Sensor
    unsigned int echoPin;         // Pino Echo do Sensor
    unsigned long maxTime = 5882;       // Tempo máximo que vai se esperar pelo echo do sensor em microssegundos
    unsigned long sentTime = 0;
    bool waiting = false;
    timer updateTimer = {0, 100, true, true, true};


    public:
    Ultrassound() : triggerPin(0), echoPin(0){}
    Ultrassound(const unsigned int trigger, const unsigned int echo) : triggerPin(trigger), echoPin(echo){}
    Ultrassound(const unsigned int trigger, const unsigned int echo, const unsigned long updateTime) 
    : triggerPin(trigger), echoPin(echo){updateTimer = timer{0, updateTime, true, true, true};}

    // Operador de atribuição
    Ultrassound& operator=(const Ultrassound& other){
        // Verifica auto-atribuição
        if (this != &other) {  
            this->triggerPin = other.triggerPin;
            this->echoPin = other.echoPin;
            this->maxTime = other.maxTime;
            this->updateTimer = other.updateTimer;
        }
        return *this;
    };

    //Inicializa o sensor
    void begin(){
        pinMode(triggerPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    // Faz uma leitura do sensor
    void update(){
        if(updateTimer.CheckTime()){
            digitalWrite(triggerPin, LOW);
            delayMicroseconds(2);
            // Ativa o trigger por 10 microssegundos
            digitalWrite(triggerPin, HIGH);
            delayMicroseconds(10);
            digitalWrite(triggerPin, LOW);
            sentTime = micros();
            waiting = true;
            // A aguarda pelo pino de echo e retorna o tempo até o pulso chegar
            unsigned long duration = pulseIn(echoPin, HIGH, maxTime);
            // retorna a distancia
            distance = duration * 0.034f / 2.0f;
        }
    }


    float getDistance(){return distance;}
    void setMaxTime(unsigned long time){maxTime = time;}
};



class Infrared : public Sensor{
    private:
    float distance;         // Distancia detectada pelo sensor
    unsigned int pin;       // Pino usado pelo sensor
    bool state;             // Estado do sensor, se está detectando ou não
    bool mode;              // Modo do sensor, pode ser ON_OFF ou ANALOG
    timer updateTimer{0, 1000, true, true, true};
    // TODO: Adicionar mais um pino e o suporte para, caso o mongol do usuario queira
    // usar o sensor Infravermelho nos dois modos ao mesmo tempo, por algum motivo

    public:
    #define DIGITAL 0    // Sensor no modo Digital
    #define ANALOG 1     // Sensor no modo Analogico

    Infrared() : pin(0){}
    Infrared(unsigned int pin) : pin(pin){mode = DIGITAL;}
    Infrared(unsigned int pin, bool mode) : pin(pin), mode(mode){}
    Infrared(unsigned int pin, bool mode, unsigned int updateTime)
     : pin(pin), mode(mode){updateTimer = timer{0, updateTime, true, true, true};}

    Infrared& operator=(const Infrared& other){
        // Verifica auto-atribuição
        if (this != &other) {  
            this->pin = other.pin;
            this->mode = other.mode;
            this->updateTimer = other.updateTimer;
        }
        return *this;
    };

    //Inicializa o sensor
    void begin(){
        pinMode(pin, INPUT);
    }

    // Faz uma leitura do sensor
    void update(){
        if(updateTimer.CheckTime()){
            if(ANALOG){
                distance = analogRead(pin);
                state = false;
            } else {
                distance = 0;
                state = digitalRead(pin);
            }
        }
    }

    bool getState(){return state;}
    float getDistance(){return distance;}
};

struct IMUReading {
        int16_t rawAccelX;
        int16_t rawAccelY;
        int16_t rawAccelZ;
        int16_t rawGyroX;
        int16_t rawGyroY;
        int16_t rawGyroZ;
        int16_t rawMagX;
        int16_t rawMagY;
        int16_t rawMagZ;
        int16_t rawPressure;
        int16_t rawTemperature;
        float accelX; // cm/s
        float accelY; // cm/s
        float accelZ; // cm/s
        float gyroX; // Graus/s
        float gyroY; // Graus/s
        float gyroZ; // Graus/s
        float magX;
        float magY;
        float magZ;
        float pressure;
        float temperature;
        float heading; // Graus
        float pitch;   // Graus
        float roll;    // Graus
    };


// Eu vou me arrepender de fazer essa classe
class InertialUnit : public Sensor{
    private:
    MPU6050 mpu;
    HMC5883L mag;
    BMP180 baro;
    timer updateTimer{0,0,100,true,true};
    float magHeading = 0;      // Heading Calculado pelo magnetometro
    float accPitch = 0;        // Pitch estimado pelo acelerometro
    float accRoll = 0;         // Roll estimado pelo acelerometro
    float mixMultiplier = 0.01;// Multiplicador de quanto receber da leitura sem referencial

    public:

    IMUReading reading;
    InertialUnit(){};

    void begin(){
        Wire.begin();
        Wire.beginTransmission(0x68);
        Wire.write(0x37);
        Wire.write(0x02);
        Wire.endTransmission();

        Wire.beginTransmission(0x68);
        Wire.write(0x6A);
        Wire.write(0x00);
        Wire.endTransmission();

        // Disable Sleep Mode
        Wire.beginTransmission(0x68);
        Wire.write(0x6B);
        Wire.write(0x00);
        Wire.endTransmission();

        mpu.initialize();
        mag.initialize();
        baro.begin();
        mag.setMode(HMC5883L_MODE_CONTINUOUS);
        mag.setDataRate(HMC5883L_RATE_15);
        #ifdef DEBUG
            Serial.println(F("Testing I2C connections..."));
            Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
            Serial.println(mag.testConnection() ? F("HMC5883L connection successful") : F("HMC5883L connection failed"));
            //Serial.println(baro.testConnection() ? F("BMP085 connection successful") : F("BMP085 connection failed"));
        #endif
    }
    // Atualiza as medições
    void update(){
        if(updateTimer.CheckTime()){
            mpu.getMotion6(&reading.rawAccelX, &reading.rawAccelY, &reading.rawAccelZ, &reading.rawGyroX, &reading.rawGyroY, &reading.rawGyroZ);
            mag.getHeading(&reading.rawMagX, &reading.rawMagY, &reading.rawMagZ);
            reading.pressure = baro.getPressure();
            reading.temperature = baro.getTemperature();
            
            reading.accelX = reading.rawAccelX;
            reading.accelY = reading.rawAccelY;
            reading.accelZ = reading.rawAccelZ;

            reading.gyroX = (float) (reading.rawGyroX * 250.0f) / (pow(2,15));
            reading.gyroY = (float) (reading.rawGyroY * 250.0f) / (pow(2,15));
            reading.gyroZ = (float) (reading.rawGyroZ * 250.0f) / (pow(2,15));
            
            // Leitura e calculo dos valores relativos ao referencial absoluto
            float magHeading = atan2(reading.rawMagY, reading.rawMagX) * 180.0f / PI;
            float accPitch = (atan2(reading.accelY, sqrt(pow(reading.accelX, 2) + pow(reading.accelZ, 2))) * 180.0f / PI);
            float accRoll = (atan2(-reading.accelX, sqrt(pow(reading.accelY, 2) + pow(reading.accelZ, 2))) * 180.0f / PI);
            

            // Caralho eu não sei como fazer essa conta
            // Atual = Atual + variação confiavel * 0.99 + Variação consistente * 0.01
            //reading.pitch = reading.gyroX * (1-mixMultiplier) + accPitch * mixMultiplier;
            //reading.roll = reading.gyroY * (1-mixMultiplier) + accRoll * mixMultiplier;
            //reading.heading = reading.gyroZ * (1-mixMultiplier) + magHeading * mixMultiplier;
            reading.pitch = accPitch;
            reading.roll = accRoll;
            reading.heading = magHeading;
        }
    }

};

#endif
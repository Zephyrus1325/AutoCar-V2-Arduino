#pragma once
#include <Arduino.h>
#include "timer.h"

/*-------------------------------------------------------------------+
|    Classe de motor - Marco Aurélio (08/09/2024)                    |
|                                                                    |
|   Arquivo com a classe de motor                                    |
|   Possui todos os parametros para controlar um motor adequadamente |
+--------------------------------------------------------------------*/


class Motor{
    private:
    // Pinagem do motor
    unsigned int dirAPin; // Pino de direção A
    unsigned int dirBPin; // Pino de direção B
    unsigned int pwmPin;  // Pino de controle de potência
    unsigned int encoderPin = 50; // Pino do encoder rotativo

    // Parametros caso queira controlar o motor com PID (espero que meu sofrimento valha a pena)
    float PIDIntegral; // Integral usada no controlador PID
    float lastSpeed;   // Ultimo valor lido, usado no calculo da derivada no controlador PID
    unsigned long lastUpdateTime = 0; // Momento do ultimo loop do PID

    float rpm;                     // RPM atual do motor
    const float timeout = 150;     // Tempo em milissegundos do timeout do sensor de rotação
    const float wheelCircunference = 6.7f * PI;    // Diametro da roda
    unsigned long lastEncoderTime = 0; // Instante da ultima leitura do encoder rotativo
    timer updateTimer{0, 50, true, true, false};    // Timer que controla a frequencia de update do PID
    const float maxSpeed = 1000.f;     // Velocidade maxima que o motor deveria chegar (evita problemas com divisão por zero)

    public:
    unsigned int motorMode = 0;    // Modo do motor (0 = PID ON | 1 = PID Off)
    int throttle = 0;                  // Controle manual do motor
    float actualSpeed; // Velocidade atual do motor
    float setpoint;    // Velocidade de objetivo do motor
    float Kp = 1;   // Parametros de calibração do controlador PID
    float Ki = 0.01;
    float Kd = 0;  
    Motor(unsigned int dirA, unsigned int dirB, unsigned int pwm) : dirAPin(dirA), dirBPin(dirB), pwmPin(pwm){}
    Motor(unsigned int dirA, unsigned int dirB, unsigned int pwm, unsigned int encoder) : dirAPin(dirA), dirBPin(dirB), pwmPin(pwm), encoderPin(encoder){}

    // Inicializador do motor
    void begin(){
        pinMode(dirAPin, OUTPUT);
        pinMode(dirBPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        pinMode(encoderPin, INPUT);
    }
    
    // Atualiza coisas do motor como PID, velocidade, entre outros
    // ** Executar esta função todos os loops **
    void update(){
        if(!motorMode){
            if(updateTimer.CheckTime()){
                float error = setpoint - actualSpeed;
                float PIDderivative = (actualSpeed - lastSpeed) * (millis() - lastUpdateTime);
                PIDIntegral += error * (millis() - lastUpdateTime);

                float proportional = Kp * error;
                float integral = Ki * PIDIntegral;
                float derivative = Kd * PIDderivative;

                throttle = (int)(proportional + integral + derivative);

                lastUpdateTime = millis();

                #ifdef DEBUG
                Serial.print("Actual Speed: ");
                Serial.print(actualSpeed);
                Serial.print(" Throttle: ");
                Serial.println(throttle);
                #endif
            }
        }
        setSpeed(throttle);
        if(millis() - lastEncoderTime > timeout){
            actualSpeed = 0;
        }
        
    }

    void setThrottle(int t){
        throttle = t;
    }

    void setMode(int mode){
        motorMode = mode;
        // Reseta a potencia caso aconteça de colocar no modo manual e o PID comandar alguma potencia ainda
        PIDIntegral = 0;
        lastUpdateTime = millis();
        throttle = 0;     
         
    }

    void setKp(float kp){
        Kp = kp;
    }

    void setKi(float ki){
        Ki = ki;
    }

    void setKd(float kd){
        Kd = kd;
    }

    void resetIntegral(){
        PIDIntegral = 0;
    }

    void setSetpoint(float speed){
        setpoint = speed;
        if(speed == 0){
            PIDIntegral = 0;
        }
    }

    float getSpeed(){
        return actualSpeed;
    }
    
    // Define direção e potência do motor
    // Recebe valor entre -255 e +255
    void setSpeed(int speed){
        if(speed < 0){
            digitalWrite(dirAPin, HIGH);
            digitalWrite(dirBPin, LOW);
        } else {
            digitalWrite(dirAPin, LOW);
            digitalWrite(dirBPin, HIGH);
        }
        analogWrite(pwmPin, constrain(abs(speed), 0, 255));
    }

    // Vou tentar fazer essa função rodar sem usar interrupções, mas se ficar muito impreciso
    // vou ter que fazer alguma magia pra converter um void (Motor::*)() para um void (*)()

    void sensorUpdate(){
        float rps = (float)50.f/(millis() - lastEncoderTime);
        actualSpeed = rps * wheelCircunference;
        actualSpeed = constrain(actualSpeed, -maxSpeed, maxSpeed);
        lastEncoderTime = millis();
    }
};

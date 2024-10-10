#include <Arduino.h>
#include "pins.h"
#include "sensorHandler.h"
#include "motor.h"
#include "navigation.h"
#include "comms.h"
#include "defines.h"
#include "buzzer.h"

// Descomente essa linha para habilitar debug de tudo
//#define DEBUG

SensorHandler sensors;
Motor leftMotor(PIN_MOTOR_LEFT_DIRA, PIN_MOTOR_LEFT_DIRB, PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_ENCODER);
Motor rightMotor(PIN_MOTOR_RIGHT_DIRA, PIN_MOTOR_RIGHT_DIRB, PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_ENCODER);
CarData carData;
Command command;
Navigation navigation;
const long ultrassoundUpdateTime = 100;
int navMode = 0;

bool walkFlag = false;

Buzzer buzzer(PIN_BUZZER);

void updateCarData(){
    carData.battery_voltage = 0;
    carData.battery_percentage = 0;
    carData.ultrassound_reading_front = sensors.getUltrassoundDistance(0);
    carData.ultrassound_reading_front_left = sensors.getUltrassoundDistance(1);
    carData.ultrassound_reading_front_right = sensors.getUltrassoundDistance(2);
    carData.ultrassound_reading_left = sensors.getUltrassoundDistance(3);
    carData.ultrassound_reading_right = sensors.getUltrassoundDistance(4);
    carData.ultrassound_reading_back_left = sensors.getUltrassoundDistance(5);
    carData.ultrassound_reading_back = sensors.getUltrassoundDistance(6);
    carData.ultrassound_reading_back_right = sensors.getUltrassoundDistance(7);
    carData.motor_left_mode = leftMotor.getMode();
    carData.motor_left_setpoint = leftMotor.getSetpoint() * FLOAT_MULTIPLIER;
    carData.motor_left_speed = leftMotor.getSpeed() * FLOAT_MULTIPLIER;
    carData.motor_left_throttle = leftMotor.getThrottle();
    carData.motor_left_kp = leftMotor.getKp() * FLOAT_MULTIPLIER;
    carData.motor_left_ki = leftMotor.getKi() * FLOAT_MULTIPLIER;
    carData.motor_left_kd = leftMotor.getKd() * FLOAT_MULTIPLIER;
    carData.motor_right_mode = rightMotor.getMode();
    carData.motor_right_setpoint = rightMotor.getSetpoint() * FLOAT_MULTIPLIER;
    carData.motor_right_speed = rightMotor.getSpeed() * FLOAT_MULTIPLIER;
    carData.motor_right_throttle = rightMotor.getThrottle();
    carData.motor_right_kp = rightMotor.getKp() * FLOAT_MULTIPLIER;
    carData.motor_right_ki = rightMotor.getKi() * FLOAT_MULTIPLIER;
    carData.motor_right_kd = rightMotor.getKd() * FLOAT_MULTIPLIER;
    carData.gyro_raw_x = sensors.getIMUReading().rawGyroX;
    carData.gyro_raw_y = sensors.getIMUReading().rawGyroY;
    carData.gyro_raw_z = sensors.getIMUReading().rawGyroZ;
    carData.gyro_treated_x = sensors.getIMUReading().gyroX * FLOAT_MULTIPLIER;
    carData.gyro_treated_y = sensors.getIMUReading().gyroY * FLOAT_MULTIPLIER;
    carData.gyro_treated_z = sensors.getIMUReading().gyroZ * FLOAT_MULTIPLIER;
    carData.acc_raw_x = sensors.getIMUReading().rawAccelX;
    carData.acc_raw_y = sensors.getIMUReading().rawAccelY;
    carData.acc_raw_z = sensors.getIMUReading().rawAccelZ;
    carData.acc_treated_x = sensors.getIMUReading().accelX * FLOAT_MULTIPLIER;
    carData.acc_treated_y = sensors.getIMUReading().accelY * FLOAT_MULTIPLIER;
    carData.acc_treated_z = sensors.getIMUReading().accelZ * FLOAT_MULTIPLIER;
    carData.mag_raw_x = sensors.getIMUReading().rawMagX;
    carData.mag_raw_y = sensors.getIMUReading().rawMagY;
    carData.mag_raw_z = sensors.getIMUReading().rawMagZ; 
    carData.mag_treated_x = sensors.getIMUReading().magX * FLOAT_MULTIPLIER;
    carData.mag_treated_y = sensors.getIMUReading().magY * FLOAT_MULTIPLIER;
    carData.mag_treated_z = sensors.getIMUReading().magZ * FLOAT_MULTIPLIER;
    carData.raw_pressure = sensors.getIMUReading().rawPressure;
    carData.raw_temperature = sensors.getIMUReading().rawTemperature;
    carData.pressure = sensors.getIMUReading().pressure * FLOAT_MULTIPLIER;
    carData.temperature = sensors.getIMUReading().temperature * FLOAT_MULTIPLIER;
    carData.navigation_mode = navMode;
    carData.navigation_position_x = navigation.getPosX();
    carData.navigation_position_y = navigation.getPosY();
    carData.navigation_position_z = navigation.getObjectiveHeading();
    carData.navigation_position_pitch = 0; //sensors.getIMUReading().pitch * FLOAT_MULTIPLIER;
    carData.navigation_position_roll = 0; //sensors.getIMUReading().roll * FLOAT_MULTIPLIER;
    carData.navigation_position_heading = navigation.getHeading() * FLOAT_MULTIPLIER; //sensors.getIMUReading().heading * FLOAT_MULTIPLIER;
    carData.navigation_destination_x = navigation.getDestinationX();
    carData.navigation_destination_y = navigation.getDestinationY();
    carData.navigation_destination_z = navigation.getDistance();;
    carData.navigation_home_x = 0;
    carData.navigation_home_y = 0;
    carData.navigation_home_z = 0;
}

void leftEncoder(){
    leftMotor.sensorUpdate();
}

void rightEncoder(){
    rightMotor.sensorUpdate();
}


void setup() {
    sensors.addUltrassound(PIN_ULTRASSOUND_FRONT_TRIGGER, PIN_ULTRASSOUND_FRONT_ECHO, ultrassoundUpdateTime);
    sensors.addUltrassound(PIN_ULTRASSOUND_FRONT_LEFT_TRIGGER, PIN_ULTRASSOUND_FRONT_LEFT_ECHO, ultrassoundUpdateTime);
    sensors.addUltrassound(PIN_ULTRASSOUND_FRONT_RIGHT_TRIGGER, PIN_ULTRASSOUND_FRONT_RIGHT_ECHO, ultrassoundUpdateTime);
    sensors.addUltrassound(PIN_ULTRASSOUND_LEFT_TRIGGER, PIN_ULTRASSOUND_LEFT_ECHO, ultrassoundUpdateTime);
    sensors.addUltrassound(PIN_ULTRASSOUND_RIGHT_TRIGGER, PIN_ULTRASSOUND_RIGHT_ECHO, ultrassoundUpdateTime);
    sensors.addUltrassound(PIN_ULTRASSOUND_BACK_LEFT_TRIGGER, PIN_ULTRASSOUND_BACK_LEFT_ECHO, ultrassoundUpdateTime);
    sensors.addUltrassound(PIN_ULTRASSOUND_BACK_TRIGGER, PIN_ULTRASSOUND_BACK_ECHO, ultrassoundUpdateTime);
    sensors.addUltrassound(PIN_ULTRASSOUND_BACK_RIGHT_TRIGGER, PIN_ULTRASSOUND_BACK_RIGHT_ECHO, ultrassoundUpdateTime);
    sensors.addInertialUnit();
    navigation.begin();
    leftMotor.begin();
    rightMotor.begin();
    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_LEFT_ENCODER), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_RIGHT_ENCODER), rightEncoder, RISING);
    buzzer.begin();
    Serial.begin(115200);
    Serial2.begin(115200);
}
void loop() {
    sensors.update();
    leftMotor.setSetpoint(navigation.getSpeedLeft());
    rightMotor.setSetpoint(navigation.getSpeedRight());
    leftMotor.update();
    rightMotor.update();
    navigation.update(sensors.getIMUReading(), leftMotor.getSpeed(), rightMotor.getSpeed());
    buzzer.update();
    updateCarData();
    sendData(&carData);
    // Se um comando foi recebido
    if(receiveData(&command) == GOOD_PACKET){
        
        #ifdef DEBUG
        Serial.print("Received Command ID: ");
        Serial.print(command.index);
        Serial.print(" Value: ");
        Serial.println(command.value);
        #endif

        switch(command.index){
            
            // Invalid case
            case 0:
                break;
            case COMMAND_MOTOR_LEFT_SETSPEED:
                leftMotor.setSetpoint((float)(command.value) / FLOAT_MULTIPLIER);
                break;
            case COMMAND_MOTOR_LEFT_SETTHROTTLE:
                leftMotor.setThrottle(command.value);
                break;
            case COMMAND_MOTOR_LEFT_SETMODE:
                leftMotor.setMode(command.value);
                break;
            case COMMAND_MOTOR_LEFT_SETKP:
                leftMotor.setKp((float) (command.value) / FLOAT_MULTIPLIER);
                break;
            case COMMAND_MOTOR_LEFT_SETKI:
                leftMotor.setKi((float) (command.value) / FLOAT_MULTIPLIER);
                leftMotor.resetIntegral();
                break;
            case COMMAND_MOTOR_LEFT_SETKD:
                leftMotor.setKd((float) (command.value) / FLOAT_MULTIPLIER);
                break;
            case COMMAND_MOTOR_RIGHT_SETSPEED:
                rightMotor.setSetpoint((float)(command.value) / FLOAT_MULTIPLIER);
                break;
            case COMMAND_MOTOR_RIGHT_SETTHROTTLE:
                rightMotor.setThrottle(command.value);
                break;
            case COMMAND_MOTOR_RIGHT_SETMODE:
                rightMotor.setMode(command.value);
                break;
            case COMMAND_MOTOR_RIGHT_SETKP:
                rightMotor.setKp((float) (command.value) / FLOAT_MULTIPLIER);
                break;
            case COMMAND_MOTOR_RIGHT_SETKI:
                rightMotor.setKi((float) (command.value) / FLOAT_MULTIPLIER);
                rightMotor.resetIntegral();
                break;
            case COMMAND_MOTOR_RIGHT_SETKD:
                rightMotor.setKd((float) (command.value) / FLOAT_MULTIPLIER);
                break;
            case COMMAND_NAVIGATION_SETMODE:
                navMode = command.value;
                break;
            case COMMAND_BUZZER_SETSTATE:
                buzzer.play(command.value);
                break;
            case COMMAND_TEMP_3_METERS:
                leftMotor.setMode(0);
                rightMotor.setMode(0);
                navigation.reset();
                if(!walkFlag){
                    navigation.goForward(100);
                    walkFlag = true;
                } else {
                    navigation.goForward(0);
                }
                
                
                break;
            default:
                break;
        }
    }
}
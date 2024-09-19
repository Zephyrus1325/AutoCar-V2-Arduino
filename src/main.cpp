#include <Arduino.h>
#include "pins.h"
#include "sensorHandler.h"
#include "motor.h"
#include "comms.h"
#include "defines.h"

SensorHandler sensors;
Motor left(PIN_MOTOR_LEFT_DIRA, PIN_MOTOR_LEFT_DIRB, PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_ENCODER);
Motor right(PIN_MOTOR_RIGHT_DIRA, PIN_MOTOR_RIGHT_DIRB, PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_ENCODER);
CarData carData;

const long ultrassoundUpdateTime = 100;

void updateCarData(){
    carData.battery_voltage = 11.1f;
    carData.battery_percentage = carData.battery_voltage * 100 / MAX_BATTERY_VOLTAGE;
    carData.ultrassound_reading_front = sensors.getUltrassoundDistance(0);
    carData.ultrassound_reading_front_left = sensors.getUltrassoundDistance(1);
    carData.ultrassound_reading_front_right = sensors.getUltrassoundDistance(2);
    carData.ultrassound_reading_left = sensors.getUltrassoundDistance(3);
    carData.ultrassound_reading_right = sensors.getUltrassoundDistance(4);
    carData.ultrassound_reading_back_left = sensors.getUltrassoundDistance(5);
    carData.ultrassound_reading_back = sensors.getUltrassoundDistance(6);
    carData.ultrassound_reading_back_right = sensors.getUltrassoundDistance(7);
    carData.motor_mode = 0;
    carData.motor_left_setpoint = 10;
    carData.motor_left_speed = 15;
    carData.motor_left_throttle = 230;
    carData.motor_left_kp = 1.1;
    carData.motor_left_ki = 1.2;
    carData.motor_left_kd = 1.3;
    carData.motor_right_setpoint = 6;
    carData.motor_right_speed = 9;
    carData.motor_right_throttle = 69;
    carData.motor_right_kp = 2.1;
    carData.motor_right_ki = 2.2;
    carData.motor_right_kd = 2.3;
    carData.gyro_raw_x = 432;
    carData.gyro_raw_y = -234;
    carData.gyro_raw_z = 13255;
    carData.gyro_treated_x = 1;
    carData.gyro_treated_y = 2;
    carData.gyro_treated_z = 3;
    carData.acc_raw_x = -514;
    carData.acc_raw_y = -2432;
    carData.acc_raw_z = 2416;
    carData.acc_treated_x = -1;
    carData.acc_treated_y = -2;
    carData.acc_treated_z = -3;
    carData.mag_raw_x = 15;
    carData.mag_raw_y = 20;
    carData.mag_raw_z = 30; 
    carData.mag_treated_x = 1;
    carData.mag_treated_y = -2;
    carData.mag_treated_z = 3;
    carData.navigation_mode = 0;
    carData.navigation_position_x = 0;
    carData.navigation_position_y = 0;
    carData.navigation_position_z = 0;
    carData.navigation_position_pitch = 0.4;
    carData.navigation_position_roll = 15.9;
    carData.navigation_position_heading = 1.5;
    carData.navigation_destination_x = 1;
    carData.navigation_destination_y = 1;
    carData.navigation_destination_z = 1;
    carData.navigation_home_x = 3;
    carData.navigation_home_y = 3;
    carData.navigation_home_z = 3;
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
    left.begin();
    right.begin();
    Serial.begin(115200);
    Serial2.begin(115200);
}
void loop() {
    sensors.update();
    updateCarData();
    sendData(&carData);
}
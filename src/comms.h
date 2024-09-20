#ifndef COMMS_H
#define COMMS_H
#include "Arduino.h"
#include "timer.h"
#include "StreamSend.h"

struct CarData{
    int32_t battery_voltage;                    // Float
    uint8_t battery_percentage;
    int32_t ultrassound_reading_front;          // Float
    int32_t ultrassound_reading_front_left;     // Float
    int32_t ultrassound_reading_front_right;    // Float    
    int32_t ultrassound_reading_left;           // Float
    int32_t ultrassound_reading_right;          // Float    
    int32_t ultrassound_reading_back;           // Float
    int32_t ultrassound_reading_back_left;      // Float    
    int32_t ultrassound_reading_back_right;     // Float    
    uint8_t motor_mode;
    int32_t motor_left_setpoint;  // Float
    int32_t motor_left_speed;     // Float
    uint8_t motor_left_throttle;
    int32_t motor_left_kp;          // Float
    int32_t motor_left_ki;          // Float
    int32_t motor_left_kd;          // Float
    int32_t motor_right_setpoint;   // Float        
    int32_t motor_right_speed;      // Float    
    uint8_t motor_right_throttle;
    int32_t motor_right_kp;  // Float
    int32_t motor_right_ki;  // Float
    int32_t motor_right_kd;  // Float
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
    int16_t gyro_treated_x;
    int16_t gyro_treated_y;
    int16_t gyro_treated_z;
    int16_t acc_raw_x;
    int16_t acc_raw_y;
    int16_t acc_raw_z;
    int16_t acc_treated_x;
    int16_t acc_treated_y;
    int16_t acc_treated_z;
    int16_t mag_raw_x;
    int16_t mag_raw_y;
    int16_t mag_raw_z;
    int16_t mag_treated_x;
    int16_t mag_treated_y;
    int16_t mag_treated_z;
    uint8_t navigation_mode;
    int16_t navigation_position_x;
    int16_t navigation_position_y;
    int16_t navigation_position_z;
    int32_t navigation_position_pitch;   // Float
    int32_t navigation_position_roll;    // Float
    int32_t navigation_position_heading; // Float
    int16_t navigation_destination_x;
    int16_t navigation_destination_y;
    int16_t navigation_destination_z;
    int16_t navigation_home_x;
    int16_t navigation_home_y;
    int16_t navigation_home_z;
    int64_t padding1;
    int32_t padding2;
    int8_t padding3;
};

// Timer responsável por enviar os dados a cada 100 milisegundos (10Hz)
timer sendTimer{0, 100, true, true, true};

// Envia os dados a cada tempo, definido pelo sendTimer
void sendData(CarData* data){
    if(sendTimer.CheckTime()){
        StreamSend::sendObject(Serial2, data, sizeof(*data));
        //Serial.print(data->ultrassound_reading_front);
        //Serial.print("|");
        //Serial.print(data->ultrassound_reading_front_left);
        //Serial.print("|");
        //Serial.print(data->ultrassound_reading_front_right);
        //Serial.print("|");
        //Serial.print(data->ultrassound_reading_left);
        //Serial.print("|");
        //Serial.print(data->ultrassound_reading_right);
        //Serial.print("|");
        //Serial.print(data->ultrassound_reading_back);
        //Serial.print("|");
        //Serial.print(data->ultrassound_reading_back_left);
        //Serial.print("|");
        //Serial.println(data->ultrassound_reading_back_right);
    }
    
}




#endif
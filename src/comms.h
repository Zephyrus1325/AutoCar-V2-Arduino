#ifndef COMMS_H
#define COMMS_H
#include "Arduino.h"
#include "timer.h"
#include "StreamSend.h"

struct CarData{
    int32_t battery_voltage;                    // Float
    int32_t battery_percentage;
    int32_t ultrassound_reading_front;          // Float
    int32_t ultrassound_reading_front_left;     // Float
    int32_t ultrassound_reading_front_right;    // Float    
    int32_t ultrassound_reading_left;           // Float
    int32_t ultrassound_reading_right;          // Float    
    int32_t ultrassound_reading_back;           // Float
    int32_t ultrassound_reading_back_left;      // Float    
    int32_t ultrassound_reading_back_right;     // Float    
    int32_t motor_left_mode;
    int32_t motor_left_setpoint;  // Float
    int32_t motor_left_speed;     // Float
    int32_t motor_left_throttle;
    int32_t motor_left_kp;          // Float
    int32_t motor_left_ki;          // Float
    int32_t motor_left_kd;          // Float
    int32_t motor_right_mode;
    int32_t motor_right_setpoint;   // Float        
    int32_t motor_right_speed;      // Float    
    int32_t motor_right_throttle;
    int32_t motor_right_kp;  // Float
    int32_t motor_right_ki;  // Float
    int32_t motor_right_kd;  // Float
    int32_t gyro_raw_x;
    int32_t gyro_raw_y;
    int32_t gyro_raw_z;
    int32_t gyro_treated_x;
    int32_t gyro_treated_y;
    int32_t gyro_treated_z;
    int32_t acc_raw_x;
    int32_t acc_raw_y;
    int32_t acc_raw_z;
    int32_t acc_treated_x;
    int32_t acc_treated_y;
    int32_t acc_treated_z;
    int32_t mag_raw_x;
    int32_t mag_raw_y;
    int32_t mag_raw_z;
    int32_t mag_treated_x;
    int32_t mag_treated_y;
    int32_t mag_treated_z;
    int32_t raw_temperature;
    int32_t temperature;    // Float
    int32_t raw_pressure;
    int32_t pressure;       // Float
    int32_t navigation_mode;
    int32_t navigation_position_x;
    int32_t navigation_position_y;
    int32_t navigation_position_z;
    int32_t navigation_position_pitch;   // Float
    int32_t navigation_position_roll;    // Float
    int32_t navigation_position_heading; // Float
    int32_t navigation_destination_x;
    int32_t navigation_destination_y;
    int32_t navigation_destination_z;
    int32_t navigation_home_x;
    int32_t navigation_home_y;
    int32_t navigation_home_z;
};

struct Command {
    int32_t index;
    int32_t value;
};

// Timer responsável por enviar os dados a cada 100 milisegundos (10Hz)
timer sendTimer{0, 100, true, true, true};

// Envia os dados a cada tempo, definido pelo sendTimer
void sendData(CarData* data){
    if(sendTimer.CheckTime()){
        StreamSend::sendObject(Serial2, data, sizeof(*data));
        // Debug Info
        //Serial.print("Sent Packet! Size: ");
        //Serial.println((int)sizeof(*data));
    }   
}

// Função de receber dados do Serial
// Incluir um pointer para a variavel que vai ser alterada com a leitura nova
byte receiveData(Command* command){
    // Ler a variavel caso tenha sido enviada algum dado no serial
    // E escrever esses dados no Struct CarData
    byte packetStatus = StreamSend::receiveObject(Serial2, command, sizeof(*command));
    return packetStatus;
    //Debugging Info
    //if(packetStatus == GOOD_PACKET){
    //    Serial.print("Received Healty Packet: ");
    //    Serial.println(sizeof(*command));
    //} else if(packetStatus == BAD_PACKET){
    //    Serial.println("Bad Packet: ");
    //    Serial.println(sizeof(*command));
    //}
}




#endif
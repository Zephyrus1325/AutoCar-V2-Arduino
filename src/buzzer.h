#pragma once
#include <Arduino.h>
#include <timer.h>
#include "defines.h"

class Buzzer{
    
    public:
    enum BuzzerCommand{
        
    };
    

    private:
    unsigned int pin;
    bool playing = false;
    bool beeping = false;
    int actual = BUZZER_NONE;
    unsigned int beepStep = 0;
    timer frequencyTimer{0,0,true,true,true};
    timer beepTime{0,0,true,true,true};
    timer waitTime{0,0,true,true,true};

    public:
    Buzzer(int p) : pin(p){}

    void begin(){
        pinMode(pin, OUTPUT);
    }

    void play(int command){
        actual = command;        
        switch(command){
            case BUZZER_NONE:
                break;
            case BUZZER_BATTERY_ALERT:
                frequencyTimer.waitMillis = 1000;
                beepTime.waitMillis = 100;
                waitTime.waitMillis = 220;
                break;
            default:
                break;
        }
    }

    void stop(){
        actual = BUZZER_NONE;
        playing = false;
        beepStep = 0;
    }

    void update(){
        switch(actual){
            case BUZZER_NONE:
                beeping = false;
                break;
            case BUZZER_BATTERY_ALERT:
                command_battery_alert();
                break;
            default:
                break;
        }
        digitalWrite(pin, beeping);
    }

    private:

    void command_battery_alert(){
        if(frequencyTimer.CheckTime()){
            playing = !playing;
        }
        if(playing){
            if(beepTime.CheckTime()){
                beeping = !beeping;
            }
        }
    }
};


#ifndef TIMER_H
#define TIMER_H
#include "Arduino.h"
/*---------------------------------------------------------------------------------------+
|    Struct de timer - Marco Aurélio (08/09/2024)                                        |
|                                                                                        |
|    Catei esse struct de timer do blog do Arduino                                       |
|    Parece ser bem útil para vários código que precisem ser ativados com o tempo,       |
|    mas que seria bom deixar ele compacto                                               |
|                                                                                        |
|    Créditos para LarryD                                                                |
|    https://forum.arduino.cc/t/switch-and-led-on-the-same-pin-suggestions-fyi/423048    |
+---------------------------------------------------------------------------------------*/

struct timer{
  
    unsigned long lastMillis;     // Ultima vez que o timer foi reiniciado
    unsigned long waitMillis;     // Tempo que o timer vai esperar
    bool          restart;        // Define se o timer deve reiniciar repetidamente ou não (0 - Rodar uma unica vez | 1 - Rodar indefinidamente)
    bool          enableFlag;     // Define se o timer esta ativo ou não
    bool          timeType;       // Define qual tipo de tempo o timer usa (0 - Microssegundos | 1 - Millisegundos)
  
    unsigned long currentTime;
  
    // Função que retorna se o timer alcançou o tempo definido ou não
    bool CheckTime(){
        // Atualiza o tempo atual
        if (timeType == true){
          currentTime = millis();
        } else {
          currentTime = micros();
        }
  
        //Se o timer estiver ativo e tiver estourado
        if (enableFlag == true && currentTime - lastMillis >= waitMillis){
            // Reiniciar?
            if (restart){
                // Preparar para proxima iteração
                lastMillis = currentTime;
            }
            // O timer estorou
            return true;
        }
        //O timer não estorou
        return false;
    }
  
};

#endif
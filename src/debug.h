#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// UNCOMMENT THIS FOR DEBUGGING
// #define DEBUG

#ifndef DEBUG
    #define PRINTLN(x) Serial.println(x)
    #define PRINT(x) Serial.print(x)
#else
    #define PRINTLN(x)
    #define PRINT(x)
#endif

#endif
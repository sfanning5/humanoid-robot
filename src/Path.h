#ifndef PATH_H
#define PATH_H

#include "debug.h"

// #define PATH_LENGTH 7
#define PATH_END 255

#define MIN_MICROS 650
#define MAX_MICROS 2250
#define MAX_ANGLE 160

#define PATH_LENGTH 8

// Controls the path of a given link and performs calculations relevant to that link
class Path 
{
    public:
        Path();
        void addMicros(int16_t micros);
        void addAngle(float angle);
        int16_t getFrame(uint8_t index);
        void setFrame(uint8_t index, int16_t micros);
        void clear();
        uint8_t size();

    // private:
        uint8_t pathSize;
        int16_t path[PATH_LENGTH+1];
};

#endif
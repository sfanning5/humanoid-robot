#include "Path.h"

Path::Path()
{
    clear(); // Initialize as empty path
}

void Path::addMicros(int16_t micros)
{
    path[pathSize] = micros;
    pathSize++;
}

void Path::addAngle(float angle)
{
    int16_t micros = (angle/MAX_ANGLE) * (MAX_MICROS-MIN_MICROS);
    addMicros(micros);
}

void Path::setFrame(uint8_t index, int16_t micros)
{
    path[index] = micros;
}

int16_t Path::getFrame(uint8_t index)
{
    if(index >= pathSize)
        return path[pathSize-1];
    
    return path[index];
}

void Path::clear()
{
    // Sets the path to one frame with value 0
    pathSize = 1;
    path[0] = 0;
}

uint8_t Path::size()
{
    return pathSize;
}



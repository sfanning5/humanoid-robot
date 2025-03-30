#include <Arduino.h>
#include "Joint.h"

#define ACC_MODIFIER 1.18

Joint::Joint(const JointData& jd, Path& p) : data(jd), path(p)
{
    writeMicros(0); // Writes to the home position

    #ifndef DEBUG
        servo.attach(data.pin);
    #endif
}

void Joint::update(uint8_t frame, float progress)
{
    // LERP angle
    int currentFrameMicros = path.getFrame(frame);
    int nextFrameMicros = path.getFrame(frame + 1);
    int micros = currentFrameMicros + (nextFrameMicros - currentFrameMicros) * progress;
    writeMicros(micros);
}

void Joint::writeAngle(float angle)
{
    int16_t micros = ((2150 - 730) / (150 - 30)) * angle;
    writeMicros(micros);
}

// Write the angle of the servo in microseconds, with 0 being the home position
void Joint::writeMicros(int16_t micros)
{
    currentMicros = micros;

    micros = data.homeMicros + micros * data.correctionModifier;// * ACC_MODIFIER; // TODO: refactor this calculation
    // PRINTLN(micros);
    if(micros != servo.readMicroseconds())
        servo.writeMicroseconds(micros);
}

int16_t Joint::getCurrentMicros()
{
    return currentMicros;
}
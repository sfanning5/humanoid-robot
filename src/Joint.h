#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include <Servo.h>
#include "avr8-stub.h"
#include "debug.h"
#include "Path.h"

#define PATH_LENGTH 7

struct JointData
{
    int homeMicros;
    uint8_t pin;
    float correctionModifier;
};

class Joint
{
public:
    Joint(const JointData& d, Path& p);
    void update(uint8_t frame, float progress);
    void writeMicros(int16_t micros);
    void writeAngle(float angle);
    int16_t getCurrentMicros();

// private:
    JointData data;
    Path& path;
    Servo servo;
    int16_t currentMicros;
};

#endif // JOINT_H

#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include <Servo.h>
#include "avr8-stub.h"
#include "debug.h"
#include <Arduino_AVRSTL.h>
#include <array>

using namespace std;

#define PATH_LENGTH 7
#define PATH_END 255

struct JointData
{
    int homeMicros;
    uint8_t pin;
    float correctionModifier;
};

class Joint
{
public:
    // Constructor
    Joint(const JointData& jd);
    void attach();
    void setPath(const array<int, PATH_LENGTH>& newPath);
    void update(uint8_t frame, float progress);
    void writeAngle(float angle); 
    void writeMicros(int micros); 
 
    Servo servo;
    float angle;
    array<int, PATH_LENGTH+1> path;
    JointData data;
    bool pathSet = false;
    uint8_t finalFrame = PATH_LENGTH - 1;
};

#endif // JOINT_H

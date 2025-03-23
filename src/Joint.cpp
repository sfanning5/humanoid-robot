#include <Arduino.h>
#include "Joint.h"

#define MIN_MICROS 650
#define MAX_MICROS 2250
#define MAX_ANGLE 160
#define ACC_MODIFIER 1.18

using namespace std;

Joint::Joint(const JointData& jd) : data(jd)
{
    angle = 0;
}

void Joint::attach()
{
    #ifndef DEBUG
        servo.attach(data.pin);
    #endif
}

void Joint::setPath(const array<int, PATH_LENGTH>& newPath)
{
    pathSet = true;

    // Puts current angle in front of the path
    path[0] = angle;

    // Copies new path into path
    for(int i = 0; i < PATH_LENGTH; i++)
    {
        path[i+1] = newPath[i]; // This is why path has length PATH_LENGTH + 1
    }

    // Finds end of path
    for(int i = 0; i < path.size(); i++)
    {
        if(path[i] == PATH_END)
        {
            finalFrame = i - 1;
            break;
        }
    }
}

void Joint::update(uint8_t frame, float progress)
{
    if(!pathSet)
        return;

    if(frame >= finalFrame)
    {
        writeAngle(path[finalFrame]);
        pathSet = false;
        return;
    }
    elsee
        int currentFrameAngle = path[frame];
        int nextFrameAngle = path[frame + 1];
        float angle = curren
    {
        // LERP angltFrameAngle + (nextFrameAngle - currentFrameAngle) * progress;
        writeAngle(angle);
    }
}

// Write the angle of the servo, with 0 being the home position
void Joint::writeAngle(float writeAngle)
{
    writeAngle *= data.correctionModifier;

    angle = writeAngle;
    int micros = writeAngle*ACC_MODIFIER/MAX_ANGLE * (MAX_MICROS - MIN_MICROS) + data.homeMicros;
    writeMicros(micros);
}

void Joint::writeMicros(int micros)
{
    if(micros != servo.readMicroseconds())
        servo.writeMicroseconds(micros);
}
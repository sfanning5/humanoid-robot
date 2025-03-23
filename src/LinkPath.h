#ifndef LINK_PATH_H
#define LINK_PATH_H

#include "Joint.h"
#include "avr8-stub.h"
#include "debug.h"

#define LINK_LENGTH 3

struct LinkData 
{
    uint8_t jointCount;
    uint8_t jointIDs[LINK_LENGTH];
    float lengths[LINK_LENGTH];
    float COMs[LINK_LENGTH];
    float masses[LINK_LENGTH];
    int8_t sideModifier; // 1 or -1, depending on the side
};

struct Point
{
    float x;
    float y;
};


// Controls the path of a given link and performs calculations relevant to that link
class LinkPath 
{
    public: 
        LinkPath(const LinkData& data, Joint** jointsPtr);
        float calculateCOMs();
        void setFixedPath(const array<array<int, PATH_LENGTH>, LINK_LENGTH>& targets, int* times);
        void setKinematicPath(Point* targets, int* times);
        void setBalancingPath(int* liftedLegAngles, float* LiftedLegCOMYs, int* times);
        void update(uint32_t time);
        uint8_t getFrameNumber(uint32_t time);
        float getFrameProgess(uint8_t frame, uint32_t time);

    private:
        Joint* getJoint(int idx);
        LinkData linkData;
        Joint** joints;
        int* frameTimes;
};

#endif
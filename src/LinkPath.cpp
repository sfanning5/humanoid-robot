#include "LinkPath.h"
#include <math.h>

// #define DEBUG

LinkPath::LinkPath(const LinkData& data, Joint** jointsPtr) : linkData(data), joints(jointsPtr)
{
}

Joint* LinkPath::getJoint(int idx)
{
    return joints[linkData.jointIDs[idx]];
}


extern int __heap_start, *__brkval;
int freeMemory() {
    int v;
    return (int) &v - ((int)__brkval);
}

void LinkPath::setKinematicPath(Point* targets, int* times)
{
    array<int, PATH_LENGTH> path1;
    array<int, PATH_LENGTH> path2;
    array<int, PATH_LENGTH> path3;

    float L1 = linkData.lengths[0];
    float L2 = linkData.lengths[1] + linkData.lengths[2];

    int i;

    for(i = 0; targets[i].x != PATH_END; i++)
    {
        float x = targets[i].x;
        float y = targets[i].y;

        // Calculate inverse kinematics
        float theta1 = -acos( (sq(L2)-sq(L1)-sq(x)-sq(y)) / (-2*L1*sqrt(sq(x) + sq(y))) ) - atan2(-x, -y);
        float theta2 = PI - acos( (sq(x)+sq(y)-sq(L1)-sq(L2)) / (-2*L1*L2) );

        // Convert to degrees and ensure correct direction
        path1[i] = (int)((float)theta1 * 180.0/PI * linkData.sideModifier); 
        path2[i] = (int)((float)theta2 * 180.0/PI * linkData.sideModifier);
        path3[i] = (int)((float)(theta1 + theta2) * 180.0/PI * linkData.sideModifier);
        PRINTLN(freeMemory());
    }

    path1[i] = PATH_END;
    path2[i] = PATH_END;
    path3[i] = PATH_END;

    getJoint(0)->setPath(path1);
    getJoint(1)->setPath(path2);
    getJoint(2)->setPath(path3);
    frameTimes = times;
}

void LinkPath::setBalancingPath(int* liftedLegAngles, float* liftedLegCOMYs, int* times)
{
    const float D = -0.2;
    const float FL = 0.33;
    const float FT = 0.41;
    const float LH = 1.25;
    const float LW = 0.2;
    const float L1 = 2.43;
    const float L3 = 4.65;

    float A = L3 - (FL*L1)/(2*FL+FT);
    float B = (FL*LW)/(2*FL+FT);

    array<int, PATH_LENGTH> path1;
    array<int, PATH_LENGTH> path2;

    int i;
    for(i = 0; liftedLegAngles[i] != PATH_END; i++)
    {
        if(liftedLegAngles[i] == 0 && liftedLegCOMYs[i] == 0)
        {
            path1[i] = 0;
            path2[i] = 0;
            continue;
        }

        float L2 = liftedLegCOMYs[i];
        float b = (float)liftedLegAngles[i] * PI / 180.0;

        float a = asin( (FL*(LH+L2*sin(b)+LW*cos(b)) + 0.5*LH*FT - D*(2.0*FL+FT)) / ((2.0*FL+FT)*(sqrt(sq(A) + sq(B)))) ) - atan2(B, A);
        a *= 180.0 / PI * linkData.sideModifier;

        path1[i] = a;
        path2[i] = a;

        path2[i] += (6.5 + 3 * (float)liftedLegAngles[i]/30) * linkData.sideModifier;
    }

    // TODO abstract this
    path1[i] = PATH_END;
    path2[i] = PATH_END;
    
    getJoint(0)->setPath(path1);
    getJoint(1)->setPath(path2);
    frameTimes = times;
}

void LinkPath::setFixedPath(const array<array<int, PATH_LENGTH>, LINK_LENGTH>& targets, int* times)
{
    frameTimes = times;
    for(int i = 0; i < linkData.jointCount; i++)
    {
        getJoint(i)->setPath(targets[i]);
    }
}

void LinkPath::update(uint32_t time)
{
    uint8_t frame = getFrameNumber(time);
    float progress = getFrameProgess(frame, time);
    // PRINTLN(frame);
    // PRINTLN(progress);
    // getJoint(0)->update(frame, progress);

    for(int i = 0; i < linkData.jointCount; i++)
    {
        getJoint(i)->update(frame, progress);
    }
}

uint8_t LinkPath::getFrameNumber(uint32_t time)
{
    for(int i = 0; i < PATH_LENGTH - 1; i++)
    {
        if(frameTimes[i+1] > time)
            return i;
    }

    return PATH_LENGTH - 1;
}

float LinkPath::getFrameProgess(uint8_t frame, uint32_t time)
{
    if(frame == PATH_LENGTH - 1)
        return 0;

    return (float)(time - frameTimes[frame]) / (frameTimes[frame + 1] - frameTimes[frame]);
}
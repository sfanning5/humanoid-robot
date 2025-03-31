#include "Link.h"
#include <math.h>

Link::Link(const LinkData &data, const JointData jointData[]) : linkData(data), frameTimes(nullptr)
{
    // Initializes the necessary amount of joints and paths
    paths = new Path[linkData.jointCount];
    joints = new Joint*[linkData.jointCount];

    for(int i = 0; i < linkData.jointCount; i++)
    {
        const uint8_t& jointID = linkData.jointIDs[i];
        joints[i] = new Joint(jointData[jointID], paths[i]);
    }
}

extern int __heap_start, *__brkval;
int freeMemory3() {
    int v;
    return (int) &v - ((int)__brkval);
}

void Link::setKinematicPath(Point targets[], int times[])
{
    resetPaths(times);

    // Calculates new paths
    float L1 = linkData.lengths[0];
    float L2 = linkData.lengths[1] + linkData.lengths[2];

    for(int i = 0; targets[i].x != PATH_END; i++)
    {
        float& x = targets[i].x;
        float& y = targets[i].y;

        // Calculate inverse kinematics
        float theta1 = -acos( (sq(L2)-sq(L1)-sq(x)-sq(y)) / (-2*L1*sqrt(sq(x) + sq(y))) ) - atan2(-x, -y);
        float theta2 = PI - acos( (sq(x)+sq(y)-sq(L1)-sq(L2)) / (-2*L1*L2) );

        // PRINTLN(freeMemory3());

        // Convert to degrees and ensure correct direction
        // TODO check what casting here is necessary
        // TODO refactor direction modifiers
        paths[0].addAngle( theta1 * 180.0/PI * linkData.sideModifier );
        paths[1].addAngle( theta2 * 180.0 / PI * linkData.sideModifier );
        paths[2].addAngle( (theta1 + theta2) * 180.0/PI * linkData.sideModifier );

        // PRINTLN(freeMemory3());
    }
}

void Link::setBalancingPath(int liftedLegAngles[], float liftedLegCOMYs[], int times[])
{
    resetPaths(times);

    const float D = -0.2;
    const float FL = 0.33;
    const float FT = 0.41;
    const float LH = 1.25;
    const float LW = 0.2;
    const float L1 = 2.43;
    const float L3 = 4.65;

    float A = L3 - (FL*L1)/(2*FL+FT);
    float B = (FL*LW)/(2*FL+FT);

    int i;
    for(i = 0; liftedLegAngles[i] != PATH_END; i++)
    {
        if(liftedLegCOMYs[i] < 0)
        {
            paths[0].addAngle(liftedLegAngles[i] * linkData.sideModifier);
            paths[1].addAngle(liftedLegAngles[i] * linkData.sideModifier);
            continue;
        }

        float L2 = liftedLegCOMYs[i];
        float b = (float)liftedLegAngles[i] * PI / 180.0;

        float a = asin( (FL*(LH+L2*sin(b)+LW*cos(b)) + 0.5*LH*FT - D*(2.0*FL+FT)) / ((2.0*FL+FT)*(sqrt(sq(A) + sq(B)))) ) - atan2(B, A);
        a *= 180.0 / PI * linkData.sideModifier;
        // PRINTLN(freeMemory3());

        paths[0].addAngle(a);
        paths[1].addAngle(a + (3.5 + 3 * (float)liftedLegAngles[i] / 30) * linkData.sideModifier);
        // PRINT("Angle: ");
        // PRINTLN(a);
        // PRINTLN(freeMemory3());
    }

}

void Link::setFixedPath(const int targets[][PATH_LENGTH], int times[])
{
    resetPaths(times);
    for(int i = 0; i < linkData.jointCount; i++)
    {
        for(int j = 0; j < PATH_LENGTH; j++)
        {
            if(targets[i][j] != PATH_END)
                paths[i].addAngle(targets[i][j] * linkData.sideModifier);
            else
                break;
        }
    }
}

// Clears current paths to prepare for new ones to be calculated
void Link::resetPaths(int newTimes[])
{
    frameTimes = newTimes;

    // Clears current paths
    for (int i = 0; i < linkData.jointCount; i++)
    {
        paths[i].clear();
    }
}

void Link::update(uint32_t time)
{
    uint8_t frame = getFrameNumber(time);
    float progress = getFrameProgess(frame, time);
    // PRINTLN(frame);
    // PRINTLN(progress);
    // getJoint(1)->update(frame, progress);

    complete = true;
    for(int i = 0; i < linkData.jointCount; i++)
    {
        // PRINT(frame);
        // PRINT(", ");
        // PRINTLN(paths[i].size());
        joints[i]->update(frame, progress);
        if(frame < paths[i].size()-1)
            complete = false;
    }
}

uint8_t Link::getFrameNumber(uint32_t time)
{
    if(!frameTimes)
        return PATH_LENGTH - 1;

    for(int i = 0; i < PATH_LENGTH - 1; i++)
    {
        if(frameTimes[i+1] > time)
            return i;
    }

    return PATH_LENGTH - 1;
}

float Link::getFrameProgess(uint8_t frame, uint32_t time)
{
    if(frame == PATH_LENGTH - 1)
        return 0;

    return (float)(time - frameTimes[frame]) / (frameTimes[frame + 1] - frameTimes[frame]);
}

void Link::startMotion()
{
    complete = false;

    for(int i = 0; i < linkData.jointCount; i++)
    {
        paths[i].setFrame(0, joints[i]->getCurrentMicros());
    }
}

bool Link::motionComplete()
{
    return complete;
}
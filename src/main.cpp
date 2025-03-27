#include "Arduino.h"
#include <Servo.h>
#include "Link.h"
#include "avr8-stub.h"
#include "app_api.h"
#include "debug.h"

// TODO: write custom servo library to allow 13+ servos
#define JOINT_COUNT 11
#define LINK_COUNT 5

#define LEFT_ANKLE           0
#define LEFT_ANKLE_ROTATOR   1
#define LEFT_KNEE            2
#define LEFT_HIP             3
#define LEFT_HIP_ROTATOR     4
// LEFT_SHOULDER
#define RIGHT_ANKLE          5
#define RIGHT_ANKLE_ROTATOR  6
#define RIGHT_KNEE           7
#define RIGHT_HIP            8
#define RIGHT_HIP_ROTATOR    9
// RIGHT_SHOULDER
#define TORSO                10

#define LEFT_SIDE 1
#define RIGHT_SIDE -1
#define FRONT_RIGHT_SIDE 1
#define FRONT_LEFT_SIDE -1

// #define DEBUG

Link* links[LINK_COUNT];
const LinkData linkData[LINK_COUNT] = 
{ 
// #  Joint IDs                                 Link Lengths          COM distance  Mass       Side Modifier
  {3, {LEFT_HIP, LEFT_KNEE, LEFT_ANKLE},        {1.69, 1.81, 0.995},  {0, 0, 0},    {0, 0, 0}, LEFT_SIDE  },
  {2, {LEFT_HIP_ROTATOR, LEFT_ANKLE_ROTATOR},   {0, 0, 0},            {0, 0, 0},    {0, 0, 0}, FRONT_LEFT_SIDE },
  {3, {RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE},     {1.69, 1.81, 0.995},  {0, 0, 0},    {0, 0, 0}, RIGHT_SIDE },
  {2, {RIGHT_HIP_ROTATOR, RIGHT_ANKLE_ROTATOR}, {0, 0, 0},            {0, 0, 0},    {0, 0, 0}, FRONT_RIGHT_SIDE },
  {1, {TORSO},                                  {0, 0, 0},            {0, 0, 0},    {0, 0, 0}, 1 }
};

const JointData jointData[JOINT_COUNT] =
{
    {1160, 8, 0.9},   // "Left Ankle"
    {1200, 11, 0.85}, // "Left Ankle Rotator"
    {1100, 13, 0.95}, // "Left Knee"
    {1300, 9, 1},     // "Left Hip"
    {1385, 2, 0.85},  // "Left Hip Rotator"
    // {1500, A0}, // "Left Shoulder"

    {1150, 6, 0.85},  // "Right Ankle"
    {1390, 12, 1},    // "Right Ankle Rotator"
    {1960, 10, 0.95}, // "Right Knee"
    {1590, 7, 0.85},  // "Right Hip"
    {1540, 3, 0.82},  // "Right Hip Rotator"
    // {1700, 5}, // "Right Shoulder"

    {1550, 4, 1} // "Torso"
};

int EVEN_SPACED_TIMES[PATH_LENGTH] = {0, 1000, 2000, 3000, 4000, 5000};

uint32_t startTime = 0;

enum State
{
  FIRST_STEP,
  RIGHT_STEP,
  LEFT_STEP,
  STANDING
};

State state = STANDING;

unsigned long fakeMillis = 0;

unsigned long time()
{
  #ifndef DEBUG
    return millis();
  #else
    return fakeMillis;
  #endif
}

void startMotion()
{
  for(Link* link : links)
    link->startMotion();

  startTime = time();
}

// ### FIRST STEP #### //

int FS_rightFrontAngles[PATH_LENGTH] = {8, 10, 18, 18, PATH_END};
float FS_rightFrontCOMYs[PATH_LENGTH] = {1.4, 1.4, 1.4, 1.4, PATH_END};

const int FS_LEFT_FRONT[LINK_LENGTH][PATH_LENGTH] = {
  {10, 10, 18, 18, PATH_END},
  {10, 10, 18, 18, PATH_END}
};

Point FS_rightSide[] = {
  {0, -4.3},
  {0.5, -3.85},
  {0.8, -3.85},
  {1.5, -3.75},
  {PATH_END, 0}
};


Point FS_leftSide[] = {
  {0, -4.49},
  {-0.2, -3.7},
  {-1.5, -3.7},
  {-0.6, -3.9},
  {PATH_END, 0}
};

int FS_TIMES[PATH_LENGTH] = {0, 1000, 1800, 2200, 3000, 5000};

const int STEPS = 8;
int stepCounter = 0;

void startFirstStep()
{
  links[0]->setKinematicPath(FS_leftSide, FS_TIMES);
  links[1]->setFixedPath(FS_LEFT_FRONT, FS_TIMES);
  links[2]->setKinematicPath(FS_rightSide, FS_TIMES);
  links[3]->setBalancingPath(FS_rightFrontAngles, FS_rightFrontCOMYs, FS_TIMES);

  state = FIRST_STEP;
  startMotion();
}

// ### RIGHT STEP #### //

// TODO refactor data types (int -> int16_t, etc.)
int RS_leftFrontAngles[PATH_LENGTH] = {15, 15, 15, 15, PATH_END};
float RS_leftFrontCOMYs[PATH_LENGTH] = {1.4, 1.4, 1.4, 1.4, PATH_END};

const int RS_RIGHT_FRONT[LINK_LENGTH][PATH_LENGTH] = {
  {-7, -10, -15, -15, PATH_END},
  {-7, -10, -15, -15, PATH_END}
};

Point RS_rightSide[] = {
  {2, -4},
  {1.5, -3.2},
  {-0.65, -3.75},
  {0.1, -3.7},
  {PATH_END, 0}
};

Point RS_leftSide[] = {
  {0.5, -3.85},
  {0.8, -3.85},
  {0.8, -3.85},
  {1.5, -3.6},
  {PATH_END, 0}
};

const int f1 = 600;
const int f2 = 400 + f1;
const int f3 = 400 + f2;
const int f4 = 200 + f3;
const int f5 = 100 + f4;
const int f6 = 100 + f5;

int STEP_TIMES[PATH_LENGTH] = {0, f1, f2, f3, f4, f5, f6};

void startRightStep()
{
  links[0]->setKinematicPath(RS_leftSide, STEP_TIMES);
  links[1]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);
  links[2]->setKinematicPath(RS_rightSide, STEP_TIMES);
  links[3]->setFixedPath(RS_RIGHT_FRONT, STEP_TIMES);

  // links[0]->getJoint(0)->path[2] -= 4;

  state = RIGHT_STEP;
  startMotion();
}

const int LS_RIGHT_FRONT[LINK_LENGTH][PATH_LENGTH] = {
  {7, 10, 15, 15, PATH_END},
  {7, 10, 15, 15, PATH_END}
};

void startLeftStep()
{
  links[0]->setKinematicPath(RS_rightSide, STEP_TIMES);
  links[1]->setFixedPath(LS_RIGHT_FRONT, STEP_TIMES);
  links[2]->setKinematicPath(RS_leftSide, STEP_TIMES);
  links[3]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);

  state = LEFT_STEP;
  startMotion();
}

extern int __heap_start, *__brkval;
int freeMemory() {
  int v;
  return (int) &v - ((int)__brkval);
}

const int TEST_PATH[LINK_LENGTH][PATH_LENGTH] = {
    {-45, 0, PATH_END},
    {30, 0, PATH_END},
    {-30, -30, 0, PATH_END},
};

Point TEST_POINTS[] = {
    {0, -4.49},
    {-0.2, -3.7},
    {0, -4.49},
    {PATH_END, 0}
};

void setup() 
{
  #ifdef DEBUG
    debug_init();
    breakpoint();
  #else
    Serial.begin(9600);
    Serial.println("Begin");
  #endif

  // Initializes Links
  for(int i = 0; i < LINK_COUNT; i++)
  {
    links[i] = new Link(linkData[i], jointData);
  }

  delay(2000);

  PRINTLN(freeMemory());

  startFirstStep();
  // startRightStep();
  // startLeftStep();
  
  PRINTLN(freeMemory());

  // joints[LEFT_HIP_ROTATOR]->writeAngle(45);
  // joints[RIGHT_HIP_ROTATOR]->writeAngle(45);

  // links[3]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);
  // links[1]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);

  startTime = time();
}

void loop() 
{
  return;
  bool nextMove = true;
  for(Link* link : links)
  {
    link->update(time() - startTime);
    if(!link->motionComplete())
      nextMove = false;
  }

  if(nextMove && stepCounter > STEPS)
    state = STANDING;
  else if(nextMove && (state == FIRST_STEP || state == LEFT_STEP))
    startRightStep();
  else if(nextMove && state == RIGHT_STEP)
    startLeftStep();

  fakeMillis += 500;
}

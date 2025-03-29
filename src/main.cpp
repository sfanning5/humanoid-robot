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
    {1230, 11, 0.85}, // "Left Ankle Rotator"
    {1100, 13, 1}, // "Left Knee"
    {1300, 9, 1},     // "Left Hip"
    {1390, 2, 0.85},  // "Left Hip Rotator"
    // {1500, A0}, // "Left Shoulder"

    {1150, 6, 0.85},  // "Right Ankle"
    {1370, 12, 1},    // "Right Ankle Rotator"
    {1960, 10, 0.95}, // "Right Knee"
    {1590, 7, 0.95},  // "Right Hip"
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

int FS_TIMES[PATH_LENGTH] = {0, 600, 1000, 1300, 1700, 2100};

int FS_rightFrontAngles[PATH_LENGTH] = {6, 6, 0, -10, PATH_END};
float FS_rightFrontCOMYs[PATH_LENGTH] = {1.4, 1.4, 1.4, 1.4, PATH_END};

const int FS_LEFT_FRONT[LINK_LENGTH][PATH_LENGTH] = {
  {4, 6, 0, -10, PATH_END},
  {4, 6, 0, -10, PATH_END}
};

Point FS_rightSide[] = {
    {0, -4.35},
    {0.2, -4.1},
    {0.5, -4.1},
    {1.2, -4.1},
    {1.5, -4.1},
    {PATH_END, 0}};

Point FS_leftSide[] = {
    {0, -4.39},
    {-0.9, -3.8},
    {-1.3, -3.8},
    {-1.1, -4.1},
    {-0.2, -4.1},
    {PATH_END, 0}};

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
int RS_leftFrontAngles[PATH_LENGTH] = {10, 10, 10, -8, PATH_END};
float RS_leftFrontCOMYs[PATH_LENGTH] = {1.4, 1.4, 1.4, 1.4, PATH_END};

const int RS_RIGHT_FRONT[LINK_LENGTH][PATH_LENGTH] = {
    {-10, -10, -10, 8, PATH_END},
    {-10, -10, -10, 6, PATH_END}};

Point RS_rightSide[] = {
    {1.8, -4.1},
    {1, -3.5},
    {-1.7, -3.5},
    {-1.9, -3.5},
    {-0.8, -4.1},
    {0.1, -4.1},
    // {-1.0, -4.0},
    {PATH_END, 0}};

Point RS_leftSide[] = {
    {0.4, -4.1},
    {0.5, -4.0},
    {0.6, -4.1},
    {0.7, -4.1},
    {1.3, -4.0},
    {1.8, -4.0},
    // {1.4, -4.1},
    {PATH_END, 0}};

const int f1 = 400;
const int f2 = 400 + f1;
const int f3 = 400 + f2;
const int f4 = 400 + f3;
const int f5 = 400 + f4;
const int f6 = 400 + f5;

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

const int LS_LEFT_FRONT[LINK_LENGTH][PATH_LENGTH] = {
  {10, 10, 10, -8, PATH_END},
  {10, 10, 10, -6, PATH_END}
};

void startLeftStep()
{
  links[0]->setKinematicPath(RS_rightSide, STEP_TIMES);
  links[1]->setFixedPath(LS_LEFT_FRONT, STEP_TIMES);
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
    {0, PATH_END},
    {60, PATH_END},
    {0, PATH_END},
};

const int TEST_PATH2[LINK_LENGTH][PATH_LENGTH] = {
    {0, PATH_END},
    {-60, PATH_END},
    {0, PATH_END},
};

Point TEST_POINTS[] = {
    {0, -4.49},
    {0.1, -4.1},
    {PATH_END, 0}};

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

  // startFirstStep();
  // startRightStep();
  // startLeftStep();
  
  PRINTLN(freeMemory());

  // links[3]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);
  // links[1]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);

  // links[0]->setKinematicPath(TEST_POINTS, EVEN_SPACED_TIMES);
  // links[2]->setKinematicPath(TEST_POINTS, EVEN_SPACED_TIMES);

  // links[0]->setFixedPath(TEST_PATH, EVEN_SPACED_TIMES);
  // links[2]->setFixedPath(TEST_PATH2, EVEN_SPACED_TIMES);

  // links[0]->joints[2]->servo.write(120);
  // links[0]->joints[2]->servo.writeMicroseconds(2400);

  // Left ankle
  // 90 -> 1500
  // 170 -> 2400
  // 130 -> 1950
  // 125 -> 1050
  // 165 -> 600

  // 30 -> 730
  // 60 -> 1100
  // 90 -> 1500
  // 120 -> 1830
  // 150 -> 2150

  startTime = time();
}

void loop()
{
  // links[0]->joints[1]->servo.writeMicroseconds(2220);
  return;
  bool nextMove = true;
  for(Link* link : links)
  {
    // link->update(time() - startTime);
    link->update(f6);
    if(!link->motionComplete())
      nextMove = false;
  }

  // if(nextMove && stepCounter > STEPS)
  //   state = STANDING;
  // if(nextMove && (state == FIRST_STEP || state == LEFT_STEP))
  //   startRightStep();
  // else if(nextMove && state == RIGHT_STEP)
  //   startLeftStep();

  fakeMillis += 500;
}

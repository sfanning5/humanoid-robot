#include "Arduino.h"
#include <Servo.h>
#include "Joint.h"
#include "LinkPath.h"
#include "avr8-stub.h"
#include "app_api.h"
#include "debug.h"

// TODO: write custom servo library to allow 13+ servos
#define JOINT_COUNT 11
#define LINK_COUNT 4

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

LinkPath* links[LINK_COUNT];
LinkData linkData[LINK_COUNT] = 
{ 
// #  Joint IDs                                 Link Lengths          COM distance  Mass       Side Modifier
  {3, {LEFT_HIP, LEFT_KNEE, LEFT_ANKLE},        {1.69, 1.81, 0.995},  {0, 0, 0},    {0, 0, 0}, LEFT_SIDE  },
  {2, {LEFT_HIP_ROTATOR, LEFT_ANKLE_ROTATOR},   {0, 0, 0},            {0, 0, 0},    {0, 0, 0}, FRONT_LEFT_SIDE },
  {3, {RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE},     {1.69, 1.81, 0.995},  {0, 0, 0},    {0, 0, 0}, RIGHT_SIDE },
  {2, {RIGHT_HIP_ROTATOR, RIGHT_ANKLE_ROTATOR}, {0, 0, 0},            {0, 0, 0},    {0, 0, 0}, FRONT_RIGHT_SIDE },
};

Joint* joints[JOINT_COUNT];
JointData jointData[JOINT_COUNT] = 
{
  {1180, 8,  0.9},// "Left Ankle"           
  {1340, 11, 0.85}, // "Left Ankle Rotator"   
  {1100, 13, 0.95}, // "Left Knee"            
  {1300, 9,  1},// "Left Hip"             
  {1350, 2,  0.9},// "Left Hip Rotator"     
  // {1500, A0}, // "Left Shoulder"        

  {1150, 6,  0.85},// "Right Ankle"          
  {1210, 12, 1}, // "Right Ankle Rotator"  
  {1960, 10, 0.95}, // "Right Knee"           
  {1590, 7,  0.85},// "Right Hip"            
  {1570, 3,  0.97},// "Right Hip Rotator"    
  // {1700, 5}, // "Right Shoulder"       

  {1550, 4,  1}   // "Torso"
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

// ### FIRST STEP #### //

int FS_rightFrontAngles[PATH_LENGTH] = {8, 10, 18, 18, PATH_END};
float FS_rightFrontCOMYs[PATH_LENGTH] = {1.4, 1.4, 1.4, 1.4, PATH_END};

const array<array<int, PATH_LENGTH>, LINK_LENGTH> FS_LEFT_FRONT = {
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

void startFirstStep()
{
  links[0]->setKinematicPath(FS_leftSide, FS_TIMES);
  links[1]->setFixedPath(FS_LEFT_FRONT, FS_TIMES);
  links[2]->setKinematicPath(FS_rightSide, FS_TIMES);
  links[3]->setBalancingPath(FS_rightFrontAngles, FS_rightFrontCOMYs, FS_TIMES);

  state = FIRST_STEP;
  startTime = millis();
}

// ### RIGHT STEP #### //

int RS_leftFrontAngles[PATH_LENGTH] = {15, 15, 15, 15, PATH_END};
float RS_leftFrontCOMYs[PATH_LENGTH] = {1.4, 1.4, 1.4, 1.4, PATH_END};

const array<array<int, PATH_LENGTH>, LINK_LENGTH> RS_RIGHT_FRONT = {
  {-7, -15, -15, -15, PATH_END},
  {-7, -15, -15, -15, PATH_END}
};

Point RS_rightSide[] = {
  {2, -4},
  {1.5, -3.2},
  {-0.85, -3.75},
  {-0.1, -4.05},
  {PATH_END, 0}
};

Point RS_leftSide[] = {
  {0, -3.85},
  {1, -3.85},
  {0.8, -3.85},
  {1.5, -3.6},
  {PATH_END, 0}
};

int STEP_TIMES[PATH_LENGTH] = {0, 1200, 1900, 2300, 2700, 3700, 4700};

void startRightStep()
{
  links[0]->setKinematicPath(RS_leftSide, STEP_TIMES);
  links[1]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);
  links[2]->setKinematicPath(RS_rightSide, STEP_TIMES);
  links[3]->setFixedPath(RS_RIGHT_FRONT, STEP_TIMES);

  state = RIGHT_STEP;
  startTime = millis();
}

const array<array<int, PATH_LENGTH>, LINK_LENGTH> LS_RIGHT_FRONT = {
  {7, 15, 15, 15, PATH_END},
  {7, 15, 15, 15, PATH_END}
};

void startLeftStep()
{
  links[0]->setKinematicPath(RS_rightSide, STEP_TIMES);
  links[1]->setFixedPath(LS_RIGHT_FRONT, STEP_TIMES);
  links[2]->setKinematicPath(RS_leftSide, STEP_TIMES);
  links[3]->setBalancingPath(RS_leftFrontAngles, RS_leftFrontCOMYs, STEP_TIMES);

  state = LEFT_STEP;
  startTime = millis();
}

extern int __heap_start, *__brkval;
int freeMemory() {
  int v;
  return (int) &v - ((int)__brkval);
}

void setup() 
{
  #ifdef DEBUG
    debug_init();
    breakpoint();
  #else
    Serial.begin(9600);
    Serial.println("Begin");
  #endif

  // Initializes all joints
  for(int i = 0; i < JOINT_COUNT; i++)
  {
    joints[i] = new Joint(jointData[i]);
  }

  for(Joint* joint : joints) // Homes all joints
  {
    joint->writeMicros(joint->data.homeMicros);
  }

  // Initializes Links
  for(int i = 0; i < LINK_COUNT; i++)
  {
    links[i] = new LinkPath(linkData[i], joints);
  }

  for(Joint* joint : joints)
  {
    joint->attach();
  }

  delay(2000);

  PRINTLN(freeMemory());

  startFirstStep();
  
  PRINTLN(freeMemory());

}

void loop() 
{
  return;
  bool nextMove = true;
  for(LinkPath* link : links)
  {
    link->update(millis() - startTime);
  }

  // Checks if next move has arrived
  for(Joint* joint : joints)
  {
    if(joint->pathSet) // Checks if all joints have finished their current path
      nextMove = false;
  }

  if(nextMove && (state == FIRST_STEP || state == LEFT_STEP))
    startRightStep();
  else if(nextMove && state == RIGHT_STEP)
    startLeftStep();

  delay(1);
}

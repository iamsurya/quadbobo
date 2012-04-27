#ifndef _AQ_KINEMATICS_
#define _AQ_KINEMATICS_

#include "GlobalDefined.h"

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

// This class is responsible for calculating vehicle attitude
byte kinematicsType = 0;
float kinematicsAngle[3] = {0.0,0.0,0.0};
float gyroAngle[2] = {0.0,0.0};
float correctedRateVector[3] = {0.0,0.0,0.0};
float earthAccel[3] = {0.0,0.0,0.0};

float accelCutoff = 0.0;

void initializeBaseKinematicsParam(float hdgX, float hdgY) {
  for (byte axis = XAXIS; axis <= ZAXIS; axis++)
    kinematicsAngle[axis] = 0.0;
  gyroAngle[XAXIS] = 0;
  gyroAngle[YAXIS] = 0;
}

void initializeKinematics(float hdgX, float hdgY);
void calculateKinematics(float rollRate,           float pitchRate,     float yawRate,       
                         float longitudinalAccel,  float lateralAccel,  float verticalAccel, 
                         float oneG,               float magX,          float magY,
                         float G_Dt);
float getGyroUnbias(byte axis);
void calibrateKinematics();
 
const float kinematicsGetDegreesHeading(byte axis) {
  float tDegrees;
    
  tDegrees = degrees(kinematicsAngle[axis]);
  if (tDegrees < 0.0)
    return (tDegrees + 360.0);
  else
    return (tDegrees);
}

#endif


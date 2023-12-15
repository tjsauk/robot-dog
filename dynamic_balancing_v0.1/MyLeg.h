#ifndef _MyLeg_h
#define _MyLeg_h

#include "MyAsyncServo.h"

#define TURN_NONE 0
#define TURN_LEFT 1
#define TURN_RIGHT 2

class MyLeg
{
public:
  MyLeg(bool left, bool front);

public:
  MyAsyncServo mTibia;
  MyAsyncServo mFemur;
  MyAsyncServo mCoax;

  char * mName;
  coords mPosition;
  coords mPositionPaw;
  coords mPositionKnee;
  coords mPositionShoulder;
  bool   mFront;
  bool   mLeft;
 

private:
  quaternion quaternionMultiply(quaternion q,quaternion p);
  quaternion quaternionConjugate(quaternion q);
  void rotateAxis(angle a);
  angle IKSolver(double x, double y, double z);
  coords StepCurve(double percentage, double rotation[2],double y_rot, double x, double z);
  double degRad(double a);


public:
  void setReverse(bool value);
  void setName(const char * name);
  void home();
  void update(unsigned long mills);
  void GoHome(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void GoCenter(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void GoZero(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void GoPosition(unsigned long mills, double x, double y, double z, unsigned long duration, double ramp = 0.0);
  void BackToPosition(unsigned long mills, unsigned long duration, double ramp);
  void addToWalk(double x, double y, double z);
  void addToWalk(coords delta);
  void updateWalkPosition(unsigned long mills, unsigned long duration, double ramp);
  void PositionWithDelta(double x, double y, double z);
  void PositionWithDelta(coords delta);
  void updateStandPosition(unsigned long mills, unsigned long duration, double ramp);
  void DoStep( double percentage, double rotation[2], double y_rot, double x , double z);
  void WalkWithDelta(coords delta);          
  coords getStep( double percentage, double rotation[2], double y_rot, double x , double z);
  legState getPoints();
};


#endif

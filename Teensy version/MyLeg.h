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
  bool   mFront;
  bool   mLeft;

private:
  coords StepCurve(double percentage, unsigned long turning);
  angle IKSolver(double x, double y, double z);

public:
  void setReverse(bool value);
  void setName(const char * name);

  coords getPosition() {return mPosition;}
  
  void home();
  void update(unsigned long mills);
  
  void GoHome(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void GoCenter(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void GoPosition(unsigned long mills, double x, double y, double z, unsigned long duration, double ramp = 0.0);
  void GoPosition(unsigned long mills, coords deltax, unsigned long duration, double ramp = 0.0);
  void DoStep(unsigned long mills, double percentage, unsigned long duration, unsigned long turning);
};


#endif

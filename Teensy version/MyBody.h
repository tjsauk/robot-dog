#ifndef _MyBody_h
#define _MyBody_h

#include "MyLeg.h"

#define GAIT_WALK 0
#define GAIT_TROT 1
#define GAIT_PACE 2
#define GAIT_CANTER 3




struct GaitFootDelays
{
  double frontLeft;
  double frontRight;
  double rearLeft;
  double rearRight;
};

class MyBody
{
public:
  MyBody();

public:
  MyLeg mLeftFront;
  MyLeg mLeftRear;
  MyLeg mRightFront;
  MyLeg mRightRear;

  
private:
  GaitFootDelays mFootDelays;

  bool          mWalking;
  unsigned long mWalkStep;
  unsigned long mLastUpdate;
  unsigned long mWalkDuration;
  unsigned long mTurning;

  double mDegree_x;
  double mDegree_y;
  double mDegree_z;
  double mPosition_x;
  double mPosition_y;
  double mPosition_z;

  void IK_Solver(double omega, double  phi, double psi, double xm, double ym, double zm, coords * fl, coords * fr, coords * rl, coords * rr);
  
public:
  void home();
  void update(unsigned long mills);
  void setGait(int mode);
  void GoHome(unsigned long mills, unsigned long duration, double ramp = 0.0);
  
  void GoWalkPosition(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void StartWalk(unsigned long single_step_duration);
  void WalkUpdate(unsigned long mills);
  void StopWalk(unsigned long mills);
  void WalkTurn(unsigned long value);
  void GoTo(unsigned long mills, unsigned long duration, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp = 0);
};


#endif

#ifndef _MyBody_h
#define _MyBody_h

#include "MyLeg.h"

#define GAIT_WALK 0
#define GAIT_TROT 1
#define GAIT_PACE 2
#define GAIT_CANTER 3
#define JACK_SPARROW 4

#define M_PI 3.141592653589793238462643


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

  
  unsigned long mWalkStep;
  unsigned long mLastUpdate;
  unsigned long mWalkDuration;

  double sVelocity;
  double sDegree_x;
  double sDegree_y;
  double sDegree_z;
  double sDirection_x;
  double sDirection_y;
  double sDirection_z;
  double sRamp;
  coords wfLeftFront;
  coords wfLeftRear;
  coords wfRightFront;
  coords wfRightRear;
  coords wfCOM;
  double COMdelay;
  //this is 4 coords object points that corresponds to the theoretical trajectory that the center of mass is supposed to follow
  //points are executed in order 0,1,2,3. COMdelay is the timing of this cycle starting from the point 0
  coords COMTrajectory[4];

  coords intersection(coords LF, coords RR, coords RF, coords LR);
  void setCOMTrajectory(double distance);
  coords linear4point(double percentage, coords trajectory[4]);
  void addOffsetTo(legState& leg,coords offset);
  legState getLegPoints(bool left,bool front);
  void COMtobody();
  quaternion quaternionMultiply(quaternion q,quaternion p);
  quaternion quaternionConjugate(quaternion v);
  corners rotateBody(double x, double z);
  double degRad(double a);
public:
  void home();
  void update(unsigned long mills);
  void setGait(int mode);
  void GoHome(unsigned long mills, unsigned long duration, double ramp = 0.0);
  
  void GoWalkPosition(unsigned long mills, unsigned long duration, double ramp = 0.0);

  void PositionUpdate(unsigned long mills);
  coords atplane(coords point);
  void GoTo(unsigned long mills, unsigned long duration, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp = 0);
  void WalkDirection(double velocity, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp);
};


#endif

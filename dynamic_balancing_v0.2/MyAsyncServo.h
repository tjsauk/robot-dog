
#ifndef _MyAsyncServo_h
#define _MyAsyncServo_h

#include <Adafruit_PWMServoDriver.h>

#define SERVO_35KG 0
#define SERVO_20KG 1

struct coords
{
  double x;
  double y;
  double z;
};
struct legState
{
  coords coax;
  coords femur;
  coords tibia;
};
struct corners
{
    coords A;
    coords B;
    coords C;
    coords D; 
};
struct quaternion
{
  double a;
  double x;
  double y;
  double z;
};

struct angle
{
    double coax_degree;
    double femur_degree;
    double tibia_degree;
};

struct Movement
{
	double         degree;
	unsigned long duration;
	double         ramp;
};

class MyAsyncServo
{
public:
	MyAsyncServo();
	~MyAsyncServo();

	static bool initialised;
	static Adafruit_PWMServoDriver pwm;
  
  double mZero;
  
private:
	void ActivatePWM();
  
	int    mId;
	int    mPwmMin;
	int    mPwmMax;
	int    mCurrentPwm;
  bool   mReverse;
  char * mName;

	double mDegreeRange;

  double mCorrection;

  
 
	double mHome;
	double mLimitLow;
	double mLimitHigh;

  double mTargetDegree;
	double mCurrentDegree;
	double mStartDegree;
	double mRamp;
	unsigned long mStartTime;
	unsigned long mEndTime;

	bool mIsMoving;

	double CutOffDegrees(double degree);
  double CutOffRamp(double ramp);

public:
	void Attach(int id, int srv_type, double home, double correction,double type);
	void SetLimits(double low, double high);
  
  void setReverse(bool value) {mReverse = value;}
  void setName(const char * owner, const char * name);
  
  void write_degree(double degree);

  void home();
	void update(unsigned long mills);

  double getZero() {return mZero;}
  double getMin() {return mLimitLow;}
  double getMax() {return mLimitHigh;}
  double getHome() {return mHome;}
  double getDegree() {return mCurrentDegree;}
 

	void StartMovement(unsigned long mills, double degree, unsigned long duration, double ramp = 0.0);
	void GoCenter(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void GoZero(unsigned long mills, unsigned long duration, double ramp = 0.0);
  void GoHome(unsigned long mills, unsigned long duration, double ramp = 0.0);

};

#endif

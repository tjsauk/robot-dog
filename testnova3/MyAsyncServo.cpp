#include <string.h>

#include "MyAsyncServo.h"
#include "MyLog.h"

bool MyAsyncServo::initialised = false;
Adafruit_PWMServoDriver MyAsyncServo::pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 60
#define OSCIL_FREQ 25000000

MyAsyncServo::MyAsyncServo()
{
	mId = -1;
	mTargetDegree = -1;
	mCurrentDegree = -1;
	mCurrentPwm = 0;
	mIsMoving = false;
  mReverse = false;
  mName = NULL;
}

MyAsyncServo::~MyAsyncServo()
{
  if (mName)
    delete []mName;
}

void MyAsyncServo::setName(const char * owner, const char * name)
{
  mName = new char[strlen(owner) + strlen(name) + 2];
  
  
  sprintf(mName, "%s-%s", owner, name);
}
  
void MyAsyncServo::ActivatePWM()
{
	 my_log(MYLOG_INFO, "ActivatePWM");

	 pwm.begin();
	 pwm.setOscillatorFrequency(OSCIL_FREQ);
	 pwm.setPWMFreq(SERVO_FREQ);

   my_log(MYLOG_INFO, "PWM intialized");

   delay(500);
}

void MyAsyncServo::Attach(int id, int srv_type, double home, double correction, double type)
{
	if (!initialised)
	{
		ActivatePWM();
		initialised = true;
	}

	mId = id;

	mHome = home;
  mCorrection = correction;
  
  
  mZero = type;
  
  
   

	if (srv_type == SERVO_20KG)
	{
		mPwmMin = 120;//was 120
		mPwmMax = 590;

		mDegreeRange = 270;//was 180.0;
	}
	else // SERVO_35KG
	{
		mPwmMin = 140;
		mPwmMax = 590;

		mDegreeRange = 270.0;
	}
}

void MyAsyncServo::write_degree(double degree)
{
	  if (mId < 0)
			return;

    degree = degree + mCorrection;

	  if (degree < 0)
		   degree = 0;
	  else if (degree > mDegreeRange)
			degree = mDegreeRange;

		mCurrentDegree = degree - mCorrection;

	  int value_in_pwm;
	  
	  if (mReverse)
	    value_in_pwm = (double)mPwmMin + ((double)mPwmMax - (double)mPwmMin) *  degree / mDegreeRange;
    else
      value_in_pwm = (double)mPwmMax - ((double)mPwmMax - (double)mPwmMin) *  degree / mDegreeRange;

		if (value_in_pwm != mCurrentPwm)
		{
			my_log(MYLOG_DEBUG, "setpwm servo %s (id %d), degree %.1f, pwm=%d", mName, mId, mCurrentDegree, value_in_pwm);

		  pwm.setPWM(mId, 0, value_in_pwm);

			mCurrentPwm = value_in_pwm;
		}
}

void MyAsyncServo::home()
{
	  my_log(MYLOG_INFO, "home servo %d", mId);

	  write_degree(mHome);
}

void MyAsyncServo::SetLimits(double low, double high)
{
	  mLimitLow = low;
		if (mLimitLow < 0.0)
			mLimitLow = 0.0;

		mLimitHigh = high;
		if (mLimitHigh > mDegreeRange)
			mLimitHigh = mDegreeRange;
}

double MyAsyncServo::CutOffDegrees(double degree)
{
	if (degree < mLimitLow)
		return mLimitLow;
	else if (degree > mLimitHigh)
		return mLimitHigh;
  return degree;
}

double MyAsyncServo::CutOffRamp(double ramp)
{
  if (ramp < 0)
    return 0;
  if (ramp > 0.4)
    return 0.4;
  return ramp;
}

void MyAsyncServo::GoHome(unsigned long mills, unsigned long duration, double ramp)
{
  StartMovement(mills, mHome, duration, ramp);
}

void MyAsyncServo::GoCenter(unsigned long mills, unsigned long duration, double ramp)
{
  StartMovement(mills, mDegreeRange / 2.0, duration, ramp);
}

void MyAsyncServo::StartMovement(unsigned long mills, double degree, unsigned long duration, double ramp)
{
	degree = CutOffDegrees(degree);
  ramp = CutOffRamp(ramp);

	//my_log(MYLOG_INFO, "StartMovement %s [%.1f, %d, %.1f]", mName, degree, duration, ramp);

  mTargetDegree = degree;
  mStartDegree = mCurrentDegree;
  mStartTime = mills;
  mEndTime = mStartTime + duration;
  mRamp = ramp;

  mIsMoving = true;
}



void MyAsyncServo::update(unsigned long mills)
{
  if (mId < 0)
    return;

	if (mIsMoving)
	{
		if (mills >= mEndTime)
		{
			my_log(MYLOG_INFO, "EndMovement %s id=%d degree=%.1f", mName, mId, getDegree());

			write_degree(mTargetDegree);
			mIsMoving = false;
		}
		else
		{
      if (mTargetDegree == mCurrentDegree)
        return;
    
      double perc = (double)(mills - mStartTime) / (double)(mEndTime - mStartTime);

		  double pos_degree;

      if (mRamp > 0.0)
      {
				double velocitymax = (mTargetDegree - mStartDegree) / (1.0 - mRamp);

				double velocity;

        if (perc < mRamp) // ramp UP
				{
					velocity = perc / mRamp *  velocitymax;

					pos_degree = mStartDegree + perc * velocity / 2.0;

          my_log(MYLOG_DEBUG, "ContinueMovement ramp UP %s perc=%.1f, velocitymax=%.1f, velocity=%.1f, pos=%.1f", mName, 100.0*perc, velocitymax, velocity, pos_degree);
				}
				else if (perc < (1.0 - mRamp)) // constant velocity
				{
					velocity = velocitymax;

					pos_degree = mStartDegree + (mRamp / 2.0 + perc - mRamp) * velocitymax;

         my_log(MYLOG_DEBUG, "ContinueMovement ramp CONST %s perc=%.1f, velocitymax=%.1f, velocity=%.1f, pos=%.1f", mName, 100.0*perc, velocitymax, velocity, pos_degree);
				}
				else // ramp DOWN
				{
					velocity = (1.0 - perc) / mRamp * velocitymax;

					pos_degree = mStartDegree + (1.0 - mRamp) * velocitymax - velocity * (1.0 - perc) / 2;

          my_log(MYLOG_DEBUG, "ContinueMovement ramp DOWN %s perc=%.1f, velocitymax=%.1f, velocity=%.1f, pos=%.1f", mName, 100.0*perc, velocitymax, velocity, pos_degree);
				}
      }
      else // No ramp
      {
		    pos_degree = mStartDegree + (mTargetDegree - mStartDegree) * perc;
        
        my_log(MYLOG_DEBUG, "ContinueMovement %s start=%.1f, end=%.1f, now=%d, end=%d, perc=%.1f --> %.1f", mName, mStartDegree, mTargetDegree, mills, mEndTime, 100.0*perc, pos_degree);
      }
			
			write_degree(pos_degree);
		}
	}
}

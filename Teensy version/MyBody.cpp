#include "MyBody.h"
#include "MyLog.h"

#include "myconfig.h"


#define MYDEG_TO_RAD(x) x*M_PI/180

void MultiplyMatrix(double m1[][4], double m2[][4], double result[][4], int size)
{
   for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
         result[i][j] = 0;
      }
   }

   for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
         for (int k = 0; k < size; k++) {
            result[i][j] += m1[i][k] * m2[k][j];
         }
      }
   }
}

void SumMatrix(double m1[][4], double m2[][4], double result[][4], int size)
{
   for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
            result[i][j] = m1[i][j] + m2[i][j];
      }
   }
}

MyBody::MyBody():mLeftFront(true,true),
  mLeftRear(true,false),
  mRightFront(false,true),
  mRightRear(false,false)
{
  mRightFront.setReverse(true);
  mRightRear.setReverse(true);

  mRightRear.setName("R rear");
  mRightFront.setName("R front");
  mLeftRear.setName("L rear");
  mLeftFront.setName("L front");

  setGait(GAIT_WALK);

  mWalking = false;
  mTurning = TURN_NONE;

  mDegree_x = 0.0;
  mDegree_y = 0.0;
  mDegree_z = 0.0;
  mPosition_x = 0.0;
  mPosition_y = 0.0;
  mPosition_z = 0.0;
}

void MyBody::setGait(int mode)
{
  if (mode == GAIT_WALK)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.75;
    mFootDelays.rearRight = 0.25;
  }
  else if (mode == GAIT_TROT)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.50;
    mFootDelays.rearRight = 0.00;
  }
  else if (mode == GAIT_PACE)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.00;
    mFootDelays.rearRight = 0.50;

  }
  else if (mode == GAIT_CANTER)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.30;
    mFootDelays.rearLeft = 0.70;
    mFootDelays.rearRight = 0.00; 
  }
  else
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.00;
    mFootDelays.rearLeft = 0.50;
    mFootDelays.rearRight = 0.50;
  }
}

void MyBody::home()
{
  my_log(MYLOG_INFO, "Body home");
  
  mLeftFront.home();
  mLeftRear.home();
  mRightFront.home();
  mRightRear.home();
}

void MyBody::update(unsigned long mills)
{
  if (mWalking)
    WalkUpdate(mills); // update 
  
  mLeftFront.update(mills);
  mLeftRear.update(mills);
  mRightFront.update(mills);
  mRightRear.update(mills);
}

// input: rotation angle (omega, phi, psi) and translation of the center of the body
// output: foot coordinates for each leg
//------------------
void MyBody::IK_Solver(double omega, double  phi, double psi, double xm, double ym, double zm, coords * fl, coords * fr, coords * rl, coords * rr)
{
  double m1[4][4] = {{1,0,0,0},
                    {0,cos(omega),-sin(omega),0},
                    {0,sin(omega),cos(omega),0},
                    {0,0,0,1}
                  };

  double m2[4][4] = {{cos(phi),0,sin(phi),0},
                    {0,1,0,0},
                    {-sin(phi),0,cos(phi),0},
                    {0,0,0,1}
                  };
  double m3[4][4] = {{cos(psi),-sin(psi),0,0},
                    {sin(psi),cos(psi),0,0},
                    {0,0,1,0},
                    {0,0,0,1}
                  };

  double tmp[4][4];
  MultiplyMatrix(m1, m2, tmp, 4);

  double tmp2[4][4];
  MultiplyMatrix(tmp, m3, tmp2, 4);

  double t[4][4] = {{0,0,0,xm},
                   {0,0,0,ym},
                   {0,0,0,zm},
                   {0,0,0,0}};

  double result[4][4];
  SumMatrix(tmp2, t, result, 4);

  double cos90 = cos(M_PI/2.0);
  double sin90 = sin(M_PI/2.0);

  double transform_FL[4][4] = {{cos90,0,sin90,BODY_LENGTH/2.0},
                             {0,1,0,0},
                             {-sin90,0,cos90,BODY_WIDTH/2.0},
                             {0,0,0,1}
                            };
  double transform_FR[4][4] = {{cos90,0,sin90,BODY_LENGTH/2.0},
                             {0,1,0,0},
                             {-sin90,0,cos90,-BODY_WIDTH/2.0},
                             {0,0,0,1}
                            };
  double transform_RL[4][4] = {{cos90,0,sin90,-BODY_LENGTH/2.0},
                             {0,1,0,0},
                             {-sin90,0,cos90,BODY_WIDTH/2.0},
                             {0,0,0,1}
                            };
  double transform_RR[4][4] = {{cos90,0,sin90,-BODY_LENGTH/2.0},
                             {0,1,0,0},
                             {-sin90,0,cos90,-BODY_WIDTH/2.0},
                             {0,0,0,1}
                            };



  double ik_FL[4][4];
  MultiplyMatrix(result, transform_FL, ik_FL, 4);
  fl->x = ik_FL[0][3] - BODY_LENGTH / 2;
  fl->y = ik_FL[1][3];
  fl->z = -(ik_FL[2][3] - BODY_WIDTH / 2);

  double ik_RL[4][4];
  MultiplyMatrix(result, transform_RL, ik_RL, 4);
  rl->x = ik_RL[0][3] + BODY_LENGTH / 2;
  rl->y = ik_RL[1][3];
  rl->z = -(ik_RL[2][3] - BODY_WIDTH / 2);

  double ik_FR[4][4];
  MultiplyMatrix(result, transform_FR, ik_FR, 4);
  fr->x = ik_FR[0][3] - BODY_LENGTH / 2;
  fr->y = ik_FR[1][3];
  fr->z = ik_FR[2][3] + BODY_WIDTH / 2;

  double ik_RR[4][4];
  MultiplyMatrix(result, transform_RR, ik_RR, 4);  
  rr->x = ik_RR[0][3] + BODY_LENGTH / 2;
  rr->y = ik_RR[1][3];
  rr->z = ik_RR[2][3] + BODY_WIDTH / 2;
}

void MyBody::GoTo(unsigned long mills, unsigned long duration, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp)
{
  coords lf, rf, lr, rr;

  IK_Solver(MYDEG_TO_RAD(degree_x), MYDEG_TO_RAD(degree_y), MYDEG_TO_RAD(degree_z), x, y, z, &lf, &rf, &lr, &rr);

  mLeftFront.GoPosition(mills, lf, duration, ramp);
  mLeftRear.GoPosition(mills, lr, duration, ramp);
  mRightFront.GoPosition(mills, rf, duration, ramp);
  mRightRear.GoPosition(mills, rr, duration, ramp);
}

void MyBody::StartWalk(unsigned long single_step_duration)
{
  if (mWalking)
    return;

  mWalking = true;
  mTurning = TURN_NONE;
  mWalkStep = 0;
  mWalkDuration = single_step_duration;

  my_log(MYLOG_INFO, "StartWalk");
}

void MyBody::WalkTurn(unsigned long value)
{
  mTurning = value;
}

void MyBody::StopWalk(unsigned long mills)
{
  mWalking = false;

  GoWalkPosition(mills, 300, 0.3);

  my_log(MYLOG_INFO, "StopWalk");
}

void MyBody::WalkUpdate(unsigned long mills)
{
  unsigned long total_step_points = 50; // the foot trajectory is divided in 50 points
  unsigned long time_delta = mWalkDuration / total_step_points;

  if (mills - mLastUpdate < time_delta)
    return;

  my_log(MYLOG_INFO, "WalkUpdate [%d]", mLastUpdate);

  mLastUpdate = mills;

  double perc_FL = (double)((mWalkStep + (int)(mFootDelays.frontLeft * total_step_points)) % total_step_points) / (double)total_step_points;
  double perc_FR = (double)((mWalkStep + (int)(mFootDelays.frontRight * total_step_points)) % total_step_points) / (double)total_step_points;
  double perc_RL = (double)((mWalkStep + (int)(mFootDelays.rearLeft * total_step_points)) % total_step_points) / (double)total_step_points;
  double perc_RR = (double)((mWalkStep + (int)(mFootDelays.rearRight * total_step_points)) % total_step_points) / (double)total_step_points;

  mRightFront.DoStep(mills, perc_FR, time_delta, mTurning);
  mLeftFront.DoStep(mills, perc_FL, time_delta, mTurning);
  mRightRear.DoStep(mills, perc_RR, time_delta, mTurning);
  mLeftRear.DoStep(mills, perc_RL, time_delta, mTurning);

  mWalkStep++;

  if (mWalkStep > total_step_points)
    mWalkStep = 0;
}


void MyBody::GoHome(unsigned long mills, unsigned long duration, double ramp)
{
  mLeftFront.GoHome(mills, duration, ramp);
  mLeftRear.GoHome(mills, duration, ramp);
  mRightFront.GoHome(mills, duration, ramp);
  mRightRear.GoHome(mills, duration, ramp);
}

void MyBody::GoWalkPosition(unsigned long mills, unsigned long duration, double ramp)
{
  double leg_len = TIBIA_SIZE+FEMUR_SIZE;

  double front_x_walk_position = STEP_SIZE / 4;
  double rear_x_walk_position = 0;
  
  double front_y_walk_position = leg_len * 0.75;
  double rear_y_walk_position = leg_len * 0.75;
  
  double z_walk_position = 0;
    
  mLeftFront.GoPosition(mills, front_x_walk_position, front_y_walk_position ,z_walk_position, duration, ramp);
  mRightFront.GoPosition(mills, front_x_walk_position, front_y_walk_position, z_walk_position, duration, ramp);
  mLeftRear.GoPosition(mills, rear_x_walk_position, rear_y_walk_position, z_walk_position, duration, ramp);
  mRightRear.GoPosition(mills, rear_x_walk_position, rear_y_walk_position, z_walk_position, duration, ramp);
}

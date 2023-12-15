#include "MyLeg.h"
#include "MyLog.h"

#define square(x) x*x

//legs have seperate coordinate systems. Origin in leg coordinates is at the point where COAX anf FEMUR are connected
//axis orientation is the same as bodys. +x is forwards -x backwards, +y is down from body and -z is left +z is right
//that translates to adding positive z for left resulting the leg going under the body and for right legs it means going away from body
//adding positive y means leg going away from body "going straight"

MyLeg::MyLeg(bool left, bool front)
{
  mFront = front;
  mLeft = left;
  if (mFront)
  {
    mPosition.x = WALK_POSITION_FRONT_X;
    mPosition.y = WALK_POSITION_FRONT_Y;
    
  }
  else
  {
    mPosition.x = WALK_POSITION_REAR_X;
    mPosition.y = WALK_POSITION_REAR_Y;
    
  }
  if (mLeft)
  {
    mPosition.z = WALK_POSITION_Z;
  }else
  {
    mPosition.z = -WALK_POSITION_Z;
  }
}
void MyLeg::setName(const char * name) 
{
  mName = new char[strlen(name) + 1];
  strcpy(mName, name);

  mTibia.setName(mName,"Tibia");
  mFemur.setName(mName,"Femur");
  mCoax.setName(mName,"Coax");
}


void MyLeg::setReverse(bool value)
{
  mTibia.setReverse(value);
  mFemur.setReverse(value);
  mCoax.setReverse(value);
}

void MyLeg::home()
{
  my_log(MYLOG_INFO, "Leg home");
  mTibia.home();
  mFemur.home();
  mCoax.home();
}

void MyLeg::update(unsigned long mills)
{
  mTibia.update(mills);
  mFemur.update(mills);
  mCoax.update(mills);
}
quaternion MyLeg::quaternionMultiply(quaternion q,quaternion p)
{
  //Multiplies the given quaternions and returns a new quaternion
  quaternion r;
  r.a = q.a*p.a-q.x*p.x-q.y*p.y-q.z*p.z;
  r.x = q.x*p.a+q.a*p.x-q.z*p.y+q.y*p.z;
  r.y = q.y*p.a+q.z*p.x+q.a*p.y-q.x*p.z;
  r.z = q.z*p.a-q.y*p.x+q.x*p.y+q.a*p.z;
  return r;
}
quaternion MyLeg::quaternionConjugate(quaternion v)
{
  //converts the given quaternion to its conjugate
  quaternion q;
  q.a = v.a;
  q.x = -v.x;
  q.y = -v.y;
  q.z = -v.z;
  return q;
}

double MyLeg::degRad(double a)
  {
    return ((M_PI / 180) * a);
  }

void MyLeg::rotateAxis(angle a)
{
  //rotates the points of shoulders and knees with the given degrees. Updates the positions as coordinates
  quaternion qx = {cos(degRad(mCoax.mZero-a.coax_degree)/2),sin(degRad(mCoax.mZero-a.coax_degree)/2),0,0};
  quaternion qz = {cos(degRad(mFemur.mZero-a.femur_degree)/2),0,0,sin(degRad(mFemur.mZero-a.femur_degree)/2)};
  quaternion qt = quaternionMultiply(qx,qz);
  quaternion qs;//shoulder
  quaternion qk;//knee
  if (mLeft)
  {
    qs = {0,0,0,COAX_SIZE};
    qk = {0,0,FEMUR_SIZE,COAX_SIZE};
  }
  else
  {
    qs = {0,0,0,-COAX_SIZE};
    qk = {0,0,FEMUR_SIZE,-COAX_SIZE};
  }
  quaternion qr = quaternionMultiply(quaternionMultiply(qx,qs),quaternionConjugate(qx));
  mPositionShoulder.x = qr.x;
  mPositionShoulder.y = qr.y;
  mPositionShoulder.z = qr.z;
  qr = quaternionMultiply(quaternionMultiply(qt,qk),quaternionConjugate(qt));
  mPositionKnee.x = qr.x;
  mPositionKnee.y = qr.y;
  mPositionKnee.z = qr.z;
}

angle MyLeg::IKSolver(double x, double y, double z)
{
  //takes in coordinates for paw (leg end point) calculates the motor angles needed for the leg to go in that position.
  //angles are good to use as is. Calculations are done from joint mZero position that corresponds leg being straight
  z = z + COAX_SIZE;

  double d1 = sqrt(square(z) + square(y) - square(COAX_SIZE));
  double d2 = sqrt(square(d1) + square(x));
  double d3 = (square(d2) - square(FEMUR_SIZE) - square(TIBIA_SIZE)) / (2 * FEMUR_SIZE * TIBIA_SIZE);

  double theta1 = -atan2(y, z) - atan2(d1, -COAX_SIZE);
  double theta3 = acos(d3);//this goes over and under -1 and 1
  double theta2 = atan2(x,d1) - atan2(TIBIA_SIZE * sin(theta3), FEMUR_SIZE + TIBIA_SIZE * cos(theta3));

  angle a;
  //Serial.println("Jalka");
  //Serial.println(this->mCoax.getZero());
  //Serial.println(((theta1 *180)/ M_PI));
  a.coax_degree  = 180.0 + this->mCoax.getZero()+((theta1 *180)/ M_PI);//kokeile sulkeita
  a.femur_degree  = this->mFemur.getZero() + ((180.0 * theta2) / M_PI);
  a.tibia_degree  = this->mTibia.getZero() + ((180.0 * theta3 )/ M_PI);
  //Serial.println(a.coax_degree);
  return a;
}


struct coords MyLeg::StepCurve(double percentage, double rotation[2],double y_rot, double x, double z)
{
  //calculates the coordinates for the end of the leg in certain stage of walk cycle. Output is coordinate delta value that is in leg coordinate system.
  //that means that the output needs to be added to the current standing or static position. Coordinates are allso calculated from leg 0 so final coordinates are to be frefreshed in every cycle step.
  //turning works by given rotation radius origo given in rotation variable where it contains (x,y) coordinates in body coordinates.
  //y_rot controls the distance of the turning point from the body and can have values between max and min radius. rotation point can allso be at 0,0 that corresponds turning in place
  //steps are fitted to walk cycle by making sure that rotation point is allways 90 degrees from straight step.
  double x0, x1, x2, x3, y0, y1, y2, y3, z0, z1, z2, z3;

  double step_size_x = 0.0 ;
  double step_size_z= 0.0;
  double step_stance_h;
  double step_swing_h;
  double stance_percentage = STEP_STANCE_TIME_PERCENTAGE;
  double x_compensate;
  double z_compensate;
  double step_lenght;


  if (y_rot == 0)//not turning just going straight
  {
    step_size_x = STEP_SIZE*x;
    //step_size_z = STEP_SIZE*z/2;
  }
  else if (y_rot < 0)//turning left
  {
    //step_size_x = step_lenght*(vastainen/hypotenuusa);
    //step_size_z = step_lenght*(viereinen/hypotenuusa);
    if (mLeft){//if leg is left leg
      double amp = 0.2+(0.8-0.8*abs(y_rot));//Minimum of 0.2 procent of stepsize
      step_size_x = STEP_SIZE*x*amp;
      //step_size_z = STEP_SIZE*z*amp/2;
      //my_log(MYLOG_INFO, "update walk position %s [%f, %f,%f,%f]",mName, step_size_x,step_size_z,y_rot,amp);
    }else{
      step_size_x = STEP_SIZE*x;
      //my_log(MYLOG_INFO, "update walk position %s [%f, %f,%f]",mName, step_size_x,step_size_z,y_rot);
      //step_size_z = STEP_SIZE*z/2;
    }
    
  }else{//Turning right
    if (mLeft){//if leg is left leg
      step_size_x = STEP_SIZE*x;
      //step_size_z = STEP_SIZE*z/2;
    }else{
      
      double amp = 0.2+(0.8-0.8*abs(y_rot));//Minimum of 0.2 procent of stepsize
      step_size_x = STEP_SIZE*x*amp;
      //step_size_z = STEP_SIZE*z*amp/2;
    }
  }
  step_size_z = STEP_SIZE*z/2;
  //my_log(MYLOG_INFO, "update walk position %s [%f, %f,%f]",mName, step_size_x,step_size_z,y_rot);
  
//set the swing and stance height
  if (mFront)
  {
    step_swing_h = STEP_SWING_HEIGHT_FRONT;
    step_stance_h = STEP_STANCE_HEIGHT_FRONT;
  }
  else
  {
    step_swing_h = STEP_SWING_HEIGHT_REAR;
    step_stance_h = STEP_STANCE_HEIGHT_FRONT;
  }
  
  struct coords coord;

  if (percentage <= stance_percentage)
  {
    double perc = percentage / stance_percentage;

    //----- coordinates of 4 points to interpolate -------
    x0 = step_size_x / 2;//25
    y0 = 0;
    z0 = step_size_z / 2;

    x1 = step_size_x / 2 -step_size_x / 3;//8.33
    y1 = 0;
    z1 = step_size_z / 2 -step_size_z / 3;
    
    x2 = step_size_x / 2 - step_size_x * 2 / 3;//
    y2 = 0;
    z2 = step_size_z / 2 - step_size_z * 2 / 3;

    x3 = step_size_x / 2 - step_size_x;
    y3 = 0;//12mm
    z3 = step_size_z / 2 - step_size_z;
    //----------------------------------------------------

    coord.x = ((1 - perc) * ((1 - perc) * ((1 - perc) * x0 + perc * x1) + perc * ((1 - perc) * x1 + perc * x2)) +
               perc * ((1 - perc) * ((1 - perc) * x1 + perc * x2) + perc * ((1 - perc) * x2 + perc * x3)));
    coord.y = -(((1 - perc) * ((1 - perc) * ((1 - perc) * y0 + perc * y1) + perc * ((1 - perc) * y1 + perc * y2)) +
               perc * ((1 - perc) * ((1 - perc) * y1 + perc * y2) + perc * ((1 - perc) * y2 + perc * y3))));
    coord.z = ((1 - perc) * ((1 - perc) * ((1 - perc) * z0 + perc * z1) + perc * ((1 - perc) * z1 + perc * z2)) +
               perc * ((1 - perc) * ((1 - perc) * z1 + perc * z2) + perc * ((1 - perc) * z2 + perc * z3)));
               
    return coord;
  }
  else//swing
  {
    double perc = (percentage - stance_percentage)/(1.0 - stance_percentage);

    //----- coordinates of 4 points to interpolate -------
    x0 = step_size_x / 2 - step_size_x;
    y0 = 0;
    z0 = step_size_z / 2 - step_size_z;
    
    x1 = step_size_x / 2 - step_size_x + step_size_x / 5;
    y1 = step_swing_h*4/3;
    z1 = step_size_z / 2 - step_size_z + step_size_z / 5;

    x2 = step_size_x / 2 - step_size_x / 5;
    y2 = step_swing_h*4/3;
    z2 = step_size_z / 2 - step_size_z / 5;

    x3 = step_size_x / 2;
    y3 = 0;
    z3 = step_size_z / 2;
    //----------------------------------------------------

    coord.x = ((1 - perc) * ((1 - perc) * ((1 - perc) * x0 + perc * x1) + perc * ((1 - perc) * x1 + perc * x2)) +
               perc * ((1 - perc) * ((1 - perc) * x1 + perc * x2) + perc * ((1 - perc) * x2 + perc * x3)));
    coord.y = -(((1 - perc) * ((1 - perc) * ((1 - perc) * y0 + perc * y1) + perc * ((1 - perc) * y1 + perc * y2)) +
               perc * ((1 - perc) * ((1 - perc) * y1 + perc * y2) + perc * ((1 - perc) * y2 + perc * y3))));
    coord.z = ((1 - perc) * ((1 - perc) * ((1 - perc) * z0 + perc * z1) + perc * ((1 - perc) * z1 + perc * z2)) +
               perc * ((1 - perc) * ((1 - perc) * z1 + perc * z2) + perc * ((1 - perc) * z2 + perc * z3)));
    return coord;
  }
}

void MyLeg::GoHome(unsigned long mills, unsigned long duration, double ramp)
{
  mTibia.GoHome(mills, duration, ramp);
  mFemur.GoHome(mills, duration, ramp);
  mCoax.GoHome(mills, duration, ramp);
}


void MyLeg::GoCenter(unsigned long mills, unsigned long duration, double ramp)
{
  mTibia.GoCenter(mills, duration, ramp);
  mFemur.GoCenter(mills, duration, ramp);
  mCoax.GoCenter(mills, duration, ramp);
}
void MyLeg::GoZero(unsigned long mills, unsigned long duration, double ramp)
{
  mTibia.GoZero(mills, duration, ramp);
  mFemur.GoZero(mills, duration, ramp);
  mCoax.GoZero(mills, duration, ramp);
}

void MyLeg::GoPosition(unsigned long mills, double x, double y, double z, unsigned long duration, double ramp)
{
  //changes the static / standing position of the robot. Doesnt take in account the last position 
  angle a = IKSolver(x, y, z);

  mPosition.x = x;//this is the standing position
  mPosition.y = y;
  mPosition.z = z;

  mPositionPaw.x = x;//this is the temperary position that is used for walking
  mPositionPaw.y = y;
  mPositionPaw.z = z;

  mTibia.StartMovement(mills, a.tibia_degree, duration, ramp);
  mFemur.StartMovement(mills, a.femur_degree, duration, ramp);
  mCoax.StartMovement(mills, a.coax_degree, duration, ramp); 
  rotateAxis(a);//this changes the knee and shoulder points to correct position
}

void MyLeg::BackToPosition(unsigned long mills, unsigned long duration, double ramp)
{
  //this takes the robot back to its current static position. For example when walk ends
  angle a = IKSolver(mPosition.x, mPosition.y, mPosition.z);

  mPositionPaw.x = mPosition.x ;
  mPositionPaw.y = mPosition.y ;
  mPositionPaw.z = mPosition.z ;

  mTibia.StartMovement(mills, a.tibia_degree, duration, ramp);
  mFemur.StartMovement(mills, a.femur_degree, duration, ramp);
  mCoax.StartMovement(mills, a.coax_degree, duration, ramp); 
  rotateAxis(a);
}
void MyLeg::addToWalk(double x, double y, double z)
{
  //this adds offset during temperary movement. For example when walking and robot needs to change its position when balancing
  //my_log(MYLOG_INFO, "Add to walk ");
  //my_log(MYLOG_INFO, "Add to walk %s before [%f,%f,%f][%f,%f,%f]",mName, mPositionPaw.x,mPositionPaw.y,mPositionPaw.z);
  mPositionPaw.x +=x ;
  mPositionPaw.y +=y ;
  mPositionPaw.z +=z ;
  //my_log(MYLOG_INFO, "Add to walk %s after [%f,%f,%f][%f,%f,%f]",mName, mPositionPaw.x,mPositionPaw.y,mPositionPaw.z);
  //if (mLeft)
  //{
    //mPositionPaw.z = mPositionPaw.z+z ; 
  //}else
  //{
    //mPositionPaw.z = mPositionPaw.z-z ; 
  //}
   
}
void MyLeg::addToWalk(coords delta)
{
  //this adds offset during temperary movement. For example when walking and robot needs to change its position when balancing
  //my_log(MYLOG_INFO, "Add to walk ");
  //my_log(MYLOG_INFO, "Add to walk %s before [%f,%f,%f]",mName, mPositionPaw.x,mPositionPaw.y,mPositionPaw.z);
  mPositionPaw.x +=delta.x ;
  mPositionPaw.y +=delta.y ;
  if (mLeft){
    mPositionPaw.z -=delta.z ; 
  }else{
    mPositionPaw.z +=delta.z ; 
  }
  
  //my_log(MYLOG_INFO, "Add to walk %s after [%f,%f,%f]",mName, mPositionPaw.x,mPositionPaw.y,mPositionPaw.z);
  //if (mLeft)
  //{
    //mPositionPaw.z = mPositionPaw.z+delta.z ; 
  //}else
  //{
    //mPositionPaw.z = mPositionPaw.z-delta.z ; 
  //} 
}
void MyLeg::updateWalkPosition(unsigned long mills, unsigned long duration, double ramp)
{
  //this is used after all temperary movement is inputted to the temperary position variable
  //and the movement needs to be executed
  angle a = IKSolver(mPositionPaw.x, mPositionPaw.y, mPositionPaw.z);
  //my_log(MYLOG_INFO, "update walk position %s [%f,%f,%f]",mName, a.tibia,a.femur,a.coax);
  mTibia.StartMovement(mills, a.tibia_degree, duration,ramp);
  mFemur.StartMovement(mills, a.femur_degree, duration,ramp);
  mCoax.StartMovement(mills, a.coax_degree, duration,ramp); 
  rotateAxis(a);
}
void MyLeg::PositionWithDelta(double x, double y, double z)
{
  //used to change the static / standing position with change in coordinates
  mPosition.x = mPosition.x + x;
  mPosition.y = mPosition.y + y;
  mPosition.z = mPosition.z + z;
  //if (mLeft)
  //{
    //mPosition.z = mPosition.z + z;
  //}else
  //{
    //mPosition.z = mPosition.z - z;
  //}
}
void MyLeg::PositionWithDelta(coords delta)
{
  //used to change the static / standing position with change in coordinates
  mPosition.x = mPosition.x + delta.x;
  mPosition.y = mPosition.y + delta.y;
  //mPosition.z = mPosition.z + delta.z;
  if (mLeft)
  {
    mPosition.z = mPosition.z + delta.z;
  }else
  {
    mPosition.z = mPosition.z - delta.z;
  }
  
}
void MyLeg::updateStandPosition(unsigned long mills, unsigned long duration, double ramp)
{
  //this function moves the robots position if its not walking but the position has been changed
  angle a = IKSolver(mPosition.x, mPosition.y, mPosition.z);

  mTibia.StartMovement(mills, a.tibia_degree, duration,ramp);
  mFemur.StartMovement(mills, a.femur_degree, duration,ramp);
  mCoax.StartMovement(mills, a.coax_degree, duration,ramp); 
  mPositionPaw.x = mPosition.x;
  mPositionPaw.y = mPosition.y;
  mPositionPaw.z = mPosition.z;
  rotateAxis(a);
}
void MyLeg::DoStep( double percentage, double rotation[2], double y_rot, double x , double z)
{
  //this call updates the temperary position for leg during walking
  coords delta = StepCurve(percentage, rotation,y_rot,x,z);
  //my_log(MYLOG_INFO, "update walk position %s [%f]",mName, delta.y);
 
  WalkWithDelta(delta);
}
coords MyLeg::getStep( double percentage, double rotation[2], double y_rot, double x , double z)
{
  //this function is used when leg position relative to body origin is needed in certain stage of walking
  //used for example when trajectory for center of mass is calculated
  coords delta = StepCurve(percentage, rotation,y_rot,x,z);
  if (mFront)
  {
    delta.x = delta.x+ mPosition.x+BODY_LENGTH/2;
  }else
  {
    delta.x = delta.x+ mPosition.x-BODY_LENGTH/2;
  }
  if (mLeft)
  {
    delta.z = delta.z+ mPosition.z+BODY_WIDTH/2;
  }else
  {
    delta.z = delta.z+ mPosition.z-BODY_WIDTH/2;
  }
  delta.y = delta.y+ mPosition.y;
  return delta;
}
void MyLeg::WalkWithDelta(coords delta)
{
  //this is used to apply the coordinate deltas from the step function during walking
   mPositionPaw.x= mPosition.x +delta.x;
    double old = mPositionPaw.z;
   mPositionPaw.y= mPosition.y +delta.y;
   if (mLeft){
    mPositionPaw.z= mPosition.z +delta.z;
   }else{
    mPositionPaw.z= mPosition.z -delta.z;
   }
   
   //my_log(MYLOG_INFO, "update walk position %s [%f,%f,%f]",mName, old,mPositionPaw.z,delta.z);
}

legState MyLeg::getPoints()
{
  legState r;
  r.coax = mPositionShoulder;
  r.femur = mPositionKnee;
  r.tibia = mPositionPaw;
  if (mLeft)
  {
    r.tibia.z = r.tibia.z+ COAX_SIZE;
  }else
  {
    r.tibia.z = r.tibia.z- COAX_SIZE;
  }
  return r;
}

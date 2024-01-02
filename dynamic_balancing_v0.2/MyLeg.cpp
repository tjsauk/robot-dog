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
double MyLeg::vectorLenght(double x, double z)
{
  return sqrt((x*x)+(z*z));
}

coords MyLeg::rotateInPlace(double percentage,double radius,double y_rot)
{
  //returns coordinates relative to current legs origo that correspond to the next goal point in current walk cycle when turning with out moving
  double step_angl = (STEP_SIZE/radius)/4;//angle for one part of a step in radians
  double leg_angl = 52*(M_PI/180);//Front right leg center position angle in radians. 52 is the angle from x axis in degrees
  double goal_angl = 0.0;//angle from x axis in radians where the leg should be with circle in this radius
  double comp_angl = 0.0;
  double step_h = STEP_SWING_HEIGHT_FRONT;
  double x0 = 141;
  double z0 = 109;
  
  coords res = {0,0,0};
  //Rotate the goal if this leg is not front right 
  if (mLeft && mFront){//left front
    comp_angl += 75*M_PI/180;
    z0 = -109.0;
  } else if (mLeft && !mFront){//left rear
    comp_angl += M_PI;
    step_h = STEP_SWING_HEIGHT_REAR;
    z0 = -109.0;
    x0 = -141.0;
  } else if (!mLeft && !mFront){//Right rear
    comp_angl += (75*M_PI/180)+M_PI;
    step_h = STEP_SWING_HEIGHT_REAR;
    x0 = -141.0;
  }
  
  //while turning 0.375 is the perc value where leg should be exactly at center position
  if (percentage <= 0.75){//calculated for body turning right when y rot is positive so it switches direction if turning left
    goal_angl = map(percentage,0,0.75,-step_angl*y_rot,step_angl*y_rot);
  }else{//switching direction if currently at lifting phase 
    goal_angl = map(percentage,0.75,1,step_angl*y_rot,-step_angl*y_rot);
    double y0,y1,y2,y3;
    y0 = 0;
    y1 = step_h*4/3;
    y2 = step_h*4/3;
    y3 = 0;
    res.y = -(((1 - percentage) * ((1 - percentage) * ((1 - percentage) * y0 + percentage * y1) + percentage * ((1 - percentage) * y1 + percentage * y2)) +
               percentage * ((1 - percentage) * ((1 - percentage) * y1 + percentage * y2) + percentage * ((1 - percentage) * y2 + percentage * y3))));
    //if (percentage < 0.875){
    //  res.y = map(percentage,0.75,0.875,0,step_h);
    //}else{
    //  res.y = map(percentage,0.875,1,step_h,0);
    //}
  }
  goal_angl += comp_angl;
  goal_angl += leg_angl;

  //the coordinate system of caclulation is 90 degrees clockwise from body coordinates
  res.z = radius*cos(goal_angl)-z0;
  res.x = radius*sin(goal_angl)-x0;
  
  //my_log(MYLOG_INFO, " %s [%f,%f,%f,%f,%f,%f,%f,%f,%f]",mName, step_angl,leg_angl,comp_angl,goal_angl,x0,z0,res.x,res.y,res.z);
  return res;
}
coords MyLeg::rotateWhileWalking(double percentage,double y_rot,double x, double z){
  //#define MIN_TURN_RADIUS 200.0
//#define MAX_TURN_RADIUS
//somehow it currently does multiple
  coords res = {0,0,0};
  double step_h = STEP_SWING_HEIGHT_FRONT;
  double xl = 141.0;//these are the coordinates of right front in body origon coordinate system
  double zl = 109.0;
    if (mLeft && mFront){//left front
     zl = -109.0;
  } else if (mLeft && !mFront){//left rear
     zl = -109.0;
     xl = -141.0;
    step_h = STEP_SWING_HEIGHT_REAR;
  } else if (!mLeft && !mFront){//Right rear
    xl = -141.0;
    step_h = STEP_SWING_HEIGHT_REAR;
  }

//center of rotation in body origin coordinates. Step in z controls x and y rot controls z 
  double xr, zr; 
  if (y_rot > 0){//Turning right 
    zr = map(y_rot,0.0,1.0,MAX_TURN_RADIUS,MIN_TURN_RADIUS);
  }else{//Turning left 
    zr = map(y_rot,0.0,-1.0,-MAX_TURN_RADIUS,-MIN_TURN_RADIUS);
  }
  if (z == 0){//Z direction to right of the robot. Rotation center goes to negative x. Z direction should be only half as effective as others
   xr = 0;
  }else{
    //xr = -abs(zr)*(z/2);
    xr = -abs(zr)*(z);
  }

  //solving what is the half of the whole step in radians.
  double theta0 = vectorLenght((x*STEP_SIZE), (z*STEP_SIZE))/(2*vectorLenght(xr,zr));
  //double theta0 = vectorLenght((x*STEP_SIZE), (z*STEP_SIZE/2))/(2*vectorLenght(xr,zr));//this is the angle in radians for one step half from walk cycle centerpoint theta0=L/(2*r)
  //Walk steps are measured in radiand from z axis in counter clockwise direction. That means that when travelling forward
  //and turning right the legs go from smaller angle towards bigger angle when touching the ground. Reversed when turning left.
  //Those are allso reversed if going backwards or leg is lifted
  //this is the actual angle that represents the wanted step cycle n current situation
  double theta1;
  if (percentage <= 0.75){//Leg is not lifted
    if (y_rot > 0){//turning right
      theta1 = map(percentage,0.0,0.75,-theta0,theta0);
    }else{//turning left
      theta1 = map(percentage,0.0,0.75,theta0,-theta0);
    }
  }else{//Leg is lifted
    if (y_rot > 0){//turning right
      theta1 = map(percentage,0.75,1.0,theta0,-theta0);
    }else{//turning left
      theta1 = map(percentage,0.75,1.0,-theta0,theta0);
    }
    double y0,y1,y2,y3;
    y0 = 0;
    y1 = step_h*4/3;
    y2 = step_h*4/3;
    y3 = 0;
    res.y = -(((1 - percentage) * ((1 - percentage) * ((1 - percentage) * y0 + percentage * y1) + percentage * ((1 - percentage) * y1 + percentage * y2)) +
               percentage * ((1 - percentage) * ((1 - percentage) * y1 + percentage * y2) + percentage * ((1 - percentage) * y2 + percentage * y3))));
    //if (percentage < 0.875){//allso add the height change for leg if its currently being lifted
    //  res.y = map(percentage,0.75,0.875,0,step_h);
    //}else{
    //  res.y = map(percentage,0.875,1.0,step_h,0);
    //}
  }
  
  if (x < 0){//if direction of travel is backwards this is reversed.
    theta1 = theta1*(-1);
  }

  //theta 1 is to be added to current legs current angle. So next thing is to solve the angle of current leg 
  //to do so the coordinate system is to be changed to rotation center coordinates. xr and zr. 
  //leg coordinates in body origo xl aand zl

  double xn = xl - xr;
  double zn = zl - zr;

  //Solve for legs current angler in rotation center coordinates
  double theta2 = atan2(xn,zn);//theta = acos(x/r)
  //now add the angles theta1 and theta 2, multiply by radius and change back to body origin coordinates. Finally change to leg coordinates
  res.x = vectorLenght(xn,zn)* sin(theta1+theta2)+xr-xl;
  res.z = vectorLenght(xn,zn)*cos(theta1+theta2)+zr-zl;
  //my_log(MYLOG_INFO, " %s [%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]",mName, xl,zl,xr,zr,xn,zn,theta0,theta1,theta2,res.x,res.z);
  //my_log(MYLOG_INFO, "theta tot,1,2 percentage  %s [%f,%f,%f,%f]",mName, (theta1+theta2),theta1,theta2,percentage);
  return res;
}
struct coords MyLeg::StepCurve(double percentage, double y_rot, double x, double z)
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

  if (y_rot == 0){//condition to be changed for linear motion y_rot == 0
    if (false){//Dumb turning vhange to false when real turning is operational
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
  step_size_z = STEP_SIZE*z;
  }else{
    step_size_x = STEP_SIZE*x;
    step_size_z = STEP_SIZE*z;
  }

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
  }else if (x == 0 && z == 0){//Condition for rotating in place x == 0 && z == 0
    return rotateInPlace(percentage,198.3,y_rot);
  }else{
    return rotateWhileWalking(percentage,y_rot,x, z);
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
  if (a.tibia_degree < 97){
    a.tibia_degree=97;
  }
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
  if (a.tibia_degree < 97){
    a.tibia_degree=97;
  }
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
coords MyLeg::getPawPosition(){
  //returns x,y,z of the paw in leg coordinates. where the end of coax is zero
  if (mLeft){
    return mPositionPaw;
  }
  else{//Right feet are reverset that results in the z axis being wrong way round. 
    coords result = mPositionPaw;
    result.z = -mPositionPaw.z;
    return result;
  }
  
}
coords MyLeg::getPawPositionInBody(){
  //returns x,y,z of the paw in body coordinates. where center of the body is zero
  coords result = mPositionPaw;
  if (mLeft){//leftside is unreversed so it can directly be subtracked
    result.z -= BODY_WIDTH/2+COAX_SIZE;
  }else{//right side is positive but the leg is reversed in z coordinates so it is done differently
    result.z = BODY_WIDTH/2+COAX_SIZE-mPositionPaw.z;
  }
  if (mFront){//front is positive x
    result.x+= BODY_LENGTH/2;
  }else{
    result.x-= BODY_LENGTH/2;
  }
  return result;
}
void MyLeg::updateWalkPosition(unsigned long mills, unsigned long duration, double ramp)
{
  //this is used after all temperary movement is inputted to the temperary position variable
  //and the movement needs to be executed
  angle a = IKSolver(mPositionPaw.x, mPositionPaw.y, mPositionPaw.z);
  //my_log(MYLOG_INFO, "update walk position %s [%f,%f,%f]",mName, a.tibia,a.femur,a.coax);
  if (a.tibia_degree < 97){
    a.tibia_degree=97;
  }
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
  //my_log(MYLOG_INFO, "update stand position %s [%f,%f,%f]",mName, a.tibia_degree,a.femur_degree,a.coax_degree);
  if (a.tibia_degree < 97){
    a.tibia_degree=97;
  }
  mTibia.StartMovement(mills, a.tibia_degree, duration,ramp);
  mFemur.StartMovement(mills, a.femur_degree, duration,ramp);
  mCoax.StartMovement(mills, a.coax_degree, duration,ramp); 
  mPositionPaw.x = mPosition.x;
  mPositionPaw.y = mPosition.y;
  mPositionPaw.z = mPosition.z;
  rotateAxis(a);
}
void MyLeg::DoStep( double percentage,  double y_rot, double x , double z)
{
  //this call updates the temperary position for leg during walking
  coords delta = StepCurve(percentage, y_rot,x,z);
  //my_log(MYLOG_INFO, "update walk position %s [%f,%f,%f]",mName, delta.x,delta.y,delta.z);
 
  WalkWithDelta(delta);
}
coords MyLeg::getStep( double percentage,  double y_rot, double x , double z)
{
  //this function is used when leg position relative to body origin is needed in certain stage of walking
  //used for example when trajectory for center of mass is calculated
  coords delta = StepCurve(percentage, y_rot,x,z);
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

#include "MyBody.h"
#include "MyLog.h"
#include <list>

#include "myconfig.h"


//Coordinates of body are defined as (x,y,z). All coordinates are from the center of body
//x axis goes with positive being forward and negative being backwards
//y axis for the body is positive down wards as (offset from body origo) and negative up
//z axis is positive at the right side of the body and negative at left side of the body
//leg coordinate systems attach to body COAX distance away from the wf corners in z direction
//For example Left Front legs coordinate zero is at x=half of body lenght y=0 and z= half of body width + coax len
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

  //steering parameters
  sVelocity = 0;
  sDegree_x = 0;
  sDegree_y = 0;
  sDegree_z = 0;
  sDirection_x = 0;
  sDirection_y = 0;
  sDirection_z = 0;
  sRamp = 0;

  //Bodys corners initialised in config file walk position
  //body coordinates are at y=0 and x,z 0 at the center of body measured from coax rotation axis
  double len = BODY_LENGTH/2;
  double wid = BODY_WIDTH/2;
  double heig = (WALK_POSITION_FRONT_Y-WALK_POSITION_REAR_Y)/2;//If front value is greater result is positive
  wfLeftFront = {len,heig,wid};
  wfLeftRear = {-len,-heig,wid};
  wfRightFront = {len,heig,-wid};
  wfRightRear = {-len,-heig,-wid};
//COM is the center of mass, it is considered to be at the center of body in y direction and offset can be added in z and x direction
  wfCOM = {COM_X_OFFSET,0.0,COM_Z_OFFSET};
  
}

void MyBody::setGait(int mode)
{
  if (mode == GAIT_WALK)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.75;
    mFootDelays.rearRight = 0.25;
    COMdelay = 0.125;
  }
  else if (mode == GAIT_TROT)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.50;
    mFootDelays.rearRight = 0.00;
  }
  else if (mode == JACK_SPARROW)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.25;
    mFootDelays.rearRight = 0.75;
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
// Function to calculate intersection
coords MyBody::intersection(const coords LF, const coords RR, const coords RF, const coords LR) {
  //this returns the point in body coordinates where lines defined by the points from (LF to RR) and (RF to LR) intersect
  //returns only x and z coordinates. Height of the point is not handled because this is supposed to be point in plane
    coords result;

    result.x = ((LF.x * RR.z - LF.z * RR.x) * (RF.x - LR.x) - (LF.x - RR.x) * (RF.x * LR.z - RF.z * LR.x)) / ((LF.x - RR.x) * (RF.z - LR.z) - (LF.z - RR.z) * (RF.x - LR.x));

    result.z = ((LF.x * RR.z - LF.z * RR.x) * (RF.z - LR.z) - (LF.z - RR.z) * (RF.x * LR.z - RF.z * LR.x)) / ((LF.x - RR.x) * (RF.z - LR.z) - (LF.z - RR.z) * (RF.x - LR.x));
    result.y = 0;   
    return result;
}
void MyBody::setCOMTrajectory(double distance) {
  //this calculetes the theoretical trajectory for center of mass during walk cycle
  //calculations and the trajectory is done by finding the intersection point of support polygon hyphotenuses
  //when step cycle switches between rear and fron legs
  //coords left corresponds the point of intersection when right front and back legs rise
  //coordinates need to be compensated for COM offset before use
        double rot_point[2] = {0, 0};
        double y_rot = 0;
        double x = 0;
        double z = 0;
        //float c_times[4] = {0.25, 0.5, 0.25, 0.5};
        //float l_times[4] = {0.5, 0.75, 0, 0.25};
        //float r_times[4] = {0, 0.25, 0.5, 0.75};

        //coords LF = mLeftFront.getStep(0.25, rot_point, y_rot, x, z);
        //coords RR = mRightRear.getStep(0.5, rot_point, y_rot, x, z);
        //coords RF = mRightFront.getStep(0.25, rot_point, y_rot, x, z);
        //coords LR = mLeftRear.getStep(0.5, rot_point, y_rot, x, z);

        //coords center = intersection(LF, RR, RF, LR);
        //COM trajectory [-17.5, 0.0, -10.0][ 2.5,  0.0,  10.0][ -17.5,  0.0,  10.0][ 2.5,  0.0,  -10.0]

        coords LF = mLeftFront.getStep(0.5, rot_point, y_rot, x, z);
        coords RR = mRightRear.getStep(0.75, rot_point, y_rot, x, z);
        coords RF = mRightFront.getStep(0.0, rot_point, y_rot, x, z);
        coords LR = mLeftRear.getStep(0.25, rot_point, y_rot, x, z);
        //my_log(MYLOG_INFO, "Set Trajectory left leg points. LF,RR,RF,LR [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f]", LF.x, LF.y, LF.z, RR.x, RR.y, RR.z, RF.x, RF.y, RF.z, LR.x, LR.y, LR.z);
        coords left = intersection(LF, RR, RF, LR);
        //my_log(MYLOG_INFO, "Set Trajectory left point.  [%.1f, %d, %.1f]", left.x, left.y, left.z);
        LF = mLeftFront.getStep(0.0, rot_point, y_rot, x, z);
        RR = mRightRear.getStep(0.25, rot_point, y_rot, x, z);
        RF = mRightFront.getStep(0.5, rot_point, y_rot, x, z);
        LR = mLeftRear.getStep(0.75, rot_point, y_rot, x, z);
        //my_log(MYLOG_INFO, "Set Trajectory right leg points. LF,RR,RF,LR [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f]", LF.x, LF.y, LF.z, RR.x, RR.y, RR.z, RF.x, RF.y, RF.z, LR.x, LR.y, LR.z);
        coords right = intersection(LF, RR, RF, LR);
        //my_log(MYLOG_INFO, "Set Trajectory right point.  [%.1f, %d, %.1f]", right.x, right.y, right.z);
        double z_pituus = sqrt(pow(left.z, 2) + pow(right.z, 2)) / 2;

        // A: back right sector -x -y, B: front left sector x y, C: back left sector -x y, D: front right sector x -y
        
        COMTrajectory[0].x = right.x - z_pituus - distance;
        COMTrajectory[0].z =right.z - distance;
        
        COMTrajectory[3].x =left.x + z_pituus + distance;
        COMTrajectory[3].z =left.z + distance;
        
        COMTrajectory[2].x =left.x - z_pituus - distance;
        COMTrajectory[2].z =left.z + distance;
        
        COMTrajectory[1].x =right.x + z_pituus + distance;
        COMTrajectory[1].z =right.z - distance;

    }




void MyBody::update(unsigned long mills)
{
  //this function is to be called constantly in the main loop to update motor statemachines and changes in steering
  PositionUpdate(mills); // update 
  
  mLeftFront.update(mills);
  mLeftRear.update(mills);
  mRightFront.update(mills);
  mRightRear.update(mills);
}

coords MyBody::linear4point(double percentage, coords trajectory[4])
{
  //executes linear change between 4 points in order A,B,C,D based on the percentage value that can be 0 to 1
  coords A = trajectory[0];
  coords B = trajectory[1];
  coords C = trajectory[2];
  coords D = trajectory[3];
  coords r;
  r.y = 0;
  if (percentage < 0.25)
  {
    r.x = A.x+(B.x-A.x)*percentage*4;
    r.z = A.z+(B.z-A.z)*percentage*4;
  }else if (percentage < 0.5)
  {
    r.x = B.x+(C.x-B.x)*(percentage-0.25)*4;
    r.z = B.z+(C.z-B.z)*(percentage-0.25)*4;    
  }else if (percentage < 0.75)
  {
    r.x = C.x+(D.x-C.x)*(percentage-0.5)*4;
    r.z = C.z+(D.z-C.z)*(percentage-0.5)*4;    
  }else
  {
    r.x = D.x+(A.x-D.x)*(percentage-0.75)*4;
    r.z = D.z+(A.z-D.z)*(percentage-0.75)*4;     
  }
  return r;
}

void MyBody::addOffsetTo(legState& leg,coords offset) {
    leg.coax.x += offset.x;
    leg.femur.x += offset.x;
    leg.tibia.x += offset.x;
    leg.coax.y += offset.y;
    leg.femur.y += offset.y;
    leg.tibia.y += offset.y;
    leg.coax.z += offset.z;
    leg.femur.z += offset.z;
    leg.tibia.z += offset.z;
}

legState MyBody::getLegPoints(bool left,bool front)
{
  legState r;
  if (front && left)//front left getPoints()
  {
    r = mLeftFront.getPoints();
    addOffsetTo(r,wfLeftFront);
  }else if (front && !left)//front right
  {
    r = mRightFront.getPoints();
    addOffsetTo(r,wfRightFront);
  }else if (left && !front)//left rear
  {
    r = mLeftRear.getPoints();
    addOffsetTo(r,wfLeftRear);
  }else//right rear
  {
    r = mRightRear.getPoints();
    addOffsetTo(r,wfRightRear);
  }
  return r;
}
void MyBody::COMtobody()
{
  //calculates the effect of leg movement to the location of center of mass
  //calculations are done by considering robot being in balance while legs are straight
  //DOES NOT take in to account the possible rotation of the body. To make that happen there needs to be 
  //static COM position that is not effected by this function and that needs to be used in place of COM_X_OFFSET and COM_Z_OFFSET
  //If this is done change the static COM allso in the function rotateBody()
  legState LF = getLegPoints(true, true);
  legState LR = getLegPoints(true, false);
  legState RF = getLegPoints(false, true);
  legState RR = getLegPoints(false,false);

  double m = COM_WEIGHT;
  double x_off = COM_X_OFFSET;
  double z_off = COM_Z_OFFSET;
  double x_com = (m*(LF.femur.x-x_off)+m*(LR.femur.x-x_off)+m*(RF.femur.x-x_off)+m*(RR.femur.x-x_off)+4*m*(x_off*(-2)))/(8*m);
  double z_com = (m*(LF.femur.z-z_off)+m*(LR.femur.z-z_off)+m*(RF.femur.z-z_off)+m*(RR.femur.z-z_off))/(4*m);
  //wfCOM.x = x_off + x_com;
  //wfCOM.z = z_off + z_com;
}
void MyBody::WalkDirection(double velocity, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp)//suunnat on asteikolla 0-1 mistä 0 on täysillä siihen suuntaan. y on korkeus + -
{
  //this function updates the steering parameters so this is to be called every time something changes
  sVelocity = velocity;
  sDegree_x=degree_x;
  sDegree_y=degree_y;
  sDegree_z=degree_z;
  sDirection_x=x;
  sDirection_y=y;
  sDirection_z=z;
  sRamp=ramp;
}

coords MyBody::atplane(coords point)
{
  //this function is used to project points using IMU to the ground plane
  return point;
}
quaternion MyBody::quaternionMultiply(quaternion q,quaternion p)
{
  //Multiplies the given quaternions and returns a new quaternion
  quaternion r;
  r.a = q.a*p.a-q.x*p.x-q.y*p.y-q.z*p.z;
  r.x = q.x*p.a+q.a*p.x-q.z*p.y+q.y*p.z;
  r.y = q.y*p.a+q.z*p.x+q.a*p.y-q.x*p.z;
  r.z = q.z*p.a-q.y*p.x+q.x*p.y+q.a*p.z;
  return r;
}
quaternion MyBody::quaternionConjugate(quaternion v)
{
  //converts the given quaternion to its conjugate
  quaternion q;
  q.a = 0;
  q.x = -v.x;
  q.y = -v.y;
  q.z = -v.z;
  return q;
}

double MyBody::degRad(double a)
  {
    return ((M_PI / 180) * a);
  }

corners MyBody::rotateBody(double x, double z)
{
  
  my_log(MYLOG_INFO, "Rotate body with angles  [%.f, %.f]", x, z);
  corners result;
  //changes the position of the pody wireframe corners takes in degrees
  quaternion qx = {cos(degRad(x)/2),sin(degRad(x)/2),0,0};//rotation in x axis
  quaternion qz = {cos(degRad(z)/2),0,0,sin(degRad(z)/2)};//rotation in z axis
  quaternion qt = quaternionMultiply(qx,qz);//total rotation
  my_log(MYLOG_INFO, "Rotate body qx [%.1f, %.1f, %.1f, %.1f]", qx.a, qx.x, qx.y,qx.z);
  my_log(MYLOG_INFO, "Rotate body qz [%.1f, %.1f, %.1f, %.1f]", qz.a, qz.x, qz.y,qz.z);
  my_log(MYLOG_INFO, "Rotate body qt [%.1f, %.1f, %.1f, %.1f]", qt.a, qt.x, qt.y,qt.z);
  //take point
  quaternion qr = {0,wfLeftFront.x,wfLeftFront.y,wfLeftFront.z};//quaternion used in rotations
  //rotate it 
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  //output results
  my_log(MYLOG_INFO, "Rotate body wfLeftFront before [%.1f, %.1f, %.1f]", wfLeftFront.x, wfLeftFront.y,wfLeftFront.z);
  wfLeftFront = {qr.x,qr.y,qr.z};
  my_log(MYLOG_INFO, "Rotate body wfLeftFront after [%.1f, %.1f, %.1f]", wfLeftFront.x, wfLeftFront.y,wfLeftFront.z);
  result.A = {qr.x,qr.y,qr.z};
  //repeate
  qr = {0,wfLeftRear.x,wfLeftRear.y,wfLeftRear.z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  wfLeftRear = {qr.x,qr.y,qr.z};
  result.B = {qr.x,qr.y,qr.z};
  
  qr = {0,wfRightRear.x,wfRightRear.y,wfRightRear.z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  wfRightRear = {qr.x,qr.y,qr.z};
  result.C = {qr.x,qr.y,qr.z};

  qr = {0,wfRightFront.x,wfRightFront.y,wfRightFront.z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  wfRightFront = {qr.x,qr.y,qr.z};
  result.D = {qr.x,qr.y,qr.z};
  //rotate the COM allso. This has basily no effect at the moment
  qr = {0,wfCOM.x,wfCOM.y,wfCOM.z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  wfCOM = {qr.x,qr.y,qr.z};
  return result;
}
void MyBody::PositionUpdate(unsigned long mills)
{
  //makes the steering parameters effective
  unsigned long total_step_points = 50; // the foot trajectory is divided in 50 points
  unsigned long time_delta = 0;
  unsigned long newDuration = (STEP_SIZE/sVelocity);
 
    //update according the steering input
  if (newDuration != mWalkDuration)
  {
    mWalkDuration = newDuration;
    time_delta = mWalkDuration / total_step_points;
  }
  else
  {
    time_delta = mWalkDuration / total_step_points;
    }

   if (mills - mLastUpdate < time_delta)
    return;

  mLastUpdate = mills;

  //check that there is steering input before doing steps
  if (sDirection_x != 0 || sDirection_z != 0 || sDegree_y != 0)
  {
    
    double perc_FL = (double)((mWalkStep + (int)(mFootDelays.frontLeft * total_step_points)) % total_step_points) / (double)total_step_points;//prosentteina kävely sykli
    double perc_FR = (double)((mWalkStep + (int)(mFootDelays.frontRight * total_step_points)) % total_step_points) / (double)total_step_points;
    double perc_RL = (double)((mWalkStep + (int)(mFootDelays.rearLeft * total_step_points)) % total_step_points) / (double)total_step_points;
    double perc_RR = (double)((mWalkStep + (int)(mFootDelays.rearRight * total_step_points)) % total_step_points) / (double)total_step_points;
    double perc_COM = (double)((mWalkStep + (int)(COMdelay * total_step_points)) % total_step_points) / (double)total_step_points;
    
    double rotation[2] = {0,0};
    if (sDegree_y > 0)
    {
      rotation[0] = (BODY_LENGTH+MAX_TURN_RADIUS*(1-sDegree_y))*sDirection_z;
      rotation[1] = (BODY_LENGTH+MAX_TURN_RADIUS*(1-sDegree_y))*sDirection_x;
    }else if (sDegree_y < 0)
    {
      rotation[0] = (-BODY_LENGTH+MAX_TURN_RADIUS*(-1-sDegree_y))*sDirection_z;
      rotation[1] = (-BODY_LENGTH+MAX_TURN_RADIUS*(-1-sDegree_y))*sDirection_x;
    }
   
    
    mRightFront.DoStep(perc_FR, rotation,sDegree_y ,sDirection_x,sDirection_z);
    mLeftFront.DoStep(perc_FL, rotation,sDegree_y ,sDirection_x,sDirection_z);
    mRightRear.DoStep(perc_RR, rotation,sDegree_y ,sDirection_x,sDirection_z);
    mLeftRear.DoStep(perc_RL, rotation,sDegree_y ,sDirection_x,sDirection_z);
  
    coords COMto = linear4point(perc_COM, COMTrajectory);

    coords COMat = atplane(wfCOM);

    coords COMmove = {(COMto.x-COMat.x+COM_X_OFFSET),(COMto.y-COMat.y),(COMto.z-COMat.z+COM_Z_OFFSET)};
    //my_log(MYLOG_INFO, "COM to [%.1f, %.1f, %.1f]", COMto.x,COMto.y,COMto.z);
     //my_log(MYLOG_INFO, "COMat [%.1f, %.1f, %.1f]", COMat.x,COMat.y,COMat.z);
     //my_log(MYLOG_INFO, "COMmove [%.1f, %.1f, %.1f]", COMmove.x,COMmove.y,COMmove.z);

    

    mRightFront.addToWalk(COMmove);
    mLeftFront.addToWalk(COMmove);
    mRightRear.addToWalk(COMmove);
    mLeftRear.addToWalk(COMmove); 
    
  
    
    mRightFront.updateWalkPosition(mills, time_delta, sRamp);
    mLeftFront.updateWalkPosition(mills, time_delta, sRamp);
    mRightRear.updateWalkPosition(mills, time_delta, sRamp);
    mLeftRear.updateWalkPosition(mills, time_delta, sRamp);

    COMtobody();
  mWalkStep++;
  }//end of walking JÄÄJUMIIN TÄHÄN SILLÄ KOKOAJAN KÄSKETÄÄN MENEE TÄHÄN ASENTOON
  else if (sDirection_y != 0 || sDegree_x != 0 || sDegree_z != 0)//not walking but body changing position
  { 
    
    //Käännöt ei rokkaa y toimii

    
    //movement in one step of one cycle
    double y_dir = (Y_STEP_SIZE/total_step_points)*sDirection_y;
    double x_rot = (X_STEP_ANGLE/total_step_points)*sDegree_x;
    double z_rot = (Z_STEP_ANGLE/total_step_points)*sDegree_z;

  coords LF;
  coords LR;
  coords RR;
  coords RF;
  
  if (sDegree_x != 0 || sDegree_z != 0)
  {
    //my_log(MYLOG_INFO, "Not walking steps before y dir x rot z rot  [%.1f, %.1f, %.1f]", y_dir,x_rot,z_rot);
    corners delta = rotateBody(x_rot, z_rot);//leftfront, leftrear,rightrear,rightfront x_rot, z_rot
    my_log(MYLOG_INFO, "Not walking steps after y dir x rot z rot  [%.1f, %.1f, %.1f]", y_dir,x_rot,z_rot);
    LF = {delta.A.x,y_dir+delta.A.y,delta.A.z};
    LR = {delta.B.x,y_dir+delta.B.y,delta.B.z};
    RR = {delta.C.x,y_dir+delta.C.y,delta.C.z};
    RF = {delta.D.x,y_dir+delta.D.y,delta.D.z};
  }
  else
  {
      LF = {0,y_dir,0};
      LR = {0,y_dir,0};
      RR = {0,y_dir,0};
      RF = {0,y_dir,0};
  }
  my_log(MYLOG_INFO, "Not walking position change LF  [%.1f, %.1f, %.1f]", LF.x, LF.y, LF.z);
  mLeftFront.PositionWithDelta(LF);
  mRightFront.PositionWithDelta(RF);
  mRightRear.PositionWithDelta(RR);
  mLeftRear.PositionWithDelta(LR);

    mRightFront.updateStandPosition(mills, time_delta, sRamp);
    mLeftFront.updateStandPosition(mills, time_delta, sRamp);
    mRightRear.updateStandPosition(mills, time_delta, sRamp);
    mLeftRear.updateStandPosition(mills, time_delta, sRamp);

    setCOMTrajectory(BALANCE_SAFETY_MARGIN);

    COMtobody();
    
  }else//not walking or changing position
  {
    GoWalkPosition(mills, time_delta, sRamp);
    COMtobody();
  }
  

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
  mLeftFront.BackToPosition(mills, duration, ramp);
  mLeftRear.BackToPosition(mills, duration, ramp);
  mRightFront.BackToPosition(mills, duration, ramp);
  mRightRear.BackToPosition(mills, duration, ramp);
  COMtobody();
  setCOMTrajectory(BALANCE_SAFETY_MARGIN);
  //my_log(MYLOG_INFO, "COM trajectory [%.1f, %.1f, %.1f][ %.1f,  %.1f,  %.1f][ %.1f,  %.1f,  %.1f][ %.1f,  %.1f,  %.1f]", COMTrajectory[0].x,COMTrajectory[0].y,COMTrajectory[0].z, COMTrajectory[1].x,COMTrajectory[1].y,COMTrajectory[1].z, COMTrajectory[2].x,COMTrajectory[2].y,COMTrajectory[2].z, COMTrajectory[3].x, COMTrajectory[3].y, COMTrajectory[3].z);

  
}

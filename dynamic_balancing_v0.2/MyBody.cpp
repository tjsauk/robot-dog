#include "MyBody.h"
#include "MyLog.h"
#include <list>
#include "myconfig.h"
#include "MPU6050_conf.h"



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
  mGait = GAIT_WALK;

  //steering parameters
  sVelocity = 0;
  sDegree_x = 0;
  sDegree_y = 0;
  sDegree_z = 0;
  sDirection_x = 0;
  sDirection_y = 0;
  sDirection_z = 0;
  sRamp = 0;
  sPositions = 1;//
  //ax;
  //Bodys corners initialised in config file walk position
  //body coordinates are at y=0 and x,z 0 at the center of body measured from coax rotation axis
  double len = BODY_LENGTH/2;
  double wid = BODY_WIDTH/2+COAX_SIZE;
  double heig = (WALK_POSITION_FRONT_Y+WALK_POSITION_REAR_Y)/2;//If front value is greater result is positive
  wfLeftFront = {len,WALK_POSITION_FRONT_Y,-wid};
  wfLeftRear = {-len,WALK_POSITION_REAR_Y,-wid};//alunperin negatiivinen on alempana
  wfRightFront = {len,WALK_POSITION_FRONT_Y,wid};
  wfRightRear = {-len,WALK_POSITION_REAR_Y,wid};
  wfCenter = {0,heig,0};
//COM is the center of mass, it is considered to be at the center of body in y direction and offset can be added in z and x direction
  double comheig = abs(WALK_POSITION_FRONT_Y-heig)/len * COM_X_OFFSET+ heig;
  wfCOM = {COM_X_OFFSET,comheig,COM_Z_OFFSET};
  balancing = {25,15,0};
  
}

void MyBody::setIMUzero(){
  //captures a reference point from home position that shows what is considered to be straight tovards the floor
  accZero = {ax,ay,az};
  gyrZero={gx,gy,gz};
}
double MyBody::getHeight()
{//wf means wireframe that is supposed to know the center and corner points of the pody, where corners are the leg coordinate 0 points
  return wfCenter.y;
}
void MyBody::setBalancing(int x, int y)
{//the two parameters used in balancing, y is the amount of change in y height dyring the cycle and x is the lenght of travel in x axis during it
  balancing.x = x;
  balancing.y = y;
}
void MyBody::setGait(int mode)
{
  if (mode == GAIT_WALK)
  {
    mFootDelays.frontLeft = 0.25;
    mFootDelays.frontRight = 0.75;
    mFootDelays.rearLeft = 0.0;
    mFootDelays.rearRight = 0.50;
    COMdelay = 0.88;//0.125+0.5;//this is the timing of the balancing cycle
    mGait = GAIT_WALK;
  }
  else if (mode == GAIT_TROT)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.50;
    mFootDelays.rearRight = 0.00;
    mGait = GAIT_TROT;
  }
  else if (mode == JACK_SPARROW)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.25;
    mFootDelays.rearRight = 0.75;
    mGait = JACK_SPARROW;
  }
  else if (mode == GAIT_PACE)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.50;
    mFootDelays.rearLeft = 0.00;
    mFootDelays.rearRight = 0.50;
    mGait = GAIT_PACE;

  }
  else if (mode == GAIT_CANTER)
  {
    mFootDelays.frontLeft = 0.00;
    mFootDelays.frontRight = 0.30;
    mFootDelays.rearLeft = 0.70;
    mFootDelays.rearRight = 0.00; 
    mGait = GAIT_CANTER;
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
coords MyBody::intersection(const coords LF, const coords RR, const coords RF, const coords LR) {//NOTUSED
  //this returns the point in body coordinates where lines defined by the points from (LF to RR) and (RF to LR) intersect
  //returns only x and z coordinates. Height of the point is not handled because this is supposed to be point in plane
    coords result;

    result.x = ((LF.x * RR.z - LF.z * RR.x) * (RF.x - LR.x) - (LF.x - RR.x) * (RF.x * LR.z - RF.z * LR.x)) / ((LF.x - RR.x) * (RF.z - LR.z) - (LF.z - RR.z) * (RF.x - LR.x));

    result.z = ((LF.x * RR.z - LF.z * RR.x) * (RF.z - LR.z) - (LF.z - RR.z) * (RF.x * LR.z - RF.z * LR.x)) / ((LF.x - RR.x) * (RF.z - LR.z) - (LF.z - RR.z) * (RF.x - LR.x));
    result.y = 0;   
    return result;
}
void MyBody::setCOMTrajectory(double distance) {//NOTUSED
  //this calculetes the theoretical trajectory for center of mass during walk cycle
  //calculations and the trajectory is done by finding the intersection point of support polygon hyphotenuses
  //when step cycle switches between rear and fron legs
  //coords left corresponds the point of intersection when right front and back legs rise
  //coordinates need to be compensated for COM offset before use
  //A=-x,-z, B=x,-z, C=-x,z, D = x,z
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

        coords LF = mLeftFront.getStep(0.5,  y_rot, x, z);
        coords RR = mRightRear.getStep(0.75,  y_rot, x, z);
        coords RF = mRightFront.getStep(0.0,  y_rot, x, z);
        coords LR = mLeftRear.getStep(0.25,  y_rot, x, z);
        //my_log(MYLOG_INFO, "Set Trajectory left leg points. LF,RR,RF,LR [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f]", LF.x, LF.y, LF.z, RR.x, RR.y, RR.z, RF.x, RF.y, RF.z, LR.x, LR.y, LR.z);
        coords left = intersection(LF, RR, RF, LR);
        //my_log(MYLOG_INFO, "Set Trajectory left point.  [%.1f, %d, %.1f]", left.x, left.y, left.z);
        LF = mLeftFront.getStep(0.0,  y_rot, x, z);
        RR = mRightRear.getStep(0.25,  y_rot, x, z);
        RF = mRightFront.getStep(0.5,  y_rot, x, z);
        LR = mLeftRear.getStep(0.75,  y_rot, x, z);
        //my_log(MYLOG_INFO, "Set Trajectory right leg points. LF,RR,RF,LR [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f] [%.1f, %d, %.1f]", LF.x, LF.y, LF.z, RR.x, RR.y, RR.z, RF.x, RF.y, RF.z, LR.x, LR.y, LR.z);
        coords right = intersection(LF, RR, RF, LR);
        //my_log(MYLOG_INFO, "Set Trajectory right point.  [%.1f, %d, %.1f]", right.x, right.y, right.z);
        double z_pituus = sqrt(pow(left.z, 2) + pow(right.z, 2)) / 2;

        //A=-x,-z, B=x,-z, C=-x,z, D = x,z
        
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
  //this calls update for all legs, with this the legs know to move. This should be called constantly from main loop
  //my_log(MYLOG_INFO, "COM to coordinates: [ %f, %f]", COMto.x,COMto.z);
  mLeftFront.update(mills);
  mLeftRear.update(mills);
  mRightFront.update(mills);
  mRightRear.update(mills);
}

void MyBody::testUpdate(unsigned long mills)
{
  mLeftFront.update(mills);
  mLeftRear.update(mills);
  mRightFront.update(mills);
  mRightRear.update(mills);
}

coords MyBody::linear4point(double percentage, coords trajectory[4])//NOTUSED
{
  //executes linear change between 4 points in order A,B,C,D based on the percentage value that can be 0 to 1
  //A=-x,-z, B=x,-z, C=-x,z, D = x,z
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
    my_log(MYLOG_INFO, "Com trajectory going to B: [%f, %f]",r.x , r.z);
  
  }else if (percentage < 0.5)
  {
    r.x = B.x+(C.x-B.x)*(percentage-0.25)*4;
    r.z = B.z+(C.z-B.z)*(percentage-0.25)*4;  
    my_log(MYLOG_INFO, "Com trajectory going to C: [%f, %f]",r.x , r.z); 
  }else if (percentage < 0.75)
  {
    r.x = C.x+(D.x-C.x)*(percentage-0.5)*4;
    r.z = C.z+(D.z-C.z)*(percentage-0.5)*4;  
    my_log(MYLOG_INFO, "Com trajectory going to D: [%f, %f]",r.x , r.z);
  }else
  {
    r.x = D.x+(A.x-D.x)*(percentage-0.75)*4;
    r.z = D.z+(A.z-D.z)*(percentage-0.75)*4; 
    my_log(MYLOG_INFO, "Com trajectory going to A: [%f, %f]",r.x , r.z);  
  }
  return r;
}

corners MyBody::linearFollowCompensate(double percentage)
{//this drives the balancing cycle. X goes two times from positive to negative and legs follow the yValues lists given targets.
  //percentage timing starts at 0.87
  //executes linear change between 4 points in order A,B,C,D based on the percentage value that can be 0 to 1
  //A=-x,-z, B=x,-z, C=-x,z, D = x,z
  //Coordinates A,B,C,D are deltas for all legs
  //balancing = {25,15,0};
  double x_comp = balancing.x*(cos(4*M_PI*percentage));//Results to a -25 when percentage is at 0. Oskillates 2 times in every period
  double y_stp = balancing.y/6;
  double yValues[50][4] = {
        {-6.0, 0.0, 0.0, 0.0},//0.0 A at -12
        {-6.0, -1.0, 0.0, 0.0},//0.02 B going down
        {-6.0, -2.0, 0.0, 0.0},//0.04
        {-6.0, -3.0, 0.0, 0.0},//0.06
        {-6.0, -4.0, 0.0, 0.0},//0.08
        {-6.0, -5.0, 0.0, 0.0},//0.10
        {-6.0, -6.0, 0.0, 0.0},//0.12 B at -12 
        {-5.0, -6.0, 0.0, 0.0},//0.14 A going up
        {-4.0, -6.0, 0.0, 0.0},//0.16
        {-3.0, -6.0, 0.0, 0.0},//0.18
        {-2.0, -6.0, 0.0, 0.0},//0.20
        {-1.0, -6.0, 0.0, 0.0},//0.22
        {0.0, -6.0, 0.0, 0.0},//0.24 B at -12 A at 0
        {0.0, -5.0, 0.0, 0.0},//0.26 B going up
        {0.0, -4.0, 0.0, 0.0},//0.28
        {0.0, -3.0, 0.0, 0.0},//0.30
        {0.0, -2.0, 0.0, 0.0},//0.32
        {0.0, -1.0, 0.0, 0.0},//0.34----------------------------------------------------
        {0.0, 0.0, 0.0, 0.0},//0.36 B at 0 C at 0
        {0.0, 0.0, -1.0, 0.0},//0.38 C going down
        {0.0, 0.0, -2.0, 0.0},//0.40
        {0.0, 0.0, -3.0, 0.0},//0.42
        {0.0, 0.0, -4.0, 0.0},//0.44
        {0.0, 0.0, -5.0, 0.0},//0.46 
        {0.0, 0.0, -6.0, 0.0},//0.48 C is at -12
        {0.0, 0.0, -6.0, 0.0},//0.50 
        {0.0, 0.0, -6.0, -1.0},//0.52 D going down
        {0.0, 0.0, -6.0, -2.0},//0.54
        {0.0, 0.0, -6.0, -3.0},//0.56
        {0.0, 0.0, -6.0, -4.0},//0.58
        {0.0, 0.0, -6.0, -5.0},//0.60
        {0.0, 0.0, -6.0, -6.0},//0.62 D at -12 --------------------------
        {0.0, 0.0, -5.0, -6.0},//0.64 C going up
        {0.0, 0.0, -4.0, -6.0},//0.66
        {0.0, 0.0, -3.0, -6.0},//0.68
        {0.0, 0.0, -2.0, -6.0},//0.70
        {0.0, 0.0, -1.0, -6.0},//0.72
        {0.0, 0.0, 0.0, -6.0},//0.74 C at 0
        {0.0, 0.0, 0.0, -6.0},//0.76
        {0.0, 0.0, 0.0, -5.0},//0.78 D going up
        {0.0, 0.0, 0.0, -4.0},//0.80
        {0.0, 0.0, 0.0, -3.0},//0.82
        {0.0, 0.0, 0.0, -2.0},//0.84
        {0.0, 0.0, 0.0, -1.0},//0.86
        {0.0, 0.0, 0.0, 0.0},//0.88 D at 0
        {-1.0, 0.0, 0.0, 0.0},//0.90 A going down
        {-2.0, 0.0, 0.0, 0.0},//0.92
        {-3.0, 0.0, 0.0, 0.0},//0.94
        {-4.0, 0.0, 0.0, 0.0},//0.96
        {-5.0, 0.0, 0.0, 0.0}//0.98
    };
  int index = 50*percentage;
  coords A = {x_comp,yValues[index][0]*y_stp,0};//left rear sector timing starts at 0.0
  coords B = {x_comp,yValues[index][1]*y_stp,0};//left front sector timing starts at 0.25
  coords C = {x_comp,yValues[index][2]*y_stp,0};//right rear sector timing starts at 0.50
  coords D = {x_comp,yValues[index][3]*y_stp,0};//right front sector timing starts at 0.75
  //my_log(MYLOG_INFO, "Com trajectory going to B: [%f, %f]",r.x , r.z);
  
  return {A,B,C,D};
}

void MyBody::WalkDirection(double velocity, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp, int posi)//suunnat on asteikolla 0-1 mistä 0 on täysillä siihen suuntaan. y on korkeus + -
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
  sPositions = posi;
}

coords MyBody::atplane(coords point)//coords point
{
  
  //this function is used to project points using IMU to the ground plane
  coords result = {point.x,0.0,point.z};
  return result;
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
  q.a = v.a;
  q.x = -v.x;
  q.y = -v.y;
  q.z = -v.z;
  return q;
}

double MyBody::degRad(double a)
  {
    return ((M_PI / 180) * a);
  }

corners MyBody::rotateBody(const double x_rot,const double z_rot, double y_dir)
{//rotate body over x ar z axis wia body origin

  double cx = wfCenter.x;
  double cy = wfCenter.y;
  double cz = wfCenter.z;

  corners result;
  //changes the position of the pody wireframe corners takes in degrees
  
  //my_log(MYLOG_INFO, "Rotate body with angles  [%f, %f]", sinx, cosx);
  quaternion qx = {cos(degRad(x_rot)/2),sin(degRad(x_rot)/2),0,0};//rotation in x axis 
  quaternion qz = {cos(degRad(z_rot)/2),0,0,sin(degRad(z_rot)/2)};//rotation in z axis
  quaternion qt = quaternionMultiply(qx,qz);//total rotation
  //my_log(MYLOG_INFO, "Rotate body qt [%f, %f, %f, %f]", qt.a, qt.x, qt.y,qt.z);
  //my_log(MYLOG_INFO, "Rotate body qz [%.1f, %.1f, %.1f, %.1f]", qz.a, qz.x, qz.y,qz.z);
  //my_log(MYLOG_INFO, "Rotate body qt [%.1f, %.1f, %.1f, %.1f]", qt.a, qt.x, qt.y,qt.z);
  //take a point and change it to body origin coordinates
  double x = wfLeftFront.x-cx;
  double y = wfLeftFront.y-cy;
  double z = wfLeftFront.z-cz;
  quaternion qr = {0,x,y,z};//quaternion used in rotations
  //rotate it 
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  //output results
  //my_log(MYLOG_INFO, "Rotate body wfLeftFront before [%f, %f, %f]", wfLeftFront.x, wfLeftFront.y,wfLeftFront.z);
  
  //Input the new coordinates for the point by adding back the change of coordinates
  wfLeftFront = {cx+qr.x,cy+qr.y+y_dir,cz+qr.z};
  
  //my_log(MYLOG_INFO, "Rotate body wfLeftFront after [%f, %f, %f]", wfLeftFront.x, wfLeftFront.y,wfLeftFront.z);
  //Add to the result the delta value from to-at
  result.A = {qr.x-x,qr.y-y,qr.z-z};
  //repeate
  x = wfLeftRear.x-cx;
  y = wfLeftRear.y-cy;
  z = wfLeftRear.z-cz;
  qr = {0,x,y,z};
  
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  

  wfLeftRear = {cx+qr.x,cy+qr.y+y_dir,cz+qr.z};
  result.B = {qr.x-x,qr.y-y,qr.z-z};
  
  x = wfRightRear.x-cx;
  y = wfRightRear.y-cy;
  z = wfRightRear.z-cz;
  qr = {0,x,y,z};
  
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);

  
  wfRightRear = {cx+qr.x,cy+qr.y+y_dir,cz+qr.z};
  result.C = {qr.x-x,qr.y-y,qr.z-z};

  x = wfRightFront.x-cx;
  y = wfRightFront.y-cy;
  z = wfRightFront.z-cz;
  qr = {0,x,y,z};

  
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);

  
  wfRightFront = {cx+qr.x,cy+qr.y+y_dir,cz+qr.z};
  result.D = {qr.x-x,qr.y-y,qr.z-z};
  wfCenter.y += y_dir;
  return result;
}

corners MyBody::swingBody(const double ax,const double az, coords point)
{
  //double x = x_rot;
  //double z = z_rot;
  bool only_y = true;
 
  corners result;
  //changes the position of the pody wireframe corners takes in degrees
  
  //my_log(MYLOG_INFO, "Rotate body with angles  [%f, %f]", sinx, cosx);
  quaternion qx = {cos(ax/2),sin(ax/2),0,0};//rotation in x axis 
  quaternion qz = {cos(az/2),0,0,sin(az/2)};//rotation in z axis
  quaternion qt = quaternionMultiply(qx,qz);//total rotation
  
  //walkYfront += y_dir;
  //walkYrear += y_dir;
//MUOKATAAN TÄTÄ SITEN ETTÄ Y ARVOT TULEE PERUSTUEN TIEDETTYIHIN Y KORKEUKSIIN JOISTA VÄHENNETÄÄN WF KOMPONENTIT
  //take point mLeftFront.mPosition.y;
  double x = wfLeftFront.x+point.x;
  double y = wfLeftFront.y+point.y;
  double z = wfLeftFront.z+point.z;
  //my_log(MYLOG_INFO, "LF before  [%f, %f, %f]", x, y, z);
  quaternion qr = {0,x,y,z};//quaternion used in rotations
  //rotate it 
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  //output results
  //result.B = {x-qr.x,(1)*(y-qr.y),z-qr.z};//at - to
  if (only_y)
  {
    result.B = {0,(qr.y-y),0};
  }else{
    result.B = {qr.x-x,(qr.y-y),qr.z-z};
  }
  
  //wfLeftFront = result.B;
  wfLeftFront = {qr.x-point.x,(qr.y-point.y),qr.z-point.z};
  //my_log(MYLOG_INFO, "LF before and after [%f, %f, %f][%f, %f, %f]", x, y, z, wfLeftFront.x, wfLeftFront.y, wfLeftFront.z);
  //repeate mLeftRear.mPosition.y; 
  x =  wfLeftRear.x+point.x;
  y =  wfLeftRear.y+point.y; 
  z =  wfLeftRear.z+point.z;
  qr = {0,x,y,z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  
  //result.A = {x-qr.x,(1)*(y-qr.y),z-qr.z};
  if (only_y)
  {
    result.A = {0,(qr.y-y),0};
  }else{
    result.A = {qr.x-x,(qr.y-y),qr.z-z};
  }
  //wfLeftRear = result.A;
  wfLeftRear = {qr.x-point.x,(qr.y-point.y),qr.z-point.z};
    
   // mRightRear.mPosition.y;
  x =  wfRightRear.x+point.x;
  y =  wfRightRear.y+point.y;
  z =  wfRightRear.z+point.z;
  
  qr = {0,x,y,z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);

  //result.C = {x-qr.x,(1)*(y-qr.y),z-qr.z};
  if (only_y)
  {
    result.C = {0,(qr.y-y),0};
  }else{
    result.C = {qr.x-x,(qr.y-y),qr.z-z};
  }
  //wfRightRear = result.C;
  wfRightRear = {qr.x-point.x,(qr.y-point.y),qr.z-point.z};
//mRightFront.mPosition.y;
  x =  wfRightFront.x+point.x;
  y =  wfRightFront.y+point.y;
  z =  wfRightFront.z+point.z;
  qr = {0,x,y,z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);

  //result.D = {x-qr.x,(1)*(y-qr.y),z-qr.z};
  if (only_y)
  {
    result.D = {0,(qr.y-y),0};
  }else{
    result.D = {qr.x-x,(qr.y-y),qr.z-z};
  }
  //wfRightFront = result.D;
  wfRightFront = {qr.x-point.x,(qr.y-point.y),qr.z-point.z};
  //rotate the COM allso. This has basily no effect at the moment
  //COM y is in body origin coordinates. By taking the mean of two corner legs the body origin height can be found
  //and added to the COM s y to get it in to the same coordinates as the corners
  x =  wfCOM.x+point.x;
  y =  wfCOM.y+point.y;
  z =  wfCOM.z+point.z;
  qr = {0,x,y,z};
  qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  wfCOM = {qr.x-point.x,qr.y-point.y,qr.z-point.z};
  my_log(MYLOG_INFO, "COM before and after [%f, %f, %f][%f, %f, %f]", x, y, z, wfCOM.x, wfCOM.y, wfCOM.z);
  //x =  wfCenter.x+point.x;
  //y =  wfCenter.y+point.y;
  //z =  wfCenter.z+point.z;
  //qr = {0,x,y,z};
  //qr = quaternionMultiply(quaternionMultiply(quaternionConjugate(qt),qr),qt);
  //wfCenter = {qr.x-point.x,qr.y-point.y,qr.z-point.z};
  return result;
}

void MyBody::moveCOMbeforeWalk(unsigned long mills)//NOTUSED
{
  //used to move the COM to a position that corresponds the walk cycle step that is about to start
  my_log(MYLOG_INFO, "Pre move COM");
    double perc_COM = (double)((mWalkStep + (int)(COMdelay * 50)) % 50) / (double)50;
    
    coords COMto = linear4point(perc_COM, COMTrajectory);

    coords COMat = atplane(wfCOM);

    coords COMmove = {(COMto.x-COMat.x),(COMto.y-COMat.y),(COMto.z-COMat.z)};
    //my_log(MYLOG_INFO, "COM to [%.1f, %.1f, %.1f]", COMto.x,COMto.y,COMto.z);
     //my_log(MYLOG_INFO, "COMat [%.1f, %.1f, %.1f]", COMat.x,COMat.y,COMat.z);
     //my_log(MYLOG_INFO, "COMmove [%.1f, %.1f, %.1f]", COMmove.x,COMmove.y,COMmove.z);
    my_log(MYLOG_INFO, "Bre walk COM to at move [%f, %f, %f][%f, %f, %f][%f, %f, %f]", COMto.x,COMto.y,COMto.z, COMat.x,COMat.y,COMat.z, COMmove.x,COMmove.y,COMmove.z);
    //addToCOM(COMmove);

    mRightFront.addToWalk(COMmove);
    mLeftFront.addToWalk(COMmove);
    mRightRear.addToWalk(COMmove);
    mLeftRear.addToWalk(COMmove); 
    
  
    
    mRightFront.updateWalkPosition(mills, 400, sRamp);
    mLeftFront.updateWalkPosition(mills, 400, sRamp);
    mRightRear.updateWalkPosition(mills, 400, sRamp);
    mLeftRear.updateWalkPosition(mills, 400, sRamp);

    //COMtobody();
    
}

void MyBody::addToCOM(coords delta)
{
  my_log(MYLOG_INFO, "Add to COM ");
  wfCOM.x += delta.x;
  wfCOM.y += delta.y;
  wfCOM.z += delta.z;
}

void MyBody::setCOM(coords pos)
{
  wfCOM.x = pos.x;
  wfCOM.y = pos.y;
  wfCOM.z = pos.z;
}

coords MyBody::calculateMovementParameters(coords target, coords current) {//NOTUSED
    // Calculate the differences between target and current coordinates
    coords result= {0,0,0};
    double dx = target.x - current.x;
    double dz = target.z - current.z;
    boolean limit_size = false;
    //3.2 in distance corresbond to 0.018 as output value these are used to define result so if changed change from there allso
    double amplitude_limit = 9.6;
    // Determine the signs based on the sector logic when target is negative the movement goes in wrong direction 
    //Calculate the detla values from to-at = delta. Extract signs from those values and rotate those using rotation matrix. 
    //Then find the amplitude
    double sign_z = 1.0 ; 
    double sign_x = -1.0;
    if (dx < 0)
    {
      sign_z = -1.0;
    }
    if (dz < 0)
    {
      sign_x = 1.0;
    }
   
    // Adjust the signs if the current overshoots the target this needs to make sure that the target and current are in same sector
   //0 -1  x    = rotated 90 digrees toi counter clockwise
   //1  0  z
    
    //my_log(MYLOG_INFO, "target current dx dz sign_X sign_z: [%f, %f][%f, %f][%f, %f][%f, %f]",(target.x/abs(target.x)),(target.z/abs(target.z)) ,(current.x/abs(current.x)),(current.z/abs(current.z)),dx,dz,sign_x,sign_z);    
    if (limit_size)
    {
      //Check that amplitude stays in limits
      if (abs(dx) > amplitude_limit)
      {
        dx = amplitude_limit;
      }
      if (abs(dz) > amplitude_limit)
      {
        dz = amplitude_limit;
      }
    }
    
    result.x = sign_x*(abs(dx)/3.3)*0.018;
    result.z = sign_z*(abs(dz)/3.3)*0.023;

    return result;
}

bool MyBody::isLegInTrajectory(coords sLeg,coords eLeg,double  y_rot, double x, double z){
//check if the legs are currently on walking trajectory or if there has been big changes that reguire legs to be lifted to position
  if (y_rot == 0){//linear case 
    coords legVec = {eLeg.x - sLeg.x, 0, eLeg.z - sLeg.z};
    double legVecMagnitude = sqrt(legVec.x * legVec.x + legVec.z * legVec.z);
    coords normLegVec = {legVec.x / legVecMagnitude, 0, legVec.z / legVecMagnitude};

        // Normalize steering vector
    double steerMagnitude = sqrt(x * x + z * z);
    coords normSteer = {x / steerMagnitude, 0, z / steerMagnitude};

        // Dot product to check if vectors are parallel and opposite dot prod is equal to -1 when lines are parallel but opposite in direction
    double dotProd = normLegVec.x * normSteer.x + normLegVec.z * normSteer.z;

    if (abs(dotProd) > -0.8) return false;//test failed if the direction is off more than 20 prosent
  }else{//circular case
    double xr, zr; //centerpoint of rotation
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
    //test if the radius for start and end points is approximatly the same for all legs
        double radiusStart = sqrt((sLeg.x - xr) * (sLeg.x - xr) + (sLeg.z - zr) * (sLeg.z - zr));
        double radiusEnd = sqrt((eLeg.x - xr) * (eLeg.x - xr) + (eLeg.z - zr) * (eLeg.z - zr));

        if (abs(radiusStart - radiusEnd) > 3) return false;

        // Check movement direction
        double angleStart = atan2(sLeg.z - zr, sLeg.x - xr);
        double angleEnd = atan2(eLeg.z - zr, eLeg.x - xr);
        bool clockwise = y_rot < 0 || x < 0;
        if (clockwise != (angleEnd < angleStart)) return false;
  }
  return true;
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
  
    //getPawPositionInBody() 

   
    HandlePosition(total_step_points);//handle the possible stance changes before calculating walking changes
    
    
    mLeftRear.DoStep(perc_RL, sDegree_y ,sDirection_x,sDirection_z);
    mLeftFront.DoStep(perc_FL,sDegree_y ,sDirection_x,sDirection_z);
    mRightRear.DoStep(perc_RR, sDegree_y ,sDirection_x,sDirection_z);
    mRightFront.DoStep(perc_FR, sDegree_y ,sDirection_x,sDirection_z);
    
    
   
    if (mGait == GAIT_WALK)//mGait == GAIT_WALK
    {
      corners deltas = linearFollowCompensate(perc_COM);
      
      mLeftRear.addToWalk(deltas.A); 
      mLeftFront.addToWalk(deltas.B);
      mRightRear.addToWalk(deltas.C);
      mRightFront.addToWalk(deltas.D);//
      
      
      
    }//PositionWithDelta

    //COMtobody();
    if (false){
      //record all parameters
        coords RF = mRightFront.getPawPositionInBody();//
        coords RR = mRightRear.getPawPositionInBody();
        coords LF = mLeftFront.getPawPositionInBody();
        coords LR = mLeftRear.getPawPositionInBody(); 
        my_log(MYLOG_INFO, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
        sVelocity,sDegree_x,sDegree_y,sDegree_z,sDirection_x,sDirection_y,
        sDirection_z,balancing.x, balancing.y, balancing.z,perc_FR,perc_RR,perc_FL,perc_RL,perc_COM,
        RF.x,RF.y,RF.z,RR.x,RR.y);
        my_log(MYLOG_INFO, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",RR.z,LF.x,LF.y,LF.z,LR.x,LR.y,LR.z,wfCenter.x,wfCenter.y,wfCenter.z,
        ax,ay,az,gx,gy,gz);
        //these are the saved zeroposition from IMU
        my_log(MYLOG_INFO, "%f, %f, %f, %f, %f, %f",accZero.x,accZero.y,accZero.z,gyrZero.x,gyrZero.y,gyrZero.z);
        
      }

    
    mLeftRear.updateWalkPosition(mills, time_delta, sRamp);
    mLeftFront.updateWalkPosition(mills, time_delta, sRamp);
    mRightRear.updateWalkPosition(mills, time_delta, sRamp);
    mRightFront.updateWalkPosition(mills, time_delta, sRamp);

    if (false){
          // Pair them
      //std::pair<coords, coords> leg1 = std::make_pair(startPoint1, endPoint1);
      //std::pair<coords, coords> leg2 = std::make_pair(startPoint2, endPoint2);
    // Pair for other legs if needed

    // Add to vector
      //std::vector<std::pair<coords, coords>> legPoints;
      //legPoints.push_back(leg1);
      //legPoints.push_back(leg2);
    }
    
    mWalkStep++;
  
  
  }
  else if (sDirection_y != 0 || sDegree_x != 0 || sDegree_z != 0)//not walking but body changing position
  { 
    HandlePosition(total_step_points);

    
    mLeftRear.updateStandPosition(mills, time_delta, sRamp);
    mLeftFront.updateStandPosition(mills, time_delta, sRamp);
    mRightRear.updateStandPosition(mills, time_delta, sRamp);
    mRightFront.updateStandPosition(mills, time_delta, sRamp);
    if (false){
        coords RF = mRightFront.getPawPositionInBody();//
        coords RR = mRightRear.getPawPositionInBody();
        coords LF = mLeftFront.getPawPositionInBody();
        coords LR = mLeftRear.getPawPositionInBody(); 
        my_log(MYLOG_INFO, "RF RR LF LR : %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f ",
        RF.x,RF.y,RF.z,RR.x,RR.y,RR.z,LF.x,LF.y,LF.z,LR.x,LR.y,LR.z);
        
      }
   
    
  }else//if steering is at zero there should not be any reason to execute this but this is here just in case
  {
    GoStandPosition(mills, 400, sRamp);
    //COMtobody();
  }
  

  if (mWalkStep > total_step_points)
    mWalkStep = 0;
}

void MyBody::HandlePosition(unsigned long total_step_points)
{
    //movement in one step of one cycle
    double y_dir = (Y_STEP_SIZE/total_step_points)*sDirection_y*4;
    double x_rot = (X_STEP_ANGLE/total_step_points)*sDegree_x*2;
    double z_rot = (Z_STEP_ANGLE/total_step_points)*sDegree_z*3;

  coords LF;
  coords LR;
  coords RR;
  coords RF;
  
  if (sDegree_x != 0 || sDegree_z != 0)
  {//testaa mitä arvoja se antaa jos syöttää esim 4 ja 7 argumenteiksi
    //my_log(MYLOG_INFO, "Not walking steps before y dir x rot z rot  [%f, %f, %f]", y_dir,x_rot,z_rot);
    corners delta = rotateBody(x_rot, z_rot,y_dir);//leftfront, leftrear,rightrear,rightfront x_rot, z_rot
    //my_log(MYLOG_INFO, "Not walking steps after y dir x rot z rot  [%.1f, %.1f, %.1f]", y_dir,x_rot,z_rot);
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
      wfCenter.y += y_dir;
  }
  //my_log(MYLOG_INFO, "Not walking position change LF  [%f, %f, %f]", LF.x, LF.y, LF.z);
  mLeftRear.PositionWithDelta(LR);
  mLeftFront.PositionWithDelta(LF);
  mRightRear.PositionWithDelta(RR);
  mRightFront.PositionWithDelta(RF);
}

void MyBody::GoHome(unsigned long mills, unsigned long duration, double ramp)
{
  mLeftFront.GoHome(mills, duration, ramp);
  mLeftRear.GoHome(mills, duration, ramp);
  mRightFront.GoHome(mills, duration, ramp);
  mRightRear.GoHome(mills, duration, ramp);
}


void MyBody::GoWalkPosition(unsigned long mills, unsigned long duration, double ramp)
{//goes to the walk position set by configuration
  mLeftRear.GoPosition(mills, WALK_POSITION_REAR_X, WALK_POSITION_REAR_Y, WALK_POSITION_Z, duration,ramp);
  mLeftFront.GoPosition(mills, WALK_POSITION_FRONT_X, WALK_POSITION_FRONT_Y, WALK_POSITION_Z, duration,ramp);
  mRightRear.GoPosition(mills, WALK_POSITION_REAR_X, WALK_POSITION_REAR_Y, WALK_POSITION_Z, duration,ramp);
  mRightFront.GoPosition(mills, WALK_POSITION_FRONT_X, WALK_POSITION_FRONT_Y, WALK_POSITION_Z, duration,ramp);
  if (false){
        coords RF = mRightFront.getPawPositionInBody();//
        coords RR = mRightRear.getPawPositionInBody();
        coords LF = mLeftFront.getPawPositionInBody();
        coords LR = mLeftRear.getPawPositionInBody(); 
        my_log(MYLOG_INFO, "RF RR LF LR : %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f ",
        RF.x,RF.y,RF.z,RR.x,RR.y,RR.z,LF.x,LF.y,LF.z,LR.x,LR.y,LR.z);
        
      }
  
}


void MyBody::GoSitPosition(unsigned long mills, unsigned long duration, double ramp)
{//goes to the walk position set by configuration
//istuma asento etupää oikein takapää väärin
//[114239] RF RR LF LR : 80.254527, 205.217223, 89.000000, -124.254527, 104.782777, 89.000000, 80.254527, 205.217223, -89.000000, -124.254527, 104.782777, -89.000000 
  coords front = {-61.0,205.0,WALK_POSITION_Z};
  coords rear = {17.0,100.0,0.0};
  mLeftFront.GoPosition(mills, front.x, front.y, front.z, duration,ramp);
  mLeftRear.GoPosition(mills, rear.x, rear.y, rear.z, duration,ramp);
  mRightFront.GoPosition(mills, front.x, front.y+13, -front.z, duration,ramp);
  mRightRear.GoPosition(mills, rear.x, rear.y, -rear.z, duration,ramp);
  
}

void MyBody::GoGivePawPosition(unsigned long mills, unsigned long duration, double ramp)
{//goes to the walk position set by configuration
//istuma asento etupää oikein takapää väärin
//[114239] RF RR LF LR : 80.254527, 205.217223, 89.000000, -124.254527, 104.782777, 89.000000, 80.254527, 205.217223, -89.000000, -124.254527, 104.782777, -89.000000 
  coords front = {-61.0,200.0,WALK_POSITION_Z};
  coords rear = {17.0,100.0,0.0};
  mLeftFront.GoPosition(mills, 90.0, 120.0, front.z, duration,ramp);//left front paw is given
  mLeftRear.GoPosition(mills, rear.x, rear.y, rear.z, duration,ramp);
  mRightFront.GoPosition(mills, front.x, front.y+13, -front.z+60, duration,ramp);
  mRightRear.GoPosition(mills, rear.x, rear.y, -rear.z, duration,ramp);
  
}

void MyBody::GoCOMposition(unsigned long mills, unsigned long duration, double ramp)
{
  double xx = COM_X_OFFSET;
  double yy = sin(acos(xx/(FEMUR_SIZE+TIBIA_SIZE-1)))*(FEMUR_SIZE+TIBIA_SIZE-1);
  double zz = 0.0;
  mLeftFront.GoPosition(mills, xx, yy, zz, duration,ramp);
  mLeftRear.GoPosition(mills, xx, yy, zz, duration,ramp);
  mRightFront.GoPosition(mills, xx, yy, zz, duration,ramp);
  mRightRear.GoPosition(mills, xx, yy, zz, duration,ramp);
}
void MyBody::GoStandPosition(unsigned long mills, unsigned long duration, double ramp)
{
  mLeftRear.BackToPosition(mills, duration, ramp);
  mLeftFront.BackToPosition(mills, duration, ramp);
  mRightRear.BackToPosition(mills, duration, ramp);
  mRightFront.BackToPosition(mills, duration, ramp);
  //COMtobody();
  //setCOMTrajectory(BALANCE_SAFETY_MARGIN);
  //my_log(MYLOG_INFO, "COM trajectory [%.1f, %.1f, %.1f][ %.1f,  %.1f,  %.1f][ %.1f,  %.1f,  %.1f][ %.1f,  %.1f,  %.1f]", COMTrajectory[0].x,COMTrajectory[0].y,COMTrajectory[0].z, COMTrajectory[1].x,COMTrajectory[1].y,COMTrajectory[1].z, COMTrajectory[2].x,COMTrajectory[2].y,COMTrajectory[2].z, COMTrajectory[3].x, COMTrajectory[3].y, COMTrajectory[3].z);

  
}//GoZero
void MyBody::GoZeroPosition(unsigned long mills, unsigned long duration, double ramp)
{
  mLeftFront.GoZero(mills, duration, ramp);
  mLeftRear.GoZero(mills, duration, ramp);
  mRightFront.GoZero(mills, duration, ramp);
  mRightRear.GoZero(mills, duration, ramp);
  
  
}
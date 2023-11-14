


#include "MyAsyncServo.h"
#include "MyLeg.h"
#include "MyBody.h"
#include "MyLog.h"

//start debug serial prints

int continueloop = 0;//value that user is asked to change to delay functions before serial monitor has started
// Instantiate the body object

MyBody body;

//----------------------------

// y akseli on pystyssä z sivuttain ja x kulkusuunnassa
void setup() {
  my_log(MYLOG_INFO, "Application started");
  

  // Attach(PWM_ID, SERVO_TYPE, HOME_DEGREE, DEGREE_CORRECTION,ZERO_DEGREE)
  //To make inverse kinematics work, change motor degree ranges correct at myasyncservo.cpp file
  //home degrees and correction term define the position the robot starts at. Start finding from the middle of your servos movement
  //zero degrees is the degree position where legs are fully extended and straight down from the body. Start allso from center.
  //--------------------------------------------------------
  body.mRightFront.mTibia.Attach(15, SERVO_35KG, 220.0, 10.0,83.0);//posi akselin suuntaan myötäpäivään
  body.mRightFront.mFemur.Attach(14, SERVO_20KG, 100.0, 0.0,176.0);
  body.mRightFront.mCoax.Attach(13, SERVO_20KG, 135.0, 6.0,135.0);
 

  body.mRightRear.mTibia.Attach(6, SERVO_35KG, 220.0, 10.0,90.0);//pos aks suun myöt päiv
  body.mRightRear.mFemur.Attach(5, SERVO_20KG, 100.0, -15.0,177.0);//nega aks suunt myötäpäivään
  body.mRightRear.mCoax.Attach(4, SERVO_20KG, 135.0, 3.0,135.0);
  
  
  body.mLeftFront.mTibia.Attach(9, SERVO_35KG, 220.0, 3.0,89.0);//suurempi on enemmän koukussa. suora koti oli 85. koukku koti oli 220 ja 3
  body.mLeftFront.mFemur.Attach(10, SERVO_20KG, 100.0, -20.0,177.0);//pienempi on takana ylhäällä. suora koti oli 110. koukku koti oli 100 ja -20
  body.mLeftFront.mCoax.Attach(11, SERVO_20KG, 135.0, -9.0,135.0);
  
  
  body.mLeftRear.mTibia.Attach(2, SERVO_35KG, 220.0, 3.0,81.0);//nega aks suunt myötäpäivään
  body.mLeftRear.mFemur.Attach(1, SERVO_20KG, 100.0, -26.0,176.0);//110 muuttamaton
  body.mLeftRear.mCoax.Attach(0, SERVO_20KG, 135.0, -12.0,134.0);//nega aks suunt myötäpäivään eli tassu liikkuu pois alta
 

  
  // PHYSICAL LIMITS IN DEGREES 
  //----------------------------
  body.mLeftFront.mTibia.SetLimits(80.0, 270.0); 
  body.mLeftRear.mTibia.SetLimits(80.0, 270.0); 
  body.mRightFront.mTibia.SetLimits(80.0, 270.0); 
  body.mRightRear.mTibia.SetLimits(80.0, 270.0); 
  
  body.mLeftFront.mFemur.SetLimits(0.0, 270.0);
  body.mRightFront.mFemur.SetLimits(0.0, 270.0);
  body.mLeftRear.mFemur.SetLimits(0.0, 270.0);
  body.mRightRear.mFemur.SetLimits(0.0, 270.0);

  body.mLeftFront.mCoax.SetLimits(50.0, 170.0);
  body.mRightFront.mCoax.SetLimits(50.0, 170.0);
  body.mLeftRear.mCoax.SetLimits(50.0, 170.0);
  body.mRightRear.mCoax.SetLimits(50.0, 170.0);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Put synchronously body in home
 
  body.home();

  delay(2000);
}


void test3(unsigned long now_mills)
{
  static int test_phase = 0;
  static unsigned int wait_until = 1000;
  
 
  if (now_mills < wait_until)
   { return;
   }
  else if (test_phase == 0)
  {
    body.setGait(GAIT_WALK); // select WALK mode GAIT_WALK
    body.GoWalkPosition(now_mills, 900, 0.3); // start stand-up at time delta=0ms then 1000ms pause (the stand-up duration is set to 900ms then 100ms do nothing) 

    test_phase++;
    wait_until = now_mills + 3000; // wait 2s (where the movement home->walk position elapse 900ms)
    return;
  }
    else if (test_phase == 1)
  {
    //WalkDirection(double velocity, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp)
    body.WalkDirection(0.05, 1.0, 0, 0, 0, 0, 0, 0.3);//nopeus m/s , x kääntö suunta,y kääntö suunta,zkääntö suunta, x suunnan nopeus 0-1, y korkeus +- kävely asennosta, z nopeus 0-1
   
    
    test_phase++;
    wait_until = now_mills + 2000; // walk for 2s
    return;
  }
    else if (test_phase == 2)
  {
    //WalkDirection(double velocity, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp)
    body.WalkDirection(0.05, 0, 0, 1.0, 0, 0, 0, 0.3);//nopeus m/s , x kääntö suunta,y kääntö suunta,zkääntö suunta, x suunnan nopeus 0-1, y korkeus +- kävely asennosta, z nopeus 0-1
   
    
    test_phase++;
    wait_until = now_mills + 2000; // walk for 2s
    return;
  }
  
  else if (test_phase == 3)/////////////
  {
    body.WalkDirection(0.25, 0, 0, 0, 0, 0, 0, 0.3);

    test_phase++;
    wait_until = now_mills + 1000; // stay stopped for 1s
    return;
  }
  
  
}
void loop() {
  static unsigned int last_mills = 0;
  int wait_for_print = 1;
  unsigned long now_mills = millis();
  if (wait_for_print != 0)//debugging
  {
    
    if (continueloop == 0)//wait for user to have serial monitor active
    {
      Serial.println("Give a non zero integer to continue.");
      continueloop = Serial.parseInt();
    }
    else
    {
       if (last_mills != now_mills)
      {
    

        test3(now_mills);
    
        body.update(now_mills);
      }
    }
  }else//not debugging
  {
    if (last_mills != now_mills)
  {
    

    test3(now_mills);
    
    body.update(now_mills);
  }
  }
 
  
}

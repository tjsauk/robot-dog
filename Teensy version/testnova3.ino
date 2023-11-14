


#include "MyAsyncServo.h"
#include "MyLeg.h"
#include "MyBody.h"
#include "MyLog.h"


// Instantiate the body object

MyBody body;

//----------------------------


void setup() {
  my_log(MYLOG_INFO, "Application started");

  // Attach(PWM_ID, SERVO_TYPE, HOME_DEGREE, DEGREE_CORRECTION)
  //
  //--------------------------------------------------------
  body.mRightFront.mTibia.Attach(15, SERVO_35KG, 270.0, 45.0);//posi on akselin suuntaan myötäpäivään
  body.mRightFront.mFemur.Attach(14, SERVO_20KG, 50.0, -3.0);
  body.mRightFront.mCoax.Attach(13, SERVO_20KG, 90.0, 4.0);

  body.mRightRear.mTibia.Attach(6, SERVO_35KG, 270.0, 45.0);//pos aks suunt myötä päivään
  body.mRightRear.mFemur.Attach(5, SERVO_20KG, 38.0, -13.0);//nega aks suunt myötäpäivään
  body.mRightRear.mCoax.Attach(4, SERVO_20KG, 90.0, 2.0);

  body.mLeftFront.mTibia.Attach(9, SERVO_35KG, 270.0, -53.0);
  body.mLeftFront.mFemur.Attach(10, SERVO_20KG, 50.0, 0.0);
  body.mLeftFront.mCoax.Attach(11, SERVO_20KG, 90.0, -6.0);
  
  body.mLeftRear.mTibia.Attach(2, SERVO_35KG, 270.0, 37.0);//negatiivinen akselin suuntaan myötäpäivään
  body.mLeftRear.mFemur.Attach(1, SERVO_20KG, 38.0, -21.0);
  body.mLeftRear.mCoax.Attach(0, SERVO_20KG, 90.0, -8.0);//negatiivinen on akselin suuntaan myötäpäivään

  
  // PHYSICAL LIMITS IN DEGREES 
  //----------------------------
  body.mLeftFront.mTibia.SetLimits(100.0, 270.0); 
  body.mLeftRear.mTibia.SetLimits(100.0, 270.0); 
  body.mRightFront.mTibia.SetLimits(100.0, 270.0); 
  body.mRightRear.mTibia.SetLimits(100.0, 270.0); 
  
  body.mLeftFront.mFemur.SetLimits(0.0, 180.0);
  body.mRightFront.mFemur.SetLimits(0.0, 180.0);
  body.mLeftRear.mFemur.SetLimits(0.0, 180.0);
  body.mRightRear.mFemur.SetLimits(0.0, 180.0);

  body.mLeftFront.mCoax.SetLimits(50.0, 120.0);
  body.mRightFront.mCoax.SetLimits(50.0, 120.0);
  body.mLeftRear.mCoax.SetLimits(50.0, 120.0);
  body.mRightRear.mCoax.SetLimits(50.0, 120.0);
  
  //pinMode(LED_BUILTIN, OUTPUT);

  // Put synchronously body in home
  //body.home();
  
  delay(1000);
}


void blink_led(unsigned long now_mills) 
{
  static unsigned long last_update = 0;
  
  unsigned long counter = (now_mills / 500) % 2; // 500ms on / 500ms off
  
  if (counter == last_update)
    return;

  last_update = counter;

  if (counter % 2)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
}


// Homing all servos
// to keep servos centered during leg assembly
void center() 
{
  body.mLeftFront.GoCenter(1000, 0.3);
  body.mLeftRear.GoCenter(1000, 0.3);
  body.mRightFront.GoCenter(1000, 0.3);
  body.mRightRear.GoCenter(1000, 0.3);
}


// Some macros to simplify calling action asynchronously
#define TEST_BEGIN_ALL() static int test_phase = 0;
#define TEST_BEGIN(x,at) if (delta < at) {if (test_phase == x){my_log(MYLOG_WARNING, "Test %d", test_phase);
#define TEST_END() test_phase++;}} else
#define TEST_END_ALL(x,at) if (delta < at) {if (test_phase == x){my_log(MYLOG_WARNING, "Test %d", test_phase);test_phase++;}} else{return true;}

// test ik
void test(unsigned long mills)
{
  static unsigned long first_update = 0;

  if (first_update == 0)
  {
    first_update = mills;
    return;
  }

  unsigned long delta = mills - first_update;

  if (delta < 50)
    return;

  TEST_BEGIN_ALL()

  TEST_BEGIN(0,1000) // executed test 0 one time at delta = 0 and do test 1 when delta = 1000
    // Go to walk initial position
    // 900 = the going to walk position will be completed in 900ms
    // 0.3 = servo speed ramp up for 30% then 60% constant then ramp dowm for 30%)
    body.GoWalkPosition(mills, 900, 0.3); // start stand-up at time delta=0ms then 1000ms pause (the stand-up duration is set to 900ms then 100ms do nothing)
  TEST_END()

  TEST_BEGIN(1,2000) // executed test 1 one time at delta = 1000 and do test 2 when delta = 2000
    body.GoTo(mills, 900, 0, 0, 15, 0, 0, 0); // rotate body to 15 degree z-axis, then 2000-1000=1000ms pause
  TEST_END()

  TEST_BEGIN(2,4000) // executed test 2 one time at delta = 2000 and do test 3 when delta = 4000
    body.GoTo(mills, 1800, 0, 0, -15, 0, 0, 0); // rotate body to -15 degree z-axis ,then 4000-2000=2000ms pause
  TEST_END()

  TEST_BEGIN(3,5000)
    body.GoTo(mills, 900, 0, 0, 0, 0, 0, 0); // rotate body to 0,0,0 degree z-axis, then 1000ms pause
  TEST_END()

  TEST_BEGIN(4,6000)
    body.GoTo(mills, 900, 45, 0, 0, 0, 0, 0); // rotate body to 45 degree x-axis, then 1000ms pause
  TEST_END()

  TEST_BEGIN(5,8000)
    body.GoTo(mills, 1800, -45, 0, 0, 0, 0, 0); // rotate body to -45 degree x-axis, then 2000ms pause
  TEST_END()

  TEST_BEGIN(6,9000)
    body.GoTo(mills, 900, 0, 0, 0, 0, 0, 0); // rotate body to 0,0,0 degree, then 1000ms pause
  TEST_END()

  TEST_BEGIN(7,10000)
    body.GoTo(mills, 900, 0, 0, 0, 0, 0, -50); // translate body to z=-50, ...
  TEST_END()

  TEST_BEGIN(8,12000)
    body.GoTo(mills, 1800, 0, 0, 0, 0, 0, 50); // translate body to z=50
  TEST_END()

  TEST_BEGIN(9,13000)
    body.GoTo(mills, 900, 0, 0, 0, 0, 0, 0); // translate body to z=0
  TEST_END()

  TEST_BEGIN(10,14000)
    body.GoWalkPosition(mills, 900, 0.3); // go to walk position 
  TEST_END()
  
  TEST_BEGIN(11,20000)
    body.setGait(GAIT_TROT); // select trot mode
    body.StartWalk(600); // start walk at 600ms per step
  TEST_END()

  TEST_BEGIN(12,24000)
    body.WalkTurn(TURN_LEFT); // start turn left
  TEST_END()

  TEST_BEGIN(13,35000)
    body.WalkTurn(TURN_NONE); // stop turn left
  TEST_END()

  TEST_BEGIN(14,36000)
    body.StopWalk(mills); // stop walk
  TEST_END()

  TEST_BEGIN(15,37000)
    body.GoHome(mills, 2000, 0.3);  // go home position
  TEST_END()
    
  TEST_END_ALL(16,40000)

  return false;
}

void loop() {
  static unsigned int last_mills = 0;
  
  unsigned long now_mills = millis();
  
  if (last_mills != now_mills)
  {
    last_mills = now_mills;
    
    //blink_led(now_mills);

    //test(now_mills);
    center();
    // Execute asynch servo movements
    
    body.update(now_mills);
  }
}

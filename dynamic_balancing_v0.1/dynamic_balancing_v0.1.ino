


#include "MyAsyncServo.h"
#include "MyLeg.h"
#include "MyBody.h"
#include "MyLog.h"
#include <RF24.h>

//NovaSMRemote NRF24 controller
//Pin wiring for Teensy 4.0:
//  pin 9 -> CE pin
//  pin 10 -> CSN pin
//  pin 13 -> SCK pin
//  pin 11 -> MOSI pin
//  pin 12 -> MISO pin
//  pin xx -> IRQ pin (unused)
// instantiate an object for the nRF24L01 transceiver
RF24 nrf_radio(9, 10);
unsigned int nrfInterval = 50;
unsigned long lastNRFUpdate = 0;
unsigned int remoteInterval = 50;
unsigned long lastRemoteUpdate = 0;

uint8_t address[2][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
const int data_num = 14;          //bytes of data to receive
bool rc_resp;
uint8_t rc_data[data_num] = {     //receive data array
  0, 0, 0, 0,                     //btn1, btn2, btn3, btn4
  0, 0,                           //sel2, sel1
  0, 0, 0, 0,                     //p1, p4, p2 , p3 index 6-9
  0, 0, 0, 0                      //lx, ly, rx, ry index 10-13
};
uint8_t tm_data[data_num] = {     //transmit data array
  0, 0, 0,                        //battery, spd, Y
  0, 0, 0,                        //x compensate, y compensate, ??
  1, 0, 0, 0,                     //Current gate, Standing up, ?? , ??
  0, 0, 0, 0                      //
};

#define BATT_MONITOR A1


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
  //Fine tune the zero degrees calibration by measuring that coax motors are in 90 degree angle relative to the frame. when robot is in standing position
  //femurs need to be calibrated next so that rear and front legs go straight relative to each other
  //then tibia is calibrated so that the standing position height is correct. Finally femur needs to be tuned so that the paw position
  //is correct measurement wise. Remember to make sure all slack in the joints is correctly positioned aka legs are under weight
  //--------------------------------------------------------
  body.mRightFront.mTibia.Attach(15, SERVO_35KG, 220.0, 10.0,89.3);//posi akselin suuntaan myötäpäivään 83
  body.mRightFront.mFemur.Attach(14, SERVO_20KG, 100.0, 0.0,170.8);
  body.mRightFront.mCoax.Attach(13, SERVO_20KG, 135.0, 6.0,139.0);//positive drives legs end to right
 

  body.mRightRear.mTibia.Attach(6, SERVO_35KG, 220.0, 10.0,90.0);//pos aks suun myöt päiv
  body.mRightRear.mFemur.Attach(5, SERVO_20KG, 100.0, -15.0,172.3);//smaller number lifts knee up
  body.mRightRear.mCoax.Attach(4, SERVO_20KG, 135.0, 3.0,139.0);//positive drives legs end to right
  
  
  body.mLeftFront.mTibia.Attach(9, SERVO_35KG, 220.0, 3.0,87.9);//suurempi on enemmän koukussa. suora koti oli 85. koukku koti oli 220 ja 3
  body.mLeftFront.mFemur.Attach(10, SERVO_20KG, 100.0, -20.0,172.9);//pienempi on takana ylhäällä. suora koti oli 110. koukku koti oli 100 ja -20
  body.mLeftFront.mCoax.Attach(11, SERVO_20KG, 135.0, -9.0,135.0);//positive drives legs end to right
  
  
  body.mLeftRear.mTibia.Attach(2, SERVO_35KG, 220.0, 3.0,85.1);//nega aks suunt myötäpäivään
  body.mLeftRear.mFemur.Attach(1, SERVO_20KG, 100.0, -26.0,172.2);//110 muuttamaton
  body.mLeftRear.mCoax.Attach(0, SERVO_20KG, 135.0, -12.0,138.0);//positive drives legs end to right
 

  
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

    //init NRF24 wireless
  
    if (!nrf_radio.begin()) {
      Serial.println(F("radio hardware is not responding!!"));
      while (1) {}
    } else {
      nrf_radio.setPALevel(RF24_PA_LOW);
      nrf_radio.setPayloadSize(sizeof(rc_data));
      nrf_radio.setChannel(124);
      nrf_radio.openReadingPipe(1, address[!radioNumber]);
      nrf_radio.enableAckPayload();
      nrf_radio.startListening();
      nrf_radio.writeAckPayload(1, &tm_data, sizeof(tm_data)); // pre-load data      if (debug8) Serial.println(F("radio hardware is ready and listening!"));
    }
  
}


void nrf_ack() {

    //set data
    int sensorValue = analogRead(BATT_MONITOR);//in a skale from 0 to 1023
    double voltage = (sensorValue * (3.3 / 1023.00) *(0.985+4.668))/0.985*1.325; 
    tm_data[0] = int(voltage*10);
    //tm_data[0] = 0;//bv;
    //tm_data[1] = 0;//spd;
    tm_data[2] = body.getHeight();//sound_vol;
    //tm_data[3] = 0;//move_steps;
    //tm_data[4] = 6;//(step_weight_factor_front * 100);
    //tm_data[5] = 6;//(step_weight_factor_rear * 100);
    //tm_data[6] = 0;//remote_select;
    //tm_data[7] = 3;//start_mode;
    
  
    nrf_radio.writeAckPayload(1, &tm_data, sizeof(tm_data)); // load the payload for the next time
}

bool nrf_check() {
  bool resp = false;
  uint8_t pipe;

  if (nrf_radio.available(&pipe)) {
    uint8_t bytes = nrf_radio.getPayloadSize();
    nrf_radio.read(&rc_data, bytes);
    resp = true;

    //send data as acknowledgement
    nrf_ack();

    //set remote control vars from nrf received data
    //fire buttons
    //btn1 = rc_data[0];
    ///btn2 = rc_data[1];
    //btn3 = rc_data[2];
    //btn4 = rc_data[3];

    //sel stop buttons
    //sel1 = rc_data[4];
    //sel2 = rc_data[5];

    //slide pots
    //p1 = rc_data[6];
    //p2 = rc_data[7];
    //p3 = rc_data[8];
    //p4 = rc_data[9];

    //joysticks
    //lx = rc_data[10];
    //ly = rc_data[11];
    //rx = rc_data[12];
    //ry = rc_data[13];

  }
  lastNRFUpdate = millis();

  return resp;
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
    //body.GoTestPosition(now_mills, 900, 0.3);
    //GoZeroPosition(unsigned long mills, unsigned long duration, double ramp)
    //body.GoZeroPosition(now_mills, 900, 0.3);
    test_phase++;
    wait_until = now_mills + 3000; // wait 2s (where the movement home->walk position elapse 900ms)
    return;
  }
    else if (test_phase == 1)
  {
    //WalkDirection(double velocity, double degree_x, double  degree_y, double degree_z, double x, double y, double z, double ramp)
    body.WalkDirection(0.04, 0.0, 0, 0, 1, 0, 0, 0.3, 0);//nopeus m/s , x kääntö suunta,y kääntö suunta,zkääntö suunta, x suunnan nopeus 0-1, y korkeus +- kävely asennosta, z nopeus 0-1
   
    
    test_phase++;
    wait_until = now_mills + 50000; // walk for 2s
    return;
  }
    
  
  else if (test_phase == 2)/////////////
  {
    body.WalkDirection(0.25, 0, 0, 0, 0, 0, 0, 0.3 , 2);//last integer 0 means walk or move, 1 is go home 
    body.GoWalkPosition(now_mills, 900, 0.3);
    test_phase++;
    wait_until = now_mills + 1000; // stay stopped for 1s
    return;
  }
  
  
}

double reMapJoystic(double val, double midvalue)
{
  double deadzone = 5;// +- 5 from center
  double result = 0.0;
  
  if (val > (midvalue - deadzone) && val < (midvalue + deadzone)) {
    return 0.0;
  } else if (val > (midvalue + deadzone)) { // from 0 to 1
    double coef = 255 - (midvalue + deadzone); // Range of values from dead zone to high max coefficient converting it to range 0-1
    result = ((double)(val - (midvalue + deadzone))*(1 / coef));
    return result;
  } else {//if (val < (midvalue - deadzone)) 
    result = (double)val * (1.0 / (midvalue - deadzone))-1;
    return result;
  }
}



void loop() {
  static unsigned int last_mills = 0;//Time of next command update
  nrf_check();//receive and send remote data
  unsigned long now_mills = millis();//current time
  
    if (now_mills >= last_mills)//time needs to be greater or equal of the set limit
  {
    //luetaan sel1 tilaa joka aktivoituessaan asettaan tm_data[7] = 3;//start_mode; joko nollaksi tai ykköseksi
    //
    
    if (int(rc_data[5])){//Left select pressed initialise or unitialise
    
      if (tm_data[7]){//if at walk position go to home
        body.GoHome(now_mills, 900, 0.3);
        tm_data[7] = 0;
      }else{
        body.GoWalkPosition(now_mills, 900, 0.3);
        tm_data[7] = 1;
      }
      last_mills = now_mills+900;
    }else if (int(rc_data[4]) ){//Right select switch pressed. Change walking gate
      if (int(tm_data[6]) == 2){//Was in trot, going to walk
          body.setGait(GAIT_WALK);//GAIT_TROT
          tm_data[6] = 1;
      }else{//Was in walk going to trot
        body.setGait(GAIT_TROT);//
          tm_data[6] = 2;
      }
      last_mills = now_mills;
    }
    else if (tm_data[7]) //is initialised
    {
      //joysticks
      double lx = reMapJoystic(int(rc_data[10]),147);
      double ly = reMapJoystic(int(rc_data[11]),109);
      double rx = reMapJoystic(int(rc_data[12]),147);
      double ry = reMapJoystic(int(rc_data[13]),109);
      //Slide pots from left to right
      //first is going to be x axis roll control
      //second
      double spd = map(double(rc_data[8]),0,255,0.01,0.2);//second pot
      tm_data[1] = int(spd*100);
      int yy = 0;//Y height goal from remote
      double xr = 0.0; // roll in x axis
      if (true)//use pots 3 and 4 for balancing parameters
      {
          int xc = map(int(rc_data[9]),0,255,0,40);//third pot
          tm_data[3] = xc;//balance compensation in x axis 
      
          int yc = map(int(rc_data[7]),0,255,0,20);//Fourth pot
          tm_data[4] = yc;//balance compensation in y axis 
          body.setBalancing(xc, yc);

          yy = map(int(rc_data[6]),0,255,125,185);//First pot Changing y height 
          //tm_data[2] = yy;//balance compensation in y axis 
      }else{//rc_data[9] is the third pot and rc_data[7] is the fourth pot
        //Here goes the other configuration of pots 1, 3 and 4 
          yy = map(int(rc_data[7]),0,255,125,185);//Fourth pot
          xr = map(int(rc_data[6]),0,255,-1,1);
          if (abs(xr) == 0.05){//rotation over x axis needs some dead zone
            xr = 0.0;
          }
      }
      double ym = 0.0;//up or down in y axis control variable

      double yrel = yy/body.getHeight();
      if (yrel > 1.01){//goal is larger than current 1.005 min value
        ym = map(yrel,1.01,1.5,0.0,-1.0);
      }else if (yrel < 0.99){//current is larger than goal 0.995 max value 
        ym = map(yrel,0.65,0.99,1.0,0.0);
      }else{
        ym = 0.0;
      }

      
      if (lx == 0 && ly == 0 && rx == 0 && ry == 0 && ym == 0.0 && xr == 0)//All movement parameters 0
      {
        //body.GoWalkPosition(now_mills, 900, 0.3);//GoStandPosition
        body.GoStandPosition(now_mills, 900, 0.3);//
        last_mills = now_mills+900;
      }else{//movement reguired
        body.WalkDirection(spd, xr, lx, ly, ry, ym, rx, 0.3, 0);
        body.PositionUpdate(now_mills);
        last_mills = now_mills;
      }

    }
    

    //test3(now_mills);
   
    //COMcalibrate(now_mills);
    //body.update(now_mills);
    //body.GoCOMposition(now_mills, 400, 0.3);
    
    
  }
  
 body.update(now_mills);
  
}

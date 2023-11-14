#ifndef _config_h
#define _config_h

// Enable serial log
// comment out to disable
#define LOG_ENABLED
#define LOG_LEVEL MYLOG_INFO

#define DEBUG_MODE 0//0= not used, 1= print walk cycle controls and info

#define COAX_SIZE 70.0
#define FEMUR_SIZE 106.5
#define TIBIA_SIZE 136.0

#define WALK_POSITION_FRONT_X 0.0
#define WALK_POSITION_REAR_X 0.0

#define WALK_POSITION_FRONT_Y 170.0
#define WALK_POSITION_REAR_Y 170.0

//Walk position Z offset is given as distance away from body. Positive is away and negative is below the body
#define WALK_POSITION_Z 0.0

#define Y_STEP_SIZE -20.0//amount of lenght in one walk cycle that body can go up and down

#define X_STEP_ANGLE 80.0 //amount of degrees the body can rotate in one walk cycle
#define Z_STEP_ANGLE 80.0

#define BODY_WIDTH 78.0
#define BODY_LENGTH 282.0

#define COM_X_OFFSET -20.0
#define COM_Z_OFFSET 0.0
#define COM_WEIGHT 80.0

#define BALANCE_SAFETY_MARGIN 20.0

#define MAX_TURN_RADIUS 4000.0//in millimeters

#define STEP_SIZE 80.0
#define STEP_SWING_HEIGHT_FRONT 35.0
#define STEP_SWING_HEIGHT_REAR 35.0
#define STEP_STANCE_HEIGHT_FRONT 0.0
#define STEP_STANCE_HEIGHT_REAR 0.0
#define STEP_STANCE_TIME_PERCENTAGE 0.75
 

#endif // _config_h

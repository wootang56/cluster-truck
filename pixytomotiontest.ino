#include <SPI.h>
#include <Pixy.h>
#include <TPixy.h>

#include <L6470.h>


//==========================================================================
//
//  Pixy Pet Robot
//
//   Adafruit invests time and resources providing this open source code, 
//  please support Adafruit and open-source hardware by purchasing 
//  products from Adafruit!
//
// Written by: Bill Earl for Adafruit Industries
//
//==========================================================================
// begin license header
//
// All Pixy Pet source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
//
// end license header
//
//==========================================================================
//
// Portions of this code are derived from the Pixy CMUcam5 pantilt example code. 
//
//==========================================================================




#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS  ((RCS_MAX_POS-RCS_MIN_POS)/2)

//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop
{
public:
  ServoLoop(int32_t proportionalGain, int32_t derivativeGain);

  void update(int32_t error);

  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_proportionalGain;
  int32_t m_derivativeGain;
};

// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
  m_pos = RCS_CENTER_POS;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x80000000L;
}

// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
  long int velocity;
  char buf[32];
  if (m_prevError!=0x80000000)
  { 
    velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;

    m_pos += velocity;
    if (m_pos>RCS_MAX_POS) 
    {
      m_pos = RCS_MAX_POS; 
    }
    else if (m_pos<RCS_MIN_POS) 
    {
      m_pos = RCS_MIN_POS;
    }
  }
  m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------

Pixy pixy;  // Declare the camera object

ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt

//motor configuration values
float maxSpeed = 400;
float fullSpeed = 1000;//1000
float accel = 100;//500
float decel = 100;
float runSpeed=100;
float microsteps= 1;//1    1,2,4,8,16,32,64 or 128

//Initialization of each stepper motor L6470 object. Note: L6470 is the driver Model
L6470 flMot(2, 48,49); // Top board A output to UL    cs, busy, reset
L6470 blMot(3, 46, 47); // Top board B output tp LL
L6470 frMot(4,44, 45); // Bottom board A output to UR
L6470 brMot(5, 42, 43); // Bottom board B output tp LR

//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  motorConfig();

  pixy.init();
}

uint32_t lastBlockTime = 0;

//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop()
{ 
  uint16_t blocks;
  blocks = pixy.getBlocks();

  // If we have blocks in sight, track and follow them
  if (blocks)
  {
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis();
  }  
  else if (millis() - lastBlockTime > 100)
  {
//    motors.setLeftSpeed(0);
//    motors.setRightSpeed(0);
      flMot.softStop();
      frMot.softStop();
      blMot.softStop();
      brMot.softStop();
    ScanForBlocks();
  }
}

int oldX, oldY, oldSignature;

//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;

  Serial.print("blocks =");
  Serial.println(blockCount);

  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }

  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;

  panLoop.update(panError);
  tiltLoop.update(tiltError);

  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  return trackedBlock;
}

//---------------------------------------
// Follow blocks via the Zumo robot drive
//
// This code makes the robot base turn 
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
  int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?

  // Size is the area of the object.
  // We keep a running average of the last 8.
  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
  size -= size >> 3;

  // Forward speed decreases as we approach the object (size is larger)
  int forwardSpeed = constrain(400 - (size/256), 1, 400);//-100,400  

  // Steering differential is proportional to the error times the forward speed
  int32_t differential = (followError + (followError * forwardSpeed))>>8;

  // Adjust the left and right speeds by the steering differential.
  int leftSpeed = constrain(forwardSpeed + differential, 0, 400);//+-400
  int rightSpeed = constrain(forwardSpeed - differential, 0, 400);//+-400
//  float leftSpeed = constrain(forwardSpeed, 1, 100);//+-400
//  float rightSpeed = constrain(forwardSpeed, 1, 100);//+-400

  // And set the motor speeds
//  motors.setLeftSpeed(leftSpeed);
//  motors.setRightSpeed(rightSpeed);
    flMot.run(REV, rightSpeed);
    blMot.run(REV, rightSpeed);
    frMot.run(FWD, leftSpeed);
    brMot.run(FWD, leftSpeed);
}

//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;

void ScanForBlocks()
{
  if (millis() - lastMove > 20)
  {
    lastMove = millis();
    panLoop.m_pos += scanIncrement;
    if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
    {
      tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
      scanIncrement = -scanIncrement;
      if (scanIncrement < 0)
      {
//        motors.setLeftSpeed(-250);
//        motors.setRightSpeed(250);
          flMot.run(FWD, 250);
          blMot.run(FWD, 250);
          frMot.run(REV, 250);
          brMot.run(REV, 250);
      }
      else
      {
//        motors.setLeftSpeed(+180);
//        motors.setRightSpeed(-180);
          flMot.run(REV, 180);
          blMot.run(REV, 180);
          frMot.run(FWD, 180);
          brMot.run(FWD, 180);
      }
      delay(random(250, 500));
    }

    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }
}

//MOTOR CONFIGURATION
void motorConfig()
{
  //FRONT LEFT MOTOR
  flMot.init();
  frMot.init();
  brMot.init();
  blMot.init();

  
  flMot.setAcc(accel); //set acceleration
  flMot.setDec(decel);
  flMot.setMaxSpeed(maxSpeed);
  flMot.setMinSpeed(1);
  flMot.setMicroSteps(microsteps); //1,2,4,8,16,32,64 or 128
  flMot.setThresholdSpeed(fullSpeed);
  flMot.setOverCurrent(6000); //set overcurrent protection
  flMot.setStallCurrent(4000);


  // FRONT RIGHT MOTOR
  frMot.setAcc(accel); //set acceleration
  frMot.setDec(decel);
  frMot.setMaxSpeed(maxSpeed);
  frMot.setMinSpeed(1);
  frMot.setMicroSteps(microsteps); //1,2,4,8,16,32,64 or 128
  frMot.setThresholdSpeed(fullSpeed);
  frMot.setOverCurrent(6000); //set overcurrent protection
  frMot.setStallCurrent(4000); 

 //BACK LEFT MOTOR
  blMot.setAcc(accel); //set acceleration
  blMot.setDec(decel);
  blMot.setMaxSpeed(maxSpeed);
  blMot.setMinSpeed(1);
  blMot.setMicroSteps(microsteps); //1,2,4,8,16,32,64 or 128
  blMot.setThresholdSpeed(fullSpeed);
  blMot.setOverCurrent(6000); //set overcurrent protection
  blMot.setStallCurrent(4000);  


  //BACK RIGHT MOTOR
  brMot.setAcc(accel); //set acceleration
  brMot.setDec(decel);
  brMot.setMaxSpeed(maxSpeed);
  brMot.setMinSpeed(1);
  brMot.setMicroSteps(microsteps); //1,2,4,8,16,32,64 or 128
  brMot.setThresholdSpeed(fullSpeed);
  brMot.setOverCurrent(6000); //set overcurrent protection
  brMot.setStallCurrent(4000); 

//  Serial.println("Done Configuring");
}


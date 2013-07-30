
//=============================================================================
// Based upon Kurt's PX Reactor arm code.
// PLEASE NOTE: There is currently a bug upon startup that causes the arm to twitch upon startup. 
// While this bug is harmless and will not damage your arm, the arm is still very powerful and caution should be used.

// This code is setup to control a WidowX Robotic Arm which is sold by Trossen
// Robotics: http://www.trossenrobotics.com/widowxrobotarm


//=============================================================================
//Armcontrol packet structure is as follows
//
// 255, XH, XL, YH, YL, ZH, ZL, WAH, WAL, WRH, WAL, GH, GL, DTIME, BUTTONS, EXT, CHECKSUM
//
// (please note, actual XYZ min/max for specific arm defined below)
//
// Protocol value ranges
//
// XH = high byte X-axis
// XL = low byte, 0-1023 (-512 through +512 via ArmControl)
//
// YH = high byte Y-axis
// YL = low byte, 0-1023
//
// ZH = high byte Z-axis
// ZL = low byte, 0-1023 
//
// WAH = high byte (unused for now, placeholder for higher res wrist angle)
// WAL = low byte, 0-180 (-90 through +90 via ArmControl)
//
// WRH = high byte 
// WRL = low byte, 0-1023. 512 center
//
// GH = high byte
// GL = low byte, 0-512. 256 center
//
// DTIME = byte. DTIME*16 = interpolation delta time
//
// Buttons = byte (not implemented)
//
// EXT = byte. Extended instruction set.
// EXT < 16 = no action
// EXT = 32 = 3D Cartesian IK
// EXT = 48 = Cylindrical IK Xaxis = 0-4096 value, untested
// EXT = 64 = BackHoe aka passthrough UNTESTED
//
// CHECKSUM = (unsigned char)(255 - (XH+XL+YH+YL+ZH+ZL+WAH+WAL+WRH+WRL+GH+GL+DTIME+BUTTONS+EXT)%256)

//  This code is a Work In Progress and is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
//  
//=============================================================================

//=============================================================================
// Define Options
//=============================================================================

#define OPT_WRISTROT          // comment this out if you are not using Wrist Rotate
#define ARBOTIX_TO  1000      // if no message for a second probably turned off...
#define DEADZONE    3        // deadzone around center of joystick values
#define SOUND_PIN    7      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial


//=============================================================================
// Global Include files
//=============================================================================
#include <ax12.h>
#include <BioloidController.h>
#include <ArmControl.h>
//#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>



//=============================================================================
//=============================================================================
/* Servo IDs */
enum {
  SID_BASE=1, SID_SHOULDER, SID_ELBOW, SID_WRIST, SID_WRISTROT, SID_GRIP};

enum {
  IKM_IK3D_CARTESIAN, IKM_CYLINDRICAL, IKM_BACKHOE};

// status messages for IK return codes..
enum {
  IKS_SUCCESS=0, IKS_WARNING, IKS_ERROR};


#define CNT_SERVOS  6 //(sizeof(p)/sizeof(pgm_axdIDs[0]))

// Define some Min and Maxs for IK Movements...
//                y   Z
//                |  /
//                |/
//            ----+----X (X and Y are flat on ground, Z is up in air...
//                |
//                |
#define IK_MAX_X  300
#define IK_MIN_X  -300

#define IK_MAX_Y  400
#define IK_MIN_Y  50

#define IK_MAX_Z  350
#define IK_MIN_Z  20

#define IK_MAX_GA  90
#define IK_MIN_GA   -90

// Define Ranges for the different servos...
#define BASE_MIN    0
#define BASE_MAX    4096

#define SHOULDER_MIN  1024 
#define SHOULDER_MAX  3072

#define ELBOW_MIN    1024
#define ELBOW_MAX    3072

#define WRIST_MIN    1024
#define WRIST_MAX    3072

#define WROT_MIN     0
#define WROT_MAX     1023

#define GRIP_MIN     0
#define GRIP_MAX     512

// Define some lengths and offsets used by the arm
#define BaseHeight          125L   // (L0)about 120mm 
#define ShoulderLength      150L   // (L1)Not sure yet what to do as the servo is not directly in line,  Probably best to offset the angle?
//                                 // X is about 140, y is about 40 so sqrt is Hyp is about 155, so maybe about 21 degrees offset
#define ShoulderServoOffset 72L    // should offset us some...
#define ElbowLength         142L   //(L2)Length of the Arm from Elbow Joint to Wrist Joint
#define WristLength         155L   // (L3)Wrist length including Wrist rotate
#define G_OFFSET            0      // Offset for static side of gripper?

#define IK_FUDGE            5     // How much a fudge between warning and error

//=============================================================================
// Global Objects
//=============================================================================
ArmControl armcontrol = ArmControl();
BioloidController bioloid = BioloidController(1000000);
//LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//=============================================================================
// Global Variables...
//=============================================================================
boolean         g_fArmActive = false;   // Is the arm logically on?
byte            g_bIKMode = IKM_IK3D_CARTESIAN;   // Which mode of operation are we in...
uint8_t         g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;
boolean         g_fServosFree = true;

// Current IK values
int            g_sIKGA;                  // IK Gripper angle..
int            g_sIKX;                  // Current X value in mm
int            g_sIKY;                  //
int            g_sIKZ;

// Values for current servo values for the different joints
int             g_sBase;                // Current Base servo value
int             g_sShoulder;            // Current shoulder target 
int             g_sElbow;               // Current
int             g_sWrist;               // Current Wrist value
int             g_sWristRot;            // Current Wrist rotation
int             g_sGrip;                // Current Grip position

// BUGBUG:: I hate too many globals...
int sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip;
int sDeltaTime = 100;


// Message informatino
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off
byte            buttonsPrev;            // will use when we wish to only process a button press once


//
#ifdef DEBUG
boolean        g_fDebugOutput = false;
#endif

// Forward references
extern void MSound(byte cNotes, ...);


//===================================================================================================
// Setup 
//====================================================================================================
void setup() {
  pinMode(0,OUTPUT);  
  // Lets initialize the Commander
  Serial.begin(38400);
//  lcd.init();        
//  lcd.backlight();
//  lcd.print("Reactor Online.");  
//  delay(2000);
//  lcd.clear();
  Serial.println("WidowX Online.");
  
  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
    delay(100);
  // Start off to put arm to sleep...


  PutArmToSleep();

  MSound(3, 60, 2000, 80, 2250, 100, 2500);

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);

}


//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() {
  boolean fChanged = false;
   
  if (armcontrol.ReadMsgs()) {
  
    digitalWrite(0,HIGH-digitalRead(0));    
    
     if(armcontrol.ext < 0x10){
        // no action
     }
    switch (armcontrol.ext){
      case 0x20:  //32
        g_bIKMode = IKM_IK3D_CARTESIAN;   
//        MoveArmToHome(); 
        break;
      case 0x30:  //48
        g_bIKMode = IKM_CYLINDRICAL;
//        MoveArmToHome(); 
        break;    
      case 0x40:  //64
        g_bIKMode = IKM_BACKHOE;
//        MoveArmToHome(); 
        break;
      case 0x50:  //80
        MoveArmToHome(); 
        break;
      case 0x60:  //96
        PutArmToSleep();
        break;
      case 0x70:  //112
        //do something
        break;
      case 0x80:  //128
        //do something
        break;        
      case 0x90:  //144
        //do something
        break;
    }
    
    // See if the Arm is active yet...
    if (g_fArmActive) {
     
      
      sBase = g_sBase;
      sShoulder = g_sShoulder;
      sElbow = g_sElbow; 
      sWrist = g_sWrist;
      sWristRot = g_sWristRot;      
      sGrip = g_sGrip;
      

// Call IKMode

//       fChanged |= ProcessUserInput3D();
        switch (g_bIKMode) {
        case IKM_IK3D_CARTESIAN:
          fChanged |= ProcessUserInput3D();
 
          break;
        case IKM_CYLINDRICAL:
          fChanged |= ProcessUserInputCylindrical();
          
          break;

        case IKM_BACKHOE:
          fChanged |= ProcessUserInputBackHoe();
          break;
        }




      // If something changed and we are not in an error condition
      if (fChanged && (g_bIKStatus != IKS_ERROR)) {
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
      }
      else if (bioloid.interpolating > 0) {
        bioloid.interpolateStep();
      }
    }
    else {
      g_fArmActive = true;
      MoveArmToHome();      
    }

//    buttonsPrev = armcontrol.buttons;
    ulLastMsgTime = millis();    // remember when we last got a message...
  }
  else {
    if (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
    }
    // error see if we exceeded a timeout
    if (g_fArmActive && ((millis() - ulLastMsgTime) > ARBOTIX_TO)) {
//      lcd.clear();
//      lcd.setCursor(0, 0);    
//      lcd.print("Good bye. :(");
      PutArmToSleep();
    
    
    }
  }
} 


//===================================================================================================
// ProcessUserInput3D: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInput3D(void) {
  
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKX;                  // Current X value in mm
  int   sIKY;                  //
  int   sIKZ;
  int   sIKGA;    

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...

  if (g_bIKStatus == IKS_SUCCESS) {
    
// Keep IK values within limits
//
    sIKX = min(max(armcontrol.Xaxis, IK_MIN_X), IK_MAX_X);    
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y), IK_MAX_Y);    
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(armcontrol.W_ang, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords..
    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;
    
  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  



  if (fChanged) {
//    LCD(sIKX, sIKY, sIKZ, sIKGA, sWristRot, sDeltaTime);    
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
  }
  return fChanged;

}




// updates needed here to compile
//===================================================================================================
// ProcessUserInputCylindrical: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputCylindrical() {
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKY;                  // Distance from base in mm
  int   sIKZ;
  int   sIKGA;

  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
  // or simply use the information from the Warning/Error from last call to the IK function...
  sIKY = g_sIKY;
  sIKZ = g_sIKZ;
  sIKGA = g_sIKGA;

  // The base rotate is real simple, just allow it to rotate in the min/max range...
  sBase = min(max((armcontrol.Xaxis+512), BASE_MIN), BASE_MAX);

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
  // Use Y for 2d distance from base
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKY > 0) && (armcontrol.Yaxis < 0)) || ((g_sIKY < 0) && (armcontrol.Yaxis > 0)))
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y), IK_MAX_Y);

  // Now Z coordinate...
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKZ > 0) && (armcontrol.Zaxis < 0)) || ((g_sIKZ < 0) && (armcontrol.Zaxis > 0)))
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z), IK_MAX_Z);

  // And gripper angle.  May leave in Min/Max here for other reasons...   

  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKGA > 0) && (armcontrol.W_ang < 0)) || ((g_sIKGA < 0) && (armcontrol.W_ang > 0)))
    sIKGA = min(max(armcontrol.W_ang, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...

    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;
   
  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);


  if (fChanged) {
    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);
  }
  return fChanged;
}



//===================================================================================================
// ProcessUserInputBackHoe: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputBackHoe() {
  // lets update positions with the 4 joystick values
  // First the base
  boolean fChanged = false;
  sBase = min(max(armcontrol.Xaxis+512, BASE_MIN), BASE_MAX);
  // Now the Boom
  sShoulder = min(max(armcontrol.Yaxis, SHOULDER_MIN), SHOULDER_MAX);
  // Now the Dipper 
  sElbow = min(max(armcontrol.Zaxis, ELBOW_MIN), ELBOW_MAX);
  // Bucket Curl
  sWrist = min(max(armcontrol.W_ang, WRIST_MIN), WRIST_MAX);
    sWristRot = min(max(armcontrol.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armcontrol.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armcontrol.dtime*16;

  fChanged = (sBase != g_sBase) || (sShoulder != g_sShoulder) || (sElbow != g_sElbow) || (sWrist != g_sWrist) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  return fChanged;
}




//===================================================================================================
// MoveArmToHome
//===================================================================================================
void MoveArmToHome(void) {

  if (g_bIKMode != IKM_BACKHOE) {
    g_bIKStatus = doArmIK(true, 0, (2*ElbowLength)/3+WristLength, BaseHeight+(2*ShoulderLength)/3, 0);
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 256, 500, true);
  }
  else {
    g_bIKStatus = IKS_SUCCESS;  // assume sucess so we will continue to move...
    MoveArmTo(2048, 2048, 2048, 2048, 512, 256, 500, true);
  }
}

//===================================================================================================
// PutArmToSleep
//===================================================================================================
void PutArmToSleep(void) {
  g_fArmActive = false;
  MoveArmTo(2048, 1024, 1024, 1700, 512, 256, 1000, true);

  // And Relax all of the servos...
  for(uint8_t i=1; i <= CNT_SERVOS; i++) {
    Relax(i);
  }
  g_fServosFree = true;
}


//===================================================================================================
// MoveArmTo
//===================================================================================================
void MoveArmTo(int sBase, int sShoulder, int sElbow, int sWrist, int sWristRot, int sGrip, int wTime, boolean fWait) {

  int sMaxDelta;
  int sDelta;

  // First make sure servos are not free...
  if (g_fServosFree) {
    g_fServosFree = false;

    for(uint8_t i=1; i <= CNT_SERVOS; i++) {
      TorqueOn(i);
    }
    bioloid.readPose();
  }


#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.print("[");
    Serial.print(sBase, DEC);
    Serial.print(" ");
    Serial.print(sShoulder, DEC);
    Serial.print(" ");
    Serial.print(sElbow, DEC);
    Serial.print(" ");
    Serial.print(sWrist, DEC);
    Serial.print(" ");
    Serial.print(sWristRot, DEC);
    Serial.print(" ");
    Serial.print(sGrip, DEC);
    Serial.println("]");
  }
#endif
  // Make sure the previous movement completed.
  // Need to do it before setNextPos calls as this
  // is used in the interpolating code...
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }

  // Also lets limit how fast the servos will move as to not get whiplash.
  bioloid.setNextPose(SID_BASE, sBase);  

  sMaxDelta = abs(bioloid.getCurPose(SID_SHOULDER) - sShoulder);
  bioloid.setNextPose(SID_SHOULDER, sShoulder);

  sDelta = abs(bioloid.getCurPose(SID_ELBOW) - sElbow);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_ELBOW, sElbow);

  sDelta = abs(bioloid.getCurPose(SID_WRIST) - sWrist);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_WRIST, sWrist);

#ifdef OPT_WRISTROT
  bioloid.setNextPose(SID_WRISTROT, sWristRot); 
#endif  

  bioloid.setNextPose(SID_GRIP, sGrip);


  // Save away the current positions...
  g_sBase = sBase;
  g_sShoulder = sShoulder;
  g_sElbow = sElbow;
  g_sWrist = sWrist;
  g_sWristRot = sWristRot;
  g_sGrip = sGrip;

  // Now start the move - But first make sure we don't move too fast.  
//  if (((long)sMaxDelta*wTime/1000L) > MAX_SERVO_DELTA_PERSEC) {
//    wTime = ((long)sMaxDelta*1000L)/ MAX_SERVO_DELTA_PERSEC;
//  }

  bioloid.interpolateSetup(wTime);

  // Do at least the first movement
  bioloid.interpolateStep();

  // And if asked to, wait for the previous move to complete...
  if (fWait) {
    while (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
      delay(3);
    }
  }
}

//===================================================================================================
// Convert radians to servo position offset. 
//===================================================================================================
int radToServo(float rads){
  float val = (rads*652);
  return (int) val;
}


//===================================================================================================
// Compute Arm IK for 3DOF+Mirrors+Gripper - was based on code by Michael E. Ferguson
// Hacked up by me, to allow different options...
//===================================================================================================
uint8_t doArmIK(boolean fCartesian, int sIKX, int sIKY, int sIKZ, int sIKGA)
{
  int t;
  int sol0;
  uint8_t bRet = IKS_SUCCESS;  // assume success
#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.print("(");
    Serial.print(sIKX, DEC);
    Serial.print(",");
    Serial.print(sIKY, DEC);
    Serial.print(",");
    Serial.print(sIKZ, DEC);
    Serial.print(",");
    Serial.print(sIKGA, DEC);
    Serial.print(")=");
  }
#endif
  if (fCartesian) {
    // first, make this a 2DOF problem... by solving base
    sol0 = radToServo(atan2(sIKX,sIKY));
    // remove gripper offset from base
    t = sqrt(sq((long)sIKX)+sq((long)sIKY));
    // BUGBUG... Not sure about G here
#define G 30   
    sol0 -= radToServo(atan2((G/2)-G_OFFSET,t));
  }
  else {
    // We are in cylindrical mode, probably simply set t to the y we passed in...
    t = sIKY;
#ifdef DEBUG
    sol0 = 0;
#endif
  }
  // convert to sIKX/sIKZ plane, remove wrist, prepare to solve other DOF           
  float flGripRad = (float)(sIKGA)*3.14159/180.0;
  long trueX = t - (long)((float)WristLength*cos(flGripRad));   
  long trueZ = sIKZ - BaseHeight - (long)((float)WristLength*sin(flGripRad));

  long im = sqrt(sq(trueX)+sq(trueZ));        // length of imaginary arm
  float q1 = atan2(trueZ,trueX);              // angle between im and X axis
  long d1 = sq(ShoulderLength) - sq(ElbowLength) + sq((long)im);
  long d2 = 2*ShoulderLength*im;
  float q2 = acos((float)d1/float(d2));
  q1 = q1 + q2;
  int sol1 = radToServo(q1-1.57);

  d1 = sq(ShoulderLength)-sq((long)im)+sq(ElbowLength);
  d2 = 2*ElbowLength*ShoulderLength;
  q2 = acos((float)d1/(float)d2);
  int sol2 = radToServo(3.14-q2);

  // solve for wrist rotate
  int sol3 = radToServo(3.2 + flGripRad - q1 - q2 );

#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.print("<");
    Serial.print(sol0, DEC);
    Serial.print(",");
    Serial.print(trueX, DEC);
    Serial.print(",");
    Serial.print(trueZ, DEC);
    Serial.print(",");
    Serial.print(sol1, DEC);
    Serial.print(",");
    Serial.print(sol2, DEC);
    Serial.print(",");
    Serial.print(sol3, DEC);
    Serial.print(">");
  }
#endif   

    // Lets calculate the actual servo values.

  if (fCartesian) {
    sBase = min(max(2048 - sol0, BASE_MIN), BASE_MAX);
  }
  sShoulder = min(max(2048 - sol1, SHOULDER_MIN), SHOULDER_MAX);

  // Magic Number 819???
  sElbow = min(max(3072 - sol2, SHOULDER_MIN), SHOULDER_MAX);

#define Wrist_Offset 2048
  sWrist = min(max(Wrist_Offset + sol3, WRIST_MIN), WRIST_MAX);

  // Remember our current IK positions
  g_sIKX = sIKX; 
  g_sIKY = sIKY;
  g_sIKZ = sIKZ;
  g_sIKGA = sIKGA;
  // Simple test im can not exceed the length of the Shoulder+Elbow joints...

  if (im > (ShoulderLength + ElbowLength)) {
    if (g_bIKStatus != IKS_ERROR) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Error");
      }
#endif
      MSound(2, 50, 3000, 50, 3000);
    }
    bRet = IKS_ERROR;  
  }
  else if(im > (ShoulderLength + ElbowLength-IK_FUDGE)) {
    if (g_bIKStatus != IKS_WARNING) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Warning");
      }
#endif
      MSound(1, 75, 2500);
    }
    bRet = IKS_WARNING;  
  }

  return bRet;
}




// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif



void LCD(int IKX, int IKY, int IKZ, int IKGA, int WristRot, int Gripper){
//    lcd.setCursor(0, 0);    
//    lcd.print(IKX);    
//    lcd.setCursor(4, 0);    
//    lcd.print(IKY);      
//    lcd.setCursor(8, 0);    
//    lcd.print(IKZ);    
//    lcd.setCursor(12, 0);    
//    lcd.print(IKGA);
//    lcd.setCursor(0, 1);    
//    lcd.print(WristRot);    
//    lcd.setCursor(8, 1);    
//    lcd.print(Gripper); 
}



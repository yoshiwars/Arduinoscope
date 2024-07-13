#include "lcdgfx.h"
#include "lcdgfx_gui.h"
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SiderealPlanets.h>
#include "BluetoothSerial.h"
#include <TimeLib.h>
#include "Preferences.h"
#include <SiderealObjects.h>
#include <List.hpp>
#include <ESP32Encoder.h>

/************************************************************************************************************************
Start Configurable Items 
*************************************************************************************************************************/
#define MOUNT_NAME "StarMax90"                    //Name of the Mount - Bluetooth Name etc.
#define FIRMWARE_VERSION "0.2"                     //Just for Informational Purposes
#define FIRMWARE_DATE "JUL 01 2024"                //Just for Informational Purposes
#define FIRMWARE_TIME "12:00:00"

//this is expecting the Alt and Az gearing to be the same.
const int GEAR_RATIO = 15;                       //where 1 is no gearing (ex. 300 Tooth gear / 20 tooth gear = 15), 15 Telescope, 1 Binoc
const double SINGLE_STEP_DEGREE =  360.0 / 400.0;    // the motor has 200 regular steps for 360 degrees (360 divided by 200 = 1.8)
const double MOTOR_GEAR_BOX = 1;                 //where 1 is no gearing (26 + (103/121)) planetary gearbox version
const float MAX_MOTOR_SPEED = 500;              //250 is probably good without GEAR_RATIO >1, 1500 Telescope, 250 Binoc 

//Define Motor setup
const int ALT_DIRECTION_PIN = 13;                 //ALT Driver Dir
const int ALT_STEP_PIN = 12;                      //ALT Driver Step
const int AZ_DIRECTION_PIN = 27;                 //AZ Driver Dir
const int AZ_STEP_PIN = 14;                      //AZ Driver Step
const int MOTOR_INTERFACE_TYPE = 1;             //AccelStepper Motor Type 1
//#define MIN_PULSE_WIDTH 50                      //use to adjust stepper pulses, comment out for default

//Define Remote setup
const int Y_JOYSTICK_PIN = 34;                  //Up/Down Pin on the Remote
const int Y_INVERT = 1;                         //1 is normal, -1 inverted , binoc -1, telescope 1
const int X_JOYSTICK_PIN = 35;                  //Left/Right Pin on the remote
const int X_INVERT = 1;                        //1 is normal, -1 inverted , binoc -1, telescope 1

const int BUTTON_PIN = 4;                      //button on Remotes
const int ANALOG_READ_RESOLUTION = 4095;        //ESP32 has this resolution, other chips may vary

//Define Encoders
const int AZ_ENCODER_1 = 26;
const int AZ_ENCODER_2 = 25;
const double AZ_ENCODER_GEAR = 15;                //300 / 20
const double AZ_ENCODER_DEGREE_PER_STEP = 0.15;   //360 / (600 * 4)
const int AZ_ENCODER_INVERT = -1;                 //change the direction of the Encoder when adding to currentAz
const int ALT_ENCODER_1 = 33;
const int ALT_ENCODER_2 = 32;
const double ALT_ENCODER_GEAR = 15;               //300 / 20
const double ALT_ENCODER_DEGREE_PER_STEP = 0.15;  //360 / (600 * 4)
const int ALT_ENCODER_INVERT = -1;                //change the direction of the Encoder when adding to currentAlt
#define ENCODERFILTER                             //If moving too fast, need to lower the debounce to 0. Comment out at slower speeds.

//Comment Out HAS_FOCUSER for no Focuser
#define HAS_FOCUSER

#ifdef HAS_FOCUSER
  const int FOCUS_DIRECTION_PIN = 15;
  const int FOCUS_STEP_PIN = 2;
  const int MAX_FOCUS_SPEED = 1024;
  const int FOCUS_INTERFACE_TYPE = 1;           //1 is with driver DRV8825
#endif

//Shift Register for motor speeds
const int LATCH_PIN = 18;                       //Pin connected to ST_CP of 74HC595
const int CLOCK_PIN = 19;                       //Pin connected to SH_CP of 74HC595
const int DATA_PIN = 5;                         //Pin connected to DS of 74HC595

//uncomment to get debugging
//#define DEBUG
//#define DEBUG_STEPS
//#define DEBUG_X_JOYSTICK
//#define DEBUG_Y_JOYSTICK 
//#define DEBUG_GPS
//#define DEBUG_COMMS
//#define DEBUG_ENCODER
/************************************************************************************************************************
End Configurable Items
*************************************************************************************************************************/

/************************************************************************************************************************
Global Variables - These should not need to change based on devices
*************************************************************************************************************************/
BluetoothSerial btComm;       //Bluetooth Setup
SiderealPlanets myAstro;      //Used for calculations
SiderealObjects myObjects;    //Used for GoTo Functions
Preferences preferences;      //Stores persistent variables

//button states
int buttonState = LOW;                // the current reading from the input pin
int lastButtonState = LOW;            // the previous reading from the input pin
unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
unsigned long debounceDelay = 50;     // the debounce time; increase if the output flickers
unsigned long longPressTime = 500;
bool longYPressActive = false;
bool longXPressActive = false;

//AltAZ motor rotation min
const double rotationDegrees = (SINGLE_STEP_DEGREE / MOTOR_GEAR_BOX); //degrees / single step per rev / gearbox

//Motor Starts
AccelStepper yStepper = AccelStepper(MOTOR_INTERFACE_TYPE,ALT_STEP_PIN,ALT_DIRECTION_PIN);
AccelStepper xStepper = AccelStepper(MOTOR_INTERFACE_TYPE,AZ_STEP_PIN,AZ_DIRECTION_PIN);
#ifdef HAS_FOCUSER
AccelStepper focuser = AccelStepper(FOCUS_INTERFACE_TYPE,FOCUS_STEP_PIN,FOCUS_DIRECTION_PIN);
#endif


ESP32Encoder altEncoder;
ESP32Encoder azEncoder;


float xSpeed = 0;
float ySpeed = 0;
float fSpeed = 0;
float fSpeedSerial = MAX_FOCUS_SPEED;
float realXSpeed = 0;
float realYSpeed = 0;
float realFSpeed = 0;

int stepperSpeed = 3;
int lastStepperSpeed = 0;
int preTrackStepperSpeed = 3;
int stepperDivider = 1;

//Menu Control
unsigned int screenMode = 0;
/*
  0 - GPS Screen
  1 - Main Menu
  2 - Slew Mode
  3 - Settings Menu
  4 - Alignment Screen
      0 - Sync 1st Object
      1 - Sync 2nd Object
      2 - Sync 3rd Object
      3 - Confirm sync
  5 - Offsets Menu
  6 - Goto Menu
  7 - Planets Menu --Not in use anymore
  8 - Target Below Horizon
*/
unsigned int returnScreenMode = 0;
const char *mainMenuItems[] = 
{
  "Move:  Disabled",
  #ifdef HAS_FOCUSER
  "Focus: Disabled",
  #endif
  "Speed: Fast",
  "Settings",
  "Info",
  "GoTo"
};

unsigned int speedIndex = 1;

const char *settingsMenuItems[] = 
{
  "<-- Back",
  "Alignment",
  "Offsets",
  "Tracking: Enabled"
};

const char *alignmentMenuItems[] =
{
  "Move:  Disabled",
  "Speed: Fast",
  "Cancel Alignment"
};

const char *offSetsMenuItems[] =
{
  "<-- Back",
  "Alt:",
  "Az: ",
  "Reset to Default"
};

const char *gotoMenuItems[] =
{
  "<-- Back",
  "Planet: Mercury",
  "Messier: M1",
  "Caldwell: 1",
  "The Moon"
};

const char *planets[] =
{
  "Mercury",
  "Venus",
  "Mars",
  "Jupiter",
  "Saturn",
  "Uranus",
  "Neptune"
};

unsigned int messierObject = 1;
unsigned int caldwellObject = 1;
unsigned int planetObject = 0;

LcdGfxMenu mainMenu(mainMenuItems, sizeof(mainMenuItems) / sizeof(char *) );
LcdGfxMenu settingsMenu(settingsMenuItems, sizeof(settingsMenuItems) / sizeof(char *) );
LcdGfxMenu alignmentMenu(alignmentMenuItems, sizeof(alignmentMenuItems) / sizeof(char *) );
LcdGfxMenu offsetsMenu(offSetsMenuItems, sizeof(offSetsMenuItems) / sizeof(char *) );
LcdGfxYesNo alignmentConfirm("Apply new offset?");

LcdGfxMenu gotoMenu(gotoMenuItems, sizeof(gotoMenuItems)/sizeof(char *));

//alignment Variables
int alignmentScreen = 0;
double alignmentAltValues[][2] = {
  {0.0,0.0},
  {0.0,0.0},
  {0.0,0.0}
};

double alignmentAzValues[][2] = {
  {0.0,0.0},
  {0.0,0.0},
  {0.0,0.0}
};

double alignmentAltOffset = 0.0;
double alignmentAzOffset = 0.0;

//JoyStick
int lastYState = 0;
int yState = 0;
unsigned long lastYDebounceTime = 0;
unsigned int yDeadZone = 0;
unsigned int xDeadZone = 0;
unsigned int maxY = ANALOG_READ_RESOLUTION;
unsigned int maxX = ANALOG_READ_RESOLUTION;

int lastXState = 0;
int xState = 0;
unsigned long lastXDebounceTime = 0;

bool moving = false;
bool yMoving = false;
bool xMoving = false;

bool focusing = false;

//communication
const byte numChars = 32;
char input[numChars];
const byte gpsNumChars = 80;
char gpsData[gpsNumChars];
bool newData = false;
bool synced = false;

//Sync Time Variables
int inHourOffset;
int inHour;
int inMinute;
int inSecond;
int inMonth;
int inDay;
int inYear;


//GPS
TinyGPSPlus gps;
bool gpsAcquired = false;
long lastGPSCheck = 0;

float flat = 0;
float flon = 0;
float faltitude;

int sats;

float gpsQueryTime;

//hold the received RA, Dec
int raHH;
int raMM;
int raSS;
int decD;
int decMM;
int decSS;

//Sync coords
double currentAlt = 0;
double currentAz = 0;
double moveOffsets[] = {1.0,1.0};
double tempMoveOffsets[] = {1.0,1.0};

//Target Coords for slewing
int targetRaHH;
int targetRaMM;
int targetRaSS;
int targetDecD;
int targetDecMM;
int targetDecSS;
double targetAlt;
double targetAz;
bool slewComplete = true;
bool tracking = true;
bool isTracking = false;
long slewScreenTimeOut = 2000;
long slewScreenUpdate = 0;

const char *sites[4] = {"","","",""};

DisplaySSD1306_128x64_I2C display(-1); // or (-1,{busId, addr, scl, sda, frequency}). This line is suitable for most platforms by default

void setup() {
  preferences.begin("scope",false);   //start up preferences
  
  if(preferences.isKey("altOffset")){
    moveOffsets[0] = preferences.getDouble("altOffset");
  }

  if(preferences.isKey("azOffset")){
    moveOffsets[1] = preferences.getDouble("azOffset");
  }

  for(int i = 0; i < 4; i++){
    if(preferences.isKey("site" + i)){
      sites[i] = preferences.getString("site" + i).c_str();
    }
  }

  #ifdef HAS_FOCUSER
    speedIndex = 2;
  #endif
  // Set all Pin Modes
  pinMode(Y_JOYSTICK_PIN, INPUT);
  pinMode(X_JOYSTICK_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
    
  //Start Comms
  #ifdef DEBUG
    Serial.begin(115200);   //USB
  #endif
  #ifndef DEBUG
    Serial.begin(9600);     //USB
  #endif


  //ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

  altEncoder.attachFullQuad(ALT_ENCODER_1, ALT_ENCODER_2);
  azEncoder.attachFullQuad(AZ_ENCODER_1,AZ_ENCODER_2);

  #ifdef ENCODERFILTER
      altEncoder.setFilter(0);
      azEncoder.setFilter(0);
  #endif

  altEncoder.clearCount();
  azEncoder.clearCount();


  btComm.begin(MOUNT_NAME);   //Bluetooth
  Serial2.begin(9600);  //start GPS
  
  #ifdef DEBUG
    Serial.println("Start Up");
    Serial.print("Mount Name: ");
    Serial.println(MOUNT_NAME);
  #endif

  #ifdef MIN_PULSE_WIDTH
    xStepper.setMinPulseWidth(MIN_PULSE_WIDTH);
    yStepper.setMinPulseWidth(MIN_PULSE_WIDTH);
  #endif

  #ifdef DEBUG_STEPS
    Serial.print("GEAR_RATIO: ");
    Serial.println(GEAR_RATIO);
    Serial.print("SINGLE_STEP_DEGREE: ");
    printDouble(SINGLE_STEP_DEGREE, 10000);
    Serial.println("");
    Serial.print("MOTOR_GEAR_BOX: ");
    printDouble(MOTOR_GEAR_BOX, 10000);
    Serial.println("");
    Serial.print("MAX_MOTOR_SPEED: ");
    Serial.println(MAX_MOTOR_SPEED);
    Serial.print("rotationDegrees: ");
    printDouble(rotationDegrees,10000);
    Serial.println("");
  #endif
  
  //-8 = PST
  myAstro.begin();
  myAstro.setTimeZone(-8);
  myAstro.rejectDST();

  myObjects.begin();

  #ifdef DEBUG
    Serial.println("Astro Started");
  #endif
  

  //start up screen
  display.setFixedFont( ssd1306xled_font6x8 );
  display.begin();
  display.clear();

  #ifdef DEBUG
    Serial.println("Screen Started");
  #endif

  yDeadZone = getDeadZone(Y_JOYSTICK_PIN);
  xDeadZone = getDeadZone(X_JOYSTICK_PIN);
  maxY = (ANALOG_READ_RESOLUTION + yDeadZone)/2;
  maxX = (ANALOG_READ_RESOLUTION + xDeadZone)/2;

  #ifdef DEBUG
    Serial.println("Joysticks Calibrated");
  #endif

  //set motor max
  yStepper.setMaxSpeed(MAX_MOTOR_SPEED);
  yStepper.setCurrentPosition(0);
  xStepper.setMaxSpeed(MAX_MOTOR_SPEED);
  xStepper.setCurrentPosition(0);
  #ifdef DEBUG
    Serial.println("Motors set");
  #endif

  #ifdef HAS_FOCUSER
    focuser.setMaxSpeed(MAX_FOCUS_SPEED);
  #endif

  //set up and display movement menu
  showInfo();  

  #ifdef DEBUG
    Serial.println("Ready");
  #endif
}

void loop() {
  
  switch (screenMode){
    case 0: //GPS Mode
    {
      infoScreensControl();
      if(millis() - lastGPSCheck > 10000){
        showInfo();
        lastGPSCheck = millis();
      }
      break;
    }        
    case 1: //Main Screen
    {
      movementButtonControl();
      if(!moving && !focusing){
        menuControl();
      }
      if(moving){ //move if in move mode
        readJoystickAndMove();  
      }
      #ifdef HAS_FOCUSER
        if(focusing){ //Focusing Mode
          readJoystickAndFocus(); 
        }
      #endif
      break;
    }      
    case 2: //Slew Mode
    {
      slewMode();
      infoScreensControl();
      break;
    }
    
    case 4: //Alignment
      movementButtonControl();
      if(!moving && !focusing){
        menuControl();
      }
      if(moving){ //move if in move mode
        readJoystickAndMove();  
      }  
      break;
    /*3 - Settings Menu
    5 - OffsetsMenu
    6 - Goto Menu
    7 - Planet Menu
    8 - Target Below Horizon */
    default:
      movementButtonControl();
      menuControl();
      break;
  }

  if(isTracking){
    doTrack();
  }
  
  while (btComm.available() > 0){
    communication(btComm);
  }

  while (Serial.available() > 0){
    communication(Serial);
  }

  while(Serial2.available() > 0){
    if(gps.encode(Serial2.read()))
      getGPS();
  }

  setStepperSpeed();

  accelerateMove();
  #ifdef HAS_FOCUSER
    accelerateFocus();
  #endif

}

#ifdef HAS_FOCUSER
void accelerateFocus(){

  if(abs(realFSpeed - fSpeed)> 1){
    if(realFSpeed < fSpeed){
      realFSpeed++;
    }else if (realFSpeed > fSpeed){
      realFSpeed--;
    }  
  }else{
    realFSpeed = fSpeed;
  }

  #ifdef DEBUG_X_JOYSTICK
    Serial.print("X:");
    Serial.println(realFSpeed);
  #endif
  
  focuser.setSpeed(realFSpeed);
  
  focuser.runSpeed();
  
}
#endif



void accelerateMove(){

  if(abs(realXSpeed - xSpeed)> 1){
    if(realXSpeed < xSpeed){
      realXSpeed++;
    }else if (realXSpeed > xSpeed){
      realXSpeed--;
    }  
  }else{
    realXSpeed = xSpeed;
  }

  if(abs(realYSpeed - ySpeed) > 1){
    if(realYSpeed < ySpeed){
      realYSpeed++;
    }else if(realYSpeed > ySpeed){
      realYSpeed--;
    }  
  }else{
    realYSpeed = ySpeed;
  }

  xStepper.setSpeed(realXSpeed);
  yStepper.setSpeed(realYSpeed);
    
  xStepper.runSpeed();
  yStepper.runSpeed();
}

void communication(Stream &aSerial)
{
  byte inByte = aSerial.read();
  static unsigned int input_pos = 0;    
  switch(inByte){
    case 6:
      aSerial.print("A");
      break;
    case '#':
      input[input_pos] = 0; //terminating null byte
  
      //terminator reached! process newdata
      newData = true;
      
      //reset buffer for next time
      input_pos = 0;    
      break;

    case '\r':   // discard line ends
      break;
    case '\n':
      break;

    default:
     // keep adding if not full ... allow for terminating null byte
    if (input_pos < (numChars - 1))
      input[input_pos++] = inByte;
    break;
  }
  
  if(newData){

    String strInput = String(input);
    #ifdef DEBUG_COMMS
      display.printFixed(0,  40, input, STYLE_NORMAL);
      Serial.println(input);
    #endif

    switch (input[1]){
      case 'C': //C - Sync Control
        commsSyncControl(input[2], aSerial);
        break;
      case 'D': //D - Distance Bars
        commsDistanceBars(aSerial);
        break;
      case 'F': //F – Focuser Control
        #ifdef HAS_FOCUSER
          commsFocuserControl(input[2]);
          #endif
        break;
      case 'G': //G – Get Telescope Information
        commsGetTelescopeInfo(aSerial);
        break;
      case 'M': //M – Telescope Movement Commands
        commsMovement(aSerial);
        break;
      case 'Q': //Q – Movement Commands - All Q commands should just stop the telescope.
        setTrack();
        break;
      case 'R': //R – Slew Rate Commands
        commsSlewRate();
        break;
      case 'S': //S – Telescope Set Commands 
        commsSetCommands(aSerial, strInput);
        break;
    }

    for(int i = 0; i < numChars; i++){
      input[i] = '\0';
    }
    newData = false;
  }
}

//C - Sync Control
void commsSyncControl(char input2, Stream &aSerial){
  /*:CM# Synchronizes the telescope's position with the currently selected database object's coordinates.
        Returns:
        LX200's - a "#" terminated string with the name of the object that was synced.
        Autostars & LX200GPS - At static string: " M31 EX GAL MAG 3.5 SZ178.0'#" */
    if(input2 == 'M'){
      
      setXSpeed(0);
      setYSpeed(0);
  
      //Slow down and stop
      while(realYSpeed != ySpeed || realXSpeed != xSpeed){
        accelerateMove();
      }

      setCurrentPositions();
            
      myAstro.setRAdec(myAstro.decimalDegrees(raHH, raMM, raSS), myAstro.decimalDegrees(decD, decMM, decSS));
      
      myAstro.doRAdec2AltAz();
      
      switch(screenMode){
        case 2: //sync while Slewing
          if(slewComplete){ //when slew is complete calculate variance from object

          }
        case 4: //alignment section
        {
          if(alignmentScreen < 3){

            if(alignmentScreen == 0){
              currentAlt = myAstro.getAltitude();
              currentAz = myAstro.getAzimuth();
              synced = true;
            }
            
            alignmentAltValues[alignmentScreen][0] = myAstro.getAltitude();
            alignmentAltValues[alignmentScreen][1] = currentAlt;
            alignmentAzValues[alignmentScreen][0] = myAstro.getAzimuth();
            alignmentAzValues[alignmentScreen][1] = currentAz;

            currentAlt = myAstro.getAltitude();
            currentAz = myAstro.getAzimuth();

            synced = true;

            alignmentScreen++;
          }
          
          if(alignmentScreen == 3){
            moving = false;
            alignmentMenuItems[0] = "Move:  Disabled";
            double alignmentAlt1 = alignmentAltValues[1][0] - alignmentAltValues[0][0];
            alignmentAlt1 = over360Under0(alignmentAlt1);

            double alignmentAlt2 = alignmentAltValues[2][0] - alignmentAltValues[1][0];
            alignmentAlt2 = over360Under0(alignmentAlt2);
            
            double currentAlt1 = alignmentAltValues[1][1] - alignmentAltValues[0][1];
            currentAlt1 = over360Under0(currentAlt1);

            double currentAlt2 = alignmentAltValues[2][1] - alignmentAltValues[1][1];
            currentAlt2 = over360Under0(currentAlt2);
            
            alignmentAltOffset = 1 + ((abs(alignmentAlt1/currentAlt1) - abs(alignmentAlt2/currentAlt2))/2);

            double alignmentAz1 = alignmentAzValues[1][0] - alignmentAzValues[0][0];
            alignmentAz1 = over360Under0(alignmentAz1);

            double alignmentAz2 = alignmentAzValues[2][0] - alignmentAzValues[1][0];
            alignmentAz2 = over360Under0(alignmentAz2);

            double currentAz1 = alignmentAzValues[1][1] - alignmentAzValues[0][1];
            currentAz1 = over360Under0(currentAz1);

            double currentAz2 = alignmentAzValues[2][1] - alignmentAzValues[1][1];
            currentAz2 = over360Under0(currentAz2);

            alignmentAzOffset = 1 + ((abs(alignmentAz1/currentAz1) - abs(alignmentAz2/currentAz2))/2);
            showAlignmentConfirm();
          }else{
            showAlignmentMenu();
          }
          break;
        }
        default:
          currentAlt = myAstro.getAltitude();
          currentAz = myAstro.getAzimuth();
          synced = true;
          break;        
      } 
      
      aSerial.print(1);      
    }
}
//D - Distance Bars
void commsDistanceBars(Stream &aSerial){
  /*:D# Requests a string of bars indicating the distance to the current library object.
      Returns:
      LX200's – a string of bar characters indicating the distance. */
    
    if(slewComplete){
      aSerial.print("#");
    }else{
      aSerial.print("-#");
    }
}

//F – Focuser Control
#ifdef HAS_FOCUSER
void commsFocuserControl(char input2){
  
    switch(input2){
      case '+': //:F+# Start Focuser moving inward (toward objective) Returns: None
        setFSpeed(fSpeedSerial);
        break;

      case '-': //:F-# Start Focuser moving outward (away from objective) Returns: None
        setFSpeed(-fSpeedSerial);
        break;

      case 'Q': //:FQ# Halt Focuser Motion Returns: Nothing
        setFSpeed(0);
        break;

      case 'F': //:FF# Set Focus speed to fastest setting Returns: Nothing
        fSpeedSerial = MAX_FOCUS_SPEED;
        break;

      case 'S': //:FS# Set Focus speed to slowest setting Returns: Nothing
        fSpeedSerial = MAX_FOCUS_SPEED/4;
        break;
      default: //:F<n># Autostar & LX200GPS – set focuser speed to <n> where <n> is an ASCII digit 1..4 Returns: Nothing LX200 – Not Supported 
        if(isDigit(input2)){
          float fSpeedMultiplier = (int) input[2];
          fSpeedMultiplier = fSpeedMultiplier/4;
          fSpeedSerial = MAX_FOCUS_SPEED * fSpeedMultiplier;
        }
        break;
    }
}
#endif

//G – Get Telescope Information
void commsGetTelescopeInfo(Stream &aSerial){
    
    switch(input[2]){
      case 'A': /*:GA# Get Telescope Altitude
                    Returns: sDD*MM# or sDD*MM’SS#
                    The current scope altitude. The returned format depending on the current precision setting*/
      {
        char decCase;
      
        if(currentAlt < 0){
          decCase = 45;
        }else{
          decCase = 43;
        }
          
        char txAlt[7];
        snprintf(txAlt, 7, "%c%02d%c%02d", decCase, getDecDeg(currentAlt), 223, getDecMM(currentAlt));
        aSerial.print(txAlt);
        aSerial.print("#");
        break;
      }

      case 'C': //:GC# Get current date. Returns: MM/DD/YY# The current local calendar date for the telescope.
        //Get Telescope Local Date
        {
          setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
          if(myAstro.getLT() > myAstro.getGMT()){
            adjustTime(-1*((24 + myAstro.getGMT()) - myAstro.getLT())*3600);
          }
          char charDate[12];
          
          snprintf(charDate, 12, "%02d/%02d/%02d#", month(), day(), (year()-2000));
                
          aSerial.print(charDate);
          break;
        }
      
      case 'c': //:Gc# Get Calendar Format Returns: 12# or 24# Depending on the current telescope format setting.
        aSerial.print("24#");
        break;

      case 'D': //:GD# Get Telescope Declination. Returns: sDD*MM# or sDD*MM’SS# Depending upon the current precision setting for the telescope.
        {
          setCurrentPositions();
          
          myAstro.setAltAz(currentAlt, currentAz);
          myAstro.doAltAz2RAdec();
          
          float aDec = myAstro.getDeclinationDec();
      
          char decCase;
      
          if(aDec < 0){
            decCase = 45;
          }else{
            decCase = 43;
          }
          
          char txDEC[11];
          snprintf(txDEC, 11, "%c%02d%c%02d:%02d#", decCase, getDecDeg(aDec), 223, getDecMM(aDec),getDecSS(aDec));
          aSerial.print(txDEC);
          break;
        }
      case 'G': //:GG# Get UTC offset time Returns: sHH# or sHH.H#
        /* The number of decimal hours to add to local time to convert it to UTC. If the number is a whole number the
        sHH# form is returned, otherwise the longer form is return. On Autostar and LX200GPS, the daylight savings
        setting in effect is factored into returned value. */
        {
          int calcOffset = (myAstro.getLT() < myAstro.getGMT()? myAstro.getGMT() : myAstro.getGMT() + 24) - myAstro.getLT();
          aSerial.print(calcOffset);
          aSerial.print("#");
          break;
        }
      case 'g': //:Gg# Get Current Site Longitude Returns: sDDD*MM# The current site Longitude. East Longitudes are expressed as negative
        {
          float lon = gps.location.lng();
          
          int lonDeg = (int) lon;
          float lonMinutesRemainder = abs(lon - lonDeg) * 60;
          int lonMin = (int) lonMinutesRemainder;

          lonDeg *= -1;

          char charLon[20];
          snprintf(charLon, 20, "%03d*%02d#", lonDeg, lonMin);
          aSerial.print(charLon);
          break;
        }
      case 'L': //:GL# Get Local Time in 24 hour format  Returns: HH:MM:SS#
        {
          char charDate[12];
          snprintf(charDate, 12, "%02d:%02d:%02d#", getRaHH(myAstro.getLT()), getRaMM(myAstro.getLT()), getRaSS(myAstro.getLT()));
          aSerial.print(charDate);
          break;
        }
      case 'M': //:GM# Get Site 1 Name Returns: <string># A ‘#’ terminated string with the name of the requested site.
        aSerial.print(sites[0]);
        aSerial.print("#");
        break;

      case 'N': //:GN# Get Site 2 Name Returns: <string>#  A ‘#’ terminated string with the name of the requested site.
        aSerial.print(sites[1]);
        aSerial.print("#");
        break;

      case 'O': //:GO# Get Site 3 Name Returns: <string># A ‘#’ terminated string with the name of the requested site.
        aSerial.print(sites[2]);
        aSerial.print("#");
        break;
      case 'P': //:GP# Get Site 4 Name Returns: <string># A ‘#’ terminated string with the name of the requested site.
        aSerial.print(sites[3]);
        aSerial.print("#");
        break;

      case 'R': //:GR# Get Telescope RA Returns: HH:MM.T# or HH:MM:SS# Depending which precision is set for the telescope
        {
          setCurrentPositions();
          
          myAstro.setAltAz(currentAlt, currentAz);
          myAstro.doAltAz2RAdec();

          float aRa = myAstro.getRAdec();
          char txRA[10];
          snprintf(txRA, 10, "%02d:%02d:%02d#", getRaHH(aRa), getRaMM(aRa), getRaSS(aRa));
          aSerial.print(txRA);
          break;
        }
      case 's': /*Get Smaller Size Limit
                    Returns: NNN'#
                    The size of the largest object returned by the FIND command expressed in arcminutes.
                */
      {
        aSerial.print("190'#"); //M31 size
        break;
      }
      case 'T': /*:GT# Get tracking rate
                      Returns: TT.T#
                      Current Track Frequency expressed in hertz assuming a synchonous motor design where a 60.0 Hz motor clock
                      would produce 1 revolution of the telescope in 24 hours.*/
      {
          double trackingRate = 2.5* 3600 * (((realXSpeed * rotationDegrees) / GEAR_RATIO) /stepperDivider);
          aSerial.print(trackingRate, 1);
          aSerial.print('#');
          break;
      }
      case 't': //:Gt# Get Current Site Latitdue Returns: sDD*MM# The latitude of the current site. Positive inplies North latitude.
        {
          float lat = gps.location.lat();
          
          int latDeg = (int) lat;
          float latMinutesRemainder = abs(lat-latDeg) * 60;
          int latMin = (int)latMinutesRemainder;

          char charLat[20];
          snprintf(charLat, 20, "%02d*%02d#", latDeg, latMin);
          aSerial.print(charLat);
          break;
        }
      case 'V': //Telescope Info
        switch(input[3]){
          case 'D': //:GVD# Get Telescope Firmware Date Returns: mmm dd yyyy#
            aSerial.print(FIRMWARE_DATE);
            aSerial.print("#");
            break;
          case 'N': //:GVN# Get Telescope Firmware Number Returns: dd.d# 
            aSerial.print(FIRMWARE_VERSION);
            aSerial.print("#");
            break;
          case 'P': //:GVP# Get Telescope Product Name Returns: <string># 
            aSerial.print(MOUNT_NAME);
            aSerial.print("#");
            break;
          case 'T': //:GVT# Get Telescope Firmware Time returns: HH:MM:SS#
            aSerial.print(FIRMWARE_TIME);
            aSerial.print("#");
        }
        break;
      case 'W': //Get Track State
        {
          aSerial.print("A");
      
          if(isTracking){
            aSerial.print("T");
          }else{
            aSerial.print("N");
          }

          if(synced){
            aSerial.print("1");
          }else{
            aSerial.print("0");
          }

          aSerial.print("#");
        break;
        }
      case 'Z': /* :GZ# Get telescope azimuth
                    Returns: DDD*MM#T or DDD*MM’SS#
                    The current telescope Azimuth depending on the selected precision. 
                */
      {
        char txAz[7];
        snprintf(txAz, 7, "%03d%c%02d", getDecDeg(currentAz), 223, getDecMM(currentAz));
        aSerial.print(txAz);
        aSerial.print("#");
        break;
      }
    }
}

//M – Telescope Movement Commands
void commsMovement(Stream &aSerial){
  isTracking = false;
  switch (input[2]){
    case 'e': //:Me# Move Telescope East at current slew rate Returns: Nothing
      setXSpeed(50);
      break;
    case 'w': //:Mw# Move Telescope West at current slew rate Returns: Nothing 
      setXSpeed(-50);
      break;
    case 'n': //:Mn# Move Telescope North at current slew rate Returns: Nothing
      setYSpeed(-50);
      break;
    case 's': //:Ms# Move Telescope South at current slew rate Returns: Nothing 
      setYSpeed(50);
      break;
      /*:MS# Slew to Target Object
        Returns:
        0 Slew is Possible
        1<string># Object Below Horizon w/string message
        2<string># Object Below Higher w/string message */
    case 'S':
      moving = false;
  
      targetRaHH = raHH;
      targetRaMM = raMM;
      targetRaSS = raSS;

      targetDecD = decD;
      targetDecMM = decMM;
      targetDecSS = decSS;
      
      startSlew();
      
      aSerial.print(1);
      break;
  }
}

//R – Slew Rate Commands
void commsSlewRate(){
  switch(input[2]){
    case 'C': //:RC# Set Slew rate to Centering rate (2nd slowest) Returns: Nothing 
      stepperSpeed = 1;
      break;
    case 'G': //:RG# Set Slew rate to Guiding Rate (slowest) Returns: Nothing
      stepperSpeed = 0;
      break;
    case 'M': //:RM# Set Slew rate to Find Rate (2nd Fastest) Returns: Nothing 
      stepperSpeed = 2;
      break;
    case 'S': //:RS# Set Slew rate to max (fastest) Returns: Nothing
      stepperSpeed = 3;
      break;
  }
}

//S – Telescope Set Commands 
void commsSetCommands(Stream &aSerial,String strInput){
  int siteID = 0;
  switch(input[2]){ /*:SCMM/DD/YY#
                      Change Handbox Date to MM/DD/YY
                      Returns: <D><string>
                        D = ‘0’ if the date is invalid. The string is the null string.
                        D = ‘1’ for valid dates and the string is “Updating Planetary Data#*/
    case 'C': {
      inMonth = strInput.substring(3,5).toInt();
      inDay = strInput.substring(6,8).toInt();
      inYear = strInput.substring(9).toInt();
      inYear += 2000;

      setTime(inHour, inMinute, inSecond, inDay, inMonth, inYear);

      adjustTime(inHourOffset * 3600);

      myAstro.setGMTdate(year(), month(), day());
      myAstro.setGMTtime(hour(), minute(), second());
      myAstro.useAutoDST();

      aSerial.print(1);
      aSerial.print("Updating Planetary Data# #");
      break;
    }
    case 'd': { /*:SdsDD*MM#
        Set target object declination to sDD*MM or sDD*MM:SS depending on the current precision setting
        Returns:
        1 - Dec Accepted
        0 – Dec invalid */
      if(input[3] == '-'){
        decD = strInput.substring(3,6).toInt();
      }else{
        decD = strInput.substring(4,6).toInt();
      }
      
      decMM = strInput.substring(7,9).toInt();
      decSS = strInput.substring(10,12).toInt();
      
      aSerial.print(1);
      break;
    }
    case 'G': {  /*:SGsHH.H#
                  Set the number of hours added to local time to yield UTC
                  Returns:
                  0 – Invalid
                  1 - Valid */
      inHourOffset = strInput.substring(3,7).toInt();
      aSerial.print(1);
      break;
    }
    case 'g': { /*:SgDDD*MM#
                  Set current site’s longitude to DDD*MM an ASCII position string
                  Returns:
                  0 – Invalid
                  1 - Valid */
      
       int inLonD = strInput.substring(3,6).toInt();
       
       float inLonM = (float)strInput.substring(7,9).toInt();

       inLonM = inLonM/60;
       flon = -1 * (inLonD + inLonM);
       

       if(flon != 0 && flat != 0){
          myAstro.setLatLong((double)flat, (double)flon);
       }
      aSerial.print(1);
      break;
    }
    case 'L': { /*:SLHH:MM:SS#
                  Set the local Time
                  Returns:
                    0 – Invalid
                    1 - Valid */
      inHour = strInput.substring(3,5).toInt();
      inMinute = strInput.substring(6,8).toInt();
      inSecond = strInput.substring(9).toInt();
      
      aSerial.print(1);
      break;
    }
    case 'P': //:SP<string># Site 4
      siteID++;
    case 'O': //:SO<string># Site 3
      siteID++;
    case 'N': //:SN<string># Site 2
      siteID++;
    case 'M': /*:SM<string>#
                Set site 1’s name to be <string>. LX200s only accept 3 character strings. Other scopes accept up to 15 characters.
                  Returns:
                  0 – Invalid
                  1- Valid */
      
      sites[siteID] = strInput.substring(3,strInput.indexOf("#")).c_str();
      preferences.putString("site" + siteID, sites[siteID]);

      aSerial.print(1);
      break;
    case 'r':{ /*:SrHH:MM.T#
                  :SrHH:MM:SS#
                  Set target object RA to HH:MM.T or HH:MM:SS depending on the current precision setting.
                  Returns:
                  0 – Invalid
                  1 - Valid */
      raHH = strInput.substring(3,5).toInt();
      raMM = strInput.substring(6,8).toInt();
      raSS = strInput.substring(9).toInt();
            
      aSerial.print(1);
      break;
    }
    case 'S':/*:SSHH:MM:SS#
          Sets the local sideral time to HH:MM:SS
          Returns:
          0 – Invalid
          1 - Valid */
    {
      //this gets calculated so just send a valid
      aSerial.print(1);
      break;
    }
    case 's':{ /*:SsNNN#
                  Set the size of the largest object the FIND/BROWSE command will return to NNNN arc minutes
                  Returns:
                  0 – Invalid
                  1 - Valid*/
              aSerial.print(1);
              break;
    }
    case 't': { /*:StsDD*MM#
                  Sets the current site latitdue to sDD*MM#
                  Returns:
                  0 – Invalid
                  1 - Valid */
      int inLatD = 0;
      
      if(input[3] == '-'){
        inLatD = strInput.substring(3,6).toInt();
      }else{
        inLatD = strInput.substring(4,6).toInt();
      }

      float inLatF = ((float)strInput.substring(7,9).toInt())/60;
      
      flat = inLatD + inLatF;
      if(flon != 0 && flat != 0){
        myAstro.setLatLong((double)flat, (double)flon);
      }
            
      aSerial.print(1);
      break;
    }
    case 'w': /*:SwN#
                Set maximum slew rate to N degrees per second. N is the range (2..8)
                Returns:
                0 – Invalid
                1 - Valid */
    {
      //just return 1 for now
      aSerial.print(1);
      break;
    }

  }
}

void startSlew(){
  screenMode = 2;
  stepperSpeed = 3;

  //get ready to move fast
  slewComplete = false;
  isTracking = false;
  slewScreenTimeOut = 2000;
  slewScreenUpdate = millis();

  display.clear();

  display.printFixed(0, 0,  "Current Alt:", STYLE_NORMAL);
  display.printFixed(0, 8,  "Current Az: ", STYLE_NORMAL);
  display.printFixed(0, 16, "Target Alt: ", STYLE_NORMAL);
  display.printFixed(0, 24, "Target Az:  ", STYLE_NORMAL);
  display.printFixed(0, 32, "Remain Alt: ", STYLE_NORMAL);
  display.printFixed(0, 40, "Remain Az:  ", STYLE_NORMAL);
}



void slewMode(){

  myAstro.setRAdec(myAstro.decimalDegrees(targetRaHH, targetRaMM, targetRaSS), myAstro.decimalDegrees(targetDecD, targetDecMM, targetDecSS));
  myAstro.doRAdec2AltAz();
  targetAlt = myAstro.getAltitude();
  targetAz = myAstro.getAzimuth();
  
  setCurrentPositions();
  
  double remainingAlt = targetAlt - currentAlt;
  double remainingAz = azDifference(currentAz, targetAz);
  
  double minStep = (rotationDegrees/GEAR_RATIO)/32;
  bool moveAz = false;
  bool moveAlt = false;
  float moveSpeed = 0;
  
  if(!slewComplete){

    if(
        abs(remainingAz) > stepperSpeed &&
        abs(remainingAz) > minStep
    ){
        moveAz = true;
    }

    if(
        abs(remainingAlt) > (stepperSpeed) &&
        abs(remainingAlt) > minStep
    ){
      moveAlt = true;
    }
    
    moveSpeed = MAX_MOTOR_SPEED;
    moveSpeed *=  (stepperSpeed + 1);
    moveSpeed /=  4;

    if(moveAz){
      setXSpeed(remainingAz > 0 ? (-1 * moveSpeed) : moveSpeed);
    }else{
      setXSpeed(0);
    }

    if(moveAlt){
      setYSpeed(remainingAlt > 0 ? ((-1 * moveSpeed)/2) : (moveSpeed/2));
    }else{
      setYSpeed(0);
    }

    if(!moveAlt && !moveAz){
      stepperSpeed--;
    }

    if(stepperSpeed < 0){
      slewComplete = true;
      stepperSpeed = 0;
      setTrack();
    }
    
  }

  if(millis() - slewScreenUpdate > slewScreenTimeOut){
      
    if(slewComplete){
      display.clear();
      display.printFixed(0, 0, "Slew Complete!", STYLE_NORMAL);
      display.printFixed(0, 16, "Press the button to return to main menu.", STYLE_NORMAL);
      slewScreenTimeOut = 60000;
    }else{
      
      int intAlt = (int) currentAlt;
      int decAlt = (100 * abs(currentAlt - intAlt));
      int intAz = (int) currentAz;
      int decAz = (100 * abs(currentAz - intAz));
  
      int intTargetAlt = (int) targetAlt;
      int decTargetAlt = (100 * abs(targetAlt - intTargetAlt));
      int intTargetAz = (int) targetAz;
      int decTargetAz = (100 * abs(targetAz - intTargetAz));
  
      int intRemainingAlt = (int) remainingAlt;
      int decRemainingAlt = (100 * abs(remainingAlt) - abs(intRemainingAlt));
      int intRemainingAz = (int) remainingAz;
      int decRemainingAz = (100 * abs(remainingAz) - abs(intRemainingAz));
      
      char charCurrentAlt[20];
      snprintf(charCurrentAlt, 20, "%d.%d", intAlt, decAlt);
      display.printFixed(72, 0, "       ", STYLE_NORMAL);
      display.printFixed(72, 0, charCurrentAlt, STYLE_NORMAL);
      
      char charCurrentAz[20];
      snprintf(charCurrentAz, 20, "%d.%d", intAz, decAz);
      display.printFixed(72,  8, "       ", STYLE_NORMAL);
      display.printFixed(72,  8, charCurrentAz, STYLE_NORMAL);
  
      char charTargetAlt[20];
      snprintf(charTargetAlt, 20, "%d.%d", intTargetAlt, decTargetAlt);
      display.printFixed(72,  16, "       ", STYLE_NORMAL);
      display.printFixed(72,  16, charTargetAlt, STYLE_NORMAL);
  
      char charTargetAz[20];
      snprintf(charTargetAz, 20, "%d.%d", intTargetAz, decTargetAz);
      display.printFixed(72,  24, "       ", STYLE_NORMAL);
      display.printFixed(72,  24, charTargetAz, STYLE_NORMAL);

      char charRemainingAlt[20];
      snprintf(charRemainingAlt, 20, "%d.%d", intRemainingAlt, decRemainingAlt);
      display.printFixed(72,  32, "       ", STYLE_NORMAL);
      display.printFixed(72,  32, charRemainingAlt, STYLE_NORMAL);

      char charRemainingAz[20];
      snprintf(charRemainingAz, 20, "%d.%d", intRemainingAz, decRemainingAz);
      display.printFixed(72,  40, "       ", STYLE_NORMAL);
      display.printFixed(72,  40, charRemainingAz, STYLE_NORMAL);
      
    }
    
    slewScreenUpdate = millis();
  }
}

void getGPS(){
  if(gps.location.isValid()){
    
    flat = gps.location.lat();
    flon = gps.location.lng();
    
    myAstro.setLatLong((double)flat, (double)flon);
    #ifdef DEBUG_GPS
      Serial.println("Location Updated");
      Serial.print("Lat: ");
      Serial.println(flat,6);
      Serial.print("Long: ");
      Serial.println(flon,6);
    #endif
    
  }
  
  if(gps.altitude.isValid()){
    faltitude = gps.altitude.meters();
    myAstro.setElevationM((double)faltitude);

    #ifdef DEBUG_GPS
      Serial.print("Alt: ");
      Serial.println(faltitude,6);
    #endif
  }
   
  
  sats = gps.satellites.value();

  #ifdef DEBUG_GPS
      Serial.print("Sats: ");
      Serial.println(sats);
    #endif
  
  if(gps.date.isValid()){
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    myAstro.setGMTdate(gps.date.year(), gps.date.month(), gps.date.day());
    myAstro.setGMTtime(gps.time.hour(), gps.time.minute(), gps.time.second());
    myAstro.useAutoDST();
    
    #ifdef DEBUG_GPS
      Serial.print("Date: ");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.print(gps.date.year());
      Serial.print(" ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second(),2);
    #endif 
  }
}

void gotoNotReady(){
  screenMode = 0;
  display.clear();
  display.printFixed(0, 0,  "Goto Unavailable.", STYLE_NORMAL);
  display.printFixed(0, 8,  "Please sync first.", STYLE_NORMAL);
  display.printFixed(0, 16, "Press the button to return to main menu.", STYLE_NORMAL);

}

void showInfo(){
  screenMode = 0;
  
  display.clear();
  
  int latDeg = (int) flat;
  float latMinutesRemainder = abs(flat-latDeg) * 60;
  int latMin = (int)latMinutesRemainder;
  float latSec = (latMinutesRemainder - latMin) * 60;
  
  int lonDeg = (int) flon;
  float lonMinutesRemainder = abs(flon - lonDeg) * 60;
  int lonMin = (int) lonMinutesRemainder;
  float lonSec = (lonMinutesRemainder - lonMin) * 60;

  //Lat
  char lat[20];
  snprintf(lat, 20, "LAT: %02d,%02d", latDeg, latMin);
  
  String latSecVal = lat;
  latSecVal.concat("'");
  latSecVal.concat(String(latSec));
  char charLat[20];
  snprintf(charLat, 20, latSecVal.c_str());
  
  display.printFixed(0,  0, charLat, STYLE_NORMAL);
  
  //Lon
  char lon[20];
  snprintf(lon, 20, "LON: %02d,%02d", lonDeg, lonMin);
  String lonSecVal = lon;
  lonSecVal.concat("'");
  lonSecVal.concat(String(lonSec));

  char charPrint[20];
  snprintf(charPrint, 20, lonSecVal.c_str());
  display.printFixed(0,  8, charPrint, STYLE_NORMAL);

  //Altitude
  snprintf(charPrint, 20, "Alt: %.02fm",faltitude);
  display.printFixed(0,  16, charPrint, STYLE_NORMAL);
  

  //Satellites
  snprintf(charPrint, 20, "Sats: %d", sats);
  display.printFixed(0,24, charPrint, STYLE_NORMAL);

  //Local Time
  snprintf(charPrint, 20, "UTC Time: %02d:%02d", hour(), minute());
  display.printFixed(0,32, charPrint, STYLE_NORMAL);
}

void infoScreensControl(){
  //Read button press
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      //Clear and return to main menu
      if (buttonState == LOW) {
        
        showMainMenu();
      }
      
    }
  }
  
  lastButtonState = reading;
}

void showMainMenu(){
  display.clear();
  if(!slewComplete){
    slewComplete = true;
    setXSpeed(0);
    setYSpeed(0);  
  }
  
  screenMode = 1;
  mainMenu.show( display );
}

void showSettingsMenu(){
  display.clear();
  screenMode = 3;
  settingsMenu.show( display );
}

void showAlignmentMenu(){
  display.clear();
  switch(alignmentScreen){
    case 0:
      alignmentMenu.show(display);
      display.printFixed(8,  40, "Align with first", STYLE_NORMAL);
      display.printFixed(8,  48, "object then sync", STYLE_NORMAL);
      break;
    case 1:
      alignmentMenu.show(display);
      display.printFixed(8,  40, "Align with second", STYLE_NORMAL);
      display.printFixed(8,  48, "object then sync", STYLE_NORMAL);
      break;
    case 2:
      alignmentMenu.show(display);
      display.printFixed(8,  40, "Align with third", STYLE_NORMAL);
      display.printFixed(8,  48, "object then sync", STYLE_NORMAL);
      break;
    case 4:
      display.printFixed(4,  0,  "Cannot align", STYLE_NORMAL);
      display.printFixed(4,  8,  "GPS not acquired", STYLE_NORMAL);
      display.printFixed(4,  16, "Please try again", STYLE_NORMAL);
      display.printFixed(4,  24, "when GPS is ready", STYLE_NORMAL);
      break;
  }
}

void showAlignmentConfirm(){
  display.clear();
  alignmentConfirm.show(display);
  char output[16];
  snprintf(output, 16, "Alt:%f", alignmentAltOffset);

  display.printFixed(4, 20,  output, STYLE_NORMAL);
  snprintf(output, 16, "Az:%f", alignmentAzOffset);
  display.printFixed(4, 28,  output, STYLE_NORMAL);
}

void showOffsetsMenu(){
  screenMode = 5;
  display.clear();
  char output[16];
  snprintf(output, 16, "Alt: %f", moveOffsets[0]); //Alt is 0
  offSetsMenuItems[1] = output;
  char output2[16];
  snprintf(output2, 16, "Az:  %f", moveOffsets[1]); //Az is 1
  offSetsMenuItems[2] = output2;
  offsetsMenu.show(display);
}

void showGotoMenu(){
  screenMode = 6;
  display.clear();
  char output[16];
  strcpy(output, "Planet: ");
  strcat(output, planets[planetObject]);
  gotoMenuItems[1] = output;

  char output2[16];
  snprintf(output2, 16, "Messier: M%u", messierObject);
  gotoMenuItems[2] = output2;
  
  char output3[16];
  snprintf(output3, 16, "Caldwell: %u", caldwellObject);
  gotoMenuItems[3] = output3;
  
  gotoMenu.show(display); 
}

void showTargetNotVisible(int returnToScreen){
  screenMode = 8;
  display.clear();
  display.printFixed(0, 0,  "Target is below", STYLE_NORMAL);
  display.printFixed(0, 8,  "the horizon", STYLE_NORMAL);
  display.printFixed(0, 16, "Press button to", STYLE_NORMAL);
  display.printFixed(0, 24, "return to GoTo menu.", STYLE_NORMAL);
  returnScreenMode = returnToScreen;
}

void gotoTarget(int returnToScreen){
  myAstro.doRAdec2AltAz();
  if(myAstro.getAltitude() > 0){          
    targetRaHH = getRaHH(myAstro.getRAdec());
    targetRaMM = getRaMM(myAstro.getRAdec());
    targetRaSS = getRaSS(myAstro.getRAdec());

    targetDecD = getDecDeg(myAstro.getDeclinationDec());
    targetDecMM = getDecMM(myAstro.getDeclinationDec());
    targetDecSS = getDecSS(myAstro.getDeclinationDec());
    startSlew();
  }else{
    showTargetNotVisible(returnToScreen);
  }
}


void returnToScreenMode(){
  switch(returnScreenMode){
    case 6:
      showGotoMenu();
      break;
    case 7:
      
      break;
  }
}

void movementButtonControl(){
  //Read button press
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      //Only Mess with the menu if LOW
      if (buttonState == LOW) {

        switch(screenMode){
          case 1:
            {
              int selected = mainMenu.selection();
              #ifndef HAS_FOCUSER
                if(selected > 0){
                selected += 1;
                }
              #endif

              switch(selected){
                case 0: //Move
                {
                  if(moving){
                    mainMenuItems[0] = "Move:  Disabled";
                    mainMenu.show(display);  
                    setTrack();
                  }else{
                    mainMenuItems[0] = "Move:  Enabled";
                    mainMenu.show(display);
                    if(isTracking){
                      stepperSpeed = preTrackStepperSpeed;
                    }
                    moving = true;
                    isTracking = false;
                  }
                  break;
                }
                
                case 1: //Focusing
                {
                  if(focusing){
                    focusing = false;
                    mainMenuItems[1] = "Focus: Disabled";
                  }else{
                    mainMenuItems[1] = "Focus: Enabled";
                    focusing = true;
                  }
                  mainMenu.show(display);
                  break;
                }

                case 3: //Settings
                {
                  showSettingsMenu();
                  break;
                }

                case 4: //GPS Mode
                {
                  showInfo();
                  break;
                }

                case 5: //Show Goto
                {
                  showGotoMenu();
                  break;
                }
              }
              break;
            }
          case 3: //Settings
            switch(settingsMenu.selection()){
              case 0:
                showMainMenu();
                break;
              case 1:
                if(flat != 0 && flon != 0){
                  alignmentScreen = 0;
                  tempMoveOffsets[0] = moveOffsets[0];
                  tempMoveOffsets[1] = moveOffsets[1];
                  moveOffsets[0] = 1.0;
                  moveOffsets[1] = 1.0;
                }else{
                  alignmentScreen = 4;
                }
                screenMode = 4;
                
                showAlignmentMenu();
                break;
              case 2:
                showOffsetsMenu();
                break;
              case 3:
                //Set Tracking
                if(tracking){
                  settingsMenuItems[3] = "Tracking: Disabled";
                  tracking = false;
                }else{
                  settingsMenuItems[3] = "Tracking: Enabled";
                  tracking = true;
                }
                showSettingsMenu();
                break;
            }
            break;
          case 4: //Alignment
            switch(alignmentScreen){
              case 0:
              case 1:
              case 2:
                switch(alignmentMenu.selection()){
                  case 0:
                    if(moving){
                      alignmentMenuItems[0] = "Move:  Disabled";
                      setTrack();
                    }else{
                      
                      alignmentMenuItems[0] = "Move:  Enabled";
                      
                      if(isTracking){
                        stepperSpeed = preTrackStepperSpeed;
                      }
                      moving = true;
                      isTracking = false;
                    }
                    showAlignmentMenu();
                    break;
                  case 2:
                    showSettingsMenu();
                    break;
                }
                break;
              case 3:
                if(alignmentConfirm.isYes()){
                  setOffsets(0, alignmentAltOffset);
                  setOffsets(1,  alignmentAzOffset);
                }else{
                  moveOffsets[0] = tempMoveOffsets[0];
                  moveOffsets[1] = tempMoveOffsets[1];
                }
                showSettingsMenu();
                break;
              case 4:
                showSettingsMenu();
                break;
            }
            break;
          case 5: //offsets
            switch(offsetsMenu.selection()){
              case 0:
                showSettingsMenu();
                break;
              case 3:
                setOffsets(0, 1.0);
                setOffsets(1, 1.0);
                
                showOffsetsMenu();
                break;
            }
            break;
          case 6: //Goto
            switch(gotoMenu.selection()){
              case 0:
                showMainMenu();
                break;
              case 1:
                myAstro.doPlans((planetObject + 1));
                gotoTarget(6);
                break;
              case 2: //Messier Objects
                myObjects.selectMessierTable(messierObject);
                myAstro.setRAdec(myObjects.getRAdec(), myObjects.getDeclinationDec());
                gotoTarget(6);
                break;
              case 3:
                myObjects.selectCaldwellTable(caldwellObject);
                myAstro.setRAdec(myObjects.getRAdec(), myObjects.getDeclinationDec());
                gotoTarget(6);
                break;
              case 4:
                myAstro.doMoon();
                gotoTarget(6);
              break;
            }
            break;
          case 7: 
            
            break;
          case 8:
            returnToScreenMode();
            break;
        }
      }
    }
  }
  
  lastButtonState = reading;
}

int getYStick(){
  int inY = analogRead(Y_JOYSTICK_PIN);
  
  #ifdef DEBUG_Y_JOYSTICK
    Serial.print("Y:");
    Serial.println(inY);
  #endif

  if(inY > maxY){
    maxY = inY;
  }
  return inY;
}

int getXStick(){
  int inX = analogRead(X_JOYSTICK_PIN);
  
  if(inX > maxX){
    maxX = inX;
  }
  return inX;
}

void menuControl(){
  int inY = getYStick();
  int y = 0;

  if(inY < (ANALOG_READ_RESOLUTION/12)){
    y = -1 * Y_INVERT;
  }
  if(inY >= (ANALOG_READ_RESOLUTION/12) && inY <= (maxY - (ANALOG_READ_RESOLUTION/12))){
    y = 0;
  }
  if(inY > (maxY - (ANALOG_READ_RESOLUTION/12))){
    y = 1 * Y_INVERT;
  }  
  
  if(y != lastYState){
    lastYDebounceTime = millis();
    longYPressActive = false;
  }

  if((millis() - lastYDebounceTime) > debounceDelay){

    if(y != yState){
      yState = y;
      
      if(y == -1){
        switch(screenMode){
          case 1:
            mainMenu.down();
            mainMenu.show(display);
            break;
          case 3:
            settingsMenu.down();
            settingsMenu.show(display);
            break;
          case 4:
            if(alignmentScreen < 3){
              alignmentMenu.down();
              showAlignmentMenu();
            }            
            break;
          case 5:
            offsetsMenu.down();
            showOffsetsMenu();
            break;
          case 6:
            gotoMenu.down();
            gotoMenu.show(display);
            break;
          case 7:
            
            break;
        }
      }else if(y == 1){
        switch(screenMode){
          case 1:
            mainMenu.up();
            mainMenu.show(display);
            break;
          case 3:
            settingsMenu.up();
            settingsMenu.show(display);
            break;
          case 4:
            if(alignmentScreen < 3){
              alignmentMenu.up();
              showAlignmentMenu();
            }            
            break;
          case 5:
            offsetsMenu.up();
            showOffsetsMenu();
            break;
          case 6:
            gotoMenu.up();
            gotoMenu.show(display);
            break;
          case 7:
            
            break;
          
        }
      }
    }else if((millis() - lastYDebounceTime) > longPressTime && longYPressActive == false)
    {
      longYPressActive = true;
      if(y == -1){
        switch(screenMode){
          //place holders for long presses
          default: //not needed yet
            break;
        }
      }else if(y == 1){
        switch(screenMode){
          //place holders for long presses
          default: //not needed at them moment
            break;
        }
      }
      
    }
  }
  lastYState = y;

  //Handle right left movement
  int inX = getXStick();
  int x = 0;

  if(inX < (ANALOG_READ_RESOLUTION/12)){
    x = -1 * X_INVERT;
  }
  if(inX >= (ANALOG_READ_RESOLUTION/12) && inX <= (maxX - (ANALOG_READ_RESOLUTION/12))){
    x = 0;
  }
  if(inX > (maxX - (ANALOG_READ_RESOLUTION/12))){
    x = 1 * X_INVERT;
  }  
  
  if(x != lastXState){
    lastXDebounceTime = millis();
    longXPressActive = false;
  }

  if((millis() - lastXDebounceTime) > debounceDelay){

    if(x != xState){
      xState = x;
     
      switch(screenMode){
        case 1:
          //Setting Speed
          if(speedIndex == mainMenu.selection()){
            toggleStepperSpeed(x);
          }
          break;
        case 4:
          if(alignmentScreen < 3){
            if(alignmentMenu.selection() == 1){
              toggleStepperSpeed(x);
            }
          }else{
            if(x < 0){
              alignmentConfirm.swapToNo();
            }else if(x > 0){
              alignmentConfirm.swapToYes();
            }
            showAlignmentConfirm();
          }
          break;
        case 6:
        {
          if(x == -1){
            switch(gotoMenu.selection()){
              case 1:
                if(planetObject == 6){
                  planetObject = 0;
                }else{
                  planetObject += 1;
                }
                break;
              case 2:
                if(messierObject == 110){
                  messierObject = 1;
                }else{
                  messierObject += 1;
                }
                break;
              case 3:
                if(caldwellObject == 109){
                  caldwellObject = 1;
                }else{
                  caldwellObject += 1;
                }
                break;
            }
          }else if( x == 1){
            switch(gotoMenu.selection()){
              case 1:
                if(planetObject == 0){
                  planetObject = 6;
                }else{
                  planetObject -= 1;
                }
                break;
              case 2:
                if(messierObject == 1){
                  messierObject = 110;
                }else{
                  messierObject -= 1;
                }
                break;
              case 3:
                if(caldwellObject == 1){
                  caldwellObject = 109;
                }else{
                  caldwellObject -= 1;
                }
                break;
            }
          }
          showGotoMenu();
          break;
        }          
      }
    }else if((millis() - lastXDebounceTime) > longPressTime && longXPressActive == false){
      longXPressActive = true;
      if(x == 1){
        switch(screenMode){
          case 6:
            switch(gotoMenu.selection()){
              case 1:
                if(planetObject < 3){
                  planetObject = 6;
                }else{
                  planetObject -= 2;
                }
                break;
              case 2:
                if(messierObject < 6){
                  messierObject = 110;
                }else{
                  messierObject -= 5;
                }
                break;
              case 3:
                if(caldwellObject < 6){
                  caldwellObject = 109;
                }else{
                  caldwellObject -= 5;
                }
                break;
            }
            showGotoMenu();
          break;
        }
      }else if(x == -1){
        switch(screenMode){
          case 6:
            switch(gotoMenu.selection()){
              case 1:
                if(planetObject > 4){
                  planetObject = 1;
                }else{
                  planetObject += 2;
                }
                break;
              case 2:
                if(messierObject > 105){
                  messierObject = 1;
                }else{
                  messierObject += 5;
                }
                break;
              case 3:
                if(caldwellObject > 104){
                  caldwellObject = 1;
                }else{
                  caldwellObject += 5;
                }
                break;
            }
            showGotoMenu();
            break;
        }
      }
    }else if((millis() - lastXDebounceTime) > (longPressTime * 2) && longXPressActive == true){
      longXPressActive = false;
    }
  }
  lastXState = x;
}

void toggleStepperSpeed(int x){
  isTracking = false;
  stepperSpeed += x;
  if(stepperSpeed < 0){
    stepperSpeed = 3;
  }
  if(stepperSpeed > 3){
    stepperSpeed = 0;
  }
  preTrackStepperSpeed = stepperSpeed;
}

#ifdef HAS_FOCUSER
  void readJoystickAndFocus(){
    
  int x = getXStick();

  int aXSpeed = 0;
  
  if(x > xDeadZone + 50){
    aXSpeed = map(x, (xDeadZone + 50), maxX, 0, MAX_FOCUS_SPEED * X_INVERT);
    xMoving = true;    
  }else if(x < xDeadZone - 50){
    aXSpeed = map(x, (xDeadZone - 50),0,0, -MAX_FOCUS_SPEED * X_INVERT);
    xMoving = true;
  }else{
    xMoving = false;
  }

  if(xMoving){
    setFSpeed(aXSpeed);
  }else{
    setFSpeed(0);
  }
  }
#endif


void readJoystickAndMove(){
  int y = getYStick();

  int aYSpeed = 0;

  if(y > yDeadZone + 50){
    aYSpeed = map(y, (yDeadZone + 50), maxY, 0, MAX_MOTOR_SPEED * Y_INVERT);
    yMoving = true;
  }else if(y < yDeadZone - 50){
    aYSpeed = map(y, (yDeadZone - 50),0,0, -MAX_MOTOR_SPEED * Y_INVERT);
    yMoving = true;
  }else{
    yMoving = false;
  }

  if(yMoving){
    setYSpeed(aYSpeed);
  }else{
    setYSpeed(0);
  }
  
  int x = getXStick();

  int aXSpeed = 0;
  
  if(x > xDeadZone + 50){
    aXSpeed = map(x, (xDeadZone + 50), maxX, 0, MAX_MOTOR_SPEED * X_INVERT);
    xMoving = true;    
  }else if(x < xDeadZone - 50){
    aXSpeed = map(x, (xDeadZone - 50),0,0, -MAX_MOTOR_SPEED * X_INVERT);
    xMoving = true;
  }else{
    xMoving = false;
  }

  if(xMoving){
    setXSpeed(aXSpeed);
  }else{
    setXSpeed(0);
  }
  
}

/*
double addSteps(int steps, double inDegrees, int altAz){
  double fSteps = double(steps);
  double degree = 0;
  if(altAz == ALT){
    
  }else if(altAz == AZ){
    degree = (moveOffsets[altAz]) * ((fSteps * rotationDegrees)/GEAR_RATIO)/stepperDivider;
  }
  
  double outDegrees = inDegrees + degree;
  outDegrees = over360Under0(outDegrees);
  
  return outDegrees;
}
*/

void setCurrentPositions(){
  #ifdef DEBUG_ENCODER
    Serial.println((AZ_ENCODER_INVERT * moveOffsets[1] * ((int32_t)azEncoder.getCount() * (AZ_ENCODER_DEGREE_PER_STEP / AZ_ENCODER_GEAR))));
    Serial.println((ALT_ENCODER_INVERT * moveOffsets[0] * ((int32_t)altEncoder.getCount() * (ALT_ENCODER_DEGREE_PER_STEP / ALT_ENCODER_GEAR))));
  #endif

  currentAz = over360Under0(currentAz + (AZ_ENCODER_INVERT * moveOffsets[1] * ((int32_t)azEncoder.getCount() * (AZ_ENCODER_DEGREE_PER_STEP / AZ_ENCODER_GEAR)))); //AzOffsets is from 1 index
  azEncoder.clearCount();

  currentAlt = over360Under0(currentAlt + (ALT_ENCODER_INVERT * moveOffsets[0] * ((int32_t)altEncoder.getCount() * (ALT_ENCODER_DEGREE_PER_STEP / ALT_ENCODER_GEAR)))); //AltOffsets is from 0 index
  altEncoder.clearCount();

  
}

void setStepperSpeed(){
  if(lastStepperSpeed != stepperSpeed){
    setXSpeed(0);
    setYSpeed(0);

    //Slow down and stop
    while(realYSpeed != ySpeed || realXSpeed != xSpeed){
      accelerateMove();
    }
    
    setCurrentPositions();
    int speedMenuIndex = 1;
    #ifdef HAS_FOCUSER
      speedMenuIndex++;
    #endif

    switch(stepperSpeed){
      // 1/32 - Fine
      case 0:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 63);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 32;
        mainMenuItems[speedMenuIndex] = "Speed: Fine";
        alignmentMenuItems[1] = "Speed: Fine";
        break;
      // 1/16 - Slow
      case 1:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 36);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 16;
        mainMenuItems[speedMenuIndex] = "Speed: Slow";
        alignmentMenuItems[1] = "Speed: Slow";
        break;
      // 1/8 - Med - HIGH, HIGH, LOW
      case 2:      
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 27);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 8;
        mainMenuItems[speedMenuIndex] = "Speed: Medium";
        alignmentMenuItems[1] = "Speed: Medium";
        break;
      // 1/4 - Fast - Low, High, Low
      case 3:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 18);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 4;
        mainMenuItems[speedMenuIndex] = "Speed: Fast";
        alignmentMenuItems[1] = "Speed: Fast";
        break;
    }

    if(screenMode == 1){
      mainMenu.show(display);
    }
    if(screenMode == 4){
      alignmentMenu.show(display);
    }
  }
  
  lastStepperSpeed = stepperSpeed;
  
}

int getRaHH(double inRa){
  return int(inRa);
}

int getRaMM(double inRa){
  float aRa = abs(inRa);
  int aRaHH = getRaHH(aRa);
  return int(((aRa)-aRaHH)*60);
}

int getRaSS(double inRa){
  double aRa = abs(inRa);
  int aRaHH = getRaHH(aRa);
  int aRaMM = getRaMM(aRa);
  return int(((((aRa)-aRaHH)*60)-aRaMM)*60);
}

double getDec(int deg, int mm, int sec){
  double outDec;
  if(deg < 0){
    outDec = double(deg) - (double(mm)/60) - (double(sec)/3600);
  }else{
    outDec = double(deg) + (double(mm)/60) + (double(sec)/3600);
  }

  return outDec;
   
}

int getDecDeg(double inDec){
  
  return int(abs(inDec));
}

int getDecMM(double inDec){
  double aDec = abs(inDec);
  int aDeg = getDecDeg(aDec);
  return abs(int((aDec - aDeg)*60));
}

int getDecSS(double inDec){
  double aDec = abs(inDec);
  int aDeg = getDecDeg(aDec);
  int aMM = getDecMM(aDec);

  return int((abs((aDec-aDeg)*60)- aMM)*60);
}

void setTrack(){
  setXSpeed(0);
  setYSpeed(0);
  preTrackStepperSpeed = stepperSpeed;
  moving = false;
  if(tracking && synced){
    isTracking = true;
    stepperSpeed = 0;
  }
}

void doTrack(){
  static long lastTrack = 0;

  if(millis() - lastTrack > 1000){

    setCurrentPositions();
    
    myAstro.setAltAz(currentAlt, currentAz);
    myAstro.doAltAz2RAdec();

    double aDec = myAstro.getDeclinationDec();
    double aRa = myAstro.getRAdec();

    adjustTime(-10);

    myAstro.setGMTdate(year(), month(), day());
    myAstro.setGMTtime(hour(), minute(), second());

    myAstro.setRAdec(myAstro.decimalDegrees(getRaHH(aRa), getRaMM(aRa), getRaSS(aRa)), myAstro.decimalDegrees(int(aDec), getDecMM(aDec), getDecSS(aDec)));
    myAstro.doRAdec2AltAz();
    double minAlt = myAstro.getAltitude();
    double minAz = myAstro.getAzimuth();

    adjustTime(20);

    myAstro.setGMTdate(year(), month(), day());
    myAstro.setGMTtime(hour(), minute(), second());

    myAstro.setRAdec(myAstro.decimalDegrees(getRaHH(aRa), getRaMM(aRa), getRaSS(aRa)), myAstro.decimalDegrees(int(aDec), getDecMM(aDec), getDecSS(aDec)));
    myAstro.doRAdec2AltAz();
    double maxAlt = myAstro.getAltitude();
    double maxAz = myAstro.getAzimuth();

    double azRate;

    if(minAz > 270 && maxAz < 90){
      azRate = (360 + maxAz - minAz)/20;
    }else if(minAz < 90 && maxAz > 270){
      azRate = (maxAz - 360 - minAz)/20;
    }else{
      azRate = (maxAz - minAz)/20;
    }
    double trackXSpeed = -1*(azRate * stepperDivider * GEAR_RATIO)/rotationDegrees;
    double trackYSpeed = -1*(((maxAlt - minAlt)/20) * stepperDivider * GEAR_RATIO)/rotationDegrees;
    
    setXSpeed(trackXSpeed);
    setYSpeed(trackYSpeed);

    adjustTime(-10);

    myAstro.setGMTdate(year(), month(), day());
    myAstro.setGMTtime(hour(), minute(), second());

    lastTrack = millis();
  }
}

void setXSpeed(float inSpeed){
  xSpeed = inSpeed;
}

void setYSpeed(float inSpeed){
  ySpeed = inSpeed;
}

void setFSpeed(float inSpeed){
  fSpeed = inSpeed;
}

int getDeadZone(int pinNumber){
  int read1 = analogRead(pinNumber);
  delay(20);
  int read2 = analogRead(pinNumber);
  delay(20);
  int read3 = analogRead(pinNumber);
  delay(20);
  int read4 = analogRead(pinNumber);
  delay(20);
  int read5 = analogRead(pinNumber);

  return (read1 + read2 + read3 + read4 + read5)/5;
}



void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    Serial.print (int(val));  //prints the int part
    Serial.print("."); // print the decimal point
    unsigned int frac;
    if(val >= 0)
      frac = (val - int(val)) * precision;
    else
       frac = (int(val)- val ) * precision;
    int frac1 = frac;
    while( frac1 /= 10 )
        precision /= 10;
    precision /= 10;
    while(  precision /= 10)
        Serial.print("0");

    Serial.println(frac,DEC) ;
}

void setOffsets(int altAz, double value){
  moveOffsets[altAz] = value;
  if(altAz == 0){
    preferences.putDouble("altOffset", value);
  }else if(altAz == 1){
    preferences.putDouble("azOffset", value);
  }  
}


double over360Under0(double value){
  //Adds +360 until the value is over 0
  while(value < 0){
    value += 360;
  }
  
  //subtract 360 until value is over 360
  while(value > 360){
    value -= 360;
  }

  return value;
}

double azDifference(double az1, double az2){
  double difference = 0;
  if(az1 > 270 && az2 < 90){
    difference = (360 + az2 - az1);
  }else if(az1 < 90 && az2 > 270){
    difference = (az2 - 360 - az1);
  }else{
    difference = az2 - az1;
  }
  return difference;
}


/* Hand Held - Ethernet Pinout
8 - Brown -       3V
7 - Brown/White - GND
6 - Green -       5V
5 - Blue/White -  D32 Button Pin
4 - Blue -        D35 X Joystick 
3 - Green/White - D34 Y Joystick
2 - Orange -      D22
1 - Orange/White -D21
*/
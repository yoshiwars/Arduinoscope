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

#include "lcdgfx.h"
#include "lcdgfx_gui.h"
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SiderealPlanets.h>
#include "BluetoothSerial.h"
#include <TimeLib.h>

/************************************************************************************************************************
Start Configurable Items
*************************************************************************************************************************/
#define BLUETOOTH_NAME "Tripod_Mount"           //Name of the Bluetooth Serial
const int GEAR_RATIO = 1;                       //where 1 is no gearing (ex. 300 Tooth gear / 20 tooth gear = 15)
const double SINGLE_STEP_DEGREE =  360.0 / 200.0;    // the motor has 200 regular steps for 360 degrees (360 divided by 200 = 1.8)
const double MOTOR_GEAR_BOX = (26.0 + (103.0/121.0)); //where 1 is no gearing (26 + (103/121)) planetary gearbox version
const float MAX_MOTOR_SPEED = 250;              //250 is probably good without GEAR_RATIO >1, 3000 is good 

//Define Motor setup
const int Y_DIRECTION_PIN = 13;                 //ALT Driver Dir
const int Y_STEP_PIN = 12;                      //ALT Driver Step
const int X_DIRECTION_PIN = 14;                 //AZ Driver Dir
const int X_STEP_PIN = 27;                      //AZ Driver Step
const int MOTOR_INTERFACE_TYPE = 1;             //AccelStepper Motor Type 1
//#define MIN_PULSE_WIDTH 50                      //use to adjust stepper pulses, comment out for default

//Define Remote setup
const int Y_JOYSTICK_PIN = 34;                  //Up/Down Pin on the Remote
const int Y_INVERT = -1;                        //1 is normal, -1 inverted
const int X_JOYSTICK_PIN = 35;                  //Left/Right Pin on the remote
const int X_INVERT = -1;                        //1 is normal, -1 inverted

const int BUTTON_PIN = 32;                      //button on Remotes
const int ANALOG_READ_RESOLUTION = 4095;        //ESP32 has this resolution, other chips may vary

//Comment Out HAS_FOCUSER for no Focuser
//#define HAS_FOCUSER

const int FOCUS_DIRECTION_PIN = 2;
const int FOCUS_STEP_PIN = 2;
const int MAX_FOCUS_SPEED = 1024;
const int FOCUS_MOTOR_MODE = 1;                 //1 is with driver DRV8825

//Shift Register for motor speeds
const int LATCH_PIN = 18;                        //Pin connected to ST_CP of 74HC595
const int CLOCK_PIN = 19;                       //Pin connected to SH_CP of 74HC595
const int DATA_PIN = 5;                        //Pin connected to DS of 74HC595

//uncomment to get debugging
//#define DEBUG
//#define DEBUG_STEPS
//#define DEBUG_X_JOYSTICK
//#define DEBUG_Y_JOYSTICK 
/************************************************************************************************************************
End Configurable Items
*************************************************************************************************************************/

/************************************************************************************************************************
Global Variables - These should not need to change based on devices
*************************************************************************************************************************/
BluetoothSerial btComm;   //Bluetooth Setup
SiderealPlanets myAstro;  //Used for calculations

//button states
int buttonState = LOW;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

//AltAZ motor rotation min
const double rotationDegrees = (SINGLE_STEP_DEGREE / MOTOR_GEAR_BOX); //degrees / single step per rev / gearbox

//Motor Starts
AccelStepper yStepper = AccelStepper(MOTOR_INTERFACE_TYPE,Y_STEP_PIN,Y_DIRECTION_PIN);
AccelStepper xStepper = AccelStepper(MOTOR_INTERFACE_TYPE,X_STEP_PIN,X_DIRECTION_PIN);
#ifdef HAS_FOCUSER
AccelStepper focuser = AccelStepper(FOCUS_INTERFACE_TYPE,FOCUS_STEP_PIN,FOCUS_DIRECTION_PIN);
#endif


float xSpeed = 0;
float ySpeed = 0;
float fSpeed = 0;
float realXSpeed = 0;
float realYSpeed = 0;
float realFSpeed = 0;
//float lastXSpeed = 0;
//float lastYSpeed = 0;

int stepperSpeed = 3;
int lastStepperSpeed = 0;
int preTrackStepperSpeed = 3;
int stepperDivider = 1;

//Menu Control
int screenMode = 0;
//LcdGfxMenu gpsMenu;

const char *movementMenuItems[] = 
{
  "Move:",
  #ifdef HAS_FOCUSER
  "Focus:",
  #endif
  "Speed:",
  "Tracking",
  "Info"
};

LcdGfxMenu movementMenu(movementMenuItems, sizeof(movementMenuItems) / sizeof(char *) );

//JoyStick
int lastYState = 0;
int yState = 0;
unsigned long lastYDebounceTime = 0;
int yDeadZone = 0;
int xDeadZone = 0;
int maxY = ANALOG_READ_RESOLUTION;
int maxX = ANALOG_READ_RESOLUTION;

int lastXState = 0;
int xState = 0;
unsigned long lastXDebounceTime = 0;

bool moving = false;
bool yMoving = false;
bool yLastMoving = false;
bool xMoving = false;
bool xLastMoving = false;

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

float flat, flon;
float faltitude;

int sats;

float gpsQueryTime;

//hold the received RA, Dec
int raHH, raMM, raSS;
int decD, decMM, decSS;

//Sync coords
double currentAlt = 0;
double currentAz = 0;

double alt=0;
double az=0;

//Target Coords
int targetRaHH, targetRaMM, targetRaSS;
int targetDecD, targetDecMM, targetDecSS;
double targetAlt;
double targetAz;
bool slewComplete = true;
bool tracking = true;
bool isTracking = false;
long slewScreenTimeOut = 2000;
long slewScreenUpdate = 0;

DisplaySSD1306_128x64_I2C display(-1); // or (-1,{busId, addr, scl, sda, frequency}). This line is suitable for most platforms by default

void setup() {
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
  btComm.begin(BLUETOOTH_NAME);   //Bluetooth
  Serial2.begin(9600);  //start GPS
  
  #ifdef DEBUG
    Serial.println("Start Up");
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
  
  display.printFixed(0,0,"Please Stand By",STYLE_NORMAL);
  display.printFixed(0,8,"Calibrate JoySticks",STYLE_NORMAL);

  yDeadZone = getDeadZone(Y_JOYSTICK_PIN);
  xDeadZone = getDeadZone(X_JOYSTICK_PIN);
  maxY = (ANALOG_READ_RESOLUTION + yDeadZone)/2;
  maxX = (ANALOG_READ_RESOLUTION + xDeadZone)/2;

  #ifdef DEBUG
    Serial.println("Joysticks Calibrated");
  #endif
 
  display.printFixed(0,16,"Set Motors",STYLE_NORMAL);
  //set motor max
  yStepper.setMaxSpeed(MAX_MOTOR_SPEED);
  yStepper.setCurrentPosition(0);
  xStepper.setMaxSpeed(MAX_MOTOR_SPEED);
  xStepper.setCurrentPosition(0);

  #ifdef HAS_FOCUSER
    focuser.setMaxSpeed(MAX_FOCUS_SPEED);
  #endif

  //set up and display movement menu
  screenMode = 0;
  showInfo();  

  #ifdef DEBUG
    Serial.println("Ready");
  #endif
}

void loop() {
  
  switch (screenMode){
    case 0:
      //GPS Mode
        infoScreensControl();
        if(millis() - lastGPSCheck > 10000){
          showInfo();
          lastGPSCheck = millis();
        }
      break;
    case 1:
      //Main Screen
      movementButtonControl();
      if(!moving && !focusing){
        menuControl();
        
      }
      
      if(moving){
        //move if in move mode
        readJoystickAndMove();  
      }

      if(focusing){
        //Focusing Mode
        readJoystickAndFocus();
        
      }
      break;
    case 2:
      //Slew Mode
      slewMode();
      infoScreensControl();
      break;
    case 3:
      //manual input mode
      manualInputMode();
  }

  if(isTracking){
    doTrack();
  }
  
/**/
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
/**/
  /*
  if(lastXSpeed != xSpeed){
    xSpeed = lastXSpeed;
  }

  if(lastYSpeed != ySpeed){
    ySpeed = lastYSpeed;
  }
    */
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

//Not sure what that is going to be
void manualInputMode(){
  
}

void communication(Stream &aSerial)
{
  byte inByte = aSerial.read();
  static unsigned int input_pos = 0;
  
  char txRA[10];
  char txDEC[11];
    
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
    
    if(input[1] == 'G' && input[2] == 'V' && input[3] == 'P'){
      //Get Telescope Name
      aSerial.print("Arduinoscope#");
    }

    if(input[1] == 'G' && input[2] == 'V' && input[3] == 'N'){
      //Get Firmware Number
      
      aSerial.print("0.1#");
    }

    if(input[1] == 'G' && input[2] == 'V' && input[3] == 'D'){
      aSerial.print("Sep 10 2022#");
    }

    if(input[1] == 'G' && input[2] == 'C'){
      //Get Telescope Local Date
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
      
      if(myAstro.getLT() > myAstro.getGMT()){
        adjustTime(-1*((24 + myAstro.getGMT()) - myAstro.getLT())*3600);
      }
      char charDate[12];
      
      sprintf(charDate, "%02d/%02d/%02d#", month(), day(), (year()-2000));
            
      aSerial.print(charDate);
    }

    if(input[1] == 'G' && input[2] == 'L'){
      char charDate[12];
      sprintf(charDate, "%02d:%02d:%02d#", getRaHH(myAstro.getLT()), getRaMM(myAstro.getLT()), getRaSS(myAstro.getLT()));
      aSerial.print(charDate);
    }

    if(input[1] == 'G' && input[2] == 'G'){
      int calcOffset = (myAstro.getLT() < myAstro.getGMT()? myAstro.getGMT() : myAstro.getGMT() + 24) - myAstro.getLT();
      
      aSerial.print(calcOffset);
      aSerial.print("#");
    }

    if(input[1] == 'G' && input[2] == 'V' && input[3] == 'T'){
      //Get Telescope Firmware Time
      char charDate[12];
      sprintf(charDate, "%02d:%02d:%02d#", 23, 13, 00);
      
      aSerial.print(charDate);
    }

    if(input[1] == 'G' && input[2] == 'g'){
      float lon = gps.location.lng();
      
      char charLon[20];

      int lonDeg = (int) lon;
      float lonMinutesRemainder = abs(lon - lonDeg) * 60;
      int lonMin = (int) lonMinutesRemainder;

      lonDeg *= -1;

      sprintf(charLon, "%03d*%02d#", lonDeg, lonMin);
      aSerial.print(charLon);
    }

    if(input[1] == 'G' && input[2] == 't'){
      float lat = gps.location.lat();
      
      char charLat[20];

      int latDeg = (int) lat;
      float latMinutesRemainder = abs(lat-latDeg) * 60;
      int latMin = (int)latMinutesRemainder;

      sprintf(charLat, "%02d*%02d#", latDeg, latMin);
      aSerial.print(charLat);
      
    }

    if(input[1] == 'G' && input[2] == 'W'){
      //Get Track State
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
    }

    

    if(input[1] == 'D'){
      if(slewComplete){
        aSerial.print("#");
      }else{
        aSerial.print("-#");
      }
    }

    //report AZ
    if (input[1] == 'G' && input[2] == 'R') {
      float totalAz = addSteps(-1*xStepper.currentPosition(), currentAz);
      float totalAlt = addSteps(-1*yStepper.currentPosition(), currentAlt);
      
      myAstro.setAltAz(totalAlt, totalAz);
      myAstro.doAltAz2RAdec();

      float aRa = myAstro.getRAdec();
      
      sprintf(txRA, "%02d:%02d:%02d#", getRaHH(aRa), getRaMM(aRa), getRaSS(aRa));
      aSerial.print(txRA);
    }
  
    if (input[1] == 'G' && input[2] == 'D') {

      float totalAz  = addSteps(-1*xStepper.currentPosition(), currentAz);
      float totalAlt = addSteps(-1*yStepper.currentPosition(), currentAlt);

      myAstro.setAltAz(totalAlt, totalAz);
      myAstro.doAltAz2RAdec();
      
      float aDec = myAstro.getDeclinationDec();
  
      char decCase;
  
      if(aDec < 0){
        decCase = 45;
      }else{
        decCase = 43;
      }
      
      sprintf(txDEC, "%c%02d%c%02d:%02d#", decCase, getDecDeg(aDec), 223, getDecMM(aDec),getDecSS(aDec));
      aSerial.print(txDEC);
    }

    //Set Hour Offset
    if(input[1] == 'S' && input[2] == 'G'){
      inHourOffset = strInput.substring(3,7).toInt();
      aSerial.print(1);
    }

    if(input[1] == 'S' && input[2] == 'L'){
      inHour = strInput.substring(3,5).toInt();
      inMinute = strInput.substring(6,8).toInt();
      inSecond = strInput.substring(9).toInt();
      
      aSerial.print(1);
    }

    if(input[1] == 'S' && input[2] == 'C'){
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
    }
  
    if(input[1] == 'S' && input[2] == 'r'){
      
      raHH = strInput.substring(3,5).toInt();
      raMM = strInput.substring(6,8).toInt();
      raSS = strInput.substring(9).toInt();
            
      aSerial.print(1);
            
    }
  
    if(input[1] == 'S' && input[2] == 'd' ){
      
      if(input[3] == '-'){
        decD = strInput.substring(3,6).toInt();
      }else{
        decD = strInput.substring(4,6).toInt();
      }
      
      decMM = strInput.substring(7,9).toInt();
      decSS = strInput.substring(10,12).toInt();
      
      aSerial.print(1);

  }

    //Sync
    if(input[1] == 'C' && input[2] == 'M'){
      
      
      //lastXSpeed = xSpeed;
      //lastYSpeed = ySpeed;
      setXSpeed(0);
      setYSpeed(0);
  
      //Slow down and stop
      while(realYSpeed != ySpeed || realXSpeed != xSpeed){
        accelerateMove();
      }

      setCurrentPositions();
            
      
      myAstro.setRAdec(myAstro.decimalDegrees(raHH, raMM, raSS), myAstro.decimalDegrees(decD, decMM, decSS));
      
      myAstro.doRAdec2AltAz();
      currentAlt = myAstro.getAltitude();
      currentAz = myAstro.getAzimuth();
      
      synced = true;
      
      aSerial.print(1);      
    }

    //Movement
    if(input[1] == 'M'){
      isTracking = false;
      if(input[2] == 'e'){
        setXSpeed(50);
      }
      if(input[2] == 'w'){
        setXSpeed(-50);
        
      }
      if(input[2] == 'n'){
        setYSpeed(-50);
      }
      if(input[2] == 's'){
        setYSpeed(50);
      }
      
    }


    //stop moving
    if(input[1] == 'Q'){
      setTrack();
      if(input[2] == 'e' || input[2] == 'w'){
        
      }else if(input[2] == 'n' || input[2] == 's'){
        
      }else{
        
      }
    }
    
    //modify Slew Rate
    if(input[1] == 'R'){
      if(input[2] == 'C'){
          stepperSpeed = 1;
      }
      if(input[2] == 'G'){
          stepperSpeed = 0;
      }
      if(input[2] == 'M'){
          stepperSpeed = 2;
      }
      if(input[2] == 'S'){
          stepperSpeed = 3;
      }
    }

    
    //Slew!
    if(input[1] == 'M' && input[2] == 'S'){
      moving = false;
      
      targetRaHH = raHH;
      targetRaMM = raMM;
      targetRaSS = raSS;

      targetDecD = decD;
      targetDecMM = decMM;
      targetDecSS = decSS;
      
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
      
      aSerial.print(1);
    }

    for(int i = 0; i < numChars; i++){
      input[i] = '\0';
    }
    newData = false;
  }
}



void slewMode(){

  myAstro.setRAdec(myAstro.decimalDegrees(targetRaHH, targetRaMM, targetRaSS), myAstro.decimalDegrees(targetDecD, targetDecMM, targetDecSS));
  myAstro.doRAdec2AltAz();
  targetAlt = myAstro.getAltitude();
  targetAz = myAstro.getAzimuth();
  
  float totalAz  = addSteps(-1*xStepper.currentPosition(), currentAz);
  float totalAlt = addSteps(-1*yStepper.currentPosition(), currentAlt);
  
  double remainingAlt = targetAlt - totalAlt;
  double remainingAz;

  if(totalAz > 270 && targetAz < 90){
    remainingAz = (360 + targetAz - totalAz);
  }else if(totalAz < 90 && targetAz > 270){
    remainingAz = (targetAz - 360 - totalAz);
  }else{
    remainingAz = targetAz - totalAz;
  }
  
  double minStep = ((rotationDegrees)/GEAR_RATIO)/32;
  bool moveAz = false;
  bool moveAlt = false;
  float moveSpeed = 0;
  
  if(!slewComplete){

      if(
          abs(remainingAz) > (stepperSpeed) &&
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
          display.printFixed(0, 0, "Press the button to return to main menu.", STYLE_NORMAL);
          slewScreenTimeOut = 60000;
        }else{
          
          int intAlt = (int) totalAlt;
          int decAlt = (100 * abs(totalAlt - intAlt));
          int intAz = (int) totalAz;
          int decAz = (100 * abs(totalAz - intAz));
      
          int intTargetAlt = (int) targetAlt;
          int decTargetAlt = (100 * abs(targetAlt - intTargetAlt));
          int intTargetAz = (int) targetAz;
          int decTargetAz = (100 * abs(targetAz - intTargetAz));
      
          int intRemainingAlt = (int) remainingAlt;
          int decRemainingAlt = (100 * abs(remainingAlt) - abs(intRemainingAlt));
          int intRemainingAz = (int) remainingAz;
          int decRemainingAz = (100 * abs(remainingAz) - abs(intRemainingAz));
          
          char charCurrentAlt[20];
          sprintf(charCurrentAlt, "%d.%d", intAlt, decAlt);
          display.printFixed(72, 0, "       ", STYLE_NORMAL);
          display.printFixed(72, 0, charCurrentAlt, STYLE_NORMAL);
          
          char charCurrentAz[20];
          sprintf(charCurrentAz, "%d.%d", intAz, decAz);
          display.printFixed(72,  8, "       ", STYLE_NORMAL);
          display.printFixed(72,  8, charCurrentAz, STYLE_NORMAL);
      
          char charTargetAlt[20];
          sprintf(charTargetAlt, "%d.%d", intTargetAlt, decTargetAlt);
          display.printFixed(72,  16, "       ", STYLE_NORMAL);
          display.printFixed(72,  16, charTargetAlt, STYLE_NORMAL);
      
          char charTargetAz[20];
          sprintf(charTargetAz, "%d.%d", intTargetAz, decTargetAz);
          display.printFixed(72,  24, "       ", STYLE_NORMAL);
          display.printFixed(72,  24, charTargetAz, STYLE_NORMAL);

          char charRemainingAlt[20];
          sprintf(charRemainingAlt, "%d.%d", intRemainingAlt, decRemainingAlt);
          display.printFixed(72,  32, "       ", STYLE_NORMAL);
          display.printFixed(72,  32, charRemainingAlt, STYLE_NORMAL);

          char charRemainingAz[20];
          sprintf(charRemainingAz, "%d.%d", intRemainingAz, decRemainingAz);
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
    #ifdef DEBUG
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

    #ifdef DEBUG
      Serial.print("Alt: ");
      Serial.println(faltitude,6);
    #endif
  }
   
  
  sats = gps.satellites.value();

  #ifdef DEBUG
      Serial.print("Sats: ");
      Serial.println(sats);
    #endif
  
  if(gps.date.isValid()){
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    myAstro.setGMTdate(gps.date.year(), gps.date.month(), gps.date.day());
    myAstro.setGMTtime(gps.time.hour(), gps.time.minute(), gps.time.second());
    myAstro.useAutoDST();
    
    #ifdef DEBUG
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



void showInfo(){
  
  display.clear();
  char lat[20], lon[20];

  int latDeg = (int) flat;
  float latMinutesRemainder = abs(flat-latDeg) * 60;
  int latMin = (int)latMinutesRemainder;
  float latSec = (latMinutesRemainder - latMin) * 60;
  
  int lonDeg = (int) flon;
  float lonMinutesRemainder = abs(flon - lonDeg) * 60;
  int lonMin = (int) lonMinutesRemainder;
  float lonSec = (lonMinutesRemainder - lonMin) * 60;

  //Lat
  sprintf(lat, "LAT: %02d,%02d", latDeg, latMin);
  
  String latSecVal = lat;
  latSecVal.concat("'");
  latSecVal.concat(String(latSec));
  char charLat[20];
  sprintf(charLat, latSecVal.c_str());
  
  display.printFixed(0,  0, charLat, STYLE_NORMAL);
  
  //Lon
  sprintf(lon, "LON: %02d,%02d", lonDeg, lonMin);
  String lonSecVal = lon;
  lonSecVal.concat("'");
  lonSecVal.concat(String(lonSec));

  char charPrint[20];
  sprintf(charPrint, lonSecVal.c_str());
  display.printFixed(0,  8, charPrint, STYLE_NORMAL);

  //Altitude
  sprintf(charPrint,"Alt: %.02fm",faltitude);
  display.printFixed(0,  16, charPrint, STYLE_NORMAL);
  

  //Satellites
  sprintf(charPrint, "Sats: %d", sats);
  display.printFixed(0,24, charPrint, STYLE_NORMAL);

  //Local Time
  sprintf(charPrint, "UTC Time: %02d:%02d", hour(), minute());
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
        
        returnToMainMenu();
      }
      
    }
  }
  
  lastButtonState = reading;
}

void returnToMainMenu(){
  display.clear();
  if(!slewComplete){
    slewComplete = true;
    setXSpeed(0);
    setYSpeed(0);  
  }
  
  screenMode = 1;
  movementMenu.show( display );
  display.printFixed(55,  8, "Disabled", STYLE_NORMAL);
  #ifdef HAS_FOCUSER
    display.printFixed(55,  16, "Disabled", STYLE_NORMAL);
  #endif
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
        int selected = movementMenu.selection();
        #ifndef HAS_FOCUSER
          if(selected > 0){
            selected += 1;
          }
        #endif

        switch(selected){
          case 0:
            if(moving){
              setTrack();
            }else{
              display.printFixed(55,  8, "Enabled ", STYLE_NORMAL);
              if(isTracking){
                stepperSpeed = preTrackStepperSpeed;
              }
              moving = true;
              isTracking = false;
            }
            break;
          
          case 1:
            if(focusing){
              focusing = false;
              display.printFixed(55,  24, "Disabled", STYLE_NORMAL);
            }else{
              display.printFixed(55,  16, "Enabled ", STYLE_NORMAL);
              focusing = true;
            }
            break;
          
          
          case 3:
            //Set Tracking
            if(tracking){
              display.printFixed(60,  32, "Disabled", STYLE_NORMAL);
              tracking = false;
            }else{
              display.printFixed(60,  32, "Enabled ", STYLE_NORMAL);  
              tracking = true;
            }
            break;
          case 4:
            //GPS Mode
            screenMode = 0;
            showInfo();
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
  
  #ifdef DEBUG_X_JOYSTICK
    Serial.print("X:");
    Serial.println(inX);
  #endif

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
  }

  if((millis() - lastYDebounceTime) > debounceDelay){

    if(y != yState){
      yState = y;
      
      if(y == -1){
        movementMenu.down();
        movementMenu.show(display);
      }else if(y == 1){
        movementMenu.up();
        movementMenu.show(display);
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
  }

  if((millis() - lastXDebounceTime) > debounceDelay){

    if(x != xState){
      xState = x;
     
      //Setting Speed
      #ifdef HAS_FOCUSER
        unsigned int speedIndex = 2;
      #endif
      #ifndef HAS_FOCUSER
        unsigned int speedIndex = 1;
      #endif

      if(speedIndex == movementMenu.selection()){
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
    }
  }
  lastXState = x;
}

void readJoystickAndFocus(){
  
  int x = getXStick();

  int aXSpeed = 0;
  
  if(x > xDeadZone + 50){
    //Top Speed: 1000?
    aXSpeed = map(x, (xDeadZone + 50), (xDeadZone * 2), 0, MAX_FOCUS_SPEED * X_INVERT); //TODO Fix for xMax
    xMoving = true;    
  }else if(x < xDeadZone - 50){
    //Top Speed: 1000?
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

  xLastMoving = xMoving;
}


void readJoystickAndMove(){
  int y = getYStick();

  int aYSpeed = 0;

  if(y > yDeadZone + 50){
    //aYSpeed = map(y, (yDeadZone + 50), 512, 0, MAX_MOTOR_SPEED * ((stepperSpeed + 1) / 4));
    aYSpeed = map(y, (yDeadZone + 50), maxY, 0, MAX_MOTOR_SPEED * Y_INVERT);
    yMoving = true;
  }else if(y < yDeadZone - 50){
    //aYSpeed = map(y, (yDeadZone - 50),-512,0, -MAX_MOTOR_SPEED * ((stepperSpeed + 1) / 4));
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

  yLastMoving = yMoving;
  
  int x = getXStick();

  int aXSpeed = 0;
  
  if(x > xDeadZone + 50){
    //aXSpeed = map(x, (xDeadZone + 50), 512, 0, MAX_MOTOR_SPEED * ((stepperSpeed + 1) / 4));
    aXSpeed = map(x, (xDeadZone + 50), maxX, 0, MAX_MOTOR_SPEED * X_INVERT);
    xMoving = true;    
  }else if(x < xDeadZone - 50){
    //aXSpeed = map(x, (xDeadZone - 50),-512,0, -MAX_MOTOR_SPEED * ((stepperSpeed + 1) / 4));
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

  xLastMoving = xMoving;
  
}

double addSteps(int steps, double inDegrees){
  double fSteps = double(steps);
  //fSteps = -1 * fSteps;
  double degree = ((fSteps * rotationDegrees)/GEAR_RATIO)/stepperDivider;
  
  double outDegrees = inDegrees + degree;

  while(outDegrees < 0){
    outDegrees += 360;
  }

  while(outDegrees > 360){
    outDegrees -= 360;
  }
  return outDegrees;
}

void setCurrentPositions(){
  currentAz = addSteps(-1*xStepper.currentPosition(), currentAz);
  xStepper.setCurrentPosition(0);
  
  currentAlt = addSteps(-1*yStepper.currentPosition(), currentAlt);
  yStepper.setCurrentPosition(0);
}

void setStepperSpeed(){
  if(lastStepperSpeed != stepperSpeed){
    //lastXSpeed = xSpeed;
    //lastYSpeed = ySpeed;
    setXSpeed(0);
    setYSpeed(0);

    //Slow down and stop
    while(realYSpeed != ySpeed || realXSpeed != xSpeed){
      accelerateMove();
    }
    
    setCurrentPositions();

    switch(stepperSpeed){
      // 1/32 - Fine
      case 0:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 63);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 32;
        if(screenMode == 1){
          display.printFixed(50,  16, "Fine", STYLE_NORMAL);  
        }
        break;
      // 1/16 - Slow
      case 1:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 36);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 16;
        if(screenMode == 1){
          display.printFixed(50,  16, "Slow", STYLE_NORMAL);  
        }       
        break;
      // 1/8 - Med (HIGH, HIGH, LOW)
      case 2:      
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 27);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 8;
        if(screenMode == 1){
          display.printFixed(50,  16, "Med ", STYLE_NORMAL);
        }
        break;
      // 1/4 - Fast (Low, High, Low)
      case 3:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 18);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 4;
        if(screenMode == 1){
          display.printFixed(50,  16, "Fast", STYLE_NORMAL);  
        } 
        break;
      // 1/2 - Not in Use
      case 4:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 9);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 2;
        break;
      //Full Speed - Not in Use
      case 5:
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);
        digitalWrite(LATCH_PIN, HIGH);
        stepperDivider = 1;
        break;
    }
  }
  
  lastStepperSpeed = stepperSpeed;
  
}

int getRaHH(float inRa){
  return int(inRa);
}

int getRaMM(float inRa){
  float aRa = abs(inRa);
  int aRaHH = getRaHH(aRa);
  return int(((aRa)-aRaHH)*60);
}

int getRaSS(float inRa){
  float aRa = abs(inRa);
  int aRaHH = getRaHH(aRa);
  int aRaMM = getRaMM(aRa);
  return int(((((aRa)-aRaHH)*60)-aRaMM)*60);
}

float getDec(int deg, int mm, int sec){
  float outDec;
  if(deg < 0){
    outDec = float(deg) - (float(mm)/60) - (float(sec)/3600);
  }else{
    outDec = float(deg) + (float(mm)/60) + (float(sec)/3600);
  }

  return outDec;
   
}

int getDecDeg(float inDec){
  
  return int(abs(inDec));
}

int getDecMM(float inDec){
  float aDec = abs(inDec);
  int aDeg = getDecDeg(aDec);
  return abs(int((aDec - aDeg)*60));
}

int getDecSS(float inDec){
  float aDec = abs(inDec);
  int aDeg = getDecDeg(aDec);
  int aMM = getDecMM(aDec);

  return int((abs((aDec-aDeg)*60)- aMM)*60);
}

void setTrack(){
  if(screenMode == 1){
    display.printFixed(55,  8, "Disabled", STYLE_NORMAL);  
  }
  
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

    float totalAz = addSteps(-1*xStepper.currentPosition(), currentAz);
    float totalAlt = addSteps(-1*yStepper.currentPosition(), currentAlt);
    
    myAstro.setAltAz(totalAlt, totalAz);
    myAstro.doAltAz2RAdec();

    float aDec = myAstro.getDeclinationDec();
    float aRa = myAstro.getRAdec();

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
  //lastXSpeed = inSpeed;
}

void setYSpeed(float inSpeed){
  ySpeed = inSpeed;
  //lastYSpeed = inSpeed;
}

void setFSpeed(float inSpeed){
  fSpeed = inSpeed;
  //lastYSpeed = inSpeed;
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
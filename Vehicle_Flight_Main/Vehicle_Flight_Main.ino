//====================================================================================================
//                                     Vehicle_Flight_Code                                             
//====================================================================================================

#include <Adafruit_GPS.h>                                             // used by: GPS
#include <math.h>                                                     // used by: GPS
#include <XBee.h>                                                     // used to receive XBee communication from transponder
#include <Wire.h>                                                     // used by BMP180 : Barometer + BN0055
#include <Adafruit_Sensor.h>                                          // used by BMP180 : Barometer + BN0055   
#include <Adafruit_BMP085_U.h>                                        // used by BMP180 : Barometer
#include <Adafruit_BNO055.h>                                          // used by BNO055
#include <utility/imumaths.h>                                         // used by BNO055
#include <PID_v1.h>                                                   // used for control loops.
#include <Servo.h>                                                    // Used for PWM outputs to Pixhawk Controller
#include <StopWatch.h>                                                // Used to adjust sampling time / detect XBee communication drop

//====================================================================================================
//                                       Creating Objects                                           
//====================================================================================================
                                                                           
#define usbSerial Serial                                              // To Screen
#define gpsSerial Serial1                                             // From GPS  
#define xbeeSerial Serial2                                            // From XBee

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);         // BMP180 Barometer.
Adafruit_BNO055 bno = Adafruit_BNO055();                              // BN0055 Accelerometer.
Adafruit_GPS GPS(&gpsSerial);                                         // GPS Hardware.

#define GPSECHO false

#define filterSamples 10
int sensSmoothArray [filterSamples];                                  // Array for holding raw barometric senor values

//====================================================================================================
//                               General_Global_Variables/Declarations                                           
//====================================================================================================

// GPS Variables.
SIGNAL(TIMER0_COMPA_vect)                                         // Interrupt is called once a millisecond, looks for any new GPS data, and stores it.
{
  GPS.read();
}
float currentLat;                                                 // Current Position
float currentLong;

String latConversion;
String longConversion;
float targetLat;                                                  // Target Position Processing
float targetLong;
int targetHeading;

boolean usingInterrupt = false;
void useInterrupt(boolean);                                       // Function prototype keeps arduino happy?

// XBee Parsing Variables.
int comma = 0;                                                    // Position of the comma delimin
int newLine = 0;                                                  // Position of the new line character
String inputString = "";                                          // XBee string data
boolean stringComplete = false;                                   // Built a new sequence of coordinates now parce it
boolean talkToXbee = true;

// Vehicle Flight Data.
double distanceToTarget;                                          // Distance in meters between target and vehicle (GPS Data)
float groundPressure;                                             // Pressure at ground level to use to compare for Altitude
double AltitudeTotal = 0;                                         // Used to average the current altitude reading
float startLat;
float startLong;
double offSet = 0;                                                // Used to try and smooth out the altitude reading from the barometer
bool flag = true;

// PWM Servo Outputs.
Servo throttle;
Servo pitch;
Servo roll;
Servo yaw;
        
// Timer Variables.
unsigned long start = millis();                                   // Used to print results
uint32_t usbPrintTimer, xbeePrintTimer;

// Blinking Update LED
int redLED = 4;                                                   // LED Status updates
int greenLED = 5;
bool checkPlease = true;

// Stop Watch for Dropped XBee Communication 
StopWatch xbeeSW;
StopWatch bmpSW;
StopWatch yawPIDTimer;
int bmpCount =0;

//====================================================================================================
//                                     PID_Variables                                          
//====================================================================================================

// Altitude 
double smoothAltitude;
double setAltitude = 1;                                           // (meters) Around 150 feet?
double currentAltitude = 0;                                       // Comes from the barometer data
double altError = 0;                                              // PID has to use type: double?
double throttleOut = 900;                                         // Pwm Output...Convert to ppm using board.
double AltAggKp=300, AltAggKi= 0, AltAggKd = 0;                   // Originally 500,10,0
double AltConsKp=100, AltConsKi=0, AltConsKd = 0;                  // Not sure how to decide startting values.
PID altPID(&currentAltitude, &throttleOut, &setAltitude,          // Specify the links and initial tuning parameters
            AltConsKp, AltConsKi, AltConsKd, DIRECT);
  
// Yaw
double yawOut;
double currentHeading;
double mappedHeading;
double desiredHeading = 0;                                      // 0 Degree N at all times.
double yawConsKp = 1, yawConsKi= 0, yawConsKd= 0;               // Not sure how to decide startting values.
PID yawPID(&mappedHeading, &yawOut, &desiredHeading,
           yawConsKp,yawConsKi,yawConsKd,DIRECT);
           
// Roll
double rollOut = 1500;
double currentRoll;
double desiredRoll;                                               // Calculated as a function of Ed and Eh.
double rollConsKp = 30, rollConsKi = 10, rollConsKd = 0;
PID rollPID(&currentRoll,&rollOut,&desiredRoll,
            rollConsKp,rollConsKi,rollConsKd,REVERSE);
            
// Pitch                                                          // Calculated as a function of Ed and Eh
double pitchOut = 1500;
double currentPitch;
double desiredPitch;
double pitchConsKp = 30, pitchConsKi = 10, pitchConsKd = 0;
PID pitchPID(&currentPitch,&pitchOut,&desiredPitch,
            pitchConsKp,pitchConsKi,pitchConsKd,REVERSE);

// Auto Land
double currentGravity;
double setGravity = 8;
double landingThrottle;
double GravConsKp=.25, GravConsKi=0, GravConsKd = 0;                 
PID autoLandPID(&currentGravity, &landingThrottle, &setGravity,          
          GravConsKp, GravConsKi, GravConsKd, DIRECT);

//====================================================================================================
//                                             SETUP
//====================================================================================================

void setup(void)
{
  // Start GPS and set desired configuration
  GPS.begin(9600);                                              // 9600 NMEA default speed
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);                 // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                    // 1 Hz update rate suggested...
  GPS.sendCommand(PGCMD_NOANTENNA);                             // turn off antenna status info
  useInterrupt(true);                                           // use interrupt to constantly pull data from GPS
  delay(1000);
  
  // XBEE 
  xbeeSerial.begin(57600);
  inputString.reserve(30);                                      // reserve 30 bytes for the inputString:

  // BMP180 Barometer.
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    usbSerial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  groundPressure = getGroundPressure();                         // Sets the ground level pressure.

  // BN0055 Accelerometer.
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    usbSerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // USB Serial Port
  usbSerial.begin(115200);
  usbSerial.println("Adafruit GPS library basic test!");
   
  // altPID                                                   // Throttle connection to pin 13
  bmpSW.start();
  throttle.attach(13);
  processBaro(groundPressure);
  altPID.SetMode(AUTOMATIC);
  altPID.SetOutputLimits(900,1200);                           // Original value was from [1100,1600]

  // yawPID                                                   // Yaw control to pin 12
  yaw.attach(12);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-500,500);
  yawPID.SetTunings(yawConsKp,yawConsKi,yawConsKd);

  // rollPID                                                  // Roll control to pin 11
  roll.attach(11);
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(1250,1750);

  // pitchPID                                                 // Pitch control to pins 10
  pitch.attach(10);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(1250,1750);

  // Auto Land PID
  autoLandPID.SetMode(AUTOMATIC);
  autoLandPID.SetOutputLimits(1000,2000);

  // Safety                                                   // Prevent while loop control if bad data or sensors
  pinMode(redLED,OUTPUT);
  pinMode(greenLED,OUTPUT);
  digitalWrite(redLED,LOW);
  digitalWrite(greenLED,LOW);
  //checkGPSfix();                                              // Loop until it has a fix.
 // getStartingLocation();                                      // Marks the starting location 
  //checkXbeeFix();         // NOT WORKING!!!!!                                    // Loop until we have parsed a non-zero value from transponder.
  //altCheck();                                                 // Loop until we have good altimeter calibration.
  digitalWrite(greenLED,HIGH);
}

//====================================================================================================
//                                             Main LOOP
//====================================================================================================

void loop(void)
{
  // Gather Data 
  processGPS();                                                // Break down data for current position 
  checkXbeeComs(checkPlease);
  if(talkToXbee)
  {
      processXbee();                                               // Receive lat,lon of target.
  }
  processBaro(groundPressure);                                 // Get current altitude.
  processIMU();                                                // Obtain Heading, Roll, and Pitch Angles.
  //usbSerial.print("current Poll:  ");usbSerial.print(currentPitch);usbSerial.print("\n");

  // Calculate.
  distanceToWaypoint();                                        // Raw Distance to target (no offset).
  courseToWaypoint();                                          // Compass heading to target.
  
  // PID Control.
//  altPIDloop();
//  yawPIDloop();
//  rollPIDloop();
//  pitchPIDloop();

   //Print (Debugging).
  printTarget();                                               // Print transponder data.
  //printScreen();                                               // Vehicle GPS Data.
  //printGraphData();                                             // Use with node.js dump script to capture data and plot in MATLAB
}





// INFINTIE LOOP TO PREVENT FLYING WHEN THERE IS A LOSS IN COMMUNICATION? 
// OR AUTOMATIC SWITCH AWAY FROM AUTONMOUS CONTROL ALGORITHM?  
// ERROR CHECK FOR FLIGHT FAILURE.  
//********************************************************************************************************
//********************************************************************************************************

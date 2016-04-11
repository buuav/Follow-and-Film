//====================================================================================================
//                                     Vehicle_Flight_Code                                              // WHAT ABOUT DELAY TIMES BOTH ON THE TRANSMITTING END AND THE RECEIVING END 
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

//====================================================================================================
//                               General_Global_Variables/Declarations                                           
//====================================================================================================

// GPS Variables.
SIGNAL(TIMER0_COMPA_vect)                                             // Interrupt is called once a millisecond, looks for any new GPS data, and stores it.
{
  GPS.read();
}
float currentLat;                                                     // Current Position
float currentLong;

String latConversion;
String longConversion;
float targetLat;                                                      // Target Position Processing
float targetLong;
int targetHeading;

boolean usingInterrupt = false;
void useInterrupt(boolean);                                           // Function prototype keeps arduino happy?

// XBee Parsing Variables.
int comma = 0;                                                        // Position of the comma delimin
int newLine = 0;                                                      // Position of the new line character
String inputString = "";                                              // XBee string data
boolean stringComplete = false;                                       // Built a new sequence of coordinates now parce it

// Vehicle Flight Data.
double distanceToTarget;                                              // Distance in meters between target and vehicle (GPS Data)
float groundPressure;                                                 // Pressure at ground level to use to compare for Altitude

// PWM Servo Outputs.
Servo throttle;
Servo pitch;
Servo roll;
Servo yaw;
Servo flightMode;
        
// Timer Variables.
unsigned long start = millis();                                       // Used to print results
uint32_t usbPrintTimer, xbeePrintTimer;

// Blinking Update LED
int redLED = 4;                                                       // LED Status updates
int greenLED = 5;

//====================================================================================================
//                                     PID_Variables                                          
//====================================================================================================

                          // ADD PD CONTROLLER FOR THE DISTANCE???

// Altitude 
double setAltitude = 50;                                       // (meters) Around 150 feet?
double currentAltitude;                                        // Comes from the barometer data
double altError;                                               // PID has to use type: double?
double throttleOut = 900;                                      // Pwm Output...Convert to ppm using board.
double AltAggKp=2, AltAggKi=0, AltAggKd=0.25;                   // Define Aggressive and Conservative Tuning Parameters.
double AltConsKp=1, AltConsKi=0, AltConsKd=0.25;              // Not sure how to decide startting values.
PID altPID(&currentAltitude, &throttleOut, &setAltitude,       // Specify the links and initial tuning parameters
            AltConsKp, AltConsKi, AltConsKd, DIRECT);
  
// Yaw
double yawOut = 1500;
double currentHeading;
double desiredHeading = 360;                                   // 0 Degree N at all times.
double yawConsKp=1, yawConsKi= 0, yawConsKd= 0.25;                // Not sure how to decide startting values.
PID yawPID(&currentHeading, &yawOut, &desiredHeading,
           yawConsKp,yawConsKi,yawConsKd,DIRECT);
           
// Roll
double rollOut = 1500;
double currentRoll;
double desiredRoll;                                           // Calculated as a function of Ed and Eh.
double rollConsKp=1, rollConsKi=0, rollConsKd=0.25;
PID rollPID(&currentRoll,&rollOut,&desiredRoll,
            rollConsKp,rollConsKi,rollConsKd,DIRECT);
            
// Pitch                                                      // Calculated as a function of Ed and Eh
double pitchOut = 1500;
double currentPitch;
double desiredPitch;
double pitchConsKp=1, pitchConsKi=0, pitchConsKd=0.25;
PID pitchPID(&currentPitch,&pitchOut,&desiredPitch,
            pitchConsKp,pitchConsKi,pitchConsKd,DIRECT);

//====================================================================================================
//                                             SETUP
//====================================================================================================

void setup(void)
{
  // Start GPS and set desired configuration
  GPS.begin(9600);                                            // 9600 NMEA default speed
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);               // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                  // 1 Hz update rate suggested...
  GPS.sendCommand(PGCMD_NOANTENNA);                           // turn off antenna status info
  useInterrupt(true);                                         // use interrupt to constantly pull data from GPS
  delay(1000);
  
  // XBEE 
  xbeeSerial.begin(57600);
  inputString.reserve(30);                                    // reserve 30 bytes for the inputString:

  // BMP180 Barometer.
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    usbSerial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  groundPressure = getGroundPressure();                      // Sets the ground level pressure.

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
   
  // altPID
  throttle.attach(13);
  processBaro(groundPressure);
  altError = currentAltitude - setAltitude;
  altPID.SetMode(AUTOMATIC);
  altPID.SetOutputLimits(1000,2000);

  // yawPID
  yaw.attach(12);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(1000,2000);

  // rollPID
  roll.attach(11);
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(1000,2000);

  // pitchPID
  pitch.attach(10);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(1000,2000);

  // Flight Mode Switch
  flightMode.attach(9);

  // Safety
  pinMode(redLED,OUTPUT);
  pinMode(greenLED,OUTPUT);
  digitalWrite(redLED,LOW);
  digitalWrite(greenLED,LOW);
  //checkGPSfix();                                           // Loop until it has a fix.
  //checkXbeeFix();                                          // Loop until we have parsed a non-zero value from transponder.
  altCheck();                                                // Loop until we have good altimeter calibration.
  digitalWrite(greenLED,HIGH);
}

//====================================================================================================
//                                             Main LOOP
//====================================================================================================

void loop(void)
{
  // Process GPS 
  processGPS();                                               // Break down data for current position
  
  // Gather Data 
  processXbee();                                              // Receive lat,lon of target.
  processBaro(groundPressure);                                // Get current altitude.
  processIMUheading();                                        // Obtain Heading, Roll, and Pitch Angles.
  
  // Calculate.
  distanceToWaypoint();                                       // Raw Distance to target (no offset).
  courseToWaypoint();                                         // Compass heading to target.
  
  // PID Control.
  altPIDloop();
  yawPIDloop();
  rollPIDloop();
  pitchPIDloop();

  // Print (Debugging).
  printTarget();                                              // Print transponder data.
  printScreen();                                              // Vehicle GPS Data.

}

// INFINTIE LOOP TO PREVENT FLYING WHEN THERE IS A LOSS IN COMMUNICATION? 
// OR AUTOMATIC SWITCH AWAY FROM AUTONMOUS CONTROL ALGORITHM?  
// ERROR CHECK FOR FLIGHT FAILURE.  
//********************************************************************************************************
//********************************************************************************************************

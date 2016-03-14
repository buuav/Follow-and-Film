//====================================================================================================
//                                     Vehicle_Flight_Code                                            // WHAT ABOUT DELAY TIMES BOTH ON THE TRANSMITTING END AND THE RECEIVING END 
//====================================================================================================

#include <Adafruit_GPS.h>                         // used by: GPS
#include <PID_v1.h>
#include <math.h>                                 // used by: GPS
#include <XBee.h>                    

// Serial Communication.
#define usbSerial Serial
#define gpsSerial Serial1
#define xbeeSerial Serial2

//====================================================================================================
//                                     General_Global_Variables/Declarations                                           
//====================================================================================================

// GPS Hardware.
#define GPSECHO false
Adafruit_GPS GPS(&gpsSerial);

// GPS Variables.
SIGNAL(TIMER0_COMPA_vect)                           // Interrupt is called once a millisecond, looks for any new GPS data, and stores i
{
  GPS.read();
}
float currentLat,currentLong;
String latConversion,longConversion;
float targetLat,targetLong;
int targetHeading;
boolean usingInterrupt = false;
void useInterrupt(boolean);                          // Function prototype keeps arduino happy?

// Timer Variables.
unsigned long start = millis();
uint32_t usbPrintTimer, xbeePrintTimer;

// GPS Parsing Variables.
int comma = 0;
int newLine = 0;
String inputString = "";
boolean stringComplete = false;

// Global Vehicle Flight Data.
double headingError;                                   

//====================================================================================================
//                                     PID_Variables                                          
//====================================================================================================

// Altitude. 
double setAltitude = 100;                               // Maybe 100 feet?
double currentAltitude = 75;                            // Comes from the barometer data
double altError;                                        // PID has to use type: double?
double throttleOut;                                     // PPM Output
double AltAggKp=4, AltAggKi=0.2, AltAggKd=1;            // Define Aggressive and Conservative Tuning Parameters.
double AltConsKp=1, AltConsKi=0.05, AltConsKd=0.25;     // Not sure how to decide startting values.
PID altPID (&currentAltitude, &throttleOut, &setAltitude,
            AltConsKp, AltConsKi, AltConsKd, DIRECT);

// Distance.
double setDistance = 50;                                 // 50 Meter Set Distance.
double distanceToTarget;
double distError;
double pitchOut;
double disAggKp=4, disAggKi=0.2, disAggKd=1;             // Define Aggressive and Conservative Tuning Parameters.
double disConsKp=1, disConsKi=0.05, disConsKd=0.25;
PID disPID(&distanceToTarget,&pitchOut,&setDistance,
           disConsKp,disConsKi,disConsKd,DIRECT);

//====================================================================================================
//                                             SETUP
//====================================================================================================

void setup()
{
  // XBEE
  xbeeSerial.begin(57600);
  inputString.reserve(30);                                 // reserve 30 bytes for the inputString:

  // USB Serial Port
  usbSerial.begin(115200);
  usbSerial.println("Adafruit GPS library basic test!");
   
  // start GPS and set desired configuration
  GPS.begin(9600);                                          // 9600 NMEA default speed
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);             // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                // 1 Hz update rate suggested...
  GPS.sendCommand(PGCMD_NOANTENNA);                         // turn off antenna status info
  useInterrupt(true);                                       // use interrupt to constantly pull data from GPS
  delay(1000);

  // altPID
  altPID.SetMode(AUTOMATIC);
}

//====================================================================================================
//                                              LOOP
//====================================================================================================

void loop()
{
  // Process GPS 
   if (GPS.newNMEAreceived())                              // check for updated GPS information
   {                                      
     if(GPS.parse(GPS.lastNMEA()) )                      // if we successfully parse it, update our data fields
     {
        // Gather Data
        processGPS();   
        processXbee();
        
        // Calculate.
        distanceToWaypoint();
        courseToWaypoint();
        
        // PID Control.
        altPIDloop();
        disPIDloop();
        
        // Print Debug.
        printTarget();
        printScreen(); 
      }
   }
}

// INFINTIE LOOP TO PREVENT FLYING WHEN THERE IS A LOSS IN COMMUNICATION? 
// OR AUTOMATIC SWITCH AWAY FROM AUTONMOUS CONTROL ALGORITHM?    


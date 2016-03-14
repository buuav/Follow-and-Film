//====================================================================================================
//                                    GPS_Transpoder_Code
//====================================================================================================

#include <Adafruit_GPS.h>                         // used by: GPS
#include <math.h>                                 // used by: GPS
#include <XBee.h>                     

#define usbSerial Serial
#define gpsSerial Serial1
#define xbeeSerial Serial2

// GPS Navigation
#define GPSECHO false
Adafruit_GPS GPS(&gpsSerial);
float currentLat,currentLong;
unsigned long start = millis();
uint32_t usbPrintTimer, xbeePrintTimer;
boolean usingInterrupt = false;
void useInterrupt(boolean);                 // Func prototype keeps arduino happy?
SIGNAL(TIMER0_COMPA_vect)                  // Interrupt is called once a millisecond, looks for any new GPS data, and stores i
{
  GPS.read();
}

//====================================================================================================
//                                             SETUP
//====================================================================================================

void setup()
{
  // XBEE
  xbeeSerial.begin(57600);

  // USB Serial Port
  usbSerial.begin(115200);
  usbSerial.println("Adafruit GPS library basic test!");
   
  // start GPS and set desired configuration
  GPS.begin(9600);                                  // 9600 NMEA default speed
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);     // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);       // 1 Hz update rate suggested...
  GPS.sendCommand(PGCMD_NOANTENNA);                // turn off antenna status info
  useInterrupt(true);                            // use interrupt to constantly pull data from GPS
  delay(1000);
}

//====================================================================================================
//                                              LOOP
//====================================================================================================

void loop()
{
  if (GPS.newNMEAreceived())                                // check for updated GPS information
  {                                      
    if(GPS.parse(GPS.lastNMEA()) )                          // if we successfully parse it, update our data fields
    {
      processGPS();   
      transmitXbee();                                       // SEND TO XBEE
      printGPS();                                           // Approximately every 2 seconds or so, print out the current stats 
     }
   }
}
        


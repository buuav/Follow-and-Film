#ifndef GPS_hpp
#define GPS_hpp

#include <Adafruit_GPS.h>                         // used by: GPS
#include <math.h>                                 // used by: GPS
#include <XBee.h>
#include <Arduino.h>

#include "Coordinate.h"                                // Include the Point structure




class GPS {
public:

//    double currentLat, currentLong;
    Coordinate currentCoord;
    Coordinate targetCoord;
    

    GPS(HardwareSerial *usbSerial, HardwareSerial *serial, HardwareSerial *xbeeSerial);
//    Adafruit_GPS gps;
    HardwareSerial* gpsSerial;
    HardwareSerial* usb;
    HardwareSerial* xbee;

    double convertDegMinToDecDeg (float);
    void process();
//    void useInterrupt(boolean v);
    void transmitXbee();
    void printData();
    float courseToWaypoint();
    float distanceToWaypoint();

    
    Adafruit_GPS getGPS();
    Coordinate getCurrentCoord();
    Coordinate getTargetCoord();

    void setTargetCoord(Coordinate coord);
};

#endif /* GPS_hpp */

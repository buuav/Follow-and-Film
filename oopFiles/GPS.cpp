#include "GPS.hpp"



GPS::GPS(HardwareSerial *usbSerial, HardwareSerial *serial, HardwareSerial *xbeeSerial){
//    Adafruit_GPS gps = Adafruit_GPS((serial));
    usb = usbSerial;
    gpsSerial = serial;
    xbee = xbeeSerial;
//    (usbSerial)->print("initialized");
}

double GPS::convertDegMinToDecDeg (float degMin)                     // converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
{
    double min = 0.0;
    double decDeg = 0.0;
    
    //get the minutes, fmod() requires doubl1
    min = fmod((double)degMin, 100.0);
    
    //rebuild coordinates in decimal degrees
    degMin = (int) ( degMin / 100 );
    decDeg = degMin + ( min / 60 );
    
    return decDeg;
}

void GPS::process()
{
    Adafruit_GPS gps = Adafruit_GPS((gpsSerial));
    currentCoord = Coordinate(GPS::convertDegMinToDecDeg(gps.latitude), GPS::convertDegMinToDecDeg(gps.longitude));
    if (gps.lat == 'S')            // make them signed
        currentCoord.lattitude *= -1;
    if (gps.lon = 'W')
        currentCoord.longitude *= -1;
}


void GPS::transmitXbee()
{
    if (millis()  > 0)                       // Do not fly for more than 50 days straight. millis() will overflow
    {
        int xbeePrintTimer = millis() + 100;
        xbee->print(currentCoord.lattitude, 6);
        xbee->print(',');
        xbee->println(currentCoord.longitude, 6);
    }
}

// returns the course, set "targetHeading = courseToWaypoint()"

float GPS::courseToWaypoint()
{
    float dlon = (targetCoord.longitude - currentCoord.longitude);
    float cLat = (currentCoord.lattitude);
    float tLat = (targetCoord.lattitude);
    float a1 = sin(dlon) * cos(tLat);
    float a2 = sin(cLat) * cos(tLat) * cos(dlon);                             // Be able to explain in paper!!!!!
    a2 = cos(cLat) * sin(tLat) - a2;
    a2 = atan2(a1, a2);
    if (a2 < 0.0)
    {
        a2 += TWO_PI;
    }
    return degrees(a2);
}

float GPS::distanceToWaypoint()
{
    float delta = (currentCoord.longitude - targetCoord.longitude);                          // copied from TinyGPS library
    float sdlong = sin(delta);                                                // returns distance in meters between two positions, both specified
    float cdlong = cos(delta);                                                // as signed decimal-degrees latitude and longitude. Uses great-circle
    float lat1 = (currentCoord.lattitude);                                         // distance computation for hypothetical sphere of radius 6372795 meters.
    float lat2 = (targetCoord.lattitude);                                          // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    float slat1 = sin(lat1);
    float clat1 = cos(lat1);
    float slat2 = sin(lat2);
    float clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return (delta * 6372795);
}

void GPS::printData()
{
    if (millis() > 0)
    {
        Adafruit_GPS gps(gpsSerial);
        int usbPrintTimer = millis() + 2000; // reset the timer
        
        usb->print("\nTime: ");
        usb->print(gps.hour, DEC); usb->print(':');
        usb->print(gps.minute, DEC); usb->print(':');
        usb->print(gps.seconds, DEC); usb->print('.');
        usb->println(gps.milliseconds);
        usb->print("Date: ");
        usb->print(gps.day, DEC); usb->print('/');
        usb->print(gps.month, DEC); usb->print("/20");
        usb->println(gps.year, DEC);
        usb->print("Fix: "); usb->print((int)gps.fix);
        usb->print(" quality: "); usb->println((int)gps.fixquality);
        if (gps.fix)
        {
            usb->print("Location: ");
            usb->print(gps.latitude, 4); usb->print(gps.lat);
            usb->print(", ");
            usb->print(gps.longitude, 4); usb->println(gps.lon);
            usb->print("Speed (knots): "); usb->println(gps.speed);
            usb->print("Angle: "); usb->println(gps.angle);
            usb->print("Altitude: "); usb->println(gps.altitude);
            usb->print("Satellites: "); usb->println((int)gps.satellites);
        }
    }
}

Adafruit_GPS GPS::getGPS(){
    return Adafruit_GPS((gpsSerial));
}

Coordinate GPS::getCurrentCoord(){
    return currentCoord;
}

Coordinate GPS::getTargetCoord(){
    return targetCoord;
}

void GPS::setTargetCoord(Coordinate coord){
    targetCoord = coord;
}


//
//  GPS.cpp
//  gps
//
//  Created by Hadi Zayer on 3/23/17.
//  Copyright © 2017 Hadi Zayer. All rights reserved.
//

#include "GPS.hpp"


GPS::GPS(){
    gps = Adafruit_GPS(&gpsSerial);
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

    currentCoord = Coordinate(GPS::convertDegMinToDecDeg(gps.latitude), GPS::convertDegMinToDecDeg(gps.longitude))
    if (gps.lat == 'S')            // make them signed
        currentCoord.lattitude *= -1;
    if (gps.lon = 'W')
        currentCoord.longitude *= -1;
}


void GPS::transmitXbee()
{
    if (millis()  > xbeePrintTimer)                       // Do not fly for more than 50 days straight. millis() will overflow
    {
        xbeePrintTimer = millis() + 100;
        xbeeSerial.print(currentCoord.lattitude, 6);
        xbeeSerial.print(',');
        xbeeSerial.println(currentCoord.longitude, 6);
    }
}

// returns the course, set "targetHeading = courseToWaypoint()"

float GPS::courseToWaypoint()
{
    float dlon = radians(targetCoord.longitude - currentCoord.longitude);
    float cLat = radians(currentCoord.lattitude);
    float tLat = radians(targetLat);
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
    float delta = radians(currentCoord.longitude - targetLong);                          // copied from TinyGPS library
    float sdlong = sin(delta);                                                // returns distance in meters between two positions, both specified
    float cdlong = cos(delta);                                                // as signed decimal-degrees latitude and longitude. Uses great-circle
    float lat1 = radians(currentLat);                                         // distance computation for hypothetical sphere of radius 6372795 meters.
    float lat2 = radians(targetLat);                                          // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
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
    if (millis() > usbPrintTimer)
    {
        usbPrintTimer = millis() + 2000; // reset the timer
        
        usbSerial.print("\nTime: ");
        usbSerial.print(gps.hour, DEC); usbSerial.print(':');
        usbSerial.print(gps.minute, DEC); usbSerial.print(':');
        usbSerial.print(gps.seconds, DEC); usbSerial.print('.');
        usbSerial.println(gps.milliseconds);
        usbSerial.print("Date: ");
        usbSerial.print(gps.day, DEC); usbSerial.print('/');
        usbSerial.print(gps.month, DEC); usbSerial.print("/20");
        usbSerial.println(gps.year, DEC);
        usbSerial.print("Fix: "); usbSerial.print((int)gps.fix);
        usbSerial.print(" quality: "); usbSerial.println((int)gps.fixquality);
        if (gps.fix)
        {
            usbSerial.print("Location: ");
            usbSerial.print(gps.latitude, 4); usbSerial.print(gps.lat);
            usbSerial.print(", ");
            usbSerial.print(gps.longitude, 4); usbSerial.println(gps.lon);
            usbSerial.print("Speed (knots): "); usbSerial.println(gps.speed);
            usbSerial.print("Angle: "); usbSerial.println(gps.angle);
            usbSerial.print("Altitude: "); usbSerial.println(gps.altitude);
            usbSerial.print("Satellites: "); usbSerial.println((int)gps.satellites);
        }
    }
}

Adafruit_GPS GPS::getGPS(){
    return gps;
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


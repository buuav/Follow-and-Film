//====================================================================================================
//                                        Functions
//====================================================================================================

void useInterrupt(boolean v) 
{
  if (v) 
  {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else 
  {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void processGPS()
{
  currentLat = convertDegMinToDecDeg(GPS.latitude);
  currentLong = convertDegMinToDecDeg(GPS.longitude);
             
  if (GPS.lat == 'S')            // make them signed
    currentLat = -currentLat;
  if (GPS.lon = 'W')  
    currentLong = -currentLong;
}
 
double convertDegMinToDecDeg (float degMin)                     // converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
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

void transmitXbee()
{
  if (millis()  > xbeePrintTimer)                       // Do not fly for more than 50 days straight. millis() will overflow
      {
        xbeePrintTimer = millis() + 100;
        xbeeSerial.print(currentLat, 6); 
        xbeeSerial.print(',');
        xbeeSerial.println(currentLong, 6);
      }
}

void printGPS()
{
  if (millis() > usbPrintTimer) 
  {
    usbPrintTimer = millis() + 2000; // reset the timer
  
    usbSerial.print("\nTime: ");
    usbSerial.print(GPS.hour, DEC); usbSerial.print(':');
    usbSerial.print(GPS.minute, DEC); usbSerial.print(':');
    usbSerial.print(GPS.seconds, DEC); usbSerial.print('.');
    usbSerial.println(GPS.milliseconds);
    usbSerial.print("Date: ");
    usbSerial.print(GPS.day, DEC); usbSerial.print('/');
    usbSerial.print(GPS.month, DEC); usbSerial.print("/20");
    usbSerial.println(GPS.year, DEC);
    usbSerial.print("Fix: "); usbSerial.print((int)GPS.fix);
    usbSerial.print(" quality: "); usbSerial.println((int)GPS.fixquality);
    if (GPS.fix) 
    {
      usbSerial.print("Location: ");
      usbSerial.print(GPS.latitude, 4); usbSerial.print(GPS.lat);
      usbSerial.print(", ");
      usbSerial.print(GPS.longitude, 4); usbSerial.println(GPS.lon);
      usbSerial.print("Speed (knots): "); usbSerial.println(GPS.speed);
      usbSerial.print("Angle: "); usbSerial.println(GPS.angle);
      usbSerial.print("Altitude: "); usbSerial.println(GPS.altitude);
      usbSerial.print("Satellites: "); usbSerial.println((int)GPS.satellites);
     }
   }
}

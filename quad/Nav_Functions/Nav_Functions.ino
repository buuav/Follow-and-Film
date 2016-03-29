//====================================================================================================
//                                          Functions
//====================================================================================================






//====================================================================================================
//                                         IMU: BNO055
//====================================================================================================

void processIMUheading()
{
  // Euler Angles (Roll & Pitch)
  sensors_event_t IMUevent;                                  
  bno.getEvent(&IMUevent);
  currentRoll = IMUevent.orientation.z;
  currentPitch = IMUevent.orientation.y;

  // Magnetometer
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  currentHeading = atan2(mag.y(),mag.x());
  float declinationAngle = 0.24;
  currentHeading += declinationAngle;
  
  if(currentHeading < 0)                                                // Correct for when signs are reversed.
  {
    currentHeading += 2*PI;
  }
  
  if(currentHeading > 2*PI)                                             // Check for wrap due to addition of declination.
  {
    currentHeading -= 2*PI;
  }
   
  currentHeading = currentHeading * 180/M_PI;                           // Convert radians to degrees for readability.

  // Display (Debugging)
  usbSerial.print("Roll:  ");
  usbSerial.print(currentRoll);
  usbSerial.print("\t");
  usbSerial.print("Pitch: ");
  usbSerial.print(currentPitch);
  usbSerial.print("\t");
  usbSerial.print("Heading: ");
  usbSerial.println(currentHeading);
  
}
//====================================================================================================
//                                         Barometer
//====================================================================================================

float getGroundPressure()
{
  sensors_event_t BNOevent;                          // Get a new sensor event.
  bmp.getEvent(&BNOevent);
  float groundLevelPressure;
  groundLevelPressure = BNOevent.pressure;
  return groundLevelPressure;
}

void processBaro(float groundPressure)
{
  sensors_event_t event;                          // Get a new sensor event.
  bmp.getEvent(&event);
  float temperature;
  bmp.getTemperature(&temperature);
  //float seaLevelPressure = 1019;                  // Adjust this value from weather network.
  currentAltitude = bmp.pressureToAltitude(groundPressure, event.pressure, temperature);
}

//====================================================================================================
//                                            GPS
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
  } else
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

// Decimanl Degree Conversion.
double convertDegMinToDecDeg (float degMin)         // converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
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

// Course_To_Waypoint.
void courseToWaypoint()
{
  float dlon = radians(targetLong - currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);                             // Be able to explain in paper!!!!!
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
}

// Distance_to_Waypoint.
void distanceToWaypoint()
{
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
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
  distanceToTarget =  delta * 6372795;
}

//====================================================================================================
//                                          XBee
//====================================================================================================

// Receive and Parse Target Coordinates.
void processXbee ()
{
  while (xbeeSerial.available() > 0)
  {
    char inChar = (char)xbeeSerial.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }
  if (stringComplete)
  {
    comma = inputString.indexOf(',');
    newLine = inputString.indexOf('\n');
    latConversion = inputString.substring(0, (comma - 1));
    longConversion = inputString.substring((comma + 1), (newLine - 1)); // MAKE THIS A PARSING FUNCTION
    targetLat = latConversion.toFloat();
    targetLong = longConversion.toFloat();
    inputString = "";
    stringComplete = false;
    latConversion = "";
    longConversion = "";
  }
}

//====================================================================================================
//                                          Safety
//====================================================================================================

void checkGPSfix()
{
  while (!GPS.fix)
  {
    if (GPS.newNMEAreceived())
    {
      GPS.parse(GPS.lastNMEA());
    }
  }
}

void checkXbeeFix()
{
  usbSerial.print("Waiting on Xbee now"); usbSerial.print("\n");
  while ((targetLat == 0) && (targetLong == 0))
  {
    processXbee();
  }
  usbSerial.print("We are now Communicating"); usbSerial.print("\n");
  delay(2000);
}

void altCheck()
{
  usbSerial.print("Checking altimeter calibration..."); usbSerial.print("\n");
  while ((currentAltitude < -1.5) || (currentAltitude > 1.5))
  {
    getGroundPressure();
    processBaro(groundPressure);
  }
}

//====================================================================================================
//                                          Debug
//====================================================================================================

// Prints Target Coordinates to Serial.
void printTarget()
{
  usbSerial.print("targetLat: "); usbSerial.print(targetLat, 6);
  usbSerial.print("\n");
  usbSerial.print("targetLong: "); usbSerial.print(targetLong, 6);
  usbSerial.print("\n");
  usbSerial.print("Distance: "); usbSerial.print(distanceToTarget);
  usbSerial.print("\n");
  usbSerial.print("Desired Heading: "); usbSerial.print(targetHeading);
  usbSerial.print("\n");
  usbSerial.print("Altitude:    ");
  usbSerial.print(currentAltitude); Serial.println(" m");
  //usbSerial.print("Ground Level:  ");
  //usbSerial.print(groundPressure); Serial.println(" m");
}

// Prints Vehicle's Coordinates to Screen.
void printScreen()
{
  if (millis() > usbPrintTimer)
  {
    usbPrintTimer = millis() + 2000; // reset the timer CHECK ALL TIMER SETUPS MAKETHISAFUNCTION!!!
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

















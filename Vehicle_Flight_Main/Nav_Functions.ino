//====================================================================================================
//                                          Functions
//====================================================================================================
/* Included Functions List:
   IMU
   Barometer
   GPS
   XBEe
   Saftey
   Debug
   Test
*/

//====================================================================================================
//                                         Smoothing
//====================================================================================================

int digitalSmooth(int rawIn, int *sensSmoothArray)                   // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
{     
  int j, k, temp, top, bottom;
  long total;
  static int i;
                                                                        // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;                                          // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                                           // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++)
  {                                                                     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                                                             // flag to know when we're done sorting              
  while(done != 1)
  {                                                                     // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++)
    {
      if (sorted[j] > sorted[j + 1])
      {                                                                 // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j];
        sorted [j] = temp;
        done = 0;
      }
    }
  }

/*
  for (j = 0; j < (filterSamples); j++){    // print the array to debug
    Serial.print(sorted[j]); 
    Serial.print("   "); 
  }
  Serial.println();
*/

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}


//====================================================================================================
//                                         IMU: BNO055
//====================================================================================================

void processIMU()
{
  // Euler Angles (Roll & Pitch)
  sensors_event_t IMUevent;
  bno.getEvent(&IMUevent);

  currentRoll = IMUevent.orientation.z;
  currentPitch = IMUevent.orientation.y;

  // Magnetometer
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  float magHeading = atan2(mag.y(), mag.x());
  float declinationAngle = 0.24;
  magHeading += declinationAngle;
  if (magHeading < 0)                                           // Correct for when signs are reversed.
  {
    magHeading += 2 * PI;
  }

  if (magHeading > 2 * PI)                                      // Check for wrap due to addition of declination.
  {
    magHeading -= 2 * PI;
  }
  magHeading = magHeading * 180 / M_PI;                         // Convert radians to degrees for readability.


  // Other Magnetic Processing
  currentHeading = IMUevent.orientation.x + 100;                // Adding offset from the Z-axis
  if (currentHeading > 360)
  {
    currentHeading = currentHeading - 360;                      // Account for angles > 360 deg
  }

  //  // Display (Debugging)
  //  usbSerial.print("Roll:  ");
  //  usbSerial.print(currentRoll);
  //  usbSerial.print("\t");
  //  usbSerial.print("Pitch: ");
  //  usbSerial.print(currentPitch);
  //  usbSerial.print("\t");
  //  usbSerial.print("Heading: ");
  //  usbSerial.println(currentHeading);
  //  usbSerial.print("\t");
  //  usbSerial.print("mag_Heading: ");
  //  usbSerial.println(magHeading);
}
//====================================================================================================
//                                         Barometer
//====================================================================================================

float getGroundPressure()                                   // Should only be called once in the setup loop
{
  sensors_event_t BNOevent;                                 // Get a new sensor event.
  bmp.getEvent(&BNOevent);
  float groundLevelPressure;
  double groundSum = 0;
  for(int i = 0; i < 20; i++)
  {
    groundLevelPressure = BNOevent.pressure;
    groundSum += groundLevelPressure;
  }
  groundLevelPressure = groundSum / 20;
  return groundLevelPressure;
}

void processBaro(float groundPressure)
{
  sensors_event_t event;                                    // Get a new sensor event.
  bmp.getEvent(&event);
  float temperature;
  bmp.getTemperature(&temperature);                          // Adjust this value from weather network.
  
  smoothAltitude = digitalSmooth (currentAltitude,sensSmoothArray)+1;

bmpSW.start();
if (bmpSW.elapsed() > 10)
{
  currentAltitude = bmp.pressureToAltitude(groundPressure, event.pressure, temperature);
  bmpSW.reset();
}
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
  if (GPS.newNMEAreceived())                                 // check for updated GPS information.
  {
    if (GPS.parse(GPS.lastNMEA()) )                          // if we successfully parse it, update our data fields.
    {
      currentLat = convertDegMinToDecDeg(GPS.latitude);
      currentLong = convertDegMinToDecDeg(GPS.longitude);

      if (GPS.lat == 'S')            // make them signed
        currentLat = -currentLat;
      if (GPS.lon = 'W')
        currentLong = -currentLong;
    }
  }
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
  float delta = radians(currentLong - targetLong);                          // copied from TinyGPS library
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
  distanceToTarget =  delta * 6372795;
}

//====================================================================================================
//                                          XBee
//====================================================================================================

// Receive and Parse Target Coordinates.
void processXbee(void)
{
  while (xbeeSerial.available() > 0)                                              // While loop to build the buffer of all chracters that are available (not if)
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
    longConversion = inputString.substring((comma + 1), (newLine - 1));           // MAKE THIS A PARSING FUNCTION
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
  usbSerial.print("looking for GPS");
  usbSerial.print("\n");
  while (!GPS.fix)
  {
    digitalWrite(greenLED, HIGH);
    delay(500);
    digitalWrite(greenLED, LOW);                                        // What is the line to test of r fix?
    delay(500);
    if (GPS.newNMEAreceived())
    {
      GPS.parse(GPS.lastNMEA());
    }
  }
  usbSerial.print("Connected to GPS!");
  usbSerial.print("\n");
}

void checkXbeeFix(void)
{
  usbSerial.print("Waiting on Xbee now"); usbSerial.print("\n");
  while (targetLat == 0)
  {
    processXbee();
    digitalWrite(redLED, HIGH);
    delay(100);
    digitalWrite(redLED, LOW);
    delay(100);
  }
  usbSerial.print("XBees are up and running!"); usbSerial.print("\n");
  delay(2000);
}

void checkXbeeComs(bool checkPlease)                                         // Instructions on what to do if we drop the wifi signal
{
  if (xbeeSerial.available() <= 0 && checkPlease)
  {
    xbeeSW.start();
    //usbSerial.print(xbeeSW.elapsed());
    //usbSerial.print("\n");
  }
  else if (xbeeSerial.available() > 0)
  {
    xbeeSW.stop();
    xbeeSW.reset();
  }
  if ((xbeeSW.elapsed() >= 5000) && checkPlease)
  {
    targetLat = startLat;
    targetLong = startLong;
    checkPlease = false;
    talkToXbee = false;
    xbeeSW.reset();
    xbeeSW.stop();
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
  }
}

void getStartingLocation(void)
{
  while (!GPS.fix)
  {
    if (GPS.newNMEAreceived())                                                  // check for updated GPS information.
    {
      if (GPS.parse(GPS.lastNMEA()) )                                            // if we successfully parse it, update our data fields.
      {
        startLat = convertDegMinToDecDeg(GPS.latitude);
        startLong = convertDegMinToDecDeg(GPS.longitude);
        if (GPS.lat == 'S')                                                     // make them signed
        {
          startLat = -startLat;
        }
        if (GPS.lon = 'W')
        {
          startLong = -startLong;
        }
        for (int i = 0; i < 5; i++)
        {
          digitalWrite(greenLED, HIGH);
          delay(100);
          digitalWrite(greenLED, LOW);
          delay(100);
        }
      }
    }
  }
  usbSerial.print("Saved starting location!");
  usbSerial.print("\n");
}

void altCheck()
{
  usbSerial.print("Checking altimeter calibration..."); usbSerial.print("\n");
  while ((currentAltitude < -1.5) || (currentAltitude > 1.5))
  {
    getGroundPressure();
    processBaro(groundPressure);
    digitalWrite(redLED, HIGH);
    delay(100);
    digitalWrite(redLED, LOW);
    delay(100);
  }
  usbSerial.print("Altitude set!");
  usbSerial.print("\n");
}

//====================================================================================================
//                                        Debug Printing
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
  usbSerial.print("Current Heading: "); usbSerial.print(currentHeading);
  usbSerial.print("\n");
  usbSerial.print("Altitude:    ");
  usbSerial.print(currentAltitude); Serial.println(" m");
  usbSerial.print("\n");
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

void printGraphData()
{
  if (millis() > usbPrintTimer)
  {
    usbSerial.print(currentAltitude);
    usbSerial.print(" ");
    usbSerial.print(throttleOut);
    //usbSerial.print(" ");
    //usbSerial.print(currentPitch);
    //usbSerial.print(" ");
    //usbSerial.print(pitchOut);
    //usbSerial.print(" ");
    //usbSerial.print(mappedHeading);
    //usbSerial.print(" ");
    //usbSerial.print(desiredRoll);
    //usbSerial.print(" ");
    //usbSerial.print(currentRoll);
    //usbSerial.print(" ");
    //usbSerial.print(rollOut);
    usbSerial.print("\n");

    
    xbeeSerial.print(currentAltitude);
    xbeeSerial.print(",");
    xbeeSerial.print(throttleOut);
    xbeeSerial.print(",");
    //usbSerial.print(pitchOut);
    //usbSerial.print(",");
    //usbSerial.print(rollOut);
    //usbSerial.print(",");
    xbeeSerial.print(yawOut);
    xbeeSerial.print("\n");
  }
}


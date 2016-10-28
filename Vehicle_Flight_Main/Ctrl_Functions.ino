//====================================================================================================
//                                          Functions
//====================================================================================================
/* These are the four functions that are needed for the PID lop control we are implementing over the 
the arduino into the pwm-ppm converter. They account for the altitude, pitch, roll, and yaw inputs
of the control algorithm.
 */
//====================================================================================================
//                                           Altitude
//====================================================================================================

void altPIDloop()
{ 
    altPID.SetTunings(AltAggKp,AltAggKi,AltAggKd);
    altPID.Compute();
    throttle.writeMicroseconds(throttleOut);                                    // PWM Control ouput sent to pin 13, THROTTLE.
//    usbSerial.print("           ");
//    usbSerial.print(throttleOut);                                               // Debug this value to see what it actialy is
//    usbSerial.print("\n");
//    xbeeSerial.print("           ");
//    xbeeSerial.print(throttleOut);                                               // Debug this value to see what it actialy is
//    xbeeSerial.print("\n");
}

//====================================================================================================
//                                             Yaw  
//====================================================================================================

void yawPIDloop()
{   
 if(currentHeading > 0 && currentHeading < 180)
 {
    mappedHeading = map(currentHeading,0,180,0,500);
//    usbSerial.print(currentHeading);
//    usbSerial.print("\n");
 }
 else if(currentHeading > 180 && currentHeading <= 360)
 {
    mappedHeading = map(currentHeading,180,360,-500,0);
//    usbSerial.print(currentHeading);
//    usbSerial.print("\n");
 }
 yawPID.Compute();
 yawPIDTimer.start();
 if(yawPIDTimer.elapsed() > 100)                                               // The timer is used to mnake sure the addition operation only occurs after one completion of the PID loop
 {
     yawOut = yawOut + 1500;
     yawPIDTimer.reset();
 }
 yawPIDTimer.start();
 yaw.writeMicroseconds(1500);                                                 // PWM Control ouput sent to pin 13, THROTTLE.
// usbSerial.print(yawOut);
// usbSerial.print("\n");
}

//====================================================================================================
//                                             Roll  
//====================================================================================================

void rollPIDloop()
{
    desiredRoll = (0.8 * abs(50)) * sin(45);
    desiredRoll = -desiredRoll;
    if (desiredRoll > 30)
    {
      desiredRoll = 30;                                                         // Make sure we do not exceed a 30 deg. bank.
    }
    else if (desiredRoll < -30)
    {
      desiredRoll = -30;
    }
//    usbSerial.print("desired roll:  "); usbSerial.print(desiredRoll);usbSerial.print("\n");
    rollPID.SetTunings(rollConsKp,rollConsKi,rollConsKd);
    rollPID.Compute();
    roll.writeMicroseconds(1500);                                           
//    usbSerial.print(rollOut);
//    usbSerial.print("\n");
}

//====================================================================================================
//                                            Pitch  
//====================================================================================================

void pitchPIDloop()
{
  desiredPitch = (0.8 * abs(50)) * cos(45) +10;                                   //distanceToTarget      targetHeading
  desiredPitch = -desiredPitch;                                               // Since cloackwise rotation is negative from the BNO055
  if (desiredPitch > 30)
    {
      desiredPitch = 30;                                                      // Make sure we do not exceed a 30 deg. bank.
    }
    else if(desiredPitch < -30)
    {
      desiredPitch = -30;
    }
//    usbSerial.print("desired pitch:  "); usbSerial.print(desiredPitch);usbSerial.print("\n");
    pitchPID.SetTunings(pitchConsKp,pitchConsKi,pitchConsKd);
    pitchPID.Compute();
    pitch.writeMicroseconds(1500);
//    usbSerial.print(pitchOut);
//    usbSerial.print("\n");
}

//====================================================================================================
//                                           Auto Land  
//====================================================================================================

void autoLand(void)
{
   imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);   // maybe a PID loop ont he gravity vector to control decent 
   currentGravity = gravity.x();
   autoLandPID.SetTunings(GravConsKp, GravConsKi, GravConsKd);
   autoLandPID.Compute();
   throttle.writeMicroseconds(landingThrottle); 
}


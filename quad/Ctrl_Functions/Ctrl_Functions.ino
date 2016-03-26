//====================================================================================================
//                                          Functions
//====================================================================================================

/* WHat will the output actually be?
   DO we need to map every thing to 0-255 range in order to have a proper analogWrite?
*/



//====================================================================================================
//                                           Altitude
//====================================================================================================

void altPIDloop()
{
  altError = currentAltitude - setAltitude;
  if(abs(altError) < 30)                                                      // If the magnitude of the distance is larger than 30 meters.
    {
      altPID.SetTunings(AltConsKp,AltConsKi,AltConsKd);
    }
    else
    {
      altPID.SetTunings(AltAggKp,AltAggKi,AltAggKd);
    }
    
    altPID.Compute();
    throttle.writeMicroseconds(throttleOut);                                    // PWM Control ouput sent to pin 13, THROTTLE.
}

//====================================================================================================
//                                              Yaw  
//====================================================================================================

void yawPIDloop()
{                                                                  
 yawPID.SetTunings(AltConsKp,AltConsKi,AltConsKd);
 yawPID.Compute();
 yaw.writeMicroseconds(yawOut);                                    // PWM Control ouput sent to pin 13, THROTTLE.
}

//====================================================================================================
//                                              Roll  
//====================================================================================================

void rollPIDloop()
{
    desiredRoll = (0.8 * distanceToTarget) * sin(targetHeading);
    if (desiredRoll > 30)
    {
      desiredRoll = 30;                                            // Make sure we do not exceed a 30 deg. bank.
    }
    rollPID.SetTunings(rollConsKp,rollConsKi,rollConsKd);
    rollPID.Compute();
    roll.writeMicroseconds(rollOut);   
}

//====================================================================================================
//                                               Pitch  
//====================================================================================================

void pitchPIDloop()
{
  desiredPitch = (0.8 * distanceToTarget) * cos(targetHeading);
  if (desiredPitch > 30)
    {
      desiredPitch = 30;                                            // Make sure we do not exceed a 30 deg. bank.
    }
    pitchPID.SetTunings(pitchConsKp,pitchConsKi,pitchConsKd);
    pitchPID.Compute();
    pitch.writeMicroseconds(pitchOut);
}
















//====================================================================================================
//                                          Functions
//====================================================================================================
void altPIDloop()
{
  if(altError < 10)
    {
      altPID.SetTunings(AltConsKp,AltConsKi,AltConsKd);
    }
    else
    {
      altPID.SetTunings(AltAggKp,AltAggKi,AltAggKd);
    }
    altPID.Compute();
}

void disPIDloop()
{
  if(distError < 30)
  {
    disPID.SetTunings(disConsKp,disConsKi,disConsKd);
  }
  else
  {
    disPID.SetTunings(disAggKp,disAggKi,disAggKd);
  }
  disPID.Compute();
}


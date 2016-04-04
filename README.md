# Follow-and-Film

Changes: 4 Apr 2016:

* Added offset to yaw euler axis to match proper compass heading
* Added Pitch and Roll PID loops
* Added Error Checking status LEDSs (red & green)

                    * To Do:

                        - Create the waypoint(target) offset so we dont hover right on top of vehicle
                        - I want to scope the PWM output from MEGA (once PID loops done) to make sure we
                        understand the value of the signal and how it will correspond to the typical input (from a 
                        transmitter) because I am worried about it being calibrated to radio T/x output values and 
                        not being able to function with the Arduino outputs.
                        - Add PWM - PPM converter
                        - Add Andrew's switch to frame and wire it
                        - Make sure only 1 ESC is powering the PixHawk
                        - Check radio calibration with APM and PixHawk
                        - Get frame ready to testfly and devise trategy for safe testing...


******************* I am sure I left a few things out but this is the most recent update *************************
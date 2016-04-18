# Follow-and-Film

Changes: 4 Apr 2016:

* Added offset to yaw euler axis to match proper compass heading
* Added Pitch and Roll PID loops
* Added Error Checking status LEDSs (red & green)

                    * To Do:

                        - Create the waypoint(target) offset so we dont hover right on top of vehicle
                        - PID values for YAW within PixHawk need to be adjusted...when flying, the inuts are delayed but smooth so I think all three terms should be incremented close to the values currently running on the pitch and roll axes. 


******************* I am sure I left a few things out but this is the most recent update *************************
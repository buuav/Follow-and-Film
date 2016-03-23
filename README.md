# Follow-and-Film

Changes: 23 Mar 2016:
                    
                    * Added functions to check for preflight communication (safety)
                    * Created 2 rough PID loops for Altitude and Yaw
                    * Incorporated the Barometer:
                                        - It calculates our alt relative to ground not sea level
                    * To Do:
                        - Create the waypoint(target) offset so we dont hover right on top of vehicle
                        - Create the other two PIDs for pitch and Roll
                        - Actually pull data from the BN0055 IMU (it is properly hooked up and setup in loop)
 


******************* I am sure I left a few things out but this is the most recent update *************************
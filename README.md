Thanks for Entering!! 

This will be a project for an Auto PID-Tuning Algorithm!

Code Structure
- Commands Folder: Have all the tuning Commands, includes kP, kD, kS, kV, etc. Those Commands output the gains in the SmartDashboard 
- HardWare Folder: Have the custom motor wrappers, those wrappers differ from the wrappers in the season code as this wrapper only includes the Canbus and the Motor ID, also have override methods 
- Motor Execute: Have Main Methods for motor controls
- AutoCalibratorCombiner: Class for having the sequence command and the ultimate tuning command -> have all the motors for each subsystem to be tuned!!!

Details
- As I go through my first season in FRC, I have realized that tuning mechanisms takes quite a long time that we are supposed to use in testing our code. So I developed a code that runs through all the motors and outputs all the PID gains by it self. The code have custom wrapper classes for TalonFX and TalonFXS. I wrote the custom wrapper classes that differ from the season-code wrappers because I thought the season-wrappers had parameters and other features that I do not need in just running the motors with certain voltages and currents. I have also wrote some commands for getting each gains and also a ensemble command that runs all the motors with the command for getting the gains.

How To Run 
- As the API for FRC Java is changing starting from Season 2027, you may have to alter some code. But the overall structure is the same. You migrate the AutoTune folder and run the code! Open SmartDashboard so that the code can read of from it. 

I hope it works!🤖🤖

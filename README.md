Thanks for Entering!! 

This will be a project for an Auto PID-Tuning Algorithm!

Code Structure
- UI/SmartDashBoard setting
- Tunable Motor Inteface -> implement with diff motors classes
- Factory Pattern Class -> Use enum to use create instances of Motors
- Motor Executing Class -> run the motor for the logger class 
- Logger Class for data and visulizing graph
- Analyzing Classes - FeedFoward Gains(kV, kS, kG), FeedBack Gains(PID), TorqueCurrentFOC FF, MMExpo Gains
- Output Class
- SIM Class for testing!

Details
- Starting with Interfaces, I want to make them simple as possible: create an Interface, implement it with different motor types.
In detail, the interface will be named "TunableMotor" and will have methods such as start(), stop(), setVoltage(), setCurrent(), getInstanceVelocity(), getInstancePos(), etc.
After that, I can just implement them with different Motors using method overriding. For the factory pattern, I will use enums to select motors. The classes of Interface, Implemening Motor classes, and factory class will be inside a folder called "hardware"
Moving on to motor executing class, I will have the class to input the gear ratios and the sensor ratios. And run the motor to certain voltages for the Logger Class. (Might or must have to add more features and inputs).
For Logger Class, I will just have the code to log the stator voltage, velocity, position, and acceleration onto the SmartDashboard with a graph logger.
For the Analyzing Classes. I will have them to calculate the gains, also seperating them according to their types.
For the output class, it will push the data to NetworkTables or directly apply the configs to TalonFX slot configs
For Sim, purely for testing stuffs using simulating features.

Motor Execution Details:
- As I think tuning all the motors one by one takes a lot of time even for a auto tuner, I don't think there will be much of a difference than tuning every subsystem manually as I did this season. I want to have the code to execute ever motor to run by itself. For example, for the 2026 Season Bot, I will have the code to execute an analysis of Torque Current FOC for the Flywheel, execute analysis of Pure MMExpo for Intake, and all the other subsystem to run by itself for timesaving. For subsystems such as the deployer, I will have special features such as soft limits and hard limits and special methods of tuning than other basic free-spinning subsystems. 

I hope it works!🤖🤖

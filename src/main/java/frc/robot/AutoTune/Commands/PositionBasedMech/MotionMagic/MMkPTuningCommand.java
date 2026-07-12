package frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;

import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands; 

//Class for caculating the kP Value for MM

//It is very important that you put the right direction of the motor -> positive or negative direction
//If you don't consider that, the motor will likely stop due to the safety check of the setMotorVoltagePOS method
public class MMkPTuningCommand {
    
    

}

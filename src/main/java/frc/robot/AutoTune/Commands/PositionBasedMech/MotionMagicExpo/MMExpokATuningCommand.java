package frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagicExpo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.PositionBasedMech.kGTuningCommandPOS;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

//A Class for tuning Expo kA for MMExpo 
//According to CTRE, motion magic expo is theoretically the same as normal kA but used differently. 
//It is used to get the volts(output for normal) to acquire a certain acceleration
//But because we want our trajectory to be smooth, we are going to adjust the normal kA value that our trajectory is crazy bumpy as high low kA means sudden acc
public class MMExpokATuningCommand {

    private enum State{
        INIT, MOVING
    }

}

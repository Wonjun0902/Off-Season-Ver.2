package frc.robot.AutoTune.Commands.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;
import frc.robot.subsystems.swerve.TunerConstants.TunerSwerveDrivetrain;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands; 

// This kA Tuning Command is only for MM and MMExpo. 
// Not necessary for standard PID 
public class kATuningCommand {
    /**
     * kA Tuning Command 
     * Give the motor sudden rush of volts multiple times to calculate the additional voltage needed 
     * @param defaultSpeed - default volts that would run the motor at a certain speed, with kS ofc
     * @param stepVolts
     * @param gearRatio 
     * @param motorExecute
     * @param duration 
     */
    private Timer stepTimer = new Timer();
    private double appliedVoltage;
    private double tunedkA;

    public Command kATuningCommand(double defaultSpeed, double stepVolts, double gearRatio, double duration, MotorExecute motorExecute, kSTuningCommand kSTuningCommand, kVTuningCommand kVTuningCommand){

        return Commands.run(() -> {
            //1. Apply the motor with the default speed with kS and kV
            double kS = kSTuningCommand.getKS();
            double kV = kVTuningCommand.getKV();
            double appliedVoltage = kS * Math.signum(defaultSpeed) + kV * defaultSpeed;

            motorExecute.setMotorVoltage(appliedVoltage);

            //Applying increasing voltage when the motor is at a steady velocity (might change the time from 1.5 sec)
            if(stepTimer.hasElapsed(1.5)){
                appliedVoltage += stepVolts;
                motorExecute.setMotorVoltage(appliedVoltage);
            }
        })
        //Run the increment until the motor has run for a certain period
        .until(() -> {
            return stepTimer.hasElapsed(duration);
        })
        .beforeStarting(() -> {
            stepTimer.start();
        })
        .finallyDo((interrupted) -> {
            double finalSpeed = motorExecute.getMotorSpeed(gearRatio).magnitude();
            double acc = (finalSpeed - defaultSpeed) / duration;
            double tunedkA = appliedVoltage / acc;

            motorExecute.stopMotor();

            if(!interrupted){
                SmartDashboard.putNumber("kA Value: ", tunedkA);
            }
        });
    }

    public double getKA(){
        return SmartDashboard.getNumber("kA Value: ",tunedkA);
    }
}

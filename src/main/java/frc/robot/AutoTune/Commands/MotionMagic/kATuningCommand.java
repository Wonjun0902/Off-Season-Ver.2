package frc.robot.AutoTune.Commands.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;

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

    private boolean isAccelerating;
    private double startSpeed;
    private double startTime;

    public Command kATuningCommand(double defaultSpeed, double stepVolts, double gearRatio, double duration, MotorExecute motorExecute, kSTuningCommand kSTuningCommand, kVTuningCommand kVTuningCommand){

        return Commands.run(() -> {
            //Apply the motor with the default speed with kS and kV
            double kS = kSTuningCommand.getKS();
            double kV = kVTuningCommand.getKV();
            appliedVoltage = kS * Math.signum(defaultSpeed) + kV * defaultSpeed;

            motorExecute.setMotorVoltage(appliedVoltage);

            //Maintain a steady velocity for a certain duration for stability(might change the time from 1.5 sec)
            if(!stepTimer.hasElapsed(1.5)){
                motorExecute.setMotorVoltage(appliedVoltage);
            }
            //Apply increasing voltage to the motor
            else{
                if(!isAccelerating){
                    isAccelerating = true;
                    startSpeed = motorExecute.getMotorSpeed(gearRatio).magnitude();
                    startTime = stepTimer.get();
                }
                motorExecute.setMotorVoltage(appliedVoltage + stepVolts);
            }
        })
        //Run the increment until the motor has run for a certain period
        .until(() -> {
            return stepTimer.hasElapsed(duration);
        })
        .beforeStarting(() -> {
            startSpeed = 0.0;
            startTime = 0.0;
            tunedkA = 0.0;
            isAccelerating = false;
            stepTimer.start();
        })
        .finallyDo((interrupted) -> {
            if(!interrupted && isAccelerating){
                double finalSpeed = motorExecute.getMotorSpeed(gearRatio).magnitude();
                double endTime = stepTimer.get();

                double dv = finalSpeed - startSpeed;
                double dt = endTime - startTime;

                double acc = dv/dt;

                tunedkA = stepVolts / acc;

                SmartDashboard.putNumber("kA Value: ", tunedkA);
            }

            motorExecute.stopMotor();
        });
    }

    public double getKA(){
        return SmartDashboard.getNumber("kA Value: ",tunedkA);
    }
}

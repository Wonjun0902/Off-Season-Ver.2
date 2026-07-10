package frc.robot.AutoTune.Commands.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;

import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands; 

public class MMCruiseVTuningCommand {

    /**
     * Cruise Velocity Tuning Command 
     * Give the motor the maximum output - 10~12V and calculate the cruise velocity
     * Make sure the motor starts from 0.0 rot per sec 
     * @param gearRatio 
     * @param duration 
     * @param maxVolts -> might be different for all subsystems so I'll put in as a parameter 
     */
    private Timer stepTimer = new Timer();
    private double tunedCruiseV;

    private double cruiseV;

    private LinearFilter speedFilter;

    public Command cruiseVTuningCommand(double gearRatio, double duration, double maxVolts, MotorExecute motorExecute){
        return Commands.run(() -> {
            //Apply almost maximum voltage to the motor 
            motorExecute.setMotorVoltage(maxVolts);

            //Gets the speed and the time right now 
            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).magnitude();

            //Update maximum Speed with Filter
            if(stepTimer.hasElapsed(0.0)){
                double rawSpeed = currentSpeed;
                double smoothedSpeed = speedFilter.calculate(rawSpeed);
                if(smoothedSpeed > cruiseV){
                    cruiseV = smoothedSpeed;
                }
            }
        })
        .until(() -> {
            return stepTimer.hasElapsed(duration);
        })
        .beforeStarting(() -> {
            cruiseV = 0.0;
            stepTimer.restart();

            speedFilter = LinearFilter.movingAverage(5);
        })
        .finallyDo((interrupted) -> {
            if(!interrupted){
                tunedCruiseV = cruiseV * 0.8;

                SmartDashboard.putNumber("Cruise Velocity Value: ", tunedCruiseV);
            }
            motorExecute.stopMotor();
        });
    }

    public double getCruiseVelocity(){
        return SmartDashboard.getNumber("Cruse Velocity Value: ", tunedCruiseV);
    }
}

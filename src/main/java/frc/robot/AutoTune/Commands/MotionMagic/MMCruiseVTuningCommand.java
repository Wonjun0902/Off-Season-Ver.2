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
    private double smoothedSpeed;

    private LinearFilter speedFilter;

    public Command cruiseVTuningCommand(double gearRatio, double duration, double maxVolts, MotorExecute motorExecute){
        return Commands.run(() -> {
            //Apply almost maximum voltage to the motor 
            motorExecute.setMotorVoltage(maxVolts);

            //Gets the speed and the time right now 
            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).magnitude();

            //Calculate the average speed of the motor using linear filter
            smoothedSpeed = speedFilter.calculate(currentSpeed);
        })
        .until(() -> {
            return stepTimer.hasElapsed(duration);
        })
        .beforeStarting(() -> {
            stepTimer.restart();
            smoothedSpeed = 0.0;
            speedFilter = LinearFilter.movingAverage(5);
        })
        .finallyDo((interrupted) -> {
            if(!interrupted){
                tunedCruiseV = smoothedSpeed * 0.9; //Multiply by 0.9 cause having the motor to spin at its fullist won't do any good on teh motor
                SmartDashboard.putNumber("Cruise Velocity Value: ", tunedCruiseV);
            }
            motorExecute.stopMotor();
        });
    }

    public double getCruiseVelocity(){
        return SmartDashboard.getNumber("Cruise Velocity Value: ", tunedCruiseV);
    }
}

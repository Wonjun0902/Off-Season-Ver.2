package frc.robot.AutoTune.Commands.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;

import java.util.spi.CurrencyNameProvider;

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

    public Command cruiseVTuningCommand(double gearRatio, double duration, double maxVolts, MotorExecute motorExecute){
        return Commands.run(() -> {
            //Apply almost maximum voltage to the motor 
            motorExecute.setMotorVoltage(maxVolts);

            //Gets the speed and the time right now 
            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).magnitude();

            //Update maximum Speed
            if(currentSpeed > cruiseV){
                cruiseV = currentSpeed;
            }
        })
        .until(() -> {
            return stepTimer.hasElapsed(duration);
        })
        .beforeStarting(() -> {
            cruiseV = 0.0;
            stepTimer.restart();
        })
        .finallyDo((interrupted) -> {
            if(!interrupted){
                tunedCruiseV = cruiseV;

                SmartDashboard.putNumber("Cruise Velocity Value: ", tunedCruiseV);
            }
            motorExecute.stopMotor();
        });
    }

    public double getCruiseVelocity(){
        return SmartDashboard.getNumber("Cruse Velocity Value: ", tunedCruiseV);
    }
}

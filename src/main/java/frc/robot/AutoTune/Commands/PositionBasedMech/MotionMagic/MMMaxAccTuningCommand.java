package frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;

import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands; 

//Class for calculating the max Acceleration -> for MM only!

//It is very important that you put the right direction of the motor -> positive or negative direction
//If you don't consider that, the motor will likely stop due to the safety check of the setMotorVoltagePOS method
public class MMMaxAccTuningCommand {

    /**
     * Max Acceleration Tuning Command 
     * Give the motor the maximum output - 10~12V and calcualte the maximum acceleration it gets
     * Make sure that the motor starts from 0.0 rot per sec
     * @param gearRatio 
     * @param duration 
     * @param maxVolts -> might be different for all subsystems so I'll put in as a parameter 
     */
    private Timer stepTimer = new Timer();
    private double tunedMaxAcc;

    private double maxAcc;
    private double lastTime;
    private double lastSpeed;

    private LinearFilter accFilter;

    public Command maxAccTuningCommand(double gearRatio, double duration, double maxVolts, MotorExecute motorExecute){
        return Commands.run(() -> {
        //Apply (almost)maximum voltage to the motor 
        motorExecute.setMotorVoltagePOS(maxVolts);
        
        //Gets the speed and the time right at teh instance
        double currentSpeed = motorExecute.getMotorSpeed(gearRatio).magnitude();
        double currentTime = stepTimer.get();
        
        //Get the derivative of the speed for acc
        double dv = currentSpeed - lastSpeed;
        double dt = currentTime - lastTime;

        //Update max acc with linear filter 
        if(dt > 0.0){
            double rawAcc = dv / dt;

            double smoothedAcc = accFilter.calculate(rawAcc);

            if(smoothedAcc > maxAcc){
                maxAcc = smoothedAcc;
            }
        }

        //Update variables for next loop 
        lastSpeed = currentSpeed;
        lastTime = currentTime;
        })
        //Run until timer has elasped the set time duration(parameter)
        .until(() -> {
            return stepTimer.hasElapsed(duration);
        })
        .beforeStarting(() -> {
            maxAcc = 0.0;
            lastTime = 0.0;
            lastSpeed = 0.0;
            stepTimer.restart();

            accFilter = LinearFilter.movingAverage(5);
        })
        .finallyDo((interrupted) -> {
            if(!interrupted){
                tunedMaxAcc = maxAcc * 0.8; //Multiply by 0.8 cause I think going to the physical limit might overload the bot

                SmartDashboard.putNumber("Max Acceleration Value: ", tunedMaxAcc);
            }

            motorExecute.stopMotor();
        });
    }

    public double getMaxAcc(){
        return tunedMaxAcc;
    }

}

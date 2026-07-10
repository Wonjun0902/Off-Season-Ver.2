package frc.robot.AutoTune.Commands.StandardPID;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; 

public class kPTuningCommand{

    private PIDController pidController;

    private double testkP = 0.0;
    private double bestkP = 0.0;
    private double lowestScore = Double.MAX_VALUE;

    private double cumulativeError = 0.0;
    private double maxWindowError = 0.0;

    private Timer stepTimer = new Timer();

    private boolean isreachedTarget = false;

    /**
     * kP Tuning Command for Free Spin 
     * This command runs some tests calculates for the kP Gain
     * @param kPIncrement increment of kP per loop
     * @param targetSpeed target speed of the mechanism 
     * @param duration test duration for accuracy 
     * @param maxkP max kP for safety issues 
     */
    public Command kPTuningCommand(double kPIncrement, double targetSpeed, double duration, double maxkP, double gearRatio, MotorExecute motorExecute, kSTuningCommand kSTuningCommand, kVTuningCommand kVTuningCommand){
        return Commands.run(() -> {

            //1. Calculate the current error of Speed
            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RadiansPerSecond);
            double absError = Math.abs(currentSpeed - targetSpeed);

            //2. Adds the absolute error to the cumulative error 
            cumulativeError += absError;

            //3. Check if there are any big errors in the last 0.5 seconds -> for big oscillation checks
            double timeRemaining = duration - stepTimer.get();
            if(timeRemaining < 0.5){
                if(absError > maxWindowError){
                    maxWindowError = absError;
                }
            }

            //4. Check if the motor speed reaches the targetSpeed
            double absCurrentSpeed = Math.abs(currentSpeed);
            double absTargetSpeed = Math.abs(targetSpeed);
            if(absCurrentSpeed > absTargetSpeed){
                isreachedTarget = true;
            }
            

            //Apply PID Voltage and FF Voltage to the Motor
            double pidVoltage = pidController.calculate(currentSpeed, targetSpeed);

            double kS = kSTuningCommand.getKS();
            double kV = kVTuningCommand.getKV();
            double ffVoltage = kS * Math.signum(targetSpeed) + kV * targetSpeed;
            
            motorExecute.setMotorVoltage(pidVoltage + ffVoltage);

            //Scoring 
            if(stepTimer.hasElapsed(duration)){
                double finalWeight = 50.0; //Might Adjust it

                //If the speed didn't reach the target, we just don't consider the kP value
                if(!isreachedTarget){
                    finalWeight = Double.MAX_VALUE;
                }

                double score = cumulativeError + (maxWindowError * finalWeight);

                if(score < lowestScore){
                    lowestScore = score;
                    bestkP = testkP;
                }

                testkP += kPIncrement;
                stepTimer.restart();
                pidController.setP(testkP);
                cumulativeError = 0.0;
                maxWindowError = 0.0;

                isreachedTarget = false;
            }
        })
        .beforeStarting(() -> {
            testkP = kPIncrement;
            pidController.reset();
            pidController.setP(testkP);

            lowestScore = Double.MAX_VALUE;
            cumulativeError = 0.0;
            maxWindowError = 0.0;

            isreachedTarget = false;
            
            stepTimer.restart();
        })
        .until(() -> {
            return testkP > maxkP;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                SmartDashboard.putNumber("kP Value: ", bestkP);
            }
        });}

    public double getkP(){
        return SmartDashboard.getNumber("kP Value: ", bestkP);
    }
}
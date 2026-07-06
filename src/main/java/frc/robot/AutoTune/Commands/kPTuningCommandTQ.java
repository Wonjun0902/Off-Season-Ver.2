package frc.robot.AutoTune.Commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; 

public class kPTuningCommandTQ {

    private PIDController pidController;

    private double testkPTQ = 0.0;
    private double bestkPTQ = 0.0;
    private double lowestScoreTQ = Double.MAX_VALUE;

    private double cumulativeError = 0.0;
    private double maxWindowError = 0.0;

    private Timer stepTimer = new Timer();

    private boolean isreachedTarget = false;

    /**
     * kP Tuning Command for Toruue Current FOC 
     * @param kPIncrement
     * @param targetSpeed
     * @param duration 
     * @param maxkP
     * @param gearRatio 
     * @param motorExecute 
     * @param kSTuningCommandTQ
     * @param kVTuningCommandTQ
     */ 
    public Command kPTuningCommandTQ(double kPIncrement, double targetSpeed, double duration, double maxkP, double gearRatio, MotorExecute motorExecute, kSTuningCommandTQ kSTuningCommandTQ, kVTuningCommandTQ kVTuningCommandTQ){
        return Commands.run(() -> {
            //1. Calculate the error
            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);
            double absError = Math.abs(currentSpeed - targetSpeed);

            //2. Add the error to the cumulative error per loop 
            cumulativeError += absError;

            //3. Check if there are any big errors, oscillation check 
            double timeRemaining = duration - stepTimer.get();
            if(timeRemaining < 0.5){
                if(absError > maxWindowError){
                    maxWindowError = absError;
                }
            }

            //4. Check if motor reaches the target 
            double absCurrentSpeed = Math.abs(currentSpeed);
            double absTargetSpeed = Math.abs(targetSpeed);
            if(absCurrentSpeed > absTargetSpeed){
                isreachedTarget = true;
            }

            //Apply PID current and FF current 
            double pidCurrent = pidController.calculate(currentSpeed, targetSpeed);
            
            double kSTQ = kSTuningCommandTQ.getKSTQ();
            double kVTQ = kVTuningCommandTQ.getkVTQ();
            double ffCurrent = kSTQ + (kVTQ * targetSpeed);

            motorExecute.setMotorCurrent(pidCurrent+ ffCurrent);

            //Scoring 
            if(stepTimer.hasElapsed(duration)){
                double finalWeight = 50.0;

                if(!isreachedTarget){
                    finalWeight = Double.MAX_VALUE;
                }

                double scoreTQ = cumulativeError + maxWindowError * finalWeight;

                if(scoreTQ < lowestScoreTQ){
                    lowestScoreTQ = scoreTQ;
                    bestkPTQ = testkPTQ;
                }

                testkPTQ += kPIncrement;
                stepTimer.restart();
                pidController.setSetpoint(testkPTQ);
                cumulativeError = 0.0;
                maxWindowError = 0.0;
            }
        })
        .beforeStarting(() -> {
            testkPTQ = kPIncrement;
            pidController.reset();
            pidController.setSetpoint(testkPTQ);
            
            lowestScoreTQ = Double.MAX_VALUE;
            cumulativeError = 0.0;
            maxWindowError = 0.0;

            stepTimer.restart();
        })
        .until(() -> {
            return testkPTQ > maxkP;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(interrupted){
                SmartDashboard.putNumber("kP Value TQFOC: ", bestkPTQ);
            }
        });
    }

    public double getkPTQ(){
        return SmartDashboard.getNumber("kP Value TQFOC", bestkPTQ);
    }

}

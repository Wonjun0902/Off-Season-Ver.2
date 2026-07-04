package frc.robot.AutoTune.Commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class kPTuningCommand extends SubsystemBase{

    private MotorExecute motorExecute;
    private kSTuningCommand kSTuningCommand;
    private kVTuningCommand kVTuningCommand;
    private PIDController pidController;

    private double testkP = 0.0;
    private double bestkP = 0.0;
    private double lowestScore = Double.MAX_VALUE;

    private double cumulativeError = 0.0;
    private double maxWindowError = 0.0;

    private Timer stepTimer = new Timer();

    /**
     * kP Tuning Command for Free Spin 
     * This command runs some tests calculates for the kP Gain
     * @param kPIncrement increment of kP per loop
     * @param targetSpeed target speed of the mechanism 
     * @param duration test duration for accuracy 
     * @param maxkP max kP for safety issues 
     */
    public Command kPTuningCommand(double kPIncrement, double targetSpeed, double duration, double maxkP){
        return run(() -> {
            //1. Calculate for the current error 
            double currentSpeed = motorExecute.getMotorSpeed().in(RadiansPerSecond);
            double error = Math.abs(targetSpeed - currentSpeed);

            cumulativeError += error;

            //Window Check
            //If we are in the last 0.5 seconds, track the worst error we see 
            double timeRemaining = duration - stepTimer.get();
            if(timeRemaining < 0.5){
                if(error < maxWindowError){
                    maxWindowError = error;
                }
            }

            //Apply PID and FF Voltage
            double pidVoltage = pidController.calculate(currentSpeed, targetSpeed);

            double kS = kSTuningCommand.getKS();
            double kV = kVTuningCommand.getKV();
            double ffVoltage = kS *Math.signum(targetSpeed) + kV * targetSpeed;

            motorExecute.setMotorVoltage(pidVoltage + ffVoltage);

            //4. Evaluate kP value when the timer is over 
            if(stepTimer.hasElapsed(duration)){
                //Multiply the worst error by 50 -> for scoring reasons 
                double finalWeight = 50.0;
                double score = cumulativeError + (maxWindowError * finalWeight);

                if(score < lowestScore){
                    bestkP = testkP;
                }

                testkP += kPIncrement;
                pidController.setP(testkP);
                cumulativeError = 0.0;
                stepTimer.restart();
            }
        })
        .beforeStarting(() -> {
            testkP = kPIncrement;
            bestkP = 0.0;
            lowestScore = Double.MAX_VALUE;
            cumulativeError = 0.0;
            maxWindowError = 0.0;

            pidController.reset();
            pidController.setP(testkP);

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
        });
    }

    public double getkP(){
        return SmartDashboard.getNumber("kP Value: ", bestkP);
    }

    private double testkPPOS = 0.0;
    private double bestkPPOS = 0.0;
    private double lowestScorePOS = Double.MAX_VALUE;

    private double cumulativeErrorPOS = 0.0;
    private double maxWindowErrorPOS = 0.0;

    private Timer stepTimerPOS = new Timer();
    
    /**
     * kP Tuning Command for Position Mechanism 
     * This command runs some tests calculates for the kP Gain
     * @param kPIncrement increment of kP per loop
     * @param targetPosition target position of the position mechanism 
     * @param duration test duration for accuracy 
     * @param maxkP max kP value for safety issues 
     */
    public Command kPTuningCommandPOS(double kPIncrement, double targetPosition, double duration, double maxkP, double cruiseVelocity){
        return run(() -> {
            //1. Calculate for the current error 
            double currentPosition = motorExecute.getMotorPosition().in(Radians);
            double error = Math.abs(targetPosition - currentPosition);

            cumulativeErrorPOS += error;

            //Window Check
            //If we are in the last 0.5 seconds, track the worst error we see 
            double timeRemaining = duration - stepTimerPOS.get();
            if(timeRemaining < 0.5){
                if(error < maxWindowErrorPOS){
                    maxWindowErrorPOS = error;
                }
            }

            //Apply PID and FF Voltage
            double pidVoltage = pidController.calculate(currentPosition, targetPosition);

            double kS = kSTuningCommand.getKS();
            double kV = kVTuningCommand.getKV();
            double ffVoltage = kS *Math.signum(cruiseVelocity) + kV * cruiseVelocity;

            motorExecute.setMotorVoltagePOS(pidVoltage + ffVoltage);

            //4. Evaluate kP value when the timer is over 
            if(stepTimerPOS.hasElapsed(duration)){
                //Multiply the worst error by 50 -> for scoring reasons 
                double finalWeight = 50.0;
                double score = cumulativeErrorPOS + (maxWindowErrorPOS * finalWeight);

                if(score < lowestScorePOS){
                    bestkPPOS = testkPPOS;
                }

                testkPPOS += kPIncrement;
                pidController.setP(testkPPOS);
                cumulativeErrorPOS = 0.0;
                stepTimerPOS.restart();
            }
        })
        .beforeStarting(() -> {
            testkPPOS = kPIncrement;
            bestkPPOS = 0.0;
            lowestScorePOS = Double.MAX_VALUE;
            cumulativeErrorPOS = 0.0;
            maxWindowErrorPOS = 0.0;

            pidController.reset();
            pidController.setP(testkPPOS);

            stepTimerPOS.restart();
        })
        .until(() -> {
            return testkPPOS > maxkP;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();
            
            if(!interrupted){
                SmartDashboard.putNumber("kP Value POS: ", bestkPPOS);
            }
        });
    }

    public double getkPPOS(){
        return SmartDashboard.getNumber("kP Value POS: ", bestkPPOS);
    }

}

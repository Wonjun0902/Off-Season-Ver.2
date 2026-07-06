package frc.robot.AutoTune.Commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Currency;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; 

public class KDTuningCommandTQ {

    private PIDController pidController;

    private double testkD = 0.0;
    private double bestkD = 0.0;
    private double lowestScore = Double.MAX_VALUE;

    private double cumulativeError = 0.0;

    private Timer stepTimer = new Timer();

    /**
     * kD Tuning Command for Torque Current FOC 
     * Again, the logic is the same 
     * @param kDIncrement 
     * @param targetSpeed
     * @param duration 
     * @param maxkD
     * @param gearRatio 
     * @param kPTuningCommandTQ
     * @param kSTuningCommandTQ
     * @param kVTuningCommandTQ
     * @param motorExecute 
     */
    public Command kDTuningCommandTQ(double kDIncrement, double targetSpeed, double duration , double maxkD, double gearRatio, kPTuningCommandTQ kPTuningCommandTQ, kSTuningCommandTQ kSTuningCommandTQ, kVTuningCommandTQ kVTuningCommandTQ, MotorExecute motorExecute){
        return Commands.run(() -> {
            //1. Calcualte for the current error of the speed 
            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RadiansPerSecond);
            double absError = Math.abs(currentSpeed - targetSpeed);

            //2. Adds up the abs Error to the cumulative error 
            cumulativeError += absError;

            //3. Apply PID Current and FF current 
            double pidCurrent = pidController.calculate(currentSpeed, targetSpeed);
            double tunedkPTQ = kPTuningCommandTQ.getkPTQ();

            double kSTQ = kSTuningCommandTQ.getKSTQ();
            double kVTQ = kVTuningCommandTQ.getkVTQ();
            double ffCurrent = kSTQ * Math.signum(currentSpeed) + kVTQ * targetSpeed; 

            motorExecute.setMotorCurrent(pidCurrent + ffCurrent);

            //Scoring 
            if(stepTimer.hasElapsed(duration)){
                double score  = cumulativeError;

                if(score < lowestScore){
                    lowestScore = score;
                    bestkD = testkD;
                }

                testkD += kDIncrement;
                stepTimer.restart();
                pidController.setD(testkD);
                pidController.setP(tunedkPTQ);
                cumulativeError = 0.0;
            }
        })
        .beforeStarting(() -> {
            double tunedKPTQ = kPTuningCommandTQ.getkPTQ();

            testkD = kDIncrement;
            pidController.reset();
            pidController.setP(tunedKPTQ);
            pidController.setD(testkD);

            lowestScore = Double.MAX_VALUE;
            cumulativeError = 0.0;

            stepTimer.restart();
        })
        .until(() -> {
            return testkD > maxkD;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                SmartDashboard.putNumber("kD Value TQFOC: ", bestkD);
            }
        });
    }

    public double getkDTQ(){
        return SmartDashboard.getNumber("kD Value TQFOC", bestkD);
    }

}

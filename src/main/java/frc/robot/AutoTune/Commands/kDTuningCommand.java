package frc.robot.AutoTune.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.security.DrbgParameters.Reseed;
import java.util.Currency;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class kDTuningCommand extends SubsystemBase{

    private MotorExecute motorExecute;
    private kSTuningCommand kSTuningCommand;
    private kVTuningCommand kVTuningCommand;
    private PIDController pidController;
    private kPTuningCommand kPTuningCommand;

    private double testkD = 0.0;
    private double bestkD = 0.0;
    private double lowestScore = Double.MAX_VALUE;

    private double cumulativeError = 0.0;

    private Timer stepTimer = new Timer();

    public double tunedkP = kPTuningCommand.getkP();

    public kDTuningCommand(MotorExecute motorExecute){
        this.motorExecute = motorExecute;
    }

    /**
     * kD Tuning Command for Free Spin 
     * This command runs multiple kDs and calculates for the optimized kD gain
     * @param kDIncrement increment for kD per loop of timer
     * @param targetSpeed target speed for the mechanism 
     * @param duration test duration for accuracy 
     * @param maxkD for jittering 
     */
    public Command kDTuningCommand(double kDIncrement, double targetSpeed, double duration, double maxkD, double gearRatio){
        return run(() -> {
            //1. Calculate for the current error of the speed
            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RadiansPerSecond);
            double absError = Math.abs(currentSpeed - targetSpeed);

            //2. Adds up the absolute errors to the cumulative error 
            cumulativeError += absError;

            //Apply PID voltage and FF voltage 
            double pidVoltage = pidController.calculate(currentSpeed, targetSpeed);

            double kS = kSTuningCommand.getKS();
            double kV = kVTuningCommand.getKV();
            double ffVoltage = kS * Math.signum(currentSpeed) + kV * targetSpeed;

            motorExecute.setMotorVoltage(pidVoltage + ffVoltage);

            //Scoring 
            if(stepTimer.hasElapsed(duration)){
                double score = cumulativeError;

                if(score < lowestScore){
                    lowestScore = score;
                    bestkD = testkD;
                }

                testkD += kDIncrement;
                stepTimer.restart();
                pidController.setD(testkD);
                pidController.setP(tunedkP);
                cumulativeError = 0.0;
            }
    })
    .beforeStarting(() -> {
         testkD = kDIncrement;
            pidController.reset();
            pidController.setD(testkD);
            pidController.setP(tunedkP);

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
            SmartDashboard.putNumber("kD Value: ", bestkD);
        }
    });
}

public double getkD(){
    return SmartDashboard.getNumber("kD Value: ", bestkD);
}
}


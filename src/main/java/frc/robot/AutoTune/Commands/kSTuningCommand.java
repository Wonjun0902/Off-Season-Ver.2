package frc.robot.AutoTune.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoTune.MotorExecute;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands; 

public class kSTuningCommand{

    /**
     * kS Tuning Command 
     * Do small increments of Voltage starting from 0V
     * But can adjust the increment 
     * As the command acts as a loop itself, we don't need to use for-loops for this tuning mechanism
     * @param voltsPerLoop for this, I will implement actual values in the combiner class for different subsystems, also may make it all the same
     */
    private double currentVolts = 0.0;
    private double tunedkS = 0.0;

    public Command kSTuningCommand(double voltsPerLoop, double gearRatio, MotorExecute motorExecute){
        //Initialize currentVolts to 0 at the start 
        currentVolts = 0.0;
        
        return Commands.run(
        () -> {
            // 1. Add the small increment (happens every 20ms)
            currentVolts += voltsPerLoop;
            
            // 2. Apply it to the motor
            motorExecute.setMotorVoltage(currentVolts);
        }
    )
    .beforeStarting(() -> {
        currentVolts = 0.0;
    })
    .until(() -> {
        double currentSpeed = Math.abs(motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond));
        return currentSpeed > 0.05;
    })
    .finallyDo((interrupted) -> {
        motorExecute.stopMotor();
        if(!interrupted){
            tunedkS = currentVolts;
            SmartDashboard.putNumber("kS Value: ", currentVolts);
        }
    });
    }

    public double getKS(){
        return SmartDashboard.getNumber("kS Value: ", tunedkS);
    }
}

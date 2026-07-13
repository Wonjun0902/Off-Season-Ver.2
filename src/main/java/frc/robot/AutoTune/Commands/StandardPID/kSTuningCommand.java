package frc.robot.AutoTune.Commands.StandardPID;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands; 

// This kS Tuning Command can be used for any type of tuning command for free spin - standard PID, MM, MMExpo as this is just a feedforwad
// But becareful to apply the volts into the right direction for position based mechanisms 
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
        if(!interrupted){
            tunedkS = currentVolts;
            SmartDashboard.putNumber("kS Value: ", currentVolts);
        }

        motorExecute.stopMotor();
    });
    }

    public double getKS(){
        return tunedkS;
    }
}

package frc.robot.AutoTune.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoTune.MotorExecute;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class kSTuningCommand extends SubsystemBase{

    private MotorExecute motorExecute;

    /**
     * kS Tuning Command 
     * Do small increments of Voltage starting from 0V
     * But can adjust the increment 
     * As the command acts as a loop itself, we don't need to use for-loops for this tuning mechanism
     * @param voltsPerLoop for this, I will implement actual values in the combiner class for different subsystems, also may make it all the same
     */
    private double currentVolts = 0.0;

    public Command kSTuningCommand(double voltsPerLoop){
    return run(
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
        double currentSpeed = Math.abs(motorExecute.getMotorSpeed().in(RotationsPerSecond));
        return currentSpeed > 0.05;
    })
    .finallyDo(() -> motorExecute.stopMotor());
    }

}

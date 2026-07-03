package frc.robot.AutoTune;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.AutoTune.MotorExecute;

public class TuningCommand extends SubsystemBase{

    /**
     * Control Commands 
     * 
     * Normal PID + MM + MMExpo
     * 1. kS tuning 
     * 2. kV tuning 
     * 3. kG tuning 
     * 4. kP tuning 
     * 5. kD tuning 
     * 6. kA tuning 
     * 
     * Torque Current FOC 
     *  Basically the same as above but in Amps
     * 1. kS tuning 
     * 2. kV tuning 
     * 3. kG tuning 
     * 4. kP tuning 
     * 5. kD tuning 
     * 6. kA tuning 
     */

    private MotorExecute motorExecute;

    //Normal PID + MM + MMExpo

    /**
     * kS Command 
     * Do small increments of Voltage starting from 0V
     * But can adjust the increment 
     * As the command acts as a loop itself, we don't need to use for-loops for this tuning mechanism
     * @param voltsPerLoop for this, I will implement actual values in the combiner class for different subsystems, also may make it all the same
     */
    private double currentVolts = 0.0;

    public Command kSTuningCommand(double voltsPerLoop){
        //Initialize the volts for 0.0 at the start
        currentVolts = 0.0; 
    return run(
        () -> {
            // 1. Add the small increment (happens every 20ms)
            currentVolts += voltsPerLoop;
            
            // 2. Apply it to the motor
            motorExecute.setMotorVoltage(currentVolts);
        }
    )
    .finallyDo(() -> motorExecute.stopMotor());
    }

    /**
     * kV Command 
     * I set a target speed and apply voltage that increments until the voltage makes the motor turn until 80% of the target speed
     * @param setVoltage
     * @param targetSpeed
     */
    public Command kVTuningCommand(double setVoltage, AngularVelocity targetSpeed){

    }
    
}

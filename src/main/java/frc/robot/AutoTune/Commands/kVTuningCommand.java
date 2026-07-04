package frc.robot.AutoTune.Commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoTune.MotorExecute;

public class kVTuningCommand extends SubsystemBase{

    private MotorExecute motorExecute;
    private kSTuningCommand kSTuningCommand;

    /**
     * kV Tuning Command 
     * I set a target speed and apply voltage that increments until the voltage makes the motor turn until 80% of the target speed
     * When the motor speed reaches 80% of the target speed the motor will stop applying the voltage 
     * Later in the analyzer class, I will get the voltage that enables the motor to reach 80% speed!
     * @param setVoltage
     * @param targetSpeed
     */
    private double currentVoltage;
    private double tunedkV;
    public Command kVTuningCommand(AngularVelocity targetSpeed, double voltsPerLoop){
        //Initialize the volts for 0.0 at the start 
        currentVoltage = 0.0;
        return run(
          () -> {
            //1. Increment Voltage
            currentVoltage += voltsPerLoop;

            //2. Apply to Motor 
            motorExecute.setMotorVoltage(currentVoltage);
          }
        )
        //Reset State before starting 
        .beforeStarting(() -> {currentVoltage = 0.0;})
        //Does this until the currentSpeed is greater or equal than the threshold, which is 80 percent of the targetSpeed
        //Basically making it stop when they are equal
        .until(() -> {
            AngularVelocity currentSpeed = motorExecute.getMotorSpeed();
            AngularVelocity threshold  = targetSpeed.times(0.87);

            return currentSpeed.gte(threshold);
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                double kS = kSTuningCommand.getKS();
                double currentSpeed = motorExecute.getMotorSpeed().in(RotationsPerSecond);
                tunedkV = (currentVoltage - kS) / (currentSpeed);

                SmartDashboard.putNumber("kV Value: ", tunedkV);
            }
        });
    }

    public double getKV(){
        return SmartDashboard.getNumber("kV Value: ", tunedkV);
    }

}

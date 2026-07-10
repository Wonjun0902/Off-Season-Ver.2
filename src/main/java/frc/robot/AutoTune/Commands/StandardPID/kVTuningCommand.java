package frc.robot.AutoTune.Commands.StandardPID;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj2.command.Commands; 

// This kS Tuning Command can be used for any type of tuning command for free spin - standard PID, MM, MMExpo as this is just a feedforwad
// But becareful to apply the volts into the right direction for position based mechanisms 
public class kVTuningCommand{

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
    public Command kVTuningCommand(AngularVelocity targetSpeed, double voltsPerLoop, double gearRatio, MotorExecute motorExecute, kSTuningCommand kSTuningCommand){
        //Initialize the volts for 0.0 at the start 
        currentVoltage = 0.0;
        return Commands.run(
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
            AngularVelocity currentSpeed = motorExecute.getMotorSpeed(gearRatio);
            AngularVelocity threshold  = targetSpeed.times(0.87);

            return currentSpeed.gte(threshold);
        })
        .finallyDo((interrupted) -> {
            if(!interrupted){
                double kS = kSTuningCommand.getKS();
                double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);
                tunedkV = (currentVoltage - kS) / (currentSpeed);

                SmartDashboard.putNumber("kV Value: ", tunedkV);
            }
            
            motorExecute.stopMotor();
        });
    }

    public double getKV(){
        return SmartDashboard.getNumber("kV Value: ", tunedkV);
    }

}

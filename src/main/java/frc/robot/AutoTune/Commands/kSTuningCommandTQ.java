package frc.robot.AutoTune.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoTune.MotorExecute;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands; 

public class kSTuningCommandTQ {

    /**
     * kS TUning Commnad for Torque Currnet FOC 
     * The logic is all the same for normal kS Tuning Command but this use Current for controling motors 
     * @param ampsPerLoop 
     * @param gearRatio 
     * @param motorexecute 
     */
    private double currentAmps = 0.0;
    private double tunedkS = 0.0;

    public Command kSTuningCommandTQ(double ampsPerLoop, double gearRatio, MotorExecute motorExecute){
        return Commands.run(() -> {
            //1. Add the small increment 
            currentAmps += ampsPerLoop;

            //2. Apply it to the motor 
            motorExecute.setMotorCurrent(currentAmps);
        })
        .beforeStarting(() -> {
            currentAmps = 0.0;
        })
        .until(() -> {
            double currentSpeed = Math.abs(motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond));
            return currentSpeed > 0.05;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                tunedkS = currentAmps;
                SmartDashboard.putNumber("kS Value TQFOC: ", currentAmps);
            }
        });
    }

    public double getKSTQ(){
        return SmartDashboard.getNumber("kS Value TQFOC", currentAmps);
    }

}

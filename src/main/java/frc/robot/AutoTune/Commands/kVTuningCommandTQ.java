package frc.robot.AutoTune.Commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj2.command.Commands; 


public class kVTuningCommandTQ {

    /**
     * kV Tuning Command for Torque Current FOC Controls 
     * The logic is the same 
     * @param targetSpeed
     * @param ampsPerLoop 
     * @param gearRatio 
     * @param motorExecute 
     * @param kSTuningCommandTQ
     */
    private double currentAmps;
    private double tunedkVTQ;
    public Command kVTuningCommandTQ(AngularVelocity targetSpeed, double ampsPerLoop, double gearRatio, MotorExecute motorExecute, kSTuningCommandTQ kSTuningCommandTQ){
        //1. Initialize the amps for 0.0 at the start 
        currentAmps = 0.0;

        return Commands.run(() -> {
            //1. Increment Current 
            currentAmps += ampsPerLoop;

            //2. Apply to Motor 
            motorExecute.setMotorCurrent(currentAmps);
        })
        .beforeStarting(() -> currentAmps = 0.0)
        .until(() -> {
            AngularVelocity currentSpeed = motorExecute.getMotorSpeed(gearRatio);
            AngularVelocity threshold = targetSpeed.times(0.87);

            return currentSpeed.gte(threshold);
        }).finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                double kSTQ = kSTuningCommandTQ.getKSTQ();
                double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);
                tunedkVTQ = (currentAmps - kSTQ) / currentSpeed;

                SmartDashboard.putNumber("kS Value TQFOC: ", tunedkVTQ);
            }
        });
    }

    public double getkVTQ(){
        return SmartDashboard.getNumber("kS Value TQFOC", tunedkVTQ);
    }

}

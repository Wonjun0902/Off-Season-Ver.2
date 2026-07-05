package frc.robot.AutoTune.Commands;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoTune.MotorExecute;

public class ArmKGTuningCommand extends SubsystemBase{

    private MotorExecute motorExecute;
    private kSTuningCommand kSTuningCommand;
    private double currentVoltage = 0.0;
    private double tunedkG = 0.0;

    /**
     * Same as the elevator tuning command, I want to have a kG tuning coammnd for an arm mechanism 
     * @param setUpVolts
     * @param setUpPosition 
     * @param voltsPerLoop
     */
    public Command armKGTuningCommand(double setUpVolts, double setUpPosition, double voltsPerLoop, double gearRatio){

        Command setUpArm= run(() -> {
            motorExecute.setMotorVoltage(setUpVolts);
        })
        .until(() -> {
            double position = motorExecute.getMotorPosition(gearRatio).in(Rotation);
            return position >= setUpPosition;
        });

        Command getKGCommand = run(() -> {
            currentVoltage += voltsPerLoop;
            motorExecute.setMotorVoltage(currentVoltage);
        })
        .beforeStarting(() -> {
            currentVoltage = 0.0;
        })
        .until(() -> {
            double armSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);
            return armSpeed >= 0.0;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                double kS = kSTuningCommand.getKS();
                tunedkG = currentVoltage - kS;

                SmartDashboard.putNumber("kG Value: ", tunedkG);
            }
        });

        return setUpArm.andThen(getKGCommand);
    }

    public double getKG(){
        return tunedkG;
    }

}

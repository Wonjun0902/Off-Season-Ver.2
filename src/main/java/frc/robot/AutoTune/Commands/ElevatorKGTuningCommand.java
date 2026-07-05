package frc.robot.AutoTune.Commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoTune.MotorExecute;

public class ElevatorKGTuningCommand extends SubsystemBase{

    private MotorExecute motorExecute;
    private kSTuningCommand kSTuningCommand;
    private double currentVoltage = 0.0;
    public double tunedkG = 0.0;
    /**
     * A kG Tuning Command for an elevator subsystem 
     * First, I move the elevator to a safe position, a point in the middle or higher with a certain voltage(param setUpVolts)
     * If there is no kG, the elevator will start to fall. 
     * I will give the motor a certain voltage starting from 0. And stop supplying the voltage until it stops 
     * @param setUpVolts to move the elevator up to a certain position 
     * @param voltsPerLoop for increments for the voltage to make the elevator stands still. 
     */
    public Command elevatorKGTuningCommand(double setUpVolts, double setUpPosition, double voltsPerLoop, double gearRatio){

        //Set up the elevator to be in a certain position 
        Command setUpElevator = run(() -> {
            motorExecute.setMotorVoltagePOS(setUpVolts);
        })
        //Stop when the motor position is gte than the setUpPosition or just when it is equal to the setUpPosition 
        .until(() -> {
           double motorPosition = motorExecute.getMotorPosition(gearRatio).in(Rotations);

           // This is in rotations!!
           return motorPosition >= setUpPosition;
        });

        //Set up the voltage until it stands still in the position 
        Command getKGCommand = run(() -> {
            //Add increments to the voltage 
            currentVoltage += voltsPerLoop;

            //Apply the voltage to the motor 
            motorExecute.setMotorVoltage(currentVoltage);
        })
        .beforeStarting(() -> currentVoltage = 0.0)
        .until(() -> {
            double elevatorSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);
            return elevatorSpeed >= 0; //Change when the direction of it is different!!!!
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                double kS = kSTuningCommand.getKS();
                tunedkG = currentVoltage - kS;

                SmartDashboard.putNumber("kG Value: ", tunedkG);
            }
        });

        return setUpElevator.andThen(getKGCommand);
    }

    public double getKG(){
        return SmartDashboard.getNumber("kG Value: ", tunedkG);
    }
}

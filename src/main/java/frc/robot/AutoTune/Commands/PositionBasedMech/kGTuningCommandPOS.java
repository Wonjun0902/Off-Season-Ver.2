package frc.robot.AutoTune.Commands.PositionBasedMech;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.AutoTune.MotorExecute;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands; 

//It is very important that you put the right direction of the motor -> positive or negative direction
//If you don't consider that, the motor will likely stop due to the safety check of the setMotorVoltagePOS method
public class kGTuningCommandPOS{

    private double currentVolts;
    public double tunedkG = 0.0;
    private double risingSign;
    private boolean isOvershoot;
    private Timer riseTimer = new Timer();
    private Timer rampTimer = new Timer();
    /**
     * A kG Tuning Command for an elevator subsystem 
     * It will be a loop where the code finds the best kG that makes the mechansim hover without falling down. 
     * For the elevator class, we don't really need to have a specific position that we need to initially set it up to
     * but for Arm mechs, we need to place the mechanism to be a place where there is maximum gravitational force - horizontal position
     * @param setUpVolts to move the elevator up to a certain position 
     * @param voltsPerLoop for increments for the voltage to make the elevator stands still. 
     */
    public Command riseCommand(double setUpVolts, double voltsPerLoop, double gearRatio, MotorExecute motorExecute){
        return Commands.run(() -> {
            //Apply both setUpVoltage and currentVolts(incrementing volts)
            motorExecute.setMotorVoltagePOS(currentVolts + setUpVolts);

            //Check direction of the motor 
            AngularVelocity risingSpeedinRotPerSec = motorExecute.getMotorSpeed(gearRatio);
            double risingSpeed = risingSpeedinRotPerSec.in(RotationsPerSecond);
            risingSign = Math.signum(risingSpeed);
        })
        .until(() -> {
            return riseTimer.hasElapsed(0.5);
        })
        .beforeStarting(() -> {
            currentVolts = voltsPerLoop;
            riseTimer.restart();
        });
    }

    public Command stayCommand(double voltsPerSec, double gearRatio, double minimumVolts, MotorExecute motorExecute){
        return Commands.run(() -> {
            //Add increments to current voltage
            currentVolts = minimumVolts + (voltsPerSec*rampTimer.get());

            //Apply only incrementing voltage to the motor 
            motorExecute.setMotorVoltagePOS(currentVolts);

            //Get the descending speed 
            AngularVelocity motorSpeedinRotPerSec = motorExecute.getMotorSpeed(gearRatio);
            double motorSpeed = motorSpeedinRotPerSec.in(RotationsPerSecond);
            double motorSpeedMagnitude = Math.abs(motorSpeed);
            double sign = Math.signum(motorSpeed);

            if(rampTimer.hasElapsed(0.5)){
                 if(risingSign == sign || motorSpeedMagnitude < 0.03){
                isOvershoot = true;
                }
                else{
                isOvershoot = false;
                }
            }
        })
        .beforeStarting(() -> {
            currentVolts = minimumVolts;
            rampTimer.restart();
            isOvershoot = false;
        })
        .until(() -> {
            return isOvershoot;
        })
        .finallyDo((interrupted) -> {
            if(!interrupted){
                tunedkG = currentVolts;
                SmartDashboard.putNumber("kG Value: ", tunedkG);
            }
            motorExecute.stopMotor();
        });
    }

    public double getKG(){
        return tunedkG;
    }
}

package frc.robot.AutoTune.Commands.PositionBasedMech;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
    private Timer stepTimer = new Timer();
    /**
     * A kG Tuning Command for an elevator subsystem 
     * It will be a loop where the code finds the best kG that makes the mechansim hover without falling down. 
     * For the elevator class, we don't really need to have a specific position that we need to initially set it up to
     * but for Arm mechs, we need to place the mechanism to be a place where there is maximum gravitational force - horizontal position
     * @param setUpVolts to move the elevator up to a certain position 
     * @param voltsPerLoop for increments for the voltage to make the elevator stands still. 
     */
    public Command elevatorKGTuningCommand(double setUpVolts, double voltsPerLoop, double gearRatio, double duration, MotorExecute motorExecute){
        return Commands.run(() ->{ 
            //1. Start Adding Increment volts to a certain position -> depends on the mechanism 
            motorExecute.setMotorVoltagePOS(currentVolts + setUpVolts);

            //Check status of the motor 
            double currentPos = motorExecute.getMotorPosition(gearRatio).magnitude();
            double currentSpeed = 

            //Check status after timer is over
            if(stepTimer.hasElapsed(duration)){

            }

        })
        .beforeStarting(() -> {
            currentVolts = voltsPerLoop;
            stepTimer.restart();
        });
    }

    public double getKG(){
        return SmartDashboard.getNumber("kG Value: ", tunedkG);
    }
}

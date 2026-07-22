package frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagicExpo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.PositionBasedMech.kGTuningCommandPOS;
import frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic.MMCruiseVTuningCommand;
import frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic.MMMaxAccTuningCommand;
import frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic.MMkATuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class MMExpokPTuningCommand {

    //Define States 
    private enum State{
        INIT_HIGH, MOVING_HIGH, INIT_LOW, MOVING_LOW, EVALUATING
    }

    //Class Variables to be changed inside the loop
    private State currentState = State.INIT_HIGH;
    private double testkP = 0.0; //Change later for optimized value
    private double bestkP = 0.0;
    private double lowestScore = Double.MAX_VALUE;
    private double cumulativeErrorPOS = 0.0;
    private double cumulativeErrorVEL = 0.0;
    private double tunedkS, tunedkV, tunedkA, tunedkG, tunedExpokA, tunedExpokV, tunedkD;

    private Timer timeOutTimer = new Timer();

     public Command mmExpokPTuningCommand(double kPIncrement, double lowTarget, double highTarget, double maxkP, double gearRatio,
        GravityTypeValue gravityType, MotorExecute motorExecute,
        MMkATuningCommand mMkATuningCommand,
        kSTuningCommand kSTuningCommand,
        kVTuningCommand kVTuningCommand,
        kGTuningCommandPOS kGTuningCommandPOS){
            return Commands.run(() -> {
            switch(currentState){
                case INIT_HIGH: 
                    motorExecute.configureMotionMagicExpo(tunedkS, tunedkV, tunedkA, tunedkG, testkP, tunedkD, tunedExpokA, tunedExpokV, gravityType);
                    motorExecute.setMMExpoTarget(Rotations.of(highTarget));
                    cumulativeErrorPOS = 0.0;
                    cumulativeErrorVEL = 0.0;

                    timeOutTimer.restart();
                    currentState = State.MOVING_HIGH;
                    break;
                
                case MOVING_HIGH:
                    accumulateError(motorExecute, gearRatio);
                    if(isAtTarget(motorExecute, highTarget, gearRatio) || timeOutTimer.hasElapsed(3.0)){ //Chnage duration if needed
                        currentState = State.INIT_LOW;
                    }
                    break;
                
                case INIT_LOW:
                    motorExecute.setMMExpoTarget(Rotations.of(lowTarget));
                    timeOutTimer.restart();
                    currentState = State.MOVING_LOW;
                    break;
                
                case MOVING_LOW:
                    accumulateError(motorExecute, gearRatio);
                    if(isAtTarget(motorExecute, lowTarget, gearRatio) || timeOutTimer.hasElapsed(3.0)){ //Change duration if needed
                        currentState = State.EVALUATING;
                    }
                    break;

                case EVALUATING:
                    double totalScore = cumulativeErrorPOS + cumulativeErrorVEL;
                    if(totalScore < lowestScore){
                        lowestScore = totalScore;

                        bestkP = testkP;
                    }
                    testkP += kPIncrement;
                    currentState = State.INIT_HIGH;
                    break;
            }
        })
        .beforeStarting(() -> {
            tunedkS = kSTuningCommand.getKS();
            tunedkV = kVTuningCommand.getKV();
            tunedkA = mMkATuningCommand.getKA();
            tunedkG = kGTuningCommandPOS.getKG();
            tunedkD = 0.0;
            tunedExpokA = mMkATuningCommand.getKA();  
            tunedExpokV = kVTuningCommand.getKV(); 

            testkP = kPIncrement;
            bestkP = 0.0;
            lowestScore = Double.MAX_VALUE;
            currentState = State.INIT_HIGH;
        })
        .until(() -> {
            return testkP > maxkP;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();
            if(!interrupted){
                SmartDashboard.putNumber("MM kP Value: ", bestkP);
            }
        });
        }

    public void accumulateError(MotorExecute motorExecute, double gearRatio){
        double referencePOS = motorExecute.getReferencePosition(gearRatio).in(Rotations);
        double actualPOS = motorExecute.getMotorPosition(gearRatio).in(Rotations);

        double referenceSpeed = motorExecute.getReferenceSpeed(gearRatio).in(RotationsPerSecond);
        double actualSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);
        
        cumulativeErrorPOS += Math.abs(referencePOS - actualPOS);
        cumulativeErrorVEL += Math.abs(referenceSpeed - actualSpeed);
    }

    private boolean isAtTarget(MotorExecute motorExecute, double target, double gearRatio){
        double currentPOS = motorExecute.getMotorPosition(gearRatio).in(Rotations);
        double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);

        return Math.abs(target - currentPOS) < 0.05 && Math.abs(currentSpeed) < 0.05;
    }

    public double getMMkP(){
        return bestkP;
    }
}

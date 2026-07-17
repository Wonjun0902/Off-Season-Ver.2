package frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.PositionBasedMech.kGTuningCommandPOS;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.reflect.GenericArrayType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.signals.GravityTypeValue;

//Class for caculating the kP Value for MM

//It is very important that you put the right direction of the motor -> positive or negative direction
//If you don't consider that, the motor will likely stop due to the safety check of the setMotorVoltagePOS method

//Motion Magic follows a Trapezoidal Trajectory that is gerenated before the motor runs its position/velocity profile
//The main point of kP in Motion Magic will be: How close does it follows the generated trapezoidal trajectory
//As there is built-in method for getting the motion magic kP, kD, kI in CTRE, we don't need ot build the entire mapping logic 

//The only thing we need to do is input the cruise velocity and max acceleration so that the built in code can follow   
public class MMkPTuningCommand {

    //Define States 
    private enum State{
        INIT_HIGH, MOVING_HIGH, INIT_LOW, MOVING_LOW, EVALUATING
    }

    //Class Variables to be changed inside the loop
    private State currentState = State.INIT_HIGH;
    private double testkP = 0.0;
    private double bestkP = 0.0;
    private double lowestScorePOS = Double.MAX_VALUE;
    private double lowestScoreVEL = Double.MAX_VALUE;
    private double cumulativeErrorPOS = 0.0;
    private double cumulativeErrorVEL = 0.0;
    private double tunedkS, tunedkV, tunedkA, tunedkG, tunedCruiseV, tunedMaxAcc;

    private Timer timeOutTimer = new Timer();

    public Command mmkPTuningCommand(double kPIncrement, double lowTarget, double highTarget, double maxkP, double gearRatio,
        GravityTypeValue gravityType, MotorExecute motorExecute,
        MMCruiseVTuningCommand mmCruiseVTuningCommand,
        MMMaxAccTuningCommand mmMaxAccTuningCommand,
        MMkATuningCommand mMkATuningCommand,
        kSTuningCommand kSTuningCommand,
        kVTuningCommand kVTuningCommand,
        kGTuningCommandPOS kGTuningCommandPOS){
        
        return Commands.run(() -> {
            switch(currentState){
                case INIT_HIGH: 
                    motorExecute.configureMotionMagic(tunedkS, tunedkV, tunedkA, tunedkG, testkP, tunedCruiseV, tunedMaxAcc, gravityType);
                    motorExecute.setMMPositionTarget(Rotations.of(highTarget));

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
                    motorExecute.setMMPositionTarget(Rotations.of(lowTarget));
                    timeOutTimer.restart();
                    currentState = State.MOVING_LOW;
                    break;
                
                case MOVING_LOW:
                    accumulateError(motorExecute, gearRatio);
                    if(isAtTarget(motorExecute, lowTarget, gearRatio) || timeOutTimer.hasElapsed(3.0)){ //Change duration if needed
                        currentState = State.EVALUATING;
                    }

                case EVALUATING:
                    if(cumulativeErrorPOS < lowestScorePOS && cumulativeErrorVEL < lowestScoreVEL){
                        lowestScorePOS = cumulativeErrorPOS;
                        lowestScoreVEL = cumulativeErrorVEL;

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
            tunedCruiseV = mmCruiseVTuningCommand.getCruiseVelocity();
            tunedMaxAcc = mmMaxAccTuningCommand.getMaxAcc();   

            testkP = kPIncrement;
            bestkP = 0.0;
            lowestScorePOS = Double.MAX_VALUE;
            lowestScoreVEL = Double.MAX_VALUE;
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

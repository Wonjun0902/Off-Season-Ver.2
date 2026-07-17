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

public class MMkDTuningCommand {

    //Define States
    private enum State{
        INIT_HIGH, MOVING_HIGH, INIT_LOW, MOVING_LOW, EVALUATING
    }

    //Class Variables to be changed inside the loop 
    private State currentState = State.INIT_HIGH;
    private double testkD = 0.0;
    private double bestkD = 0.0;
    private double lowestScorePOS = Double.MAX_VALUE;
    private double lowestScoreVEL = Double.MAX_VALUE;
    private double cumulativeErrorPOS = 0.0;
    private double cumulativeErrorVEL = 0.0;
    private double tunedkS, tunedkV, tunedkA, tunedkG, tunedCruiseV, tunedMaxAcc, tunedkP;

    private Timer timeOutTimer = new Timer();

    public Command mmkDTuningCommand(double kDIncrement, double lowTarget, double highTarget, double maxkD, double gearRatio,
        GravityTypeValue gravityType, MotorExecute motorExecute,
        MMCruiseVTuningCommand mmCruiseVTuningCommand,
        MMMaxAccTuningCommand mmMaxAccTuningCommand,
        MMkATuningCommand mMkATuningCommand,
        kSTuningCommand kSTuningCommand,
        kVTuningCommand kVTuningCommand,
        kGTuningCommandPOS kGTuningCommandPOS, 
        MMkPTuningCommand mMkPTuningCommand){

        return Commands.run(() -> {
            switch(currentState){
                case INIT_HIGH:
                    motorExecute.configureMotionMagic(tunedkS, tunedkV, tunedkA, tunedkG, tunedkP, testkD, tunedCruiseV, tunedMaxAcc, gravityType);
                    motorExecute.setMMPositionTarget(Rotations.of(highTarget));
                    currentState = State.MOVING_HIGH;
                    break;

                case MOVING_HIGH:
                    cumulateError(motorExecute, gearRatio);
                    if(isReachedTarget(motorExecute, gearRatio, highTarget)){
                        currentState = State.INIT_LOW;
                    }
                    break;

                case INIT_LOW:
                    motorExecute.setMMPositionTarget(Rotations.of(lowTarget));
                    currentState = State.MOVING_LOW;
                    break;

                case MOVING_LOW:
                    cumulateError(motorExecute, gearRatio);
                    if(isReachedTarget(motorExecute, gearRatio, lowTarget)){
                        currentState = State.EVALUATING;
                    }
                    break;

                case EVALUATING:
                    if(cumulativeErrorPOS < lowestScorePOS && cumulativeErrorVEL < lowestScoreVEL){
                        lowestScorePOS = cumulativeErrorPOS;
                        lowestScoreVEL = cumulativeErrorVEL;

                        bestkD = testkD;
                    }
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
            tunedkP = mMkPTuningCommand.getMMkP();

            testkD = kDIncrement;
            bestkD = 0.0;
            lowestScorePOS = Double.MAX_VALUE;
            lowestScoreVEL = Double.MAX_VALUE;
            currentState = State.INIT_HIGH;
        })
        .until(() -> {
            return testkD > maxkD;
        })
        .finallyDo((interrupted) -> {
            motorExecute.stopMotor();

            if(!interrupted){
                SmartDashboard.putNumber("MM kD Value: ", bestkD);
            }
        });

    }

    public void cumulateError(MotorExecute motorExecute, double gearRatio){
        double referencePOS = motorExecute.getReferencePosition(gearRatio).in(Rotations);
        double actualPOS = motorExecute.getMotorPosition(gearRatio).in(Rotations);

        double referenceSpeed = motorExecute.getReferenceSpeed(gearRatio).in(RotationsPerSecond);
        double actualSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);

        cumulativeErrorPOS += Math.abs(referencePOS - actualPOS);
        cumulativeErrorVEL += Math.abs(referenceSpeed - actualSpeed);
    }

    public boolean isReachedTarget(MotorExecute motorExecute, double gearRatio, double target){
        double currentPOS = motorExecute.getMotorPosition(gearRatio).in(Rotations);
        double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);

        return Math.abs(target - currentPOS) < 0.05 && Math.abs(currentSpeed) < 0.05;
    }

    public double getMMkD(){
        return bestkD;
    }

}

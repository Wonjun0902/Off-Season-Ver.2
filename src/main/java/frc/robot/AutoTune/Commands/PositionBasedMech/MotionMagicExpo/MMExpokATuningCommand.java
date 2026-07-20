package frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagicExpo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.PositionBasedMech.kGTuningCommandPOS;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;
import frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic.MMkATuningCommand; 

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

//A Class for tuning Expo kA for MMExpo 
//According to CTRE, motion magic expo is theoretically the same as normal kA but used differently. 
//It is used to get the volts(output for normal) to acquire a certain acceleration
//But because we want our trajectory to be smooth, we are going to adjust the normal kA value that our trajectory is crazy bumpy as high low kA means sudden acc
public class MMExpokATuningCommand {

    private enum State{
        INIT_LOW, MOVING_LOW, INIT_HIGH, MOVING_HIGH, EVALUATING
    }

    private State currentState = State.INIT_HIGH;
    private double testExpokA;
    private double bestExpokA;
    private double tunedkS, tunedkV, tunedkA, tunedkG, tunedkP, tunedkD;
    private double lowestScore;

    private Timer timeOutTimer = new Timer();

    private double previousAcc = 0.0;
    private double maxJerk = 0.0;

    public Command mmExpokATuningCommand(double defaultVolts, double lowTarget, double highTarget, double gearRatio, 
        GravityTypeValue gravityType, MotorExecute motorExecute,   
        MMkATuningCommand mMkATuningCommand,
        kSTuningCommand kSTuningCommand,
        kVTuningCommand kVTuningCommand,
        kGTuningCommandPOS kGTuningCommandPOS){

        return Commands.run(() -> {
            switch(currentState){
                case INIT_HIGH:
                    motorExecute.configureMotionMagicExpo(tunedkS, tunedkV, tunedkA, tunedkG, tunedkP, tunedkD, testExpokA,  tunedkV, gravityType);
                    motorExecute.setMMExpoTarget(Rotations.of(highTarget));

                    timeOutTimer.restart();
                    currentState = State.MOVING_HIGH;
                    break;

                case MOVING_HIGH:
                    getJerk(motorExecute, gearRatio);
                    

            }
        })
        .beforeStarting(() -> {
            tunedkS = kSTuningCommand.getKS();
            tunedkV = kVTuningCommand.getKV();
            tunedkA = mMkATuningCommand.getKA();
            tunedkG = kGTuningCommandPOS.getKG();
            tunedkD = 0.0;
            tunedkP = 0.0;

            testExpokA = tunedkA;
            bestExpokA = tunedkA;

            lowestScore = 0.0;
            
            currentState = State.INIT_HIGH;
        });
    }

    public void getJerk(MotorExecute motorExecute, double gearRatio){
        double currentAcc = motorExecute.getAcceleration(gearRatio).in(RotationsPerSecondPerSecond);
        double currentJerk = Math.abs(currentAcc - previousAcc) / 0.02;

        if(currentJerk > maxJerk){
            maxJerk = currentJerk;
        }

        previousAcc = currentAcc;
    }
}

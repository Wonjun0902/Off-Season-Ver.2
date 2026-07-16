package frc.robot.AutoTune.Commands.PositionBasedMech.MotionMagic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTune.MotorExecute;
import frc.robot.AutoTune.Commands.PositionBasedMech.kGTuningCommandPOS;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

//Class for caculating the kP Value for MM

//It is very important that you put the right direction of the motor -> positive or negative direction
//If you don't consider that, the motor will likely stop due to the safety check of the setMotorVoltagePOS method

//Motion Magic follows a Trapezoidal Trajectory that is gerenated before the motor runs its position/velocity profile
//The main point of kP in Motion Magic will be: How close does it follows the generated trapezoidal trajectory
//As there is built-in method for getting the motion magic kP, kD, kI in CTRE, we don't need ot build the entire mapping logic 

//The only thing we need to do is input the cruise velocity and max acceleration so that the built in code can follow   
public class MMkPTuningCommand {

    private double testkP = 0.0;
    private double bestkP = 0.0;
    private double lowestScore = 0.0;

    private double cumulativeErrorPos = 0.0;
    private double cumulativeErrorSpeed = 0.0;
    private double maxWindowError = 0.0;

    private boolean isreachedTarget = false;

    private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    /**
     * kP Tuning Command for MM - Position Mech 
     * This Command will map out the trajectory using the motion magic voltage object 
     * Test with a few kP values and return the best kP gain by scoring (score by errors)
     * @param kPIncrement
     * @param targetPosition 
     * @param duration 
     * @param maxkP
     */
    public Command mmkPTuningCommand(double kPIncrement, double highTarget, double lowTarget, double maxkP, double gearRatio, GravityTypeValue gravityTypeValue, MotorExecute motorExecute, MMCruiseVTuningCommand mmCruiseVTuningCommand, MMMaxAccTuningCommand mmMaxAccTuningCommand, MMkATuningCommand mMkATuningCommand, kSTuningCommand kSTuningCommand, kVTuningCommand kVTuningCommand, kGTuningCommandPOS kGTuningCommandPOS){
        return Commands.run(() -> {
            
            //1. Apply Config
            double tunedkS = kSTuningCommand.getKS();
            double tunedkV = kVTuningCommand.getKV();
            double tunedkA = mMkATuningCommand.getKA();
            double tunedMaxAcc = mmMaxAccTuningCommand.getMaxAcc();
            double tunedCruiseV = mmCruiseVTuningCommand.getCruiseVelocity();
            double tunedkG = kGTuningCommandPOS.getKG();

            config.Slot0.kS = tunedkS;
            config.Slot0.kV = tunedkV;
            config.Slot0.kA = tunedkA;
            config.Slot0.kG = tunedkG;
            config.Slot0.GravityType = gravityTypeValue;

            config.Slot0.kP = testkP;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            config.MotionMagic.MotionMagicAcceleration = tunedMaxAcc;
            config.MotionMagic.MotionMagicCruiseVelocity = tunedCruiseV;

            motorExecute.configureMotionMagic(tunedkS, tunedkV, tunedkA, tunedkG, maxkP, tunedCruiseV, tunedMaxAcc, gravityTypeValue);

            motorExecute.setMMPositionTarget(Rotations.of(lowTarget));

            //2. Calculate error - Position and Speed
            double currentPos = motorExecute.getMotorPosition(gearRatio).in(Rotations);
            double referencePos = motorExecute.getReferencePosition(gearRatio).in(Rotations);
            double errorPos = Math.abs(referencePos- currentPos);

            double currentSpeed = motorExecute.getMotorSpeed(gearRatio).in(RotationsPerSecond);
            double referenceSpeed = motorExecute.getReferenceSpeed(gearRatio).in(RotationsPerSecond);
            double errorSpeed = Math.abs(referenceSpeed - currentSpeed);

            cumulativeErrorPos += errorPos;
            cumulativeErrorSpeed += errorSpeed;

            //3. Overshooting Check
            double absCurrentPos = Math.abs(currentPos);
            double absTargetPos = Math.abs(referencePos);
            if(absCurrentPos >= absTargetPos){
                isreachedTarget = true;
            }
        })
        .andThen(() -> {
            motorExecute.setMMPositionTarget(Rotations.of(highTarget));
        })
        .andThen(() -> {
            motorExecute.setMMPositionTarget(Rotations.of(highTarget));

            //4. Scoring 
            if(errorPos < 0.05 && errorSpeed < 0.01){
                double finalWeight = 5;

                if(!isreachedTarget){
                    finalWeight = Double.MAX_VALUE;
                }

                double score = (cumulativeErrorPos + cumulativeErrorSpeed)*finalWeight;

                if(score < lowestScore){
                    lowestScore = score;
                    bestkP = testkP;
                }
            }
        });
    }

}

//NOTE FOR MYSELF
//Config가 하나의 커맨드에서만 활성화되는지 알아보고, 안되면 Class Varaible로 놓기
//에러 계산도 에러를 Class Variable에서 계속 쌓기 -> final loop에서 cumulative error 측정후 점수 매기기 

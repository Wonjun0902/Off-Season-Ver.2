package frc.robot.AutoTune;

import frc.robot.AutoTune.HardWare.TunableMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class MotorExecute {

    private final TunableMotor m_motor;

    //For position mechanism subsystems
    private final Angle m_forwardSoftLimit;
    private final Angle m_reverseSoftLimit;
    private final DigitalInput m_hardLimitSwitch;

    public MotorExecute(TunableMotor motor, Angle forwardLimit, Angle reverseLimit, DigitalInput hardLimitSwitch){
        this.m_motor = motor;
        this.m_forwardSoftLimit = forwardLimit;
        this.m_reverseSoftLimit = reverseLimit;
        this.m_hardLimitSwitch = hardLimitSwitch;
    }

    public MotorExecute(TunableMotor motor){
        this(motor, Rotations.of(Double.POSITIVE_INFINITY), Rotations.of(Double.NEGATIVE_INFINITY), (DigitalInput) null);
    }

    /**
     * I'll have all the motor controls to be new here, seperate from the TunableMotor Classes 
     * I want to do that because I want to have all motor controls to have forward and reverse soft limits
     * So there will be new methods that consider the mechanism of the motor: free spin or position mechanism 
     */

    // Voltage Control for Position Mechanism Subsystems
    public void setMotorVoltagePOS(double volts){
        //1. Check if the hard limit switch is on
        if(m_hardLimitSwitch != null && m_hardLimitSwitch.get()){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond limits: ", "HARD LIMIT");
            throw new IllegalStateException("Motor Stopped: over hard limit");
        }
        //2. Gets the current motor position 
        Angle currentPos = m_motor.getMotorPosition();

        //3. Check if the motor is beyond the soft limits 
        if(volts > 0 && currentPos.gt(m_forwardSoftLimit)){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond the limits: ", "forward soft limit");
            return;
        }

        if(volts < 0 && currentPos.lt(m_reverseSoftLimit)){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond the limits: ", "reverse soft limit");
            return;
        }

        //4. Run the motor with given voltage
        m_motor.setMotorVoltage(volts);
    }

    //Current Controls for Position Mechanism Subsystems
    public void setMotorCurrentPOS(double current){
        //1. Check if the hard limit switch is on
        if(m_hardLimitSwitch != null && m_hardLimitSwitch.get()){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond limits: ", "HARD LIMIT");
            throw new IllegalStateException("Motor Stopped: over hard limit");
        }
        //2. Gets the current motor position 
        Angle currentPos = m_motor.getMotorPosition();

        //3. Check if the motor is beyond the soft limits 
        if(current > 0 && currentPos.gt(m_forwardSoftLimit)){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond the limits: ", "forward soft limit");
            return;
        }

        if(current < 0 && currentPos.lt(m_reverseSoftLimit)){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond the limits: ", "reverse soft limit");
            return;
        }

        //4. Run the motor with given voltage
        m_motor.setMotorCurrent(current);
    }

    //For non-Position mech subsystems
    public void setMotorCurrent(double current){
        m_motor.setMotorCurrent(current);
    }

    //For non-position mech subsystems
    public void setMotorVoltage(double volts){
        m_motor.setMotorVoltage(volts);
    }

    //Added Gear Ratio
    public AngularVelocity getMotorSpeed(double gearRatio){
        AngularVelocity currentVelocity = m_motor.getMotorSpeed();
        return currentVelocity.div(gearRatio);
    }

    //Added Gear Ratio
    public Angle getMotorPosition(double gearRatio){
        Angle currentPosition = m_motor.getMotorPosition();
        return currentPosition.div(gearRatio);
    }

    //Added Gear Ratio 
    public AngularAcceleration getAcceleration(double gearRatio){
        AngularAcceleration currentAcc = m_motor.getAcceleration();
        return currentAcc.div(gearRatio);
    }

    //the same 
    public Current getCurrent(){
        return m_motor.getCurrent();
    }

    //the same 
    public Voltage getVoltage(){
        return m_motor.getVoltage();
    }

    //the same 
    public void stopMotor(){
        m_motor.stopMotor();
    }

    public void configureMotionMagic(double kS, double kV, double kA, double kG, double kP, double cruiseV, double maxAcc, GravityTypeValue gravityTypeValue){
        m_motor.configureMotionMagic(kS,kV, kA, kG, kP, cruiseV, maxAcc, gravityTypeValue);
    }

    public Angle getReferencePosition(double gearRatio){
        Angle currentRef = m_motor.getReferencePosition();
        return currentRef.div(gearRatio);
    }

    public AngularVelocity gerReferenceSpeed(double gearRatio){
        AngularVelocity currentRef = m_motor.getReferenceSpeed();
        return currentRef;
    }

    public void setMMPositionTarget(Angle target){
        m_motor.setMMPositionTarget(target, 0);
    }

    public void setMMExpoTarget(Angle target){
        m_motor.setMMExpoTarget(target, 0);
    }
}
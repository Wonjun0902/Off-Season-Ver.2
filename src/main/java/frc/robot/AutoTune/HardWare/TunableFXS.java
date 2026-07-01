package frc.robot.AutoTune.HardWare;

import frc.lib.LazyFXS;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TunableFXS implements TunableMotor{

    private LazyFXS m_motor;

    public TunableFXS(LazyFXS motor){
        this.m_motor = motor;
    }

    @Override
    public void setMotorVoltage(double volts){
        m_motor.setVoltageControl(Volts.of(volts));
    }

    @Override 
    public void setMotorCurrent(double current){
        m_motor.setCurrentControl(Amps.of(current));
    }   

    @Override 
    public void stopMotor(){
        m_motor.stop();
    }

    @Override
    public AngularVelocity getMotorSpeed(){
        return m_motor.getVelocity();
    }

    @Override 
    public Angle getMotorPosition(){
        return m_motor.getPosition();
    }

    @Override
    public AngularAcceleration getAcceleration(){
        return m_motor.getAcceleration();
    }

    @Override
    public Current getCurrent(){
        return m_motor.getStatorCurrent();
    }

    @Override 
    public Voltage getVoltage(){
        return m_motor.getStatorVoltage();
    }
}

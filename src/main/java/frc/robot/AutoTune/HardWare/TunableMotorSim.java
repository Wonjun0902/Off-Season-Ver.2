package frc.robot.AutoTune.HardWare;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public class TunableMotorSim implements TunableMotor{

    private DCMotorSim simMotor;

    public TunableMotorSim(){}

    @Override
    public void setMotorVoltage(double volts){
        simMotor.setInputVoltage(volts);
    }

    @Override 
    public void setMotorCurrent(double current){
        //Get the current speed of the motor
        double angularVel = simMotor.getAngularVelocityRadPerSec();

        //calculates for the target torque 
        double targetTorque = simMotor.getGearbox().getTorque(current);

        //gets the needed voltage for the current angular velocity and the torque created by the current 
        double neededVoltage = simMotor.getGearbox().getVoltage(targetTorque, angularVel);

        //Applies the needed voltage
        simMotor.setInputVoltage(neededVoltage);
    }

    @Override 
    public void stopMotor(){
        simMotor.setInputVoltage(0);
    }

    @Override
    public AngularVelocity getMotorSpeed(){
        return simMotor.getAngularVelocity();
    }

    @Override 
    public Angle getMotorPosition(){
        return simMotor.getAngularPosition();
    }

    @Override 
    public AngularAcceleration getAcceleration(){
        return simMotor.getAngularAcceleration();
    }

    @Override 
    public Current getCurrent(){
        return Amps.of(simMotor.getCurrentDrawAmps());
    }

    @Override 
    public Voltage getVoltage(){
        return Volts.of(simMotor.getInputVoltage());
    }

}

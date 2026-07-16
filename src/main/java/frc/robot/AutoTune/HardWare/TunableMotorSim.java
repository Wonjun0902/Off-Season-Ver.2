package frc.robot.AutoTune.HardWare;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class TunableMotorSim implements TunableMotor{
    private DCMotor gearbox;
    private DCMotorSim simMotor;

    public TunableMotorSim(int motorID, CANBus canBus, double gearRatio){
        simMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.1, 1), gearbox);
    }
    
    public TunableMotorSim(int motorID, double gearRatio) {
    this(motorID, new com.ctre.phoenix6.CANBus(), gearRatio); 
    }

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

    @Override 
    public void setMMExpoTarget(Angle setPoint, int slot){
    }

    @Override 
    public void setMMPositionTarget(Angle setPoint, int slot){}

    @Override 
    public void configureMotionMagic(double kS, double kV, double kA, double kG, double kP, double cruiseV, double maxAcc, GravityTypeValue gravityTypeValue){}

    @Override
    public Angle getReferencePosition(){
        return null;
    }

    @Override 
    public AngularVelocity getReferenceSpeed(){
        return null;
    }
}

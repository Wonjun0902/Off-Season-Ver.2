package frc.robot.AutoTune.HardWare;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.LazyCTRE;
import frc.lib.LazyCTRE.MotorTelemetry;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

public class LazyTunableFXS implements TunableMotor{

    private TalonFXS motor;

    private boolean enableFOC = true;

    private final MotionMagicVoltage mmPosVoltage = new MotionMagicVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicExpoVoltage mmPosExpVoltage = new MotionMagicExpoVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicVelocityVoltage mmVelVoltage = new MotionMagicVelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityTorqueCurrentFOC velTFOC = new VelocityTorqueCurrentFOC(0.0);
    private final PositionVoltage posVoltage = new PositionVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityVoltage velVoltage = new VelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VoltageOut voltageOut = new VoltageOut(Volts.of(0.0)).withEnableFOC(enableFOC);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(Amps.of(0.0));

    public LazyTunableFXS(int motorID, CANBus canBus, double gearRatio){
        motor = new TalonFXS(motorID, canBus);

        if (motor.getIsProLicensed().getValue() ==  false) DriverStation.reportWarning("Motor" + motor.getDeviceID() + " on CANbus" + motor.getNetwork(), false);
        BaseStatusSignal.setUpdateFrequencyForAll(250, motor.getPosition(),motor.getVelocity(),motor.getAcceleration(),motor.getStatorCurrent(),motor.getSupplyCurrent());
        motor.optimizeBusUtilization();
    }

    public LazyTunableFXS(int motorID, double gearRatio) {
    this(motorID, new com.ctre.phoenix6.CANBus(), gearRatio); 
    }
    
    @Override
    public void setMotorVoltage(double volts){
        this.motor.setControl(voltageOut.withOutput(Volts.of(volts)));
    }

    @Override
    public void setMotorCurrent(double current){
        this.motor.setControl(torqueCurrentFOC.withOutput(Amps.of(current)));
    }

    @Override
    public void stopMotor(){
        this.motor.stopMotor();
    }

    @Override
    public AngularVelocity getMotorSpeed(){
        return this.motor.getVelocity().getValue();
    }

    @Override
    public Angle getMotorPosition(){
        return this.motor.getPosition().getValue();
    }

    @Override 
    public Current getCurrent(){
        return this.motor.getStatorCurrent().getValue();
    }

    @Override
    public Voltage getVoltage(){
        return this.motor.getMotorVoltage().getValue();
    }

    @Override 
    public AngularAcceleration getAcceleration(){
        return this.motor.getAcceleration().getValue();
    }

    @Override 
    public void setMMPositionTarget(Angle setPoint, int slot){
        this.motor.setControl(mmPosVoltage.withPosition(setPoint));
    }

    @Override
    public void setMMExpoTarget(Angle setPoint, int slot){
        this.motor.setControl(mmPosExpVoltage.withPosition(setPoint));
    }

    @Override 
    public void configureMotionMagic(double kS, double kV, double kA, double kG, double kP, double kD, double cruiseV, double maxAcc, GravityTypeValue gravityTypeValue){
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        config.Slot0.kI = 0.0;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        config.Slot0.GravityType = gravityTypeValue;

        config.MotionMagic.MotionMagicAcceleration = maxAcc;
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseV;

        this.motor.getConfigurator().apply(config);
    }

    @Override 
    public void configureMotionMagicExpo(double kS, double kV, double kA, double kG, double kP, double kD, double expokA, double expokV, GravityTypeValue gravityTypeValue){
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        config.Slot0.kI = 0.0;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;

        config.MotionMagic.MotionMagicCruiseVelocity = 0.0; //Optional for MMExpo  
        config.MotionMagic.MotionMagicExpo_kA = expokA;
        config.MotionMagic.MotionMagicExpo_kV = expokV;

        this.motor.getConfigurator().apply(config);
    }

    @Override 
    public Angle getReferencePosition(){
        return Rotations.of(this.motor.getClosedLoopReference().getValueAsDouble());
    }

    @Override 
    public AngularVelocity getReferenceSpeed(){
        return RotationsPerSecond.of(this.motor.getClosedLoopReferenceSlope().getValueAsDouble());
    }
}



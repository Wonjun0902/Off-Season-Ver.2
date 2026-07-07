package frc.robot.AutoTune.HardWare;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;


public class LazyTunableTalon implements TunableMotor{

    private TalonFX motor;

    private boolean enableFOC = true;

    private final MotionMagicVoltage mmPosVoltage = new MotionMagicVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicExpoVoltage mmPosExpVoltage = new MotionMagicExpoVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicVelocityVoltage mmVelVoltage = new MotionMagicVelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityTorqueCurrentFOC velTFOC = new VelocityTorqueCurrentFOC(0.0);
    private final PositionVoltage posVoltage = new PositionVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityVoltage velVoltage = new VelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VoltageOut voltageOut = new VoltageOut(Volts.of(0.0)).withEnableFOC(enableFOC);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(Amps.of(0.0));

    public LazyTunableTalon(int motorID, CANBus canBus, double gearRatio){
        motor = new TalonFX(motorID, canBus);

        if (motor.getIsProLicensed().getValue() ==  false) DriverStation.reportWarning("Motor" + motor.getDeviceID() + " on CANbus" + motor.getNetwork(), false);
        BaseStatusSignal.setUpdateFrequencyForAll(250, motor.getPosition(),motor.getVelocity(),motor.getAcceleration(),motor.getStatorCurrent(),motor.getSupplyCurrent());
        motor.optimizeBusUtilization();
    }

    // Inside LazyTunableTalon.java
    public LazyTunableTalon(int motorID, double gearRatio) {
    this(motorID, new com.ctre.phoenix6.CANBus(), gearRatio); // Automatically passes a default bus
    }

    @Override
    public AngularAcceleration getAcceleration() {
        return this.motor.getAcceleration().getValue();
    }

    @Override
    public void setMotorVoltage(double volts){
        this.motor.setControl(voltageOut.withOutput(Volts.of(volts)));
    }

    @Override
    public void setMotorCurrent(double current){
        this.motor.setControl(torqueCurrentFOC.withOutput(current));
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
}

package frc.robot.AutoTune;

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
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.LazyCTRE;
import frc.lib.LazyCTRE.MotorTelemetry;
import frc.robot.AutoTune.HardWare.TunableMotor;

public class LazyTunableFXS implements LazyCTRE, TunableMotor{

    private TalonFXS motor;

    private boolean enableFOC = true;
    private Voltage inputVoltage;
    private Current inputCurrent;

    private final MotionMagicVoltage mmPosVoltage = new MotionMagicVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicExpoVoltage mmPosExpVoltage = new MotionMagicExpoVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicVelocityVoltage mmVelVoltage = new MotionMagicVelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityTorqueCurrentFOC velTFOC = new VelocityTorqueCurrentFOC(0.0);
    private final PositionVoltage posVoltage = new PositionVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityVoltage velVoltage = new VelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VoltageOut voltageOut = new VoltageOut(inputVoltage);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(inputCurrent);

    public LazyTunableFXS(int motorID, CANBus canBus, double gearRatio){
        motor = new TalonFXS(motorID, canBus);

        if (motor.getIsProLicensed().getValue() ==  false) DriverStation.reportWarning("Motor" + motor.getDeviceID() + " on CANbus" + motor.getNetwork(), false);
        BaseStatusSignal.setUpdateFrequencyForAll(250, motor.getPosition(),motor.getVelocity(),motor.getAcceleration(),motor.getStatorCurrent(),motor.getSupplyCurrent());
        motor.optimizeBusUtilization();
    }

    @Override
    public void setCurrentControl(Current inputCurrent){
        this.motor.setControl(torqueCurrentFOC.withOutput(inputCurrent));
    }

    @Override
    public void setVoltageControl(Voltage inputVoltage){
        this.motor.setControl(voltageOut.withOutput(inputVoltage));
    }

    @Override
    public void setMMPositionTarget(Angle setpoint, int slot) {
        this.motor.setControl(mmPosVoltage.withPosition(setpoint).withSlot(slot));
    }

    @Override
    public void setMMExpoTarget(Angle setpoint, int slot) {
        this.motor.setControl(mmPosExpVoltage.withPosition(setpoint).withSlot(slot));
    }

    @Override
    public void setMMVelocityTarget(AngularVelocity velocity, int slot) {
        this.motor.setControl(mmVelVoltage.withVelocity(velocity).withSlot(slot));
    }

    @Override
    public void setTFOCVelocityTarget(AngularVelocity velocity, Current feedforward, int slot) {
        this.motor.setControl(velTFOC.withVelocity(velocity).withFeedForward(feedforward).withSlot(slot));
    }

    @Override
    public void setPIDPositionTarget(Angle setpoint, int slot) {
        this.motor.setControl(posVoltage.withPosition(setpoint).withSlot(slot));
    }

    @Override
    public void setPIDVelocityTarget(AngularVelocity velocity, int slot) {
        this.motor.setControl(velVoltage.withVelocity(velocity).withSlot(slot));
    }

    @Override
    public void setDutyCycle(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setCoast() {
        this.motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void setBrake() {
        this.motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void enableFOC() {
        this.enableFOC = true;
    }

    @Override
    public void disableFOC() {
        this.enableFOC = false;
    }

    @Override
    public void stop() {
        this.motor.stopMotor();
    }

    @Override
    public Angle getPosition() {
        return this.motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return this.motor.getVelocity().getValue();
    }

    @Override
    public AngularAcceleration getAcceleration() {
        return this.motor.getAcceleration().getValue();
    }

    @Override
    public Current getStatorCurrent() {
        return this.motor.getStatorCurrent().getValue();
    }

    @Override
    public Voltage getStatorVoltage() {
        return this.motor.getMotorVoltage().getValue();
    }

    @Override
    public void updateTelemetry(MotorTelemetry telemetry) {
        telemetry.position = motor.getPosition().getValue();
        telemetry.velocity = motor.getVelocity().getValue();
        telemetry.acceleration = motor.getAcceleration().getValue();
        telemetry.statorCurrent = motor.getStatorCurrent().getValue();
        telemetry.supplyCurrent = motor.getSupplyCurrent().getValue();
    }
    
    public TalonFXS getMotor() {
        return motor;
    }

    public TalonFXS getFollower() {
        return null;
    }

    public CANcoder getCanCoder() {
        return null;
    }

    @Override
    public void setMotorVoltage(double volts){
        this.motor.setVoltage(volts);
    }

    @Override
    public void setMotorCurrent(double current){
        this.motor.setControl(torqueCurrentFOC.withOutput(inputCurrent));
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



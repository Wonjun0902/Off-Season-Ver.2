package frc.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyTalon implements LazyCTRE {
    private TalonFX motor, follower;
    private CANcoder canCoder;
    private boolean enableFOC = true;
    private Voltage inputVoltage;

    private final MotionMagicVoltage mmPosVoltage = new MotionMagicVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicExpoVoltage mmPosExpVoltage = new MotionMagicExpoVoltage(0.0).withEnableFOC(this.enableFOC);
    private final MotionMagicVelocityVoltage mmVelVoltage = new MotionMagicVelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityTorqueCurrentFOC velTFOC = new VelocityTorqueCurrentFOC(0.0);
    private final PositionVoltage posVoltage = new PositionVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VelocityVoltage velVoltage = new VelocityVoltage(0.0).withEnableFOC(this.enableFOC);
    private final VoltageOut voltageOut = new VoltageOut(inputVoltage);


    public LazyTalon(int motorID, String canBus, TalonFXConfiguration configuration, int followID, String followCanBus,
            TalonFXConfiguration followConfiguration, MotorAlignmentValue followerInverted, int canCoderID,
            String canCoderCanBus, CANcoderConfiguration canCoderConfiguration) {
        motor = new TalonFX(motorID, canBus);
        motor.getConfigurator().apply(configuration);

        if (followConfiguration != null) {
            follower = new TalonFX(followID, followCanBus);
            follower.getConfigurator().apply(followConfiguration);
            follower.setControl(new Follower(motorID, followerInverted));
        }

        if (canCoderConfiguration != null) {
            canCoder = new CANcoder(canCoderID, canCoderCanBus);
            canCoder.getConfigurator().apply(canCoderConfiguration);
        }
    if (motor.getIsProLicensed().getValue() ==  false) DriverStation.reportWarning("Motor" + motor.getDeviceID() + " on CANbus" + motor.getNetwork(), false);
        BaseStatusSignal.setUpdateFrequencyForAll(250, motor.getPosition(),motor.getVelocity(),motor.getAcceleration(),motor.getStatorCurrent(),motor.getSupplyCurrent());
        motor.optimizeBusUtilization();
        for (int i = 0; i < 5; i++) {
            var error = motor.getConfigurator().apply(configuration);
            if (error.isOK()) {
                break;
            } else {
                DriverStation.reportWarning(
                        "Warning: Your motor configuration on motor " + motor.getDeviceID() + " was not applied!",
                        false);
            }
        }        
    motor.setNeutralMode(NeutralModeValue.Brake);
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
    public void updateTelemetry(MotorTelemetry telemetry) {
        telemetry.position = motor.getPosition().getValue();
        telemetry.velocity = motor.getVelocity().getValue();
        telemetry.acceleration = motor.getAcceleration().getValue();
        telemetry.statorCurrent = motor.getStatorCurrent().getValue();
        telemetry.supplyCurrent = motor.getSupplyCurrent().getValue();
    }

    public TalonFX getMotor() {
        return motor;
    }

    public TalonFX getFollower() {
        return follower;
    }

    public CANcoder getCanCoder() {
        return canCoder;
    }
}
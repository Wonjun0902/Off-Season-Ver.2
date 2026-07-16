package frc.robot.AutoTune.HardWare;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TunableMotor {

    public void setMotorVoltage(double volts);

    public void setMotorCurrent(double current);

    public void stopMotor();

    public AngularVelocity getMotorSpeed();

    public Angle getMotorPosition();

    public AngularAcceleration getAcceleration();

    public Current getCurrent();

    public Voltage getVoltage();

    public void setMMExpoTarget(Angle setPoint, int slot);

    public void setMMPositionTarget(Angle setPoint, int slot);

    public void configureMotionMagic(double kS, double kV, double kA, double kG, double kP, double cruiseV, double maxAcc, GravityTypeValue gravityTypeValue);

    public Angle getReferencePosition();

    public AngularVelocity getReferenceVelocity();

    public AngularAcceleration getReferenceAcceleration();
}

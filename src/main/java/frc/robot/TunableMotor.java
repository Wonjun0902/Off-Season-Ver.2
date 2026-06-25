package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TunableMotor {

    public void createMotor();

    public void setMotorSpeed();

    public void setMotorVoltage();

    public void setMotorCurrent();

    public void stopMotor();

    public AngularVelocity getMotorVelocity();

    public Angle getMotorPosition();

    public AngularAcceleration getAcceleration();

    public Current getCurrent();

    public Voltage getVoltage();
}

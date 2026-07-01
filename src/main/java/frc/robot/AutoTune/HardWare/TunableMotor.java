package frc.robot.AutoTune.HardWare;

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
}

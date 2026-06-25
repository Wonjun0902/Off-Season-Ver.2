package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    public void setShooterTargetSpeed(AngularVelocity velocity);
    public void setShooterAngle(Angle angle);    
    public void stop();
    public void periodic();

    public Angle getHoodAngle();
    public AngularVelocity getLeftShooterVelocity();
    public AngularVelocity getRightShooterVelocity();
}

package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ThroatIO {
    public void setSpeed(AngularVelocity velocity);

    public void stop();

    public AngularVelocity getVelocity();

}

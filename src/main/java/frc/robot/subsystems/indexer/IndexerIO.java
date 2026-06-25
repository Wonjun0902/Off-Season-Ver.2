package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public interface IndexerIO {
    public void spinLeftVelocity(AngularVelocity speed);
    public void spinRightVelocity(AngularVelocity speed);
    public void stop();
    public void runDutyCycleLeft(double dutyCycle);
    public void runDutyCycleRight(double dutyCycle);

    public Distance getLeftRange();
    public Distance getRightRange();
    public AngularVelocity getLeftVelocity();
    public Current getLeftCurrent();
    public AngularVelocity getRightVelocity();
    public Current getRightCurrent();
    public void periodic();
}

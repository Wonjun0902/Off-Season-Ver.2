package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    /**
     * Runs the intake motor at a given velocity.
     * @param velocity
     */
    public void runIntake(AngularVelocity velocity);

    /**
     * Moves the deployer to a given setpoint using motion magic.
     * @param setpoint
     */
    public void moveDeployerTo(Angle setpoint);

    /**
     * Stops the intake motor.
     */
    public void stopIntake();

    /**
     * Stops the deployer motor.
     */
    public void stopDeployer();

    public Voltage getTopIntakeVoltage();
    public Current getTopIntakeSupplyCurrent();
    public AngularVelocity getTopIntakeVelocity();
    public Angle getTopIntakePosition();
    public Voltage getBottomIntakeVoltage();
    public Current getBottomIntakeSupplyCurrent();
    public AngularVelocity getBottomIntakeVelocity();
    public Angle getBottomIntakePosition();

    public Voltage getDeployerVoltage();
    public Current getDeployerSupplyCurrent();
    public AngularVelocity getDeployerVelocity();
    public Angle getDeployerPosition();

    public void periodic();
}

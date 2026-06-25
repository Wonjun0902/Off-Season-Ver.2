package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.shooter.ShooterConstants.Configurations.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.Configurations.Hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOSim implements ShooterIO {
    private DCMotor gearboxes = DCMotor.getKrakenX60(2);
    private DCMotorSim HOOD = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxes, 0.00001, 1), gearboxes);
    private FlywheelSim FLYWHEEL_LEFT = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearboxes, 0.1, 1), gearboxes);
    private FlywheelSim FLYWHEEL_RIGHT = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearboxes, 0.1, 1), gearboxes);
    private double shooterTarget=0.0;
    private double hoodTarget=0.0;

    public ShooterIOSim() {
    }

    @Override
    public void setShooterTargetSpeed(AngularVelocity velocity) {
        shooterTarget=velocity.in(RotationsPerSecond);
    }

    @Override
    public void setShooterAngle(Angle angle) {
        hoodTarget=angle.magnitude();
    }

    @Override
    public void stop() {
        hoodTarget=HOOD.getAngularPosition().in(Degrees);
        shooterTarget=0;
    }
    @Override
    public AngularVelocity getLeftShooterVelocity() {
        return FLYWHEEL_LEFT.getAngularVelocity();
    }
    @Override
    public AngularVelocity getRightShooterVelocity() {
        return FLYWHEEL_RIGHT.getAngularVelocity();
    }
    @Override
    public Angle getHoodAngle() {
        return Degrees.of(HOOD.getAngularPosition().in(Degrees));
    }
    @Override
    public void periodic() {
        double leftError = shooterTarget - FLYWHEEL_LEFT.getAngularVelocity().in(RotationsPerSecond);
        double rightError = shooterTarget - FLYWHEEL_RIGHT.getAngularVelocity().in(RotationsPerSecond);
        double hoodError = hoodTarget - getHoodAngle().magnitude();
        FLYWHEEL_LEFT.setInput(MathUtil.clamp(Shooter.Left.kP*leftError,-12,12));
        FLYWHEEL_RIGHT.setInput(MathUtil.clamp(Shooter.Right.kP*rightError,-12,12));
        HOOD.setInput(Math.min(Hood.kP*hoodError,12));
        SmartDashboard.putNumber("Shooter Left Speed Error", leftError);
        SmartDashboard.putNumber("Shooter Right Speed Error", rightError);
        SmartDashboard.putNumber("Shooter Hood Error", hoodError);
        SmartDashboard.putNumber("Shooter Hood Angle", getHoodAngle().magnitude());
        SmartDashboard.putNumber("Shooter Left Speed ", FLYWHEEL_LEFT.getAngularVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Shooter Right Speed ", FLYWHEEL_RIGHT.getAngularVelocity().in(RotationsPerSecond));
        FLYWHEEL_LEFT.update(0.02);
        FLYWHEEL_RIGHT.update(0.02);
        HOOD.update(0.02);
    }

}

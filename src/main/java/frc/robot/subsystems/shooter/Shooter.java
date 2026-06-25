package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.LookupTables.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    ShooterIO io;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public Pose2d lastSeenRobotPose=drivetrain.getCachedState().Pose;
    private Angle aimOffset = Rotations.of(0.0); //0.1;
    private AngularVelocity speedOffset = RotationsPerSecond.of(0.0); //40.0;

    public Angle getHoodAngle() {
        double dist = (AutoAlign.getCachedHub().getDistance(lastSeenRobotPose.getTranslation()));
        return Rotations.of(HOOD_LOOKUP_TABLE.get(dist));
    }
    public AngularVelocity getShooterSpeed() {
        double dist = (AutoAlign.getCachedHub().getDistance(lastSeenRobotPose.getTranslation()));
        return RotationsPerSecond.of(SHOOTER_LOOKUP_TABLE.get(dist));
    }

    public void updateCurrentPose(Pose2d currentRobotPose) {
            lastSeenRobotPose = currentRobotPose;
    }

    public Command shootFromEverywhere() {
        return runEnd(() -> {
            double dist = (AutoAlign.getCachedHub().getDistance(lastSeenRobotPose.getTranslation()));
            io.setShooterAngle(Rotations.of(HOOD_LOOKUP_TABLE.get(dist)).plus(aimOffset));
            io.setShooterTargetSpeed(RotationsPerSecond.of(SHOOTER_LOOKUP_TABLE.get(dist)).plus(speedOffset));
        },() -> {
            io.stop();
        });
    }

    /**
     * Shoot with specified values with offsets
     * @return
     */
    public Command shoot(Angle hoodAngle, AngularVelocity shooterSpeed) {
        return runEnd(() -> {
            io.setShooterAngle(hoodAngle.plus(aimOffset));
            io.setShooterTargetSpeed(shooterSpeed.plus(speedOffset));
        }, () -> {
            io.stop();
        });
    }
    /**
     * Shoot with specified values with offsets from the HUB
     * @return
     */
    public Command shootHUB() {
        return shoot(HOOD_HUB_ANGLE, SHOOTER_HUB_SPEED);
    }
    /**
     * Shoot with specified values with offsets from the HUB
     * @return
     */
    public Command shootTOWERFRONT() {
        return shoot(HOOD_TOWER_ANGLE, SHOOTER_TOWER_SPEED);
    }
    /**
     * Shoot with specified values with offsets from the HUB
     * @return
     */
    public Command shootTRENCH() {
        return shoot(HOOD_TRENCH_ANGLE, SHOOTER_TRENCH_SPEED);
    }
    /**
     * Shoot with specified values with offsets from the HUB
     * @return
     */
    public Command shootTOWERBACK() {
        return shoot(HOOD_BACK_ANGLE, SHOOTER_BACK_SPEED);
    }


    public Command spinShooterAt(AngularVelocity vel) {
        return run(() -> io.setShooterTargetSpeed(vel));
    }
    
    /**
     * Debug purposes: shoot using values of the speed offsets
     * @return
     */
    public Command trimmedShoot() {
        return run(() -> io.setShooterTargetSpeed(speedOffset));
    }
    
    /**
     * Debug purposes: move hood to position of the aim offset
     * @return
     */
    public Command trimmedAngle() {
        return run(() -> io.setShooterAngle(aimOffset));
    }


    public Runnable trimSpeedUp() {
        return () -> speedOffset = speedOffset.plus(TRIM_SPEED_INCREMENT);
    }
    public Runnable trimSpeedDown() {
        return () -> speedOffset = speedOffset.minus(TRIM_SPEED_INCREMENT);
    }
    public Runnable trimAimUp() {
        return () -> aimOffset = aimOffset.plus(TRIM_AIM_INCREMENT);
    }
    public Runnable trimAimDown() {
        return () -> aimOffset = aimOffset.minus(TRIM_AIM_INCREMENT);
    }

    public Runnable resetTrims() {
        return () -> {
            speedOffset = RotationsPerSecond.of(0.0);
            aimOffset = Rotations.of(0.0);
        };
    }

    public Command stop() {
        return runOnce(() -> io.setShooterTargetSpeed(RotationsPerSecond.of(0.0)));
    }

    @Override
    public void periodic() {
        updateCurrentPose(drivetrain.getCachedState().Pose);
        // io.periodic();

        SmartDashboard.putNumber("Hood angle (rot)", io.getHoodAngle().in(Rotations));
        SmartDashboard.putNumber("Left shooter speed (rps)", io.getLeftShooterVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Right shooter speed (rps)", io.getRightShooterVelocity().in(RotationsPerSecond));
       
        // SmartDashboard.putNumber("Speed Target of SOE", getShooterSpeed().in(RotationsPerSecond));
        // SmartDashboard.putNumber("Hood Target of SOE", getHoodAngle().in(Rotations));
        // SmartDashboard.putNumber("Angle to Hub", AutoAlign.getAngleToHub().getDegrees());

        SmartDashboard.putNumber("Hood offset (rot)", aimOffset.in(Rotations));
        SmartDashboard.putNumber("Speed offset (rps)", speedOffset.in(RotationsPerSecond));

        // used to be called "LL to tag", but that was not the intended value
        // we dont know if the value is correct to its name "bot to hub", but that is what its intended to be.
        // it *should* be correct though
        SmartDashboard.putNumber("Bot to hub distance", AutoAlign.getCachedHub().getDistance(lastSeenRobotPose.getTranslation()));
    
        SmartDashboard.putNumber("Current Bot Angle", drivetrain.getCachedState().Pose.getRotation().getDegrees());
    }
}

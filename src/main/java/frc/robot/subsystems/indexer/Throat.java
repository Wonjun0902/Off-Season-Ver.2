package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.ThroatConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Throat extends SubsystemBase {
    private ThroatIO io;

    public Throat(ThroatIO io) {
        this.io = io;
    }

    public Command feed() {
        return runEnd(() -> io.setSpeed(FEED_SPEED), () -> io.stop());
    }

    public Command stopAll() {
        return runOnce(() -> {
            io.stop();
        });
    }

    public Command outtake() {
        return runEnd(() -> io.setSpeed(OUTTAKE_SPEED), () -> io.stop());
    }

    private double trimmedSpeed = 90;
    public Command trimmedFeed() {
        return run(() -> io.setSpeed(RotationsPerSecond.of(trimmedSpeed)));
    }
    public Command trimSpeedUp() {
        return runOnce(() -> trimmedSpeed += 3);
    }
    public Command trimSpeedDown() {
        return runOnce(() -> trimmedSpeed -= 3);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Throat feeding", io.getVelocity().gt(RotationsPerSecond.of(1)));
    }
}

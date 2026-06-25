package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    private IndexerIO io;

    // If either throat stream is jammed or not being run (no balls can be detected)
    public final Trigger noBallsInSightLeft;
    public final Trigger noBallsInSightRight;
    public Trigger debouncedLeft, debouncedRight;

    public Indexer(IndexerIO io) {
        this.io = io;

        noBallsInSightLeft = new Trigger(() -> io.getLeftRange().gte(CANRANGE_LEFT_TRIGGER));
        noBallsInSightRight = new Trigger(() -> io.getRightRange().gte(CANRANGE_RIGHT_TRIGGER));

        debouncedLeft = noBallsInSightLeft.debounce(JAM_TIME_TOLERANCE.in(Seconds));
        debouncedRight = noBallsInSightRight.debounce(JAM_TIME_TOLERANCE.in(Seconds));
    }

    public AngularVelocity getFeedSpeed() {
        // Note: temporary
        return RotationsPerSecond.of(-60.0);
    }
    
    public Command feed() {
        // Note: get feed speed will be static (computed once and never updated)
        return feed(getFeedSpeed());
    }
    public Command feedWithUnjam() {
        return feedWithUnjam(getFeedSpeed());
    }

    public Command feed(AngularVelocity feedSpeed) {
        return runEnd(() -> {
            io.spinLeftVelocity(feedSpeed);
            io.spinRightVelocity(feedSpeed);
        }, () -> {
            io.stop();
        });
    }

    public Command outFeed() {
        return runEnd(() -> {
            io.spinLeftVelocity(getFeedSpeed().unaryMinus());
            io.spinRightVelocity(getFeedSpeed().unaryMinus());
        }, () -> {
            io.stop();
        });
    }

    public Command feedWithUnjam(AngularVelocity feedSpeed) {
        Timer leftUnjamTimer = new Timer();
        leftUnjamTimer.stop();
        leftUnjamTimer.reset();
        Timer rightUnjamTimer = new Timer();
        rightUnjamTimer.stop();
        rightUnjamTimer.reset();

        return new SequentialCommandGroup(
            runOnce(() -> {
                debouncedLeft = noBallsInSightLeft.debounce(JAM_TIME_TOLERANCE.in(Seconds));
                debouncedRight = noBallsInSightRight.debounce(JAM_TIME_TOLERANCE.in(Seconds));
            }),
            runEnd(() -> {
                // LEFT SIDE
                boolean isLeftUnjamming = leftUnjamTimer.get() > 0;

                if (!isLeftUnjamming) { // NOT UNJAMMING ###
                    if (debouncedLeft.getAsBoolean()) {
                        leftUnjamTimer.restart();
                        io.spinLeftVelocity(INDEXER_UNJAM_SPEED);
                    } else {
                        // Normal operation
                        io.spinLeftVelocity(feedSpeed);
                    }
                } else { // UNJAMMING ###
                    if (leftUnjamTimer.hasElapsed(UNJAM_DURATION.in(Seconds))) {
                        // Cycle over. Begin normal operations
                        leftUnjamTimer.stop();
                        leftUnjamTimer.reset();
                        io.spinLeftVelocity(feedSpeed);

                        debouncedLeft = noBallsInSightLeft.debounce(JAM_TIME_TOLERANCE.in(Seconds));
                    } else {
                        // Keep reversing
                        io.spinLeftVelocity(INDEXER_UNJAM_SPEED);
                    }
                }

                // RIGHT SIDE
                boolean isRightUnjamming = rightUnjamTimer.get() > 0;

                if (!isRightUnjamming) {  // NOT UNJAMMING ###
                    if (debouncedRight.getAsBoolean()) {
                        rightUnjamTimer.restart();
                        io.spinRightVelocity(INDEXER_UNJAM_SPEED);
                    } else {
                        // Normal operation
                        io.spinRightVelocity(feedSpeed);
                    }
                } else { // UNJAMMING ###
                    if (rightUnjamTimer.hasElapsed(UNJAM_DURATION.in(Seconds))) {
                        // Cycle over. Begin normal operations
                        rightUnjamTimer.stop();
                        rightUnjamTimer.reset();
                        io.spinRightVelocity(feedSpeed);
                        
                        debouncedRight = noBallsInSightRight.debounce(JAM_TIME_TOLERANCE.in(Seconds));
                    } else {
                        // Keep reversing
                        io.spinRightVelocity(INDEXER_UNJAM_SPEED);
                    }
                }
            }, () -> io.stop())
        );
    }

    public Command idle_indexer(AngularVelocity speed) {
        return runEnd(() -> {
            io.spinLeftVelocity(speed);
            io.spinRightVelocity(speed);
        }, () -> io.stop());
    }

    public Command stop() {
        return runOnce(() -> io.stop());
    }

    public AngularVelocity getVelocity() {
        return io.getLeftVelocity();
    }

    private double trimmedSpeed = 0.0;
    public Command trimmedFeed() {
        return idle_indexer(RotationsPerSecond.of(trimmedSpeed));
    }

    public Command trimSpeedUp() {
        return runOnce(() -> trimmedSpeed += 3);
    }

    public Command trimSpeedDown() {
        return runOnce(() -> trimmedSpeed -= 3);
    }

    @Override
    public void periodic() {
        io.periodic();

        // SmartDashboard.putNumber("Left ind State", (int) (io.getLeftRange().in(Meters) * 100));
        // SmartDashboard.putNumber("Right ind State", (int) (io.getRightRange().in(Meters) * 100));
        // SmartDashboard.putBoolean("Left jammed State", io.getLeftRange().gte(CANRANGE_LEFT_TRIGGER));
        // SmartDashboard.putBoolean("Right jammed State", io.getRightRange().gte(CANRANGE_RIGHT_TRIGGER));
        // SmartDashboard.putBoolean("Agitating State", this.agitatingTrigger.getAsBoolean());
        SmartDashboard.putBoolean("Indexer left feeding", io.getLeftVelocity().lt(RotationsPerSecond.of(-1)));
        SmartDashboard.putBoolean("Indexer right feeding", io.getRightVelocity().lt(RotationsPerSecond.of(-1)));
        // SmartDashboard.putNumber("Velocity", getVelocity().magnitude());
        // SmartDashboard.putNumber("Indexer Target Rotation", trimmedSpeed);
    }

}
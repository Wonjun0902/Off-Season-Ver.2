package frc.robot.subsystems.stageManager;

import static frc.robot.RobotContainer.autoAligner;
import static frc.robot.subsystems.indexer.IndexerConstants.INDEXER_LONG_SPEED;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Throat;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class MrKrabs {
    private Intake intake;
    private Indexer indexer;
    private Throat throat;
    private Shooter shooter;

    // FIXME: rotation angle
    public final SwerveRequest.FieldCentricFacingAngle rotate = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(new Rotation2d(90.0));

    // TODO: add throat
    public MrKrabs(Intake intake, Indexer indexer, Throat throat, Shooter shooter) {
        this.intake = intake;
        this.indexer = indexer;
        this.throat = throat;
        this.shooter = shooter;
    }

    /**
     * DEFAULT/RESET
     * Reset (retract if applicable) and stop everything.
     * If applicable, prefer idling over stopping (indexer and throat).
     * 
     * @return
     */
    public Command reset() {
        return new ParallelCommandGroup(
                intake.stopIntake().andThen(intake.retract()),
                indexer.stop(),
                throat.stopAll(),
                shooter.stop()
            );
    }

    /**
     * IDLE
     * Default reset position, except the intake doesn't retract.
     * @return
     */
    public Command idle() {
        return new ParallelCommandGroup(
                intake.stopIntake(),
                indexer.stop(),
                throat.stopAll(),
                shooter.stop()
            );
    }

    /**
     * INTAKING
     * Intake deployed
     * Intake running
     * 
     * @return
     */
    public Command beginIntaking() {
        return intake.deploy()
                .andThen(intake.intake());
    }

    public Command outtake() {
        return intake.outtake();
    }

    /**
     * STOP INTAKE
     * Intake stopped
     * 
     * @return
     */
    public Command stopIntake(){
        return intake.stopIntake();
    }

    /**
     * RETRACT INTAKE
     * Intake retracted
     * Intake stopped
     * 
     * @return
     */
    public Command retractIntake() {
        return intake.stopIntake()
                .andThen(intake.retract());
    }

    public Command shoot(Angle hoodAngle, AngularVelocity shooterSpeed, AngularVelocity indexerSpeed) {
        return new ParallelCommandGroup(
                // intake.stopIntake(),
                shooter.shoot(hoodAngle, shooterSpeed),
                Commands.waitSeconds(0.4).andThen(throat.feed()),
                Commands.waitSeconds(0.8).andThen(indexer.feedWithUnjam(indexerSpeed))   
            );
    }
    public Command shootFromEverywhere() {
        return new ParallelCommandGroup(
                // intake.stopIntake(),
                shooter.shootFromEverywhere(),
                Commands.waitSeconds(0.4).andThen(throat.feed()),
                Commands.waitSeconds(0.8).andThen(indexer.feedWithUnjam(INDEXER_LONG_SPEED))   
            );
    }
    
    /**
     * Begin scoring, running the throat, indexer, and shooter.
     * The bot is auto-aligned to face the hub.
     * The intake is stopped (without retracting)
     * @return
     */
    public Command beginShooting(Angle hoodAngle, AngularVelocity shooterSpeed, AngularVelocity indexerSpeed, Angle robotAngle) {
        return new ParallelCommandGroup(
            autoAligner.align(robotAngle),
            shoot(hoodAngle, shooterSpeed, indexerSpeed)
        );
    }
    
    /**
     * Begin scoring with specified values, running the throat, indexer, and shooter.
     * The bot is auto-aligned to face the hub.
     * The intake is stopped (without retracting)
     * @param hoodAngle
     * @param shooterSpeed
     * @param indexerSpeed
     * @param leftRobotAngle
     * @param rightRobotAngle
     * @return
     */
    public Command beginShooting(Angle hoodAngle, AngularVelocity shooterSpeed, AngularVelocity indexerSpeed, Angle leftRobotAngle, Angle rightRobotAngle) {
        return new ParallelCommandGroup(
            autoAligner.alignLeftOrRight(leftRobotAngle, rightRobotAngle),
            shoot(hoodAngle, shooterSpeed, indexerSpeed)
        );
    }
}

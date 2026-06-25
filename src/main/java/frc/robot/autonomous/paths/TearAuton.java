package frc.robot.autonomous.paths;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotContainer.PATHFINDER;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutonParams;
import frc.robot.autonomous.Poses;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.LookupTables;
import static frc.robot.RobotContainer.mrKrabs;


public class TearAuton {
    public static Command tearAuton(boolean blue, boolean right) {
            var allianceBump = Poses.configureColorSide(blue, right, Poses.TEAR_RED_ALLIANCE_BUMP);
            var preCrossBump = Poses.configureColorSide(blue, right, Poses.TEAR_RED_PRE_CROSS_BUMP);
            var crossBump = Poses.configureColorSide(blue, right, Poses.TEAR_RED_CROSS_BUMP);
            var preIntake = Poses.configureColorSide(blue, right, Poses.TEAR_RED_PRE_INTAKE);
            var slantIntake = Poses.configureColorSide(blue, right, Poses.TEAR_RED_INTAKE);
            var secondIntake = Poses.configureColorSide(blue, right, Poses.TEAR_RED_SECOND_INTAKE);
            var slideToHub = Poses.configureColorSide(blue, right, Poses.TEAR_RED_THIRD_INTAKE);
            var arrivalPose = Poses.configureColorSide(blue, right, Poses.TEAR_ARRIVAL_POSE);
            var shootingPose = Poses.configureColorSide(blue, right, Poses.TEAR_SHOOTING_POSE);
            var expShootingPose = Poses.configureColorSide(blue, right, Poses.TEAR_SHOOTING_EXP);
            var secondSweep = Poses.configureColorSide(blue, right, Poses.SWEEP_SECOND_INTAKE);
            var crossBumpReverse = Poses.configureColorSide(blue, right, Poses.CROSS_BUMP_REVERSE);
            var preCrossReverse = Poses.configureColorSide(blue, right, Poses.PRE_CROSS_BUMP_REVERSE);

        
        return new SequentialCommandGroup(
            Commands.runOnce(() -> drivetrain.resetTranslation(allianceBump.getTranslation())),
                        Poses.limeResetTranslation(),

            // new WaitCommand(1.0),

            
            // PATHFINDER.moveToPose(preCrossBump, AutonParams.DRIVING_NO_EXIT),
            new SequentialCommandGroup(
                PATHFINDER.moveToPose(crossBump, AutonParams.DRIVING),
                PATHFINDER.moveToPose(preIntake, AutonParams.DRIVING)
            ).alongWith(intake.deploy().andThen(intake.intakeWithoutRunEnd())),
                
            PATHFINDER.moveToPose(slantIntake, AutonParams.DRIVING),
            PATHFINDER.moveToPose(secondIntake, AutonParams.DRIVING),
            // PATHFINDER.moveToPose(slideToHub, AutonParams.DRIVING),

            PATHFINDER.moveToPose(crossBump,AutonParams.DRIVING),

            intake.stopIntake(),

            PATHFINDER.moveToPose(arrivalPose, AutonParams.DRIVING_LARGE_EXIT),

            // PATHFINDER.moveToPose(shootingPose, MoveToPoseParams.empty())
            PATHFINDER.moveToPose(expShootingPose, AutonParams.SLOW_DRIVE),
            drivetrain.stop(),

            // new ParallelDeadlineGroup(
            //     new WaitCommand(Seconds.of(4)),
            //     Commands.runOnce(
            //         () -> {
            //             intake.stopIntake();
            //             shooter.shoot(LookupTables.HOOD_TOWER_ANGLE.minus(Rotations.of(0.05)), LookupTables.SHOOTER_TOWER_SPEED);
            //             Commands.waitSeconds(0.4).andThen(throat.feed());
            //             Commands.waitSeconds(0.8).andThen(indexer.feedWithUnjam(IndexerConstants.INDEXER_SHORT_SPEED));
            //         }
            //     )
            // ),
            new ParallelDeadlineGroup(
                mrKrabs.shoot(LookupTables.HOOD_HUB_ANGLE, LookupTables.SHOOTER_RENEE_SPEED, IndexerConstants.INDEXER_SHORT_SPEED).withTimeout(4),
                
                new WaitCommand(Seconds.of(1.5)).andThen(intake.agitateM2().repeatedly())
            ),

            intake.deploy(),

            PATHFINDER.moveToPose( 
                new Pose2d(
                    crossBump.getX(),
                    crossBump.getY() + (blue ? -0.25 : 0.25),
                    crossBump.getRotation()
                ), AutonParams.DRIVING),

            PATHFINDER.moveToPose(crossBump, AutonParams.DRIVING),

            intake.intakeWithoutRunEnd(),

            PATHFINDER.moveToPose(secondSweep, AutonParams.SLOW_DRIVE),
            PATHFINDER.moveToPose(crossBump, AutonParams.DRIVING),
            PATHFINDER.moveToPose(arrivalPose, AutonParams.DRIVING_LARGE_EXIT),
            PATHFINDER.moveToPose(expShootingPose, AutonParams.SLOW_DRIVE),
            drivetrain.stop(),

            new ParallelCommandGroup(
                mrKrabs.shoot(LookupTables.HOOD_HUB_ANGLE, LookupTables.SHOOTER_RENEE_SPEED, IndexerConstants.INDEXER_SHORT_SPEED),
                intake.agitateM2().repeatedly()
            )
        );
    }
}

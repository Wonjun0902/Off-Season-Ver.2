package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonParams;
import frc.robot.autonomous.Poses;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.LookupTables;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotContainer.PATHFINDER;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.intake;
import static frc.robot.RobotContainer.mrKrabs;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.Pathfinder.MoveToPoseParams;

public class BumpAutonRight {
    public static Command bumpAutonRight(boolean blue, boolean right) {
        final Pose2d allianceBump = blue ? Poses.flipAlliance(Poses.BUMP_RED_ALLIANCE_BUMP)
                : Poses.BUMP_RED_ALLIANCE_BUMP;
        final Pose2d preCrossBump = blue ? Poses.flipAlliance(Poses.BUMP_RED_PRE_CROSS_BUMP)
                : Poses.BUMP_RED_PRE_CROSS_BUMP;
        final Pose2d crossBump = blue ? Poses.flipAlliance(Poses.BUMP_RED_CROSS_BUMP) : Poses.BUMP_RED_CROSS_BUMP;
        final Pose2d intaking = blue ? Poses.flipAlliance(Poses.BUMP_RED_INTAKING) : Poses.BUMP_RED_INTAKING;
        final Pose2d secondIntaking = blue ? Poses.flipAlliance(Poses.BUMP_RED_SECOND_INTAKING)
                : Poses.BUMP_RED_SECOND_INTAKING;
        final Pose2d thirdIntaking = blue ? Poses.flipAlliance(Poses.BUMP_RED_THIRD_INTAKING)
                : Poses.BUMP_RED_THIRD_INTAKING;
        final Pose2d secondRun = blue ? Poses.flipAlliance(Poses.BUMP_RED_SECOND_RUN) : Poses.BUMP_RED_SECOND_RUN;
        final Pose2d arrivalPose = blue ? Poses.flipAlliance(Poses.BUMP_RED_ARRIVAL_POSE) : Poses.BUMP_RED_ARRIVAL_POSE;
        final Pose2d shoot = blue ? Poses.flipAlliance(Poses.BUMP_RED_SHOOT) : Poses.BUMP_RED_SHOOT;

        return new SequentialCommandGroup(
                Commands.runOnce(() -> drivetrain.resetTranslation(allianceBump.getTranslation())),
                        Poses.limeResetTranslation(),


                PATHFINDER.moveToPose(preCrossBump, AutonParams.DRIVING_NO_EXIT),

                // cross over to mid
                drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
                        .withVelocityX(3.5)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0)).withTimeout(Seconds.of(1.75)),

                // reset pose To cross bump manually
                Commands.runOnce(() -> drivetrain.resetPose(
                        new Pose2d(crossBump.getTranslation(), drivetrain.getCachedState().Pose.getRotation()))),

                //
                new ParallelRaceGroup(
                        PATHFINDER.moveToPose(intaking, AutonParams.SEE_TAG_SPECIAL)
                                .andThen(PATHFINDER.moveToPose(secondIntaking, AutonParams.SLOW_DRIVE))
                                .andThen(PATHFINDER.moveToPose(thirdIntaking, AutonParams.SLOW_DRIVE))
                                .andThen(PATHFINDER.moveToPose(secondRun, AutonParams.DRIVING_NO_EXIT)).andThen(drivetrain.stop()),
                        intake.deploy().andThen(intake.intake().withTimeout(Seconds.of(0.5))).andThen(intake.intake())),

                drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
                        .withVelocityX(-3.5)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0)).withTimeout(Seconds.of(1.38)),

                Commands.runOnce(() -> drivetrain.resetPose(
                        new Pose2d(arrivalPose.getTranslation(), drivetrain.getCachedState().Pose.getRotation()))),
                                PATHFINDER.moveToPose(shoot, MoveToPoseParams.empty()),
                                drivetrain.stop(),

                mrKrabs.shoot(LookupTables.HOOD_HUB_ANGLE, LookupTables.SHOOTER_HUB_SPEED, IndexerConstants.INDEXER_SHORT_SPEED).withTimeout(2.6)
        );
    }
}

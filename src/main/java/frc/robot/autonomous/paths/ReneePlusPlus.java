package frc.robot.autonomous.paths;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotContainer.PATHFINDER;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.intake;
import static frc.robot.RobotContainer.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Pathfinder.MoveToPoseParams;
import frc.robot.autonomous.AutonParams;
import frc.robot.autonomous.Poses;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.LookupTables;
import static frc.robot.RobotContainer.mrKrabs;
import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotContainer.throat;
import static frc.robot.RobotContainer.indexer;
import static edu.wpi.first.units.Units.*;


public class ReneePlusPlus {
    public static Command ReneePlusPlus(boolean blue, boolean right) {
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
            
            var secondSweepIntake = Poses.configureColorSide(blue, right, Poses.TEAR_RED_THIRD_INTAKE_REVERSE);
            var secondSweepSecondIntake = Poses.configureColorSide(blue, right, Poses.TEAR_RED_SECOND_INTAKE_REVERSE);
            var secondSweepThirdIntake = Poses.configureColorSide(blue, right, Poses.TEAR_RED_INTAKE_REVERSE);
            var secondSweepPreCross = Poses.configureColorSide(blue, right, Poses.TEAR_RED_PRE_INTAKE_REVERSE);

        
        return new SequentialCommandGroup(
            Commands.runOnce(() -> drivetrain.resetTranslation(allianceBump.getTranslation())),
                        Poses.limeResetTranslation(),
            
            
            new SequentialCommandGroup(
                new SequentialCommandGroup(
                    PATHFINDER.moveToPose(crossBump, AutonParams.DRIVING),
                    PATHFINDER.moveToPose(preIntake, AutonParams.DRIVING)
                ).alongWith(intake.deploy().andThen(intake.intakeWithoutRunEnd())),
                    
                PATHFINDER.moveToPose(slantIntake, AutonParams.DRIVING),
                PATHFINDER.moveToPose(secondIntake, AutonParams.DRIVING),
                PATHFINDER.moveToPose(slideToHub, AutonParams.DRIVING_LARGE_EXIT),
                PATHFINDER.moveToPose(slideToHub, AutonParams.SLOW_DRIVE),
                PATHFINDER.moveToPose(crossBump,AutonParams.DRIVING),

                intake.stopIntake(),

                PATHFINDER.moveToPose(arrivalPose, AutonParams.DRIVING_LARGE_EXIT),


                new ParallelDeadlineGroup(
                    new ParallelCommandGroup(
                        new WaitCommand(Seconds.of(0.5)).andThen(mrKrabs.shoot(LookupTables.HOOD_HUB_ANGLE, LookupTables.SHOOTER_RENEE_SPEED, IndexerConstants.INDEXER_SHORT_SPEED).withTimeout(4)),

                        PATHFINDER.moveToPose(expShootingPose, AutonParams.SLOW_DRIVE).andThen(drivetrain.stop())
                    ),
                    new WaitCommand(Seconds.of(1.5)).andThen(intake.agitateM2().repeatedly())
                )
            ),

            new SequentialCommandGroup( //bascially the exact same thing lol
                new SequentialCommandGroup(
                    PATHFINDER.moveToPose( 
                        new Pose2d(
                            crossBump.getX(),
                            crossBump.getY() + (blue ? -0.25 : 0.25),
                            crossBump.getRotation()
                        ), AutonParams.DRIVING)

                ).alongWith(intake.deploy().andThen(intake.intakeWithoutRunEnd())),
                    
                PATHFINDER.moveToPose(secondSweepIntake, AutonParams.INTAKING_DRIVING),
                PATHFINDER.moveToPose(secondSweepSecondIntake, AutonParams.INTAKING_DRIVING),
                PATHFINDER.moveToPose(secondSweepThirdIntake, AutonParams.INTAKING_DRIVING),
                PATHFINDER.moveToPose(secondSweepPreCross, AutonParams.INTAKING_DRIVING),
                PATHFINDER.moveToPose(crossBump,AutonParams.DRIVING),

                intake.stopIntake(),

                PATHFINDER.moveToPose(arrivalPose, AutonParams.DRIVING_LARGE_EXIT),

                new ParallelDeadlineGroup(
                    new ParallelCommandGroup(
                        new WaitCommand(Seconds.of(0.5)).andThen(mrKrabs.shoot(LookupTables.HOOD_HUB_ANGLE, LookupTables.SHOOTER_RENEE_SPEED, IndexerConstants.INDEXER_SHORT_SPEED)),

                        PATHFINDER.moveToPose(expShootingPose, AutonParams.SLOW_DRIVE).andThen(drivetrain.stop())
                    ),
                    new WaitCommand(Seconds.of(1.5)).andThen(intake.agitateM2().repeatedly())
                )
            )
        );
    }
}

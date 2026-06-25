package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.autonomous.Poses;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.LookupTables;
import frc.robot.autonomous.AutonParams;
import static frc.robot.RobotContainer.intake;
import static frc.robot.RobotContainer.mrKrabs;

import static frc.robot.RobotContainer.PATHFINDER;
import static frc.robot.RobotContainer.drivetrain;


import frc.lib.Pathfinder.MoveToPoseParams;

public class DoubleSweep {
        public static Command DoubleSweepAuton(boolean blue, boolean right) {

                var allianceBump = Poses.configureColorSide(blue, right, Poses.BUMP_RED_ALLIANCE_BUMP);
                var preCrossBump = Poses.configureColorSide(blue, right, Poses.BUMP_RED_PRE_CROSS_BUMP);
                var crossBump = Poses.configureColorSide(blue, right, Poses.BUMP_RED_CROSS_BUMP);
                var preIntaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_INTAKING); //PRE INTAKING
                var intaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_INTAKING);
                var secondIntaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_SECOND_INTAKING);
                var thirdIntaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_THIRD_INTAKING);
                var secondSweepReset = Poses.configureColorSide(blue, right, Poses.SWEEP_SECOND_RESET);
                var secondSweepIntake = Poses.configureColorSide(blue, right, Poses.SWEEP_SECOND_INTAKE);
                var secondRun = Poses.configureColorSide(blue, right, Poses.BUMP_RED_SECOND_RUN);
                var arrivalPose = Poses.configureColorSide(blue, right, Poses.BUMP_RED_ARRIVAL_POSE);
                var shoot = Poses.configureColorSide(blue, right, Poses.BUMP_RED_SHOOT);

                return new SequentialCommandGroup(
                        Commands.runOnce(() -> drivetrain.resetTranslation(allianceBump.getTranslation())),
                        Poses.limeResetTranslation(),

                        // PATHFINDER.moveToPose(preCrossBump, AutonParams.DRIVING_NO_EXIT),
                        
                        new SequentialCommandGroup(
                                PATHFINDER.moveToPose(crossBump, AutonParams.DRIVING),
                                PATHFINDER.moveToPose(crossBump, MoveToPoseParams.builder().earlyExitRange(0.10).build()),
                                PATHFINDER.moveToPose(preIntaking, AutonParams.DRIVING)
                        ).alongWith(intake.deploy().andThen(intake.intakeWithoutRunEnd())),
                                
                        PATHFINDER.moveToPose(intaking, AutonParams.DRIVING),
                        PATHFINDER.moveToPose(secondIntaking, AutonParams.DRIVING),
                        
                        PATHFINDER.moveToPose(thirdIntaking, AutonParams.SLOW_DRIVE),
                        
                        PATHFINDER.moveToPose(secondSweepReset, AutonParams.DRIVING),
                        PATHFINDER.moveToPose(secondSweepIntake, AutonParams.SLOW_DRIVE),

                        PATHFINDER.moveToPose(secondRun, AutonParams.DRIVING),
                
                        intake.stopIntake(),

                        PATHFINDER.moveToPose(arrivalPose, AutonParams.DRIVING),
                        PATHFINDER.moveToPose(shoot, AutonParams.DRIVING_NO_EXIT),

                        drivetrain.stop(),

                        new ParallelCommandGroup(
                                mrKrabs.shoot(LookupTables.HOOD_HUB_ANGLE, LookupTables.SHOOTER_HUB_SPEED, IndexerConstants.INDEXER_SHORT_SPEED),
                                new SequentialCommandGroup(
                                        intake.agitateM2(),
                                        intake.agitateM2()
                                )
                        )       
                );
        }
}

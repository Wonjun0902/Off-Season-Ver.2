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

public class HookAuton {
        public static Command hookAuton(boolean blue, boolean right) {

                var allianceBump = Poses.configureColorSide(blue, right, Poses.BUMP_RED_ALLIANCE_BUMP);
                var preCrossBump = Poses.configureColorSide(blue, right, Poses.BUMP_RED_PRE_CROSS_BUMP);
                var crossBump = Poses.configureColorSide(blue, right, Poses.BUMP_RED_CROSS_BUMP);
                var preIntaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_INTAKING); //PRE INTAKING
                var intaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_INTAKING);
                var secondIntaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_SECOND_INTAKING);
                var thirdIntaking = Poses.configureColorSide(blue, right, Poses.BUMP_RED_THIRD_INTAKING);
                var thirdToSweep = Poses.configureColorSide(blue, right, Poses.THIRD_TO_SWEEP_TRANSITION);
                var sweeping = Poses.configureColorSide(blue, right, Poses.BUMP_RED_SWEEEP); //SWEEP
                var preSecondBump = Poses.configureColorSide(blue, right, Poses.PRE_SECOND_BUMP);
                var secondRun = Poses.configureColorSide(blue, right, Poses.BUMP_RED_SECOND_RUN);
                var arrivalPose = Poses.configureColorSide(blue, right, Poses.BUMP_RED_ARRIVAL_POSE);
                var shoot = Poses.configureColorSide(blue, right, Poses.BUMP_RED_SHOOT);

                return new SequentialCommandGroup(
                        Commands.runOnce(() -> drivetrain.resetTranslation(allianceBump.getTranslation())),
                        Poses.limeResetTranslation(),

                        new SequentialCommandGroup(
                                PATHFINDER.moveToPose(crossBump, AutonParams.DRIVING),
                                PATHFINDER.moveToPose(crossBump, MoveToPoseParams.builder().earlyExitRange(0.10).build()),
                                PATHFINDER.moveToPose(preIntaking, AutonParams.DRIVING)
                        ).alongWith(intake.deploy().andThen(intake.intakeWithoutRunEnd())),
                                
                        PATHFINDER.moveToPose(intaking, AutonParams.DRIVING),
                        PATHFINDER.moveToPose(secondIntaking, AutonParams.DRIVING),
                                
                        
                        PATHFINDER.moveToPose(thirdIntaking, AutonParams.SLOW_DRIVE),
                        PATHFINDER.moveToPose(thirdToSweep, AutonParams.DRIVING),
                        PATHFINDER.moveToPose(sweeping, AutonParams.SLOW_DRIVE),
                        PATHFINDER.moveToPose(preSecondBump, AutonParams.SLOW_DRIVE),

                        //NEW PATH FOR ROTATION

                        PATHFINDER.moveToPose(secondRun,
                                MoveToPoseParams.builder().maxSpeed(0.1).earlyExitRange(0.10).build()),
                
                        intake.stopIntake(),

                        PATHFINDER.moveToPose(arrivalPose, AutonParams.DRIVING),
                        PATHFINDER.moveToPose(shoot, MoveToPoseParams.empty()),

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

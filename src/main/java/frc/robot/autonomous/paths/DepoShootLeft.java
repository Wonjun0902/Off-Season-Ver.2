package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonParams;
import frc.robot.autonomous.Poses;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.LookupTables;

import static frc.robot.RobotContainer.PATHFINDER;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.intake;
import static frc.robot.RobotContainer.mrKrabs;


public class DepoShootLeft {
        public static Command DepoAutonLeft(boolean blue) {
                final var startDepo = Poses.configureColorSide(blue, true, Poses.DEPO_RED_START_DEPO);
                final var shoot = Poses.configureColorSide(blue, true, Poses.DEPO_RED_SHOOT);
                final var endDepo = Poses.configureColorSide(blue, true, Poses.DEPO_RED_END_DEPO);

                return new SequentialCommandGroup(
                        Commands.runOnce(() -> drivetrain.resetTranslation(shoot.getTranslation())),
                                Poses.limeResetTranslation(),

                        PATHFINDER.moveToPose(startDepo, AutonParams.DRIVING_LARGE_EXIT),   
                        PATHFINDER.moveToPose(startDepo, AutonParams.SLOW_DRIVE).alongWith(
                                intake.deploy().andThen(intake.intakeWithoutRunEnd())),
                        PATHFINDER.moveToPose(endDepo, AutonParams.SLOW_DEPO),
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

package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.LookupTables;

import static frc.robot.RobotContainer.mrKrabs;

public class ShootEightBalls {
    public static Command create() {
        return  mrKrabs.shoot(LookupTables.HOOD_HUB_ANGLE, LookupTables.SHOOTER_HUB_SPEED, IndexerConstants.INDEXER_SHORT_SPEED).withTimeout(10.0);

    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;


import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.Throat;
import frc.robot.subsystems.indexer.ThroatIOReal;
import frc.robot.subsystems.indexer.ThroatIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.AutoAlign;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.stageManager.MrKrabs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.lib.Pathfinder;
import frc.robot.subsystems.limelight.Limelight;
import frc.lib.LimelightHelpers;

import frc.robot.autonomous.paths.*;

public class RobotContainer {
  public static MrKrabs mrKrabs;

  public static Intake intake;
  public static Indexer indexer;
  public static Throat throat;
  public static Shooter shooter;
  public static AutoAlign autoAligner; // utility for shooting auto-align

  public static final Swerve drivetrain = TunerConstants.createDrivetrain();
  public static final Pathfinder PATHFINDER = new Pathfinder(drivetrain);

  public static final Limelight limelightFRONT = new Limelight("limelight-four", drivetrain);
  public static final Limelight limelightBACK = new Limelight("limelight-five", drivetrain);
  // private final LimelightDebug limelightDebugFRONT = new LimelightDebug(limelightFRONT);
  // private final LimelightDebug limelightDebugBACK = new LimelightDebug(limelightBACK);

  private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

  private SendableChooser<Command> autoSideSelector = new SendableChooser<>();
  private SendableChooser<Command> autoSelector = new SendableChooser<>();

  private static Optional<Alliance> alliance = null;

  public static Boolean right = false;

  public RobotContainer() {
    if (RobotBase.isReal()) {
      drivetrain.registerTelemetry(logger::telemeterize);
      intake = new Intake(new IntakeIOReal());
      indexer = new Indexer(new IndexerIOReal());
      throat = new Throat(new ThroatIOReal());
      shooter = new Shooter(new ShooterIOReal(Rotations.of(0.0)));
      autoAligner = new AutoAlign();
      mrKrabs = new MrKrabs(intake, indexer, throat, shooter);

      // limelightDebugFRONT.startPeriodic(0.02);
      // limelightDebugBACK.startPeriodic(0.02);

    } else {
      drivetrain.registerTelemetry(logger::telemeterize);
      intake = new Intake(new IntakeIOSim());
      indexer = new Indexer(new IndexerIOSim());
      throat = new Throat(new ThroatIOSim());
      shooter = new Shooter(new ShooterIOSim());
      autoAligner = new AutoAlign();
      mrKrabs = new MrKrabs(intake, indexer, throat, shooter);
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    Boolean blue = alliance.get().equals(Alliance.Blue);

    // autoSideSelector.setDefaultOption("right", sideSelect(true));
    // autoSideSelector.addOption("right", sideSelect(true));
    // autoSideSelector.addOption("left", sideSelect(false));  

    autoSelector.setDefaultOption("right renee", TearAuton.tearAuton(blue, true));
		autoSelector.addOption("depo left", DepoShootLeft.DepoAutonLeft(blue));
		autoSelector.addOption("depo mid", DepoShootMid.DepoAutonMid(blue));
		autoSelector.addOption("shoot 8 balls", ShootEightBalls.create());

    autoSelector.addOption("renee++ right",  ReneePlusPlus.ReneePlusPlus(blue, true));
    autoSelector.addOption("renee++ left",  ReneePlusPlus.ReneePlusPlus(blue, false));

    autoSelector.addOption("right hook", HookAuton.hookAuton(blue, true));
		autoSelector.addOption("right renee", TearAuton.tearAuton(blue, true));

    autoSelector.addOption("left hook", HookAuton.hookAuton(blue, false));
		autoSelector.addOption("left renee", TearAuton.tearAuton(blue, false));
		
		SmartDashboard.putData(autoSelector);
  	SmartDashboard.putData(autoSideSelector);

		SmartDashboard.putBoolean("Intake", false);

    configureBindings();
  }

  public static Optional<Alliance> getCachedAlliance() {
    if (alliance == null) alliance = DriverStation.getAlliance();
    return alliance;
  }

  public static Command sideSelect(boolean r) {
    return Commands.runOnce(() -> right = r);
  }

  /**
   * Start with mode 1 on the limelight, which uses MT1 for heading estimation
   * with a very low alpha.
   */
  public void limelightCalibration() {
    LimelightHelpers.SetIMUAssistAlpha(limelightFRONT.getLimelightName(), 0.001);
    LimelightHelpers.SetIMUMode(limelightFRONT.getLimelightName(), 1);
    LimelightHelpers.SetIMUAssistAlpha(limelightBACK.getLimelightName(), 0.001);
    LimelightHelpers.SetIMUMode(limelightBACK.getLimelightName(), 1);

    // note to self, instead of TV use trust.
  }

  public void setLimelightUseMegaTag2(boolean use) {
    limelightFRONT.setUseMegaTag2(use);
	  limelightBACK.setUseMegaTag2(use);
  }

  /**
   * Called during disabled periodic to let limelights attempt a full pose + heading
   * correction when confident. Also ensures limelights are in IMU mode 1.
   */
  public void disabledPeriodicVisionUpdate() {
    if (limelightFRONT.estimate != null && limelightFRONT.estimate.tagCount >= 2)
    {
      limelightFRONT.disabledUpdate();
    } else {
      limelightBACK.disabledUpdate();
    }
  }

  public void setLimelightIMUMode(String name, int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }


  // private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //           .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //           .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  // private static final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);
  // private static final SlewRateLimiter slewRateX = new SlewRateLimiter(SLEW_RATE_TRANSLATION.magnitude());
  // private static final SlewRateLimiter slewRateY = new SlewRateLimiter(SLEW_RATE_TRANSLATION.magnitude());

  private void configureBindings() {
    // TestControls.configureKingstonBindings();
    // TestControls.configureLimelightBindings();
    DriverControls.configureBindings();
    OperatorControls.configureBindings();
  }

    public Command getAutonomousCommand() {
      return autoSelector.getSelected();
    }
}

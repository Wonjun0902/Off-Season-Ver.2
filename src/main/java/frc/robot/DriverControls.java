package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.RobotContainer.autoAligner;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.mrKrabs;
import static frc.robot.subsystems.swerve.TunerConstants.MaxAngularRate;
import static frc.robot.subsystems.swerve.TunerConstants.MaxSpeed;
import static frc.robot.subsystems.swerve.TunerConstants.SLEW_RATE_TRANSLATION;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.AutoAlign;
import frc.robot.subsystems.swerve.TunerConstants;

public class DriverControls {
	public static final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);

	public static final SlewRateLimiter slewRateX = new SlewRateLimiter(SLEW_RATE_TRANSLATION.magnitude());
	public static final SlewRateLimiter slewRateY = new SlewRateLimiter(SLEW_RATE_TRANSLATION.magnitude());
	/* Setting up bindings for necessary control of the swerve drive platform */
	public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	public static final SwerveRequest.FieldCentricFacingAngle testDrive = new SwerveRequest.FieldCentricFacingAngle() 
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
			.withHeadingPID(5.0, 0, 0.0);
			
	// private static final SwerveRequest.PointWheelsAt point = new
	// SwerveRequest.PointWheelsAt();
	// private static final SwerveRequest.FieldCentricFacingAngle angleLock = new
	// SwerveRequest.FieldCentricFacingAngle()
	// .withHeadingPID(3.0, 0.0, 0.1);speedLimiter

	/**
	 * Default drive controls
	 * 
	 * @param maxSpeed value from 0 to 1
	 * @return
	 */
	public static final SwerveRequest defaultDriveControls(double maxSpeed) {
		return drive
				.withVelocityX(slewRateX.calculate(-DRIVER_CONTROLLER.getLeftY()
						* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
						* maxSpeed))
				.withVelocityY(slewRateY.calculate(-DRIVER_CONTROLLER.getLeftX()
						* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
						* maxSpeed))
				.withRotationalRate(-DRIVER_CONTROLLER.getRightX()
						* TunerConstants.MaxAngularRate
						* TunerConstants.rotationalSpeedLimiter);
	}

	private static Rotation2d m_desiredHeading = new Rotation2d();

	public static final SwerveRequest testDefaultDriveControls(double maxSpeed) {
		double thetaInput = -DRIVER_CONTROLLER.getRightX();

		if (Math.abs(thetaInput) > 0.1) {
			double angleChange = thetaInput * TunerConstants.MaxAngularRate * TunerConstants.rotationalSpeedLimiter * 0.02;
			m_desiredHeading = m_desiredHeading.plus(Rotation2d.fromRadians(angleChange));
		}

		return testDrive
				.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)

				.withVelocityX(slewRateX.calculate(-DRIVER_CONTROLLER.getLeftY()
						* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
						* maxSpeed))
				.withVelocityY(slewRateY.calculate(-DRIVER_CONTROLLER.getLeftX()
						* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
						* maxSpeed))
						
				.withTargetDirection(m_desiredHeading);
	}

	// private static Rotation2d getAngleToHub() {
	// // 1. Get the translation (ignore rotation)
	// Translation2d robotLocation =
	// drivetrain.getCachedState().Pose.getTranslation();

	// // 2. Subtract Robot from Hub to get the vector pointing TO the hub
	// // Vector = Target - Source
	// Translation2d targetHub = RobotContainer.getCachedAlliance()
	// .map(alliance -> (alliance == Alliance.Red) ? RED_HUB_POSE : BLUE_HUB_POSE)
	// .orElse(BLUE_HUB_POSE);
	// Translation2d targetVector = targetHub.minus(robotLocation);

	// // 3. Get the angle of that vector relative to the field's X-axis
	// // This returns an angle where 0 is the positive X axis.
	// Rotation2d fieldRelativeAngle = targetVector.getAngle();

	// // LOGGING: Check if this angle makes sense
	// // If you are standing at (5,5) and hub is at (10,5), angle should be 0.
	// // If you are at (10,10) and hub is at (10,5), angle should be -90.
	// SmartDashboard.putNumber("Field Target Angle",
	// fieldRelativeAngle.getDegrees());
	// SmartDashboard.putNumber("Current Target Angle",
	// drivetrain.getCachedState().Pose.getRotation().getDegrees());
	// return new Rotation2d(Radians.of(fieldRelativeAngle.getRadians()));
	// }

	public static void configureBindings() {
		SmartDashboard.putString("Drive mode", "Normal");

		/**
		 * DRIVING
		 * 
		 * Default driving based. This is field-centric.
		 */
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
			Commands.runOnce(() -> SmartDashboard.putString("Drive mode", "Normal")).alongWith(
				drivetrain.applyRequest(
						() -> defaultDriveControls(TunerConstants.speedLimiter))
			)
		);

		DRIVER_CONTROLLER.y().whileTrue(
			autoAligner.alignToHub().alongWith(
			mrKrabs.shootFromEverywhere()));

		// back up hub preset
		DRIVER_CONTROLLER.povUp().whileTrue(
				(autoAligner.align(AutoAlign.ROBOT_HUB_ANGLE).withTimeout(0.5)
				.andThen(drivetrain.applyRequest(() -> brake)))
				.alongWith(mrKrabs.shoot(ShooterConstants.LookupTables.HOOD_HUB_ANGLE,
										ShooterConstants.LookupTables.SHOOTER_HUB_SPEED,
										IndexerConstants.INDEXER_LONG_SPEED)));

		DRIVER_CONTROLLER.x().whileTrue(
				(autoAligner.align(AutoAlign.ROBOT_TOWER_FRONT_ANGLE).withTimeout(0.5)
				.andThen(drivetrain.applyRequest(() -> brake)))
				.alongWith(mrKrabs.shoot(ShooterConstants.LookupTables.HOOD_TOWER_ANGLE,
										ShooterConstants.LookupTables.SHOOTER_TOWER_SPEED,
										IndexerConstants.INDEXER_LONG_SPEED)));
		// SHOOT @ SIDE OF TOWER

		DRIVER_CONTROLLER.a().whileTrue(
				(autoAligner.alignLeftOrRight(AutoAlign.ROBOT_TOWER_LEFT_ANGLE, AutoAlign.ROBOT_TOWER_RIGHT_ANGLE).withTimeout(0.5)
				.andThen(drivetrain.applyRequest(() -> brake)))
				.alongWith(mrKrabs.shoot(ShooterConstants.LookupTables.HOOD_BACK_ANGLE,
										ShooterConstants.LookupTables.SHOOTER_BACK_SPEED,
										IndexerConstants.INDEXER_LONG_SPEED)));

		DRIVER_CONTROLLER.leftTrigger().whileTrue(
				(autoAligner.alignLeftOrRight(AutoAlign.RENEE_BUMP_ANGLE_LEFT, AutoAlign.RENEE_BUMP_ANGLE_RIGHT).withTimeout(0.5)
				.andThen(drivetrain.applyRequest(() -> brake)))
				.alongWith(mrKrabs.shoot(ShooterConstants.LookupTables.HOOD_HUB_ANGLE,
										ShooterConstants.LookupTables.SHOOTER_RENEE_SPEED,
										IndexerConstants.INDEXER_LONG_SPEED)));

		// PASSING
		DRIVER_CONTROLLER.b()
				.whileTrue(
				  	autoAligner.alignForPassing(
						() -> slewRateX.calculate(-DRIVER_CONTROLLER.getLeftY()
							* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
							* TunerConstants.speedLimiter),
						() -> slewRateY.calculate(-DRIVER_CONTROLLER.getLeftX()
							* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
							* TunerConstants.speedLimiter)
					).alongWith(
				mrKrabs.shootFromEverywhere()
				)
			);

		DRIVER_CONTROLLER.leftBumper().onTrue(Commands.runOnce(() -> autoAligner.alignToLeft()));
		DRIVER_CONTROLLER.rightBumper().onTrue(Commands.runOnce(() -> autoAligner.alignToRight()));

		// OVERDRIVEEE!!!!!!!!!!!!!!!!!!!
		DRIVER_CONTROLLER.rightTrigger().whileTrue(
			Commands.runOnce(() -> SmartDashboard.putString("Drive mode", "OVERDRIVE")).alongWith(
				drivetrain.applyRequest(
						() -> defaultDriveControls(TunerConstants.overdriveSpeedLimiter))
			)
		);

		// .and(DRIVER_CONTROLLER.povUp()).whileTrue(
		// drivetrain.applyRequest(() ->
		// drive
		// .withVelocityX(CROSS_SPEED)
		// .withVelocityY(0.0)
		// .withRotationalRate(0.0)
		// )
		// );
		// // cross bump down
		// DRIVER_CONTROLLER.leftTrigger().and(DRIVER_CONTROLLER.povDown()).whileTrue(
		// drivetrain.applyRequest(() ->
		// drive
		// .withVelocityX(-CROSS_SPEED)
		// .withVelocityY(0.0)
		// .withRotationalRate(0.0)
		// )
		// );

		/**
		 * BRAKE
		 */
		// DRIVER_CONTROLLER.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
		// brake));

		/**
		 * SEED FIELD CENTRIC
		 */
		DRIVER_CONTROLLER.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		/**
		 * ##########################################
		 * DEBUG CONTROLS
		 * ##########################################
		 */

		/*
		 * 
		 * // Idle while the robot is disabled. This ensures the configured
		 * // neutral mode is applied to the drive motors while disabled.
		 * final var idle = new SwerveRequest.Idle();
		 * RobotModeTriggers.disabled().whileTrue(
		 * drivetrain.applyRequest(() -> idle).ignoringDisable(true));
		 * 
		 * DRIVER_CONTROLLER.a().whileTrue(drivetrain.applyRequest(() -> brake));
		 * DRIVER_CONTROLLER.b().whileTrue(drivetrain.applyRequest(
		 * () -> point.withModuleDirection(new Rotation2d(-DRIVER_CONTROLLER.getLeftY(),
		 * -DRIVER_CONTROLLER.getLeftX()))
		 * ));
		 * 
		 * // Run SysId routines when holding back/start and X/Y.
		 * // Note that each routine should be run exactly once in a single log.
		 * DRIVER_CONTROLLER.back().and(DRIVER_CONTROLLER.y()).whileTrue(drivetrain.
		 * sysIdDynamic(Direction.kForward));
		 * DRIVER_CONTROLLER.back().and(DRIVER_CONTROLLER.x()).whileTrue(drivetrain.
		 * sysIdDynamic(Direction.kReverse));
		 * DRIVER_CONTROLLER.start().and(DRIVER_CONTROLLER.y()).whileTrue(drivetrain.
		 * sysIdQuasistatic(Direction.kForward));
		 * DRIVER_CONTROLLER.start().and(DRIVER_CONTROLLER.x()).whileTrue(drivetrain.
		 * sysIdQuasistatic(Direction.kReverse));
		 * 
		 * // reset the field-centric heading on left bumper press
		 * DRIVER_CONTROLLER.leftBumper().onTrue(drivetrain.runOnce(() ->
		 * drivetrain.seedFieldCentric()));
		 * DRIVER_CONTROLLER.x().whileTrue(shooter.shoot());
		 * DRIVER_CONTROLLER.leftTrigger().whileTrue(
		 * drivetrain.applyRequest(
		 * () -> angleLock
		 * //.withVelocityX(slewRateX.calculate(DRIVER_CONTROLLER.getLeftY() *
		 * MaxSpeed)) // Drive
		 * //.withVelocityY(slewRateY.calculate(DRIVER_CONTROLLER.getLeftX() *
		 * MaxSpeed))
		 * //.withTargetDirection(getAngleToHub())
		 * 
		 * )
		 * .until(() ->
		 * drivetrain.getCachedState().Pose.getRotation().getMeasure().isNear(
		 * getAngleToHub().getMeasure(), 2))
		 * .until(() ->
		 * drivetrain.getCachedState().Pose.getRotation().getMeasure().isNear(Degrees.of
		 * (60), 2))
		 * .andThen(drivetrain.applyRequest(() -> brake))
		 * );
		 * 
		 */
	}
}

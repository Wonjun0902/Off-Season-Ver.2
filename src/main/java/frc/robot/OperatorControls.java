package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.RobotContainer.indexer;
import static frc.robot.RobotContainer.intake;
import static frc.robot.RobotContainer.shooter;
import static frc.robot.RobotContainer.throat;

public class OperatorControls {
	public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);
	
	public static void configureBindings() {
		/**
		 * Agitation
		 */
		OPERATOR_CONTROLLER.y().onTrue(intake.agitateM1());
		OPERATOR_CONTROLLER.b().onTrue(intake.agitateM2());
		OPERATOR_CONTROLLER.a().onTrue(intake.agitateFull());

		/**
		 * Override intake @ full speed
		 */
		// OPERATOR_CONTROLLER.leftTrigger().whileTrue(intake.intake(INTAKE_FULL_SPEED));

		/**+
		 * Auto-align to left or right position (for trench and back tower positions)
		 */
		OPERATOR_CONTROLLER.rightBumper().whileTrue(throat.outtake().alongWith(indexer.outFeed()));
		OPERATOR_CONTROLLER.leftBumper().onTrue(intake.deploy());
		OPERATOR_CONTROLLER.x().onTrue(intake.retract());

		OPERATOR_CONTROLLER.leftTrigger().whileTrue(intake.outtake());
		OPERATOR_CONTROLLER.rightTrigger().whileTrue(intake.intake());

		OPERATOR_CONTROLLER.start().onTrue(Commands.runOnce(shooter.resetTrims()));

		OPERATOR_CONTROLLER.povUp().onTrue(Commands.runOnce(shooter.trimAimUp()));
        OPERATOR_CONTROLLER.povDown().onTrue(Commands.runOnce(shooter.trimAimDown()));
        OPERATOR_CONTROLLER.povRight().onTrue(Commands.runOnce(shooter.trimSpeedUp()));
        OPERATOR_CONTROLLER.povLeft().onTrue(Commands.runOnce(shooter.trimSpeedDown()));
	}
}

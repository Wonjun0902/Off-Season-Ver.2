package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DriverControls;
import frc.robot.subsystems.swerve.TunerConstants;

import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.getCachedAlliance;

import java.util.function.Supplier;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;


/**
 * Class that manages the bot auto-angle-alignment logic for shooting at set locations.
 */
public class AutoAlign {
    /** 
     * When using buttons for shooting at preset bot positions,
     * should we use left side values or right side values to 
     * calculate auto-alignment for the bot's rotation 
     */
    private boolean isAutoAlignLeft = true;

    private static final SwerveRequest.FieldCentricFacingAngle angleLock = new SwerveRequest.FieldCentricFacingAngle()
	    .withHeadingPID(5.5, 0.0, 0.3)
        .withMaxAbsRotationalRate(TunerConstants.MaxAutoAlignAngularRate);

    /**
     * Set alignment to left for future alignment requests
     * @return
     */
    public void alignToLeft() {
        isAutoAlignLeft = true;
    }

    /**
     * Set alignment to right for future alignment requests
     * @return
     */
    public void alignToRight() {
        isAutoAlignLeft = false;
    }

    /**
     * Get the alignment value.
     * Left alignment is true
     * Right alignment is false
     * @return
     */
    public boolean getAlignmentValue() {
        return isAutoAlignLeft;
    }

    /**
     * Apply a drivetrain request to align to the given setpoint, considering the offset.
     * @param setpoint
     * @return
     */
    public Command align(Angle setpoint) {
        // Note: beware of incorrect poses that might mess up the auto alignment

        Rotation2d target = new Rotation2d(setpoint);
        
        return drivetrain.applyRequest(() ->
                angleLock
                    .withTargetDirection(target)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                    .withVelocityX(0)
                    .withVelocityY(0)
            ).beforeStarting(() -> angleLock.HeadingController.reset());
    }
    

    /**
     * Apply a drivetrain request to align to the given setpoint, considering the offset.
     * Use left or right based on the previously set alignment value.
     * @param leftSetpoint
     * @param rightSetpoint
     * @return
     */
    public Command alignLeftOrRight(Angle leftSetpoint, Angle rightSetpoint) {
        return Commands.either(
            align(leftSetpoint).until(() -> !isAutoAlignLeft), 
            align(rightSetpoint).until(() -> isAutoAlignLeft), 
            () -> isAutoAlignLeft
        );
    }

    public Command alignForPassing(Supplier<Double> vx, Supplier<Double> vy) {
        Rotation2d target = new Rotation2d(Degrees.of(180));
        return drivetrain.applyRequest(() -> {
            return angleLock 
                .withTargetDirection(target)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(vx.get())
                .withVelocityY(vy.get());
        }).beforeStarting(() -> angleLock.HeadingController.reset());
    }
    public static Translation2d getCachedHub() {
        return getCachedAlliance().get().equals(Alliance.Blue) ? ShooterConstants.Field.BLUE_HUB_POSE : ShooterConstants.Field.RED_HUB_POSE;
    }
    /**
     * Calculate the angle from the robot to the hub, which will be used for auto-alignment.
     * @return the angle to the hub in field-relative coordinates, where 0 degrees is along the +X axis of the field
     */
    public static Rotation2d getAngleToHub() {
		// 1. Get the translation (ignore rotation)
		Translation2d robotLocation = drivetrain.getCachedState().Pose.getTranslation();
		
		// 2. Subtract Robot from Hub to get the vector pointing TO the hub
		// Vector = Target - Source
		Translation2d targetHub = getCachedHub();
		Translation2d targetVector = targetHub.minus(robotLocation);
		
		// 3. Get the angle of that vector relative to the field's X-axis
		// This returns an angle where 0 is the positive X axis.
		Rotation2d fieldRelativeAngle = targetVector.getAngle();
		
		// LOGGING: Check if this angle makes sense
		// If you are standing at (5,5) and hub is at (10,5), angle should be 0.
		// If you are at (10,10) and hub is at (10,5), angle should be -90.
		SmartDashboard.putNumber("Field Target Angle", fieldRelativeAngle.getDegrees());
		return fieldRelativeAngle;
	}

    /**
     * Apply a drivetrain request to align to the hub, considering the offset.
     * @return the command to align the drivetrain such that it faces the hub
     */
    public Command alignToHub() {
        return drivetrain.applyRequest(() -> {
            // Rotation2d current = drivetrain.getCachedState().Pose.getRotation();

            // double error = target.minus(current).getDegrees();
            // if (Math.abs(error) <= 2.0) return DriverControls.brake;

            return angleLock
                .withTargetDirection(getAngleToHub())
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withVelocityX(0)
                .withVelocityY(0);
        }).beforeStarting(() -> angleLock.HeadingController.reset())
        .withTimeout(1.0)
        .until(() -> {
            Rotation2d target = getAngleToHub();
            Rotation2d current = drivetrain.getCachedState().Pose.getRotation();

            double error = target.minus(current).getDegrees();
            return (Math.abs(error) <= 2.0);
        }).andThen(drivetrain.applyRequest(() -> DriverControls.brake));
    }
}

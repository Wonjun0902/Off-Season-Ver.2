package frc.lib;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TunerConstants;

import java.util.OptionalDouble;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

public class Pathfinder {
    private final Swerve m_swerve;
    private final PIDController moveController = new PIDController(2.5, 0, 0.1);
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public Pathfinder(Swerve swerve) {
        this.m_swerve = swerve;
        moveController.setTolerance(0.05);
    }

    /**
     * Parameters to customize the behavior of the MoveToPose command.
     */
    public static class MoveToPoseParams {
        private final OptionalDouble minSpeed;
        private final OptionalDouble maxSpeed;
        private final OptionalDouble earlyExitRange;

        private MoveToPoseParams(OptionalDouble minSpeed, OptionalDouble maxSpeed, OptionalDouble earlyExitRange) {
            this.minSpeed = minSpeed;
            this.maxSpeed = maxSpeed;
            this.earlyExitRange = earlyExitRange;
        }

        public OptionalDouble minSpeed() {
            return minSpeed;
        }

        public OptionalDouble maxSpeed() {
            return maxSpeed;
        }

        public OptionalDouble earlyExitRange() {
            return earlyExitRange;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static MoveToPoseParams empty() {
            return builder().build();
        }

        public static class Builder {
            private OptionalDouble min = OptionalDouble.empty();
            private OptionalDouble max = OptionalDouble.empty();
            private OptionalDouble early = OptionalDouble.empty();

            public Builder minSpeed(double v) {
                this.min = OptionalDouble.of(v);
                return this;
            }

            public Builder maxSpeed(double v) {
                this.max = OptionalDouble.of(v);
                return this;
            }

            public Builder earlyExitRange(double v) {
                this.early = OptionalDouble.of(v);
                return this;
            }

            public MoveToPoseParams build() {
                return new MoveToPoseParams(min, max, early);
            }
        }
    }

    /**
     * Drives the robot to a field-space coordinate using a direct-path vector.
     * * @param x Target X coordinate on the field in meters.
     * 
     * @param y      Target Y coordinate on the field in meters.
     * @param theta  Target rotation in degrees.
     * @param params Configuration for speed limits and exit conditions.
     * @return A command that ends when the robot arrives at the target.
     */
    public Command moveToPose(double x, double y, double theta, MoveToPoseParams params) {
        return m_swerve.run(() -> {
            Pose2d currentPose = m_swerve.getCachedState().Pose;

            double xError = x - currentPose.getX();
            double yError = y - currentPose.getY();
            double distance = Math.hypot(xError, yError);

            // the setpoint should stay the same but measurement be changing
            // closing the distance will now be negative but we use aps of this anyway
            double translationOutput = moveController.calculate(distance, 0);

            double configuredMax = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * TunerConstants.speedLimiter;
            double configuredMaxTurn = TunerConstants.MaxAutonomousAngularRate;

            double allowedMax = configuredMax;
            if (params != null && params.maxSpeed().isPresent()) {
                allowedMax = Math.min(params.maxSpeed().getAsDouble(), configuredMax);
            }

            double allowedMin = 0.0;
            if (params != null && params.minSpeed().isPresent()) {
                // should be the smaller of the min speed or the allowable max (not configured
                // max)
                // prevents a logic error of min = 2, max = 1 being passed for path
                allowedMin = Math.min(params.minSpeed().getAsDouble(), allowedMax);
            }

            double speed = Math.abs(translationOutput);
            if (speed > 1e-9) {
                speed = MathUtil.clamp(speed, allowedMin, allowedMax);
            } else {
                speed = 0;
            }

            double xSpeed = 0;
            double ySpeed = 0;
            if (distance > 1e-9) {
                xSpeed = (xError / distance) * speed;
                ySpeed = (yError / distance) * speed;
            }

            double currentAngle = currentPose.getRotation().getRadians();
            double targetAngle = Math.toRadians(theta);
            double angleError = Math.atan2(Math.sin(targetAngle - currentAngle), Math.cos(targetAngle - currentAngle));
            double kPRot = 4.0;
            double rotRate = MathUtil.clamp(kPRot * angleError, -configuredMaxTurn, configuredMaxTurn);

            m_swerve.setControl(
                    driveRequest
                            .withVelocityX(xSpeed)
                            .withVelocityY(ySpeed)
                            .withRotationalRate(rotRate)
                            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance));
        })
                .beforeStarting(moveController::reset)
                .until(() -> {
                    Pose2d currentPose = m_swerve.getCachedState().Pose;
                    double xErr = Math.abs(currentPose.getX() - x);
                    double yErr = Math.abs(currentPose.getY() - y);

                    double angleError = Math.abs(currentPose.getRotation().getDegrees() - theta);
                    if (angleError > 180)
                        angleError = 360 - angleError;

                    if (params != null && params.earlyExitRange().isPresent()) {
                        double dist = currentPose.getTranslation()
                                .getDistance(new edu.wpi.first.math.geometry.Translation2d(x, y));
                        if (dist < params.earlyExitRange().getAsDouble())
                            return true;
                    }

                    return xErr < 0.05 && yErr < 0.05 && (angleError < 2.0);
                })
                .withTimeout(3.0)
                .finallyDo(() -> m_swerve.stop());
    }

    /**
     * Convenience overload for moveToPose using default parameters.
     * * @param target The target Pose2d on the field.
     * 
     * @return A command to move to the target.
     */
    public Command moveToPose(Pose2d target) {
        return moveToPose(target.getX(), target.getY(), target.getRotation().getDegrees(), MoveToPoseParams.empty());
    }

    /**
     * Convenience overload for moveToPose using a Pose2d and custom parameters.
     * * @param target The target Pose2d on the field.
     * 
     * @param params Configuration for speed limits and exit conditions.
     * @return A command to move to the target.
     */
    public Command moveToPose(Pose2d target, MoveToPoseParams params) {
        return moveToPose(target.getX(), target.getY(), target.getRotation().getDegrees(), params);
    }
}
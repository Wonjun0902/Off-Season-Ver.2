package frc.robot.subsystems.shooter;

import frc.lib.LazyFXS;
import frc.lib.LazyFXSBuilder;
import frc.lib.LazyTalon;
import frc.lib.LazyTalonBuilder;
import frc.robot.subsystems.shooter.ShooterConstants.Configurations.Shooter;
import frc.robot.subsystems.intake.IntakeConstants.Configurations;
import frc.robot.subsystems.shooter.ShooterConstants.Configurations.Hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOReal implements ShooterIO {

    private final LazyTalon leftShooter;
    private final LazyTalon rightShooter;
    private final LazyFXS hood;

    // private final CANcoder hoodCoder;

    public ShooterIOReal(Angle SENSOR_OFFSET) {
        leftShooter = new LazyTalonBuilder(
                Shooter.Left.MOTOR_ID,
                Configurations.CANBUS,
                Shooter.SENSOR_TO_MECH_RATIO, 
                InvertedValue.Clockwise_Positive, 
                Shooter.STATOR_CURRENT_LIMIT.magnitude(), 
                Shooter.SUPPLY_CURRENT_LIMIT.magnitude(),Shooter.SUPPLY_LOWER_LIMIT.in(Amps),Shooter.SUPPLY_LOWER_TIME.in(Seconds))
            .withPIDFConfiguration(
                Shooter.Left.kP, Shooter.Left.kI, Shooter.Left.kD, Shooter.Left.kS, Shooter.Left.kG, Shooter.Left.kV, Shooter.Left.kA, 
                GravityTypeValue.Elevator_Static, // Assuming logic remains Elevator_Static
               StaticFeedforwardSignValue.UseVelocitySign,
               0
            ).withFollower(Shooter.Left.FOLLOWER_LEFT_ID,Configurations.CANBUS,MotorAlignmentValue.Aligned).build();
        leftShooter.setCoast();

        rightShooter = new LazyTalonBuilder(
            Shooter.Right.MOTOR_ID,
            Configurations.CANBUS,
            Shooter.SENSOR_TO_MECH_RATIO, 
            InvertedValue.CounterClockwise_Positive, 
            Shooter.STATOR_CURRENT_LIMIT.magnitude(), 
            Shooter.SUPPLY_CURRENT_LIMIT.magnitude(),Shooter.SUPPLY_LOWER_LIMIT.in(Amps),Shooter.SUPPLY_LOWER_TIME.in(Seconds))
        .withPIDFConfiguration(
            Shooter.Right.kP, Shooter.Right.kI, Shooter.Right.kD, Shooter.Right.kS, Shooter.Right.kG, Shooter.Right.kV, Shooter.Right.kA, 
            GravityTypeValue.Elevator_Static, // Assuming logic remains Elevator_Static
            StaticFeedforwardSignValue.UseVelocitySign,
            0
            ).withFollower(Shooter.Right.FOLLOWER_RIGHT_ID,Configurations.CANBUS,MotorAlignmentValue.Aligned).build();
        rightShooter.setCoast();
       
        hood = new LazyFXSBuilder(
                Hood.MOTOR_ID,
                Configurations.CANBUS,
                MotorArrangementValue.NEO550_JST,
                Hood.SENSOR_MECH_RATIO, 
                InvertedValue.CounterClockwise_Positive,
                Hood.STATOR_CURRENT_LIMIT.magnitude(), 
                Hood.SUPPLY_CURRENT_LIMIT.magnitude()
            )
            .withRotorSensorRatio(Hood.ROTOR_SENSOR_RATIO)
            .withSoftLimits(true, Hood.SOFT_HI.magnitude(), true, Hood.SOFT_LO.magnitude())
            .withCANCoder(Hood.CANCODER_ID, Configurations.CANBUS, ExternalFeedbackSensorSourceValue.RemoteCANcoder, Hood.SENSOR_OFFSET, Hood.SENSOR_DIRECTION)
            .withPIDFConfiguration(
                Hood.kP, Hood.kI, Hood.kD, Hood.kS, Hood.kG, Hood.kV, Hood.kA, 
                GravityTypeValue.Elevator_Static,
                StaticFeedforwardSignValue.UseVelocitySign,
                0
            )
            .withMotionMagicConfiguration(Hood.EXPO_V, Hood.EXPO_A, Hood.CRUISE_VELOCITY, Hood.MAX_ACCELERATION).build();

    }

    @Override
    public void setShooterTargetSpeed(AngularVelocity rpm) {
        leftShooter.setTFOCVelocityTarget(rpm, Shooter.Left.CURRENT_FF, 0);
        rightShooter.setTFOCVelocityTarget(rpm, Shooter.Right.CURRENT_FF, 0);
    }

    @Override
    public void setShooterAngle(Angle angle) {
        hood.setMMPositionTarget(angle, 0);
    }

    @Override
    public void stop() {
        leftShooter.stop();
        rightShooter.stop();
        hood.stop();
    }

    @Override
    public void periodic() {
        // throw new UnsupportedOperationException("Operation unsupported");
    }

    @Override
    public Angle getHoodAngle() {
        return hood.getPosition();
    }
    @Override
    public AngularVelocity getLeftShooterVelocity() {
        return leftShooter.getVelocity();
    }
    @Override
    public AngularVelocity getRightShooterVelocity() {
        return rightShooter.getVelocity();
    }
}
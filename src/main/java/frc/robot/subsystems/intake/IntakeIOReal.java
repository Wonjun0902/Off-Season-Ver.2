package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.Configurations;
import frc.robot.subsystems.intake.IntakeConstants.Configurations.Intake;
import frc.robot.subsystems.intake.IntakeConstants.Configurations.Deployer;
import frc.lib.LazyTalon;
import frc.lib.LazyTalonBuilder;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeIOReal implements IntakeIO {
    private LazyTalon topRollerMotor, bottomRollerMotor, deployerMotor;

    public IntakeIOReal() {
        topRollerMotor = new LazyTalonBuilder(
            Intake.Top.MOTOR_ID,
            Configurations.CANBUS,
            Intake.Top.SENSOR_TO_MECH_RATIO, 
            Intake.Top.INVERTED_VALUE,
            Intake.Top.STATOR_CURRENT_LIMIT.in(Amps),
            Intake.Top.SUPPLY_CURRENT_LIMIT.in(Amps)
        )
            .withPIDFConfiguration(
                Intake.Top.kP,
                Intake.Top.kI,
                Intake.Top.kD,
                Intake.Top.kS,
                Intake.Top.kG,
                Intake.Top.kV,
                Intake.Top.kA,
                Intake.Top.GRAVITY_TYPE, 
                StaticFeedforwardSignValue.UseVelocitySign,
                0
            )
            .build();
        topRollerMotor.setCoast();
        
        bottomRollerMotor = new LazyTalonBuilder(
            Intake.Bottom.MOTOR_ID,
            Configurations.CANBUS,
            Intake.Bottom.SENSOR_TO_MECH_RATIO, 
            Intake.Bottom.INVERTED_VALUE,
            Intake.Bottom.STATOR_CURRENT_LIMIT.in(Amps),
            Intake.Bottom.SUPPLY_CURRENT_LIMIT.in(Amps)
        )
            .withPIDFConfiguration(
                Intake.Bottom.kP,
                Intake.Bottom.kI,
                Intake.Bottom.kD,
                Intake.Bottom.kS,
                Intake.Bottom.kG,
                Intake.Bottom.kV,
                Intake.Bottom.kA,
                Intake.Bottom.GRAVITY_TYPE, 
                StaticFeedforwardSignValue.UseVelocitySign,
                0
            )
            .build();
        bottomRollerMotor.setCoast();

        deployerMotor = new LazyTalonBuilder(
            Deployer.MOTOR_ID, 
            Configurations.CANBUS,
            1.0, 
            Deployer.INVERTED_VALUE,
            Deployer.STATOR_CURRENT_LIMIT.in(Amps),
            Deployer.SUPPLY_CURRENT_LIMIT.in(Amps)
        )
            .withRotorSensorRatio(Deployer.ROTOR_TO_SENSOR_RATIO)
            .withPIDFConfiguration(
                Deployer.kP,
                Deployer.kI,
                Deployer.kD,
                Deployer.kS,
                Deployer.kG,
                Deployer.kV,
                Deployer.kA,                
                Deployer.GRAVITY_TYPE, 
                StaticFeedforwardSignValue.UseVelocitySign,
                0
                )
            .withMotionMagicConfiguration(
                Deployer.EXPO_VELOCITY,
                Deployer.EXPO_ACCELERATION,
                Deployer.CRUISE_VELOCITY,
                Deployer.MAX_ACCELERATION
            )
            .withCANCoder(CANCODER_ID, Configurations.CANBUS, CANCODER_TYPE, CANCODER_OFFSET,CANCODER_DIRECTION)
            .withSoftLimits(true, Configurations.Deployer.FORWARD_LIMIT, true, Configurations.Deployer.REVERSE_LIMIT)
            .build();
        deployerMotor.setBrake();
    }

    @Override
    public void moveDeployerTo(Angle pos) {
        deployerMotor.setMMPositionTarget(pos,0);
    }

    @Override
    public void runIntake(AngularVelocity vel) {
        topRollerMotor.setTFOCVelocityTarget(vel,Intake.Top.CURRENT_FF,0);
        bottomRollerMotor.setTFOCVelocityTarget(vel,Intake.Bottom.CURRENT_FF, 0);
    }

    @Override
    public void stopIntake() {
        topRollerMotor.stop();
        bottomRollerMotor.stop();
    }

    @Override
    public void stopDeployer() {
        deployerMotor.stop();
    }

    @Override
    public void periodic() {
        throw new Error("IntakeIOReal's periodic() should not be called");
    }

    @Override
    public Voltage getTopIntakeVoltage() {
        return topRollerMotor.getMotor().getMotorVoltage().getValue();
    }
    @Override
    public Current getTopIntakeSupplyCurrent() {
        return topRollerMotor.getMotor().getSupplyCurrent().getValue();
    }
    @Override
    public AngularVelocity getTopIntakeVelocity() {
        return topRollerMotor.getVelocity();
    }
    @Override
    public Angle getTopIntakePosition() {
        return topRollerMotor.getPosition();
    }
    @Override
    public Voltage getBottomIntakeVoltage() {
        return bottomRollerMotor.getMotor().getMotorVoltage().getValue();
    }
    @Override
    public Current getBottomIntakeSupplyCurrent() {
        return bottomRollerMotor.getMotor().getSupplyCurrent().getValue();
    }
    @Override
    public AngularVelocity getBottomIntakeVelocity() {
        return bottomRollerMotor.getVelocity();
    }
    @Override
    public Angle getBottomIntakePosition() {
        return bottomRollerMotor.getPosition();
    }

    @Override
    public Voltage getDeployerVoltage() {
        return deployerMotor.getMotor().getMotorVoltage().getValue();
    }
    @Override
    public Current getDeployerSupplyCurrent() {
        return deployerMotor.getMotor().getSupplyCurrent().getValue();
    }
    @Override
    public AngularVelocity getDeployerVelocity() {
        return deployerMotor.getVelocity();
    }
    @Override
    public Angle getDeployerPosition() {
        return deployerMotor.getPosition();
    }
}
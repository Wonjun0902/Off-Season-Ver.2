package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.subsystems.intake.IntakeConstants.Configurations.Deployer;
import frc.robot.subsystems.intake.IntakeConstants.Configurations.Intake;

public class IntakeIOSim implements IntakeIO {
    private TalonFX intakeMotor, deployerMotor;
    private TalonFXSimState intakeMotorSim, deployerMotorSim;

    private final DCMotorSim intakeMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.08, Intake.Bottom.SENSOR_TO_MECH_RATIO
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim deployerMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.08, Deployer.ROTOR_TO_SENSOR_RATIO
        ),
        DCMotor.getKrakenX60Foc(1)
    );
    
    public IntakeIOSim() {
        intakeMotor = new TalonFX(Intake.Bottom.MOTOR_ID);
        deployerMotor = new TalonFX(Deployer.MOTOR_ID);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.Slot0.kP = Intake.Bottom.kP;
        intakeConfig.Slot0.kI = Intake.Bottom.kI;
        intakeConfig.Slot0.kD = Intake.Bottom.kD;
        intakeConfig.Feedback.SensorToMechanismRatio = Intake.Bottom.SENSOR_TO_MECH_RATIO;
        intakeConfig.MotionMagic.MotionMagicCruiseVelocity = Intake.Bottom.CRUISE_VELOCITY;
        intakeConfig.MotionMagic.MotionMagicAcceleration = Intake.Bottom.MAX_ACCELERATION;
        intakeConfig.MotionMagic.MotionMagicExpo_kV = Intake.Bottom.kV;
        intakeConfig.MotionMagic.MotionMagicExpo_kA = Intake.Bottom.kA;
        intakeMotor.getConfigurator().apply(intakeConfig);


        TalonFXConfiguration deployerConfig = new TalonFXConfiguration();
        deployerConfig.Slot0.kP = Deployer.kP;
        deployerConfig.Slot0.kI = Deployer.kI;
        deployerConfig.Slot0.kD = Deployer.kD;
        deployerConfig.Feedback.SensorToMechanismRatio = Deployer.ROTOR_TO_SENSOR_RATIO;
        deployerConfig.MotionMagic.MotionMagicCruiseVelocity = Deployer.CRUISE_VELOCITY.magnitude();
        deployerConfig.MotionMagic.MotionMagicAcceleration = Deployer.MAX_ACCELERATION.magnitude();
        deployerConfig.MotionMagic.MotionMagicExpo_kV = Deployer.kV;
        deployerConfig.MotionMagic.MotionMagicExpo_kA = Deployer.kA;
        deployerMotor.getConfigurator().apply(deployerConfig);

        // Get sim state AFTER creating motors
        intakeMotorSim = intakeMotor.getSimState();
        deployerMotorSim = deployerMotor.getSimState();
        
        // CRITICAL: Set supply voltage for simulation
        intakeMotorSim.setSupplyVoltage(12.0);
        deployerMotorSim.setSupplyVoltage(12.0);
    }

    @Override
    public void moveDeployerTo(Angle pos) {
        deployerMotor.setControl(new MotionMagicExpoVoltage(pos));
    }

    @Override
    public void runIntake(AngularVelocity vel) {
        intakeMotor.setControl(new MotionMagicVelocityVoltage(vel));
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    @Override
    public void stopDeployer() {
        deployerMotor.stopMotor();
    }

    public void periodic() {
        // 1. Apply voltage from Talon to the physics sim
        intakeMotorModel.setInputVoltage(intakeMotorSim.getMotorVoltage());
        deployerMotorModel.setInputVoltage(deployerMotorSim.getMotorVoltage());
        
        // 2. Advance the physics world (20ms)
        intakeMotorModel.update(0.020);
        deployerMotorModel.update(0.020);
        
        // 3. Update the Talon's internal sensors from the physics world
        intakeMotorSim.setRawRotorPosition(intakeMotorModel.getAngularPositionRotations());
        deployerMotorSim.setRawRotorPosition(deployerMotorModel.getAngularPositionRotations());

        intakeMotorSim.setRotorVelocity(intakeMotorModel.getAngularVelocityRPM() / 60.0);
        deployerMotorSim.setRotorVelocity(deployerMotorModel.getAngularVelocityRPM() / 60.0);
    }

    @Override
    public Voltage getTopIntakeVoltage() {
        return intakeMotorSim.getMotorVoltageMeasure();
    }
    @Override
    public Current getTopIntakeSupplyCurrent() {
        return intakeMotorSim.getSupplyCurrentMeasure();
    }
    @Override
    public AngularVelocity getTopIntakeVelocity() {
        return intakeMotor.getVelocity().getValue();
    }
    @Override
    public Angle getTopIntakePosition() {
        return intakeMotor.getPosition().getValue();
    }
    @Override
    public Voltage getBottomIntakeVoltage() {
        return getTopIntakeVoltage();
    }
    @Override
    public Current getBottomIntakeSupplyCurrent() {
        return getTopIntakeSupplyCurrent();
    }
    @Override
    public AngularVelocity getBottomIntakeVelocity() {
        return getTopIntakeVelocity();
    }
    @Override
    public Angle getBottomIntakePosition() {
        return getTopIntakePosition();
    }

    public Voltage getDeployerVoltage() {
        return deployerMotorSim.getMotorVoltageMeasure();
    }
    public Current getDeployerSupplyCurrent() {
        return deployerMotorSim.getSupplyCurrentMeasure();
    }
    public AngularVelocity getDeployerVelocity() {
        return deployerMotor.getVelocity().getValue();
    }
    public Angle getDeployerPosition() {
        return deployerMotor.getPosition().getValue();
    }
}

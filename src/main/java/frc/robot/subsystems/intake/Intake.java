package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private IntakeIO io;

    public final Trigger isIntaking;
    public final Trigger isOuttaking;

    public final Trigger isAtM1;
    public final Trigger isAtM2;

    public final Trigger isDeployed;
    public final Trigger isRetracted;

    public Intake(IntakeIO io) {
        this.io = io;

        isIntaking = new Trigger(() -> io.getTopIntakeVelocity().lt(RotationsPerSecond.of(-1.0)));
        isOuttaking = new Trigger(() -> io.getTopIntakeVelocity().gt(RotationsPerSecond.of(1.0)));
        
        isDeployed = new Trigger(() -> io.getDeployerPosition().isNear(DEPLOYED_SETPOINT, TOLERANCE_POSITION_DEPLOY));
        isRetracted = new Trigger(() -> io.getDeployerPosition().isNear(RETRACTED_SETPOINT, TOLERANCE_POSITION_DEPLOY));

        isAtM1 = new Trigger(() -> io.getDeployerPosition().isNear(MID_1_SETPOINT, TOLERANCE_POSITION_DEPLOY));
        isAtM2 = new Trigger(() -> io.getDeployerPosition().isNear(MID_2_SETPOINT, TOLERANCE_POSITION_DEPLOY));
    }

    /**
     * Deploy command
     * @return
     */
    public Command deploy() {
        return runOnce(() -> io.moveDeployerTo(DEPLOYED_SETPOINT));
    }

    /**
     * Retract command
     * @return
     */
    public Command retract() {
        return runOnce(() -> io.moveDeployerTo(RETRACTED_SETPOINT));
    }

    private Command agitateTo(Angle setpoint, double duration) {
        return new SequentialCommandGroup(
                run(() -> {
                    io.moveDeployerTo(setpoint);
                    //io.runIntake(INTAKE_SPEED);
                }).withTimeout(duration),

                run(() -> {
                    io.moveDeployerTo(DEPLOYED_SETPOINT);
                    //io.runIntake(INTAKE_SPEED);
                }).withTimeout(duration)
            );
    }

    /**
     * Agitate to lowest agitation position while running intake
     * On end, go to lowest agitation position
     * @return
     */
    public Command agitateM1() {
        return agitateTo(MID_1_SETPOINT, 0.35);
    }

    /**
     * Agitate to mid position while running intake
     * On end, go to mid position
     * @return
     */
    public Command agitateM2() {
        return agitateTo(MID_2_SETPOINT, 0.5);
    }

    /**
     * Agitate to the fully retracted position while running intake
     * On end, fully-retract
     * @return
     */
    public Command agitateFull(){
        return agitateTo(RETRACTED_SETPOINT, 0.75);
    }

    /**
     * Intake command. The intake gets interrupted after a set time if no game piece is detected.
     * @return
     */
    public Command intake() {
        return runEnd(() -> {
            io.runIntake(INTAKE_FULL_SPEED);
        }, () -> {
            io.stopIntake();
        });
    }

    /**
     * Intake command that doesn't end. The intake will only stop if stopIntake is called.
     * @return
     */
    public Command intakeWithoutRunEnd() {
        return runOnce(() -> {
            io.runIntake(INTAKE_FULL_SPEED);
        });
    }


    public Command intake(AngularVelocity speed) {
        return runEnd(() -> {
            io.runIntake(speed);
        }, () -> {
            io.stopIntake();
        });
    }
    
    /**
     * Idle command
     * @return
     */
    public Command idle() {
        return runEnd(() -> {
            io.runIntake(IDLE_SPEED);
        }, () -> {
            io.stopIntake();
        });
    }

    /**
     * Outtake command
     * @return
     */
    public Command outtake() {
        return runEnd(() -> {
            io.runIntake(OUTTAKE_SPEED);
        }, () -> {
            io.stopIntake();
        });
    }


    /**
     * Stop the intake
     * @return
     */
    public Command stopIntake() {
        return runOnce(() -> io.stopIntake());
    }

    /**
     * Stop the deployer motor
     * @return
     */
    public Command stopDeployer() {
        return runOnce(() -> io.stopDeployer());
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Bottom intake voltage", io.getBottomIntakeVoltage().magnitude());
        // SmartDashboard.putNumber("Bottom intake supply current", io.getBottomIntakeSupplyCurrent().magnitude());
        // SmartDashboard.putNumber("Bottom intake velocity", io.getBottomIntakeVelocity().magnitude());
        // SmartDashboard.putNumber("Bottom intake position", io.getBottomIntakePosition().magnitude());

        SmartDashboard.putBoolean("Intake running", isIntaking.getAsBoolean());
        SmartDashboard.putBoolean("Intake deployed", isDeployed.getAsBoolean());

        // SmartDashboard.putNumber("Deployer voltage", io.getDeployerVoltage().magnitude());
        // SmartDashboard.putNumber("Deployer supply current", io.getDeployerSupplyCurrent().magnitude());
        // SmartDashboard.putNumber("Deployer velocity", io.getDeployerVelocity().magnitude());
        SmartDashboard.putNumber("Deployer position", io.getDeployerPosition().magnitude());

        SmartDashboard.putBoolean("Intake state", io.getBottomIntakeVelocity().abs(RotationsPerSecond) > 1.0);
    }

    @Override
    public void simulationPeriodic() {
        io.periodic();
    }
}

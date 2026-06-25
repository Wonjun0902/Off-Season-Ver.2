package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerIOSim extends SubsystemBase implements IndexerIO{
    private DCMotor gearbox;
    private DCMotorSim motorSim;

    public IndexerIOSim() {
        gearbox = DCMotor.getKrakenX60(2);
        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.1, 1), gearbox); 
    }

    @Override
    public Distance getLeftRange() { return Inches.of(0); }
    @Override
    public Distance getRightRange() { return Inches.of(0); }
    
    @Override
    public AngularVelocity getLeftVelocity() {
        return motorSim.getAngularVelocity();
    }
    @Override
    public Current getLeftCurrent() {
        return Amps.of(motorSim.getCurrentDrawAmps());
    }


    @Override
    public void spinLeftVelocity(AngularVelocity speed) {
        motorSim.setInputVoltage(speed.magnitude());
        SmartDashboard.putNumber("Speed Passed",speed.magnitude());        
    }
    @Override
    public void runDutyCycleLeft(double dutyCycle) {
        motorSim.setInputVoltage(dutyCycle);        
    }
    @Override
    public void stop() {
        motorSim.setInputVoltage(0);        
    }

    @Override
    public void periodic() {
        motorSim.update(0.02);
    }

    @Override
    public void spinRightVelocity(AngularVelocity speed) {
        // TODO Auto-generated method stub
    }


    @Override
    public void runDutyCycleRight(double dutyCycle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runDutyCycleRight'");
    }

    @Override
    public AngularVelocity getRightVelocity() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'getRightVelocity'");

        return RotationsPerSecond.of(0);
    }

    @Override
    public Current getRightCurrent() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'getRightCurrent'");

        return Amps.of(0);
    }
}

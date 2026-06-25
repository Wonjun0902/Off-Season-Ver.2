package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.Configurations.*;
import frc.robot.subsystems.indexer.IndexerConstants.Configurations.Spindexer;
import frc.robot.subsystems.indexer.IndexerConstants.Configurations.Canranges;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.lib.LazyTalon;
import frc.lib.LazyTalonBuilder;

public class IndexerIOReal implements IndexerIO{
    public LazyTalon indexerLeft;
    public LazyTalon indexerRight;
    public CANrange leftCanrange, rightCanrange;

    public IndexerIOReal() {
        // RIGHT
        indexerLeft = new LazyTalonBuilder(
            Spindexer.Left.LEFT_ID, 
            CANBUS, 
            Spindexer.Left.SENSOR_TO_MECH_RATIO, 
            InvertedValue.CounterClockwise_Positive, 
            Spindexer.Left.STATOR_LIMIT.in(Amps), 
            Spindexer.Left.SUPPLY_LIMIT.in(Amps)
        )
            .withPIDFConfiguration(
                Spindexer.Left.kP, 
                Spindexer.Left.kI, 
                Spindexer.Left.kD, 
                Spindexer.Left.kS, 
                Spindexer.Left.kG, 
                Spindexer.Left.kV, 
                Spindexer.Left.kA,
                GravityTypeValue.Elevator_Static, 
                StaticFeedforwardSignValue.UseClosedLoopSign,
                0
            )
            .build();
        indexerLeft.setCoast();

        indexerRight = new LazyTalonBuilder(
            Spindexer.Right.RIGHT_ID, 
            CANBUS, 
            Spindexer.Right.SENSOR_TO_MECH_RATIO, 
            InvertedValue.Clockwise_Positive, 
            Spindexer.Right.STATOR_LIMIT.in(Amps), 
            Spindexer.Right.SUPPLY_LIMIT.in(Amps)
        )
            .withPIDFConfiguration(
                Spindexer.Right.kP, 
                Spindexer.Right.kI, 
                Spindexer.Right.kD, 
                Spindexer.Right.kS, 
                Spindexer.Right.kG, 
                Spindexer.Right.kV, 
                Spindexer.Right.kA,
                GravityTypeValue.Elevator_Static, 
                StaticFeedforwardSignValue.UseClosedLoopSign,
                0
            )
            .build();
        indexerRight.setCoast();
        
        leftCanrange = new CANrange(Canranges.Left.CANRANGE_ID, CANBUS);
        rightCanrange = new CANrange(Canranges.Right.CANRANGE_ID, CANBUS);

        CANrangeConfiguration config = new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs()
                .withFOVCenterX(-10)
                .withFOVCenterY(-10)
                .withFOVRangeX(8)
                .withFOVRangeY(8)
            )
            .withProximityParams(new ProximityParamsConfigs()
                .withMinSignalStrengthForValidMeasurement(2500)
            );
        rightCanrange.getConfigurator().apply(config);
        leftCanrange.getConfigurator().apply(config);
    }

    @Override
    public Distance getLeftRange() {
        return leftCanrange.getDistance().getValue();
    }
    @Override
    public Distance getRightRange() {
        return rightCanrange.getDistance().getValue();
    }

    @Override
    public AngularVelocity getLeftVelocity() {
        return indexerLeft.getVelocity();
    }
    @Override
    public Current getLeftCurrent() {
        return indexerLeft.getMotor().getStatorCurrent().getValue();
    }
    
    @Override
    public void spinLeftVelocity(AngularVelocity speed) {
        indexerLeft.setTFOCVelocityTarget(speed,Spindexer.Left.CURRENT_FF , 0);
    }
    @Override
    public void spinRightVelocity(AngularVelocity speed) {
        indexerRight.setTFOCVelocityTarget(speed, Spindexer.Right.CURRENT_FF, 0);
    }
    @Override
    public void stop() {
        indexerLeft.stop();
        indexerRight.stop();
    }
    @Override
    public void runDutyCycleLeft(double dutyCycle) {
        indexerLeft.setDutyCycle(dutyCycle);
    }
    @Override
    public void runDutyCycleRight(double dutyCycle) {
        indexerRight.setDutyCycle(dutyCycle);
    }

    @Override
    public void periodic() {
    
    }

    @Override
    public AngularVelocity getRightVelocity() {
        return indexerRight.getVelocity();
    }

    @Override
    public Current getRightCurrent() {
    return indexerRight.getMotor().getStatorCurrent().getValue();
    }
}

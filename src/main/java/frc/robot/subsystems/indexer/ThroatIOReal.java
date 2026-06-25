package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.ThroatConstants.Configurations.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LazyTalon;
import frc.lib.LazyTalonBuilder;

public class ThroatIOReal implements ThroatIO {
    public LazyTalon throatMotor;

    public ThroatIOReal() {
        throatMotor = new LazyTalonBuilder(
            MOTOR_ID, 
            CANBUS, 
            SENSOR_TO_MECH_RATIO, 
            InvertedValue.CounterClockwise_Positive,
            STATOR_LIMIT.magnitude(), 
            SUPPLY_LIMIT.magnitude(),SUPPLY_LIMIT_LOWER.magnitude(),SUPPLY_LIMIT_LOWER_TIME.magnitude()
        )
            .withPIDFConfiguration(kP, kI, kD, kS, kG, kV, kA, GravityTypeValue.Elevator_Static,
                    StaticFeedforwardSignValue.UseVelocitySign, 0)
            .build();
        throatMotor.setCoast();
    }

    
    @Override
    public void setSpeed(AngularVelocity targetVel) {
        throatMotor.setTFOCVelocityTarget(targetVel, CURRENT_FF, 0);
    }

    @Override
    public void stop() {
        throatMotor.stop();
    }

    @Override
    public AngularVelocity getVelocity() {
        return throatMotor.getVelocity();
    }
}

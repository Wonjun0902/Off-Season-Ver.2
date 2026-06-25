package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class ThroatConstants {
    public static final AngularVelocity FEED_SPEED = RotationsPerSecond.of(36.0);
    public static final AngularVelocity OUTTAKE_SPEED = RotationsPerSecond.of(-36.0);

    public class Configurations {
        public static final String CANBUS = "canDoAttitude";

        public static final int MOTOR_ID = 12;

        public static final double SENSOR_TO_MECH_RATIO = (34.0/11.0) * (22.0/34.0);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.0);
        public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(0.0);

        public static final double EXPO_V = 0;
        public static final double EXPO_A = 0;

        public static final Current SUPPLY_LIMIT = Amps.of(45.0);
        public static final Current SUPPLY_LIMIT_LOWER = Amps.of(45.0);
        public static final Time SUPPLY_LIMIT_LOWER_TIME = Seconds.of(0.05);
        public static final Current STATOR_LIMIT = Amps.of(70.0);
        
        public static final double kP = 5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final Current CURRENT_FF = Amps.of(26.0);
    }    
}
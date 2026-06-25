package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class IndexerConstants {
    public static final AngularVelocity INDEXER_IDLE_SPEED = RotationsPerSecond.of(-1.0);
    public static final AngularVelocity INDEXER_SHORT_SPEED = RotationsPerSecond.of(-10);
    public static final AngularVelocity INDEXER_LONG_SPEED = RotationsPerSecond.of(-10);
    public static final Distance SHORTSHOT_DISTANCE = Inches.of(82.5);

    public static final AngularVelocity INDEXER_UNJAM_SPEED = RotationsPerSecond.of(9);

    public static final Distance CANRANGE_LEFT_TRIGGER = Meters.of(0.028);
    public static final Distance CANRANGE_RIGHT_TRIGGER = Meters.of(0.03);

    public static final Time JAM_TIME_TOLERANCE = Seconds.of(4); // how long until begin unjam
    public static final Time UNJAM_DURATION = Seconds.of(0.5); // 0.35how long to unjam for

    public class Configurations {
        public static String CANBUS = "canDoAttitude";

        public class Spindexer {
            public class Left {
                public static final int LEFT_ID = 10;
               public static final double SENSOR_TO_MECH_RATIO = 9.0;
                public static final Current STATOR_LIMIT = Amps.of(30);
                public static final Current SUPPLY_LIMIT = Amps.of(30);

                public static final double kP = 6.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;

                public static final double kS = 0.0;
                public static final double kG = 0.0;
                public static final double kV = 0.0; // 10 volts for 80.5 rpm
                public static final double kA = 0.0;

                public static final Current CURRENT_FF = Amps.of(9.0);

                public static final double EXPO_A = 0.0;
                public static final double EXPO_V = 0.0;

                public static final double CRUISE_VELOCITY = 80.0;
                public static final double MAX_ACCELERATION = 160.0;
            }

            public class Right {
                public static final int RIGHT_ID = 11;
                public static final double SENSOR_TO_MECH_RATIO = 9.0;
                public static final Current STATOR_LIMIT = Amps.of(30);
                public static final Current SUPPLY_LIMIT = Amps.of(30);

                public static final double kP = 4.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;

                public static final double kS = 0.0;
                public static final double kG = 0.0;
                public static final double kV = 0.0; // 10 volts for 80.5 rpm
                public static final double kA = 0.0;

                public static final Current CURRENT_FF = Amps.of(9.0);

                public static final double EXPO_A = 0.0;
                public static final double EXPO_V = 0.0;

                public static final double CRUISE_VELOCITY = 80.0;
                public static final double MAX_ACCELERATION = 160.0;
            }
        }

        public class Canranges {
            public class Left {
                public static final int CANRANGE_ID = 3;
            }

            public class Right {
                public static final int CANRANGE_ID = 5;
            }
        }
    }
}
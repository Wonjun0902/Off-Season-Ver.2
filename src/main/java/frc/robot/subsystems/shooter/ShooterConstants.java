package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class ShooterConstants {
    public static final Angle TRIM_AIM_INCREMENT = Rotations.of(0.025);
    public static final AngularVelocity TRIM_SPEED_INCREMENT = RotationsPerSecond.of(2);

    public class Configurations {
        public static final String CANBUS = "canDoAttitude";

        public static class Shooter {
            public static final double SENSOR_TO_MECH_RATIO = 1; // free spinning, ignore gear ratio
            
            public static class Left {
                public static final int MOTOR_ID = 14;
                public static final int FOLLOWER_LEFT_ID = 21;
                
                public static final double kP = 6.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kF = 0.0;
                public static final double kS = 0.0;
                public static final double kV = 0.0;
                public static final double kG = 0.0;
                public static final double kA = 0.0;

                public static final Current CURRENT_FF = Amps.of(16);

                public static final double EXPO_A = 0.0;
                public static final double EXPO_V = 0.0;
            }
            public static class Right {
                public static final int MOTOR_ID = 13;
                public static final int FOLLOWER_RIGHT_ID = 22;

                // NOT DONE
                public static final double kP = 6.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kF = 0.0;
                public static final double kS = 0.0;
                public static final double kV = 0.0;
                public static final double kG = 0.0;
                public static final double kA = 0.0;

                public static final Current CURRENT_FF = Amps.of(16);

                public static final double EXPO_A = 0.0;
                public static final double EXPO_V = 0.0;
            }

            public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(80);
            public static final AngularVelocity MAX_ACCELERATION = RotationsPerSecond.of(160);

            public static final Current STATOR_CURRENT_LIMIT = Amps.of(90);
            public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
            public static final Current SUPPLY_LOWER_LIMIT = Amps.of(30);
            public static final Time SUPPLY_LOWER_TIME = Seconds.of(0.25);
        }


        public static class Hood {
            public static final int MOTOR_ID = 15;
            public static final int CANCODER_ID = 15;

            public static final double SENSOR_OFFSET = -0.119873046875;
            public static final SensorDirectionValue SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;
            public static final double ROTOR_SENSOR_RATIO = 25.0; 
            public static final double SENSOR_MECH_RATIO =1.0; 

            public static final double kP = 11.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;
            public static final double kS = 0.4;
            public static final double kV = 1.4;
            public static final double kG = 0.0;
            public static final double kA = 0.0;

            public static final double EXPO_A = 0.0;
            public static final double EXPO_V = 0.0;

            public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(7.3);
            public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(21);


            public static final Current STATOR_CURRENT_LIMIT = Amps.of(20);
            public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

            public static final Angle SOFT_HI = Rotations.of(0.95); // soft limits
            public static final Angle SOFT_LO = Rotations.of(0.05);
        }
    }

    public class AutoAlign {
        public static final Angle ALIGN_TOLERANCE = Degrees.of(2.0);
        public static final Angle AUTO_ALIGN_MAX_OFFSET = Degrees.of(30.0);
        /**
         * TRENCH ROBOT ANGLE (80 deg offset)
         * RIGHT: 80 deg
         * LEFT : 280 deg (flipped)
         */

        /**
         * TOWER ROBOT ANGLE (19 deg offset)
         * RIGHT: 19 deg
         * LEFT : 341 deg
        */

        // BLUE ALLIANCE PERSPECTIVE
        public static final Angle ROBOT_HUB_ANGLE = Degrees.of(0.0);
        public static final Angle SCRAPE_ANGLE_LEFT = Degrees.of(225.0);
        public static final Angle SCRAPE_ANGLE_RIGHT = Degrees.of(135.0);

        public static final Angle ROBOT_TOWER_FRONT_ANGLE = Degrees.of(0.0);
        public static final Angle ROBOT_TRENCH_RIGHT_ANGLE = Degrees.of(80.0);
        public static final Angle ROBOT_TRENCH_LEFT_ANGLE = Degrees.of(280.0);

        public static final Angle ROBOT_TOWER_RIGHT_ANGLE = Degrees.of(19.0);
        public static final Angle ROBOT_TOWER_LEFT_ANGLE = Degrees.of(341.0);

        public static final Angle RENEE_BUMP_ANGLE_RIGHT = Degrees.of(45.0);
        public static final Angle RENEE_BUMP_ANGLE_LEFT = Degrees.of(315.0);
    }

    public class LookupTables {
        public static final InterpolatingDoubleTreeMap HOOD_LOOKUP_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap SHOOTER_LOOKUP_TABLE = new InterpolatingDoubleTreeMap();

        public static final double          HALF_HUB                 = 47.0 / 2.0;
        public static final double          HALF_BUMPER              = 16.5;
        public static final double          HALF_ROBOT_NO_BUMPER     = Centimeters.of(34.25).in(Inches);

        public static final Distance        HUB_POSITION             = Inches.of(8.75 + HALF_BUMPER+HALF_HUB);
        public static final Distance        TOWER_FRONT_POSITION     = Inches.of(79   + HALF_BUMPER+HALF_HUB);
        public static final Distance        TRENCH_POSITION          = Inches.of(1000 + HALF_BUMPER+HALF_HUB);
        public static final Distance        TOWER_BACK_POSITION      = Inches.of(2000 + HALF_BUMPER+HALF_HUB);

        public static final AngularVelocity SHOOTER_HUB_SPEED        = RotationsPerSecond.of(26.0);
        public static final AngularVelocity SHOOTER_RENEE_SPEED      = RotationsPerSecond.of(32.0);
        public static final AngularVelocity SHOOTER_TOWER_SPEED      = RotationsPerSecond.of(44.0);
        public static final AngularVelocity SHOOTER_TRENCH_SPEED     = RotationsPerSecond.of(60.0);
        public static final AngularVelocity SHOOTER_BACK_SPEED       = RotationsPerSecond.of(50.0);
        public static final AngularVelocity SHOOTER_PASS_SPEED       = RotationsPerSecond.of(60.0);
        
        public static final Angle           HOOD_HUB_ANGLE           = Rotations.of(0.1);
        public static final Angle           HOOD_TOWER_ANGLE         = Rotations.of(0.3);
        public static final Angle           HOOD_TRENCH_ANGLE        = Rotations.of(0.6);
        public static final Angle           HOOD_BACK_ANGLE          = Rotations.of(0.425);
        public static final Angle           HOOD_PASS_ANGLE          = Rotations.of(0.85); // +0.525
       

        static {
            HOOD_LOOKUP_TABLE   .put(1.3,        HOOD_HUB_ANGLE     .in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(1.3,        SHOOTER_HUB_SPEED  .in(RotationsPerSecond));

            HOOD_LOOKUP_TABLE   .put(1.77, 0.0  +HOOD_HUB_ANGLE     .in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(1.77, 2    +SHOOTER_HUB_SPEED  .in(RotationsPerSecond));

            HOOD_LOOKUP_TABLE   .put(2.25, 0.15 +HOOD_HUB_ANGLE     .in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(2.25, 2    +SHOOTER_HUB_SPEED  .in(RotationsPerSecond));

            HOOD_LOOKUP_TABLE   .put(2.95, 0.2  +HOOD_HUB_ANGLE     .in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(2.95, 4    +SHOOTER_HUB_SPEED  .in(RotationsPerSecond));

            HOOD_LOOKUP_TABLE   .put(3.75, 0.2  +HOOD_HUB_ANGLE     .in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(3.75, 8   +SHOOTER_HUB_SPEED  .in(RotationsPerSecond));

            HOOD_LOOKUP_TABLE   .put(4.05, 0.25 +HOOD_HUB_ANGLE     .in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(4.05, 8   +SHOOTER_HUB_SPEED  .in(RotationsPerSecond));
            
            HOOD_LOOKUP_TABLE   .put(4.95, 0.25 +HOOD_HUB_ANGLE     .in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(4.95, 12   +SHOOTER_HUB_SPEED  .in(RotationsPerSecond));



            // passing (out of range)
            HOOD_LOOKUP_TABLE   .put(5.85, 0.35 +HOOD_HUB_ANGLE.in(Rotations));
            SHOOTER_LOOKUP_TABLE.put(5.85, 38   +SHOOTER_HUB_SPEED.in(RotationsPerSecond));

            HOOD_LOOKUP_TABLE.put(10.65,0.65);
            SHOOTER_LOOKUP_TABLE.put(10.65, 100.0);
        }

        // private static double cmIn(double cm) {
        //     return Centimeters.of(cm).in(Inches) + HALF_ROBOT_NO_BUMPER + HALF_HUB;
        // }
    }

    public class Field {
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
        public static final double FIELD_WIDTH = Inches.of(47.0).in(Meters);
        public static final double FIELD_LENGTH = FIELD_LAYOUT.getFieldLength();
        public static final Translation2d BLUE_HUB_POSE = new Translation2d(
            4.625, // 4.625594, 
            4.034536 // 4.034536
        );
        public static final Translation2d RED_HUB_POSE = new Translation2d(
            11.905,// 12.896088,
            4.034536 // 4.034536
        );
    }
}
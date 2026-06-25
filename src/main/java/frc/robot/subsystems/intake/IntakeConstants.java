package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeConstants {
    public static final int CANCODER_ID = 17;
    public static final double CANCODER_OFFSET = 0.183837890625;
    public static final SensorDirectionValue CANCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    public static final FeedbackSensorSourceValue CANCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
        
    public static final Angle DEPLOYED_SETPOINT = Rotations.of(0.0);
    public static final Angle MID_1_SETPOINT = Rotations.of(-0.2);
    public static final Angle MID_2_SETPOINT = Rotations.of(-0.28);
    public static final Angle RETRACTED_SETPOINT = Rotations.of(-0.36); // -0.36 is correct; -0.4 is temp fix cuz pid

    // Note: this might be too much. Maybe reduce to 5 degrees?
    public static final Angle TOLERANCE_POSITION_DEPLOY = Degrees.of(10.0);
    
    // into the bot is negative (intake, etc.)
    // out of the bot is positive (outtake, etc.)
    public static final AngularVelocity INTAKE_SPEED = RotationsPerSecond.of(-30.0);
    public static final AngularVelocity INTAKE_FULL_SPEED = RotationsPerSecond.of(-43.0);
    public static final AngularVelocity OUTTAKE_SPEED = RotationsPerSecond.of(43.0);
    public static final AngularVelocity IDLE_SPEED = RotationsPerSecond.of(-20);
 
    public class Configurations {
        public static final String CANBUS = "canDoAttitude";

        public class Intake {
            public class Top {
                public static final int MOTOR_ID = 20;

                public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
                public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

                public static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive; 
                public static final double SENSOR_TO_MECH_RATIO = 32.0/14;
                
                public static final double kS = 0.0;
                public static final double kG = 0; 
                public static final double kV = 0.0;
                public static final double kA = 0.0;
                public static final double kP = 1.0;
                public static final double kI = 0;
                public static final double kD = 0;

                public static final Current CURRENT_FF = Amps.of(2.4);

                public static final double EXPO_VELOCITY = 0.0;
                public static final double EXPO_ACCELERATION = 0.0;

                public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;
                
                public static final double CRUISE_VELOCITY = 10;
                public static final double MAX_ACCELERATION = 20;
            }
            public class Bottom {
                public static final int MOTOR_ID = 19;

                public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
                public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

                public static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive; 
                public static final double SENSOR_TO_MECH_RATIO = 32.0 /14;
                
                public static final double kS = 0.0;
                public static final double kG = 0; 
                public static final double kV = 0.0;
                public static final double kA = 0.0;
                public static final double kP = 4.0;
                public static final double kI = 0;
                public static final double kD = 0;

                public static final Current CURRENT_FF = Amps.of(10.0);

                public static final double EXPO_VELOCITY = 0.12;
                public static final double EXPO_ACCELERATION = 0.01;

                public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;
                
                public static final double CRUISE_VELOCITY = 10;
                public static final double MAX_ACCELERATION = 20;
            }
        }
        // 0.57 bottom
        // 0.479 top
        
        public class Deployer {
            public static final int MOTOR_ID = 17;

            public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
            public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

            public static final double ROTOR_TO_SENSOR_RATIO = 40.0 / 18.0 * 45.0;
            
            public static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;

            public static final double kS = 0.29;
            public static final double kG = 0.35; 
            public static final double kV = 14.3;
            public static final double kA = 0.0;
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double EXPO_VELOCITY = 0.12;
            public static final double EXPO_ACCELERATION = 0.01;

            public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
            public static final double FORWARD_LIMIT = 0.005;
            public static final double REVERSE_LIMIT = -0.37;
            
            public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.8);
            public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(1.6);
        }
    }
}
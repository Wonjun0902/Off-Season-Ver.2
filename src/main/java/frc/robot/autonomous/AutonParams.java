package frc.robot.autonomous;

import frc.lib.Pathfinder.MoveToPoseParams;

public class AutonParams {
        public static final MoveToPoseParams DRIVING = MoveToPoseParams.builder()
                        .minSpeed(2.0)
                        .maxSpeed(3.0)
                        .earlyExitRange(0.5)
                        .build();

        public static final MoveToPoseParams INTAKING_DRIVING = MoveToPoseParams.builder()
                        .minSpeed(2.0)
                        .maxSpeed(2.0)
                        .earlyExitRange(0.25)
                        .build();

        public static final MoveToPoseParams DRIVING_LARGE_EXIT = MoveToPoseParams.builder()
                        .minSpeed(2.0)
                        .maxSpeed(3.0)
                        .earlyExitRange(1.0)
                        .build();

        public static final MoveToPoseParams DRIVING_NO_EXIT = MoveToPoseParams.builder()
                        .maxSpeed(3.0)
                        .earlyExitRange(0.05)
                        .build();

        public static final MoveToPoseParams SLOW_DRIVE = MoveToPoseParams.builder()
                        .minSpeed(1.0)
                        .maxSpeed(1.5)
                        .earlyExitRange(0.1)
                        .build();

        public static final MoveToPoseParams FAST_EXIT_SLOW_DRIVE = MoveToPoseParams.builder()
                        .minSpeed(1.0)
                        .maxSpeed(1.5)
                        .earlyExitRange(0.25)
                        .build();


        public static final MoveToPoseParams SEE_TAG_SPECIAL = MoveToPoseParams.builder()
                        .maxSpeed(3.5)
                        .minSpeed(0.5)
                        .earlyExitRange(0.5)
                        .build();

        public static final MoveToPoseParams SLOW_DEPO = MoveToPoseParams.builder()
                        .minSpeed(1.0)
                        .maxSpeed(1.0)
                        .build();
}

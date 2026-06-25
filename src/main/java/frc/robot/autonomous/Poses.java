package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class Poses {
    // Bump (right) poses
    public static final Pose2d BUMP_RED_ALLIANCE_BUMP = new Pose2d(12.9, 5.65, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BUMP_RED_PRE_CROSS_BUMP = new Pose2d(13.3, 5.65, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BUMP_RED_CROSS_BUMP = new Pose2d(10.5, 5.65, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BUMP_RED_INTAKING = new Pose2d(10.0, 6.9, new Rotation2d(Math.toRadians(270)));
    public static final Pose2d BUMP_RED_SECOND_INTAKING = new Pose2d(8.6, 6.9, new Rotation2d(Math.toRadians(270)));
    public static final Pose2d BUMP_RED_THIRD_INTAKING = new Pose2d(8.6, 4.5, new Rotation2d(Math.toRadians(270)));
    public static final Pose2d BUMP_RED_SWEEEP = new Pose2d(10.8,4.3,new Rotation2d(Math.toRadians(90)));
    public static final Pose2d BUMP_RED_SECOND_RUN = new Pose2d(10.8, 5.75, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BUMP_RED_ARRIVAL_POSE = new Pose2d(13.3, 5.75, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BUMP_RED_SHOOT = new Pose2d(13.4, 4.0, new Rotation2d(Math.toRadians(180)));

    // Bump (left) poses (these are the red-side values used for the left bump auton in original file)
    public static final Pose2d BUMP_LEFT_RED_ALLIANCE_BUMP = new Pose2d(12.9, 8 - 5.65, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BUMP_LEFT_RED_PRE_CROSS_BUMP = new Pose2d(13.3, 8 - 5.65, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BUMP_LEFT_RED_CROSS_BUMP = new Pose2d(10.4, 8 - 5.65, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BUMP_LEFT_RED_INTAKING = new Pose2d(10.1, 8 - 6.8, new Rotation2d(Math.toRadians(360 - 270)));
    public static final Pose2d BUMP_LEFT_RED_SECOND_INTAKING = new Pose2d(8.9, 8 - 6.8, new Rotation2d(Math.toRadians(360 - 270)));
    public static final Pose2d BUMP_LEFT_RED_THIRD_INTAKING = new Pose2d(8.9, 8 - 4.3, new Rotation2d(Math.toRadians(360 - 270)));
    public static final Pose2d BUMP_LEFT_RED_SECOND_RUN = new Pose2d(10.3, 8 - 5.25, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BUMP_LEFT_RED_ARRIVAL_POSE = new Pose2d(12.3, 8 - 5.25, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BUMP_LEFT_RED_SHOOT = new Pose2d(13.4, 8 - 4.0, new Rotation2d(Math.toRadians(180)));

    // Depo shoot poses
    public static final Pose2d DEPO_RED_START = new Pose2d(12.75, 2.9, new Rotation2d(Math.toRadians(135)));
    public static final Pose2d DEPO_RED_LIME_POSE = new Pose2d(14.2, 2.6, new Rotation2d(Math.toRadians(130)));
    public static final Pose2d DEPO_RED_SECOND_LIME = new Pose2d(13.8, 2.1, new Rotation2d(Math.toRadians(100)));
    public static final Pose2d DEPO_RED_START_DEPO = new Pose2d(15.95 , 3.0, new Rotation2d(Math.toRadians(-45)));
    public static final Pose2d DEPO_RED_SHOOT = new Pose2d(13.1, 4.0, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d DEPO_RED_END_DEPO = new Pose2d(15.95, 1.4, new Rotation2d(Math.toRadians(-45)));

    // Center shoot poses
    public static final Pose2d CENTER_START = new Pose2d(3.644, 5.945, new Rotation2d(Math.toRadians(23.538)));
    public static final Pose2d CENTER_SHOOT = new Pose2d(2.847, 4.923, new Rotation2d(Math.toRadians(-4.917)));
    public static final Pose2d CENTER_CENTER = new Pose2d(0.459, 5.945, new Rotation2d(Math.toRadians(180.000)));

    //Directly cross middle poses (TearAuton) 
    public static final Pose2d TEAR_RED_ALLIANCE_BUMP = new Pose2d(12.9, 5.65, new Rotation2d(Math.toRadians(225)));
    public static final Pose2d TEAR_RED_PRE_CROSS_BUMP = new Pose2d(13.3, 5.65, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d TEAR_RED_CROSS_BUMP = new Pose2d(10.8, 5.65, new Rotation2d(Math.toRadians(225)));

    //turned 45? 
    public static final Pose2d TEAR_RED_PRE_INTAKE = new Pose2d(9.2,6.1, new Rotation2d(Math.toRadians(200)));
    public static final Pose2d TEAR_RED_INTAKE = new Pose2d(8.537, 6.0, new Rotation2d(Math.toRadians(-75)));
    public static final Pose2d TEAR_RED_SECOND_INTAKE = new Pose2d(8.537, 4.1, new Rotation2d(Math.toRadians(-75)));
    public static final Pose2d TEAR_RED_THIRD_INTAKE = new Pose2d(10.0, 4.6, new Rotation2d(Math.toRadians(35)));

    //end (bump cross)
    public static final Pose2d TEAR_ARRIVAL_POSE = new Pose2d(13.6,5.65, new Rotation2d(Math.toRadians(225)));
    public static final Pose2d TEAR_SHOOTING_POSE = new Pose2d(13.4, 4.0, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d TEAR_SHOOTING_EXP = new Pose2d(13.6, 5.65, new Rotation2d(Math.toRadians(225)));
    public static final Pose2d PRE_CROSS_BUMP_REVERSE = new Pose2d(12.9, 5.65, new Rotation2d(Math.toRadians(0)));

     // reverse tear
    public static final Pose2d TEAR_RED_PRE_INTAKE_REVERSE = new Pose2d(9.2,6.0, new Rotation2d(Math.toRadians(20)));
    public static final Pose2d TEAR_RED_INTAKE_REVERSE = new Pose2d(8.8, 6.0, new Rotation2d(Math.toRadians(75)));
    public static final Pose2d TEAR_RED_SECOND_INTAKE_REVERSE = new Pose2d(8.8, 4.1, new Rotation2d(Math.toRadians(175)));
    public static final Pose2d TEAR_RED_THIRD_INTAKE_REVERSE = new Pose2d(10.6, 4.2, new Rotation2d(Math.toRadians(-90)));

    
    // Hook auton
    public static final Pose2d THIRD_TO_SWEEP_TRANSITION = new Pose2d(10.0, 4.0, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d PRE_SECOND_BUMP = new Pose2d(10.8, 5.60, new Rotation2d(Math.toRadians(90)));

    // double sweep
    public static final Pose2d SWEEP_SECOND_RESET = new Pose2d(10.5, 6.9, new Rotation2d(Math.toRadians(270)));
    public static final Pose2d SWEEP_SECOND_INTAKE = new Pose2d(10.5, 4.1, new Rotation2d(Math.toRadians(270)));

    // misc

    public static final Pose2d CROSS_BUMP_REVERSE = new Pose2d(10.5, 5.65, new Rotation2d(Math.toRadians(0)));


    /**
     * helper function to swap poses (flip alliance) - same logic as original
     */
    public static final Pose2d flipAlliance(Pose2d original) {
        return original.rotateAround(new Translation2d(8.27, 4), new Rotation2d(Units.degreesToRadians(180)));
    }

    /**
     * helper function to swap poses (flip sides) - same logic as original
     */
    public static final Pose2d flipSide(Pose2d original) {
        double flippedY = 8.0 - original.getY();
        Rotation2d flippedRot = original.getRotation().unaryMinus();
        return new Pose2d(original.getX(), flippedY, flippedRot);
    }

    /**
     * Returns the pose adjusted for the given alliance.
     * If blue==true, returns the flipped (mirrored) pose for the blue side.
     * Otherwise returns the provided red-side pose unchanged.
     * !!! IMPORTANT !!! All autons have been written on red field right side, except depo autons which have to be on the left
     */
    public static Pose2d configureColorSide(boolean blue, boolean right, Pose2d redPose) {
        Pose2d sideFlipped = right ? redPose : flipSide(redPose);
        return blue ? flipAlliance(sideFlipped) : sideFlipped;
    }

    public static final Command limeResetTranslation() {
        if (RobotContainer.limelightFRONT.estimate != null)
            return Commands.runOnce(() -> RobotContainer.drivetrain.resetTranslation(RobotContainer.limelightBACK.estimate.pose.getTranslation()));
        
        return Commands.none();
    }
}

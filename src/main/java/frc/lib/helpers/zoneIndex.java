package frc.lib.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Predefined rectangular zones and helpers to test membership.
 */
public final class zoneIndex {
    private zoneIndex() {}

    // Loading zone
    public static final Pose2d[] LOADING_ZONE = new Pose2d[] {
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),   // bottom left
        new Pose2d(1.2, 0.0, Rotation2d.fromDegrees(0.0)),   // bottom right
        new Pose2d(1.2, 1.2, Rotation2d.fromDegrees(0.0)),   // top right
        new Pose2d(0.0, 1.2, Rotation2d.fromDegrees(0.0))    // top left
    };

    // Shooting zone
    public static final Pose2d[] SHOOTING_ZONE = new Pose2d[] {
        new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(0.0)),   // bottom left
        new Pose2d(4.0, 1.5, Rotation2d.fromDegrees(0.0)),   // bottom right
        new Pose2d(4.0, 6.5, Rotation2d.fromDegrees(0.0)),   // top right
        new Pose2d(1.5, 6.5, Rotation2d.fromDegrees(0.0))    // top left
    };

    public static boolean isInLoadingZone(Pose2d pose) {
        return geometryUtils.inZone(pose, LOADING_ZONE);
    }

    public static boolean isInShootingZone(Pose2d pose) {
        return geometryUtils.inZone(pose, SHOOTING_ZONE);
    }
}
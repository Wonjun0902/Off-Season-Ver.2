package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LimelightHelpers;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Debug/telemetry helper that supports multiple Limelight instances.
 * Keeps SmartDashboard/NetworkTables and applying vision-to-odometry
 * out of the main Limelight class to keep it focused on limelight-specific
 * logic.
 */
public class LimelightDebug {
    private final List<Limelight> limelights = new ArrayList<>();
    private final Map<String, Field2d> fields = new HashMap<>();
    private final Map<String, StructPublisher<Pose2d>> publishers = new HashMap<>();
    // private final Map<String, HttpCamera> cameras = new HashMap<>();

    private Notifier notifier = null;

    public LimelightDebug() {
    }

    public LimelightDebug(List<Limelight> limelights) {
        limelights.forEach(this::addLimelight);
    }

    public LimelightDebug(Limelight... limelights) {
        for (Limelight l : limelights)
            addLimelight(l);
    }

    public void addLimelight(Limelight limelight) {
        String name = limelight.getLimelightName();
        if (fields.containsKey(name))
            return;

        limelights.add(limelight);
        Field2d field = new Field2d();
        fields.put(name, field);

        publishers.put(name, NetworkTableInstance.getDefault()
                .getStructTopic(name + "_frNT", Pose2d.struct)
                .publish());

        SmartDashboard.putData("Limelight Field - " + name, field);

        HttpCamera fr = new HttpCamera("fr", "http://limelight-fr.local:5800");
        // HttpCamera fl = new HttpCamera("fr", "http://10.85.3.201:5800");
        // HttpCamera br = new HttpCamera("fr", "http://10.85.3.202:5800");
        // HttpCamera bl = new HttpCamera("fr", "http://10.85.3.203:5800");

        CameraServer.startAutomaticCapture(fr);
        // CameraServer.startAutomaticCapture(fl);
        // CameraServer.startAutomaticCapture(br);
        // CameraServer.startAutomaticCapture(bl);

    }

    public void removeLimelight(String name) {
        limelights.removeIf(l -> l.getLimelightName().equals(name));
        fields.remove(name);
        publishers.remove(name);
    }

    /**
     * Publish one telemetry frame for every configured limelight; safe to call from
     * robotPeriodic or Notifier.
     */
    public void periodic() {
        for (Limelight limelight : limelights) {
            final String name = limelight.getLimelightName();

            if (limelight.isIgnored()) {
                SmartDashboard.putString("Limelight/ignored/" + name, "true");
                continue;
            }

            LimelightHelpers.PoseEstimate est = limelight.getPoseEstimate();

            boolean tv = LimelightHelpers.getTV(name);
            SmartDashboard.putBoolean("Limelight/" + name + "/tv", tv);

            if (tv) {
                double tx = LimelightHelpers.getTX(name);
                double ty = LimelightHelpers.getTY(name);
                double ta = LimelightHelpers.getTA(name);
                double pipeline = LimelightHelpers.getCurrentPipelineIndex(name);
                String pipelineType = LimelightHelpers.getCurrentPipelineType(name);
                double latencyPipe = LimelightHelpers.getLatency_Pipeline(name);
                double latencyCap = LimelightHelpers.getLatency_Capture(name);

                SmartDashboard.putNumber("Limelight/" + name + "/tx", tx);
                SmartDashboard.putNumber("Limelight/" + name + "/ty", ty);
                SmartDashboard.putNumber("Limelight/" + name + "/ta", ta);
                SmartDashboard.putNumber("Limelight/" + name + "/pipelineIndex", pipeline);
                SmartDashboard.putString("Limelight/" + name + "/pipelineType", pipelineType);
                SmartDashboard.putNumber("Limelight/" + name + "/latency_pipeline_ms", latencyPipe);
                SmartDashboard.putNumber("Limelight/" + name + "/latency_capture_ms", latencyCap);
            } else {
                SmartDashboard.putNumber("Limelight/" + name + "/tx", 0);
                SmartDashboard.putNumber("Limelight/" + name + "/ty", 0);
                SmartDashboard.putNumber("Limelight/" + name + "/ta", 0);
                SmartDashboard.putNumber("Limelight/" + name + "/pipelineIndex", 0);
                SmartDashboard.putString("Limelight/" + name + "/pipelineType", "no_target");
                SmartDashboard.putNumber("Limelight/" + name + "/latency_pipeline_ms", 0);
                SmartDashboard.putNumber("Limelight/" + name + "/latency_capture_ms", 0);
            }

            if (est != null && est.pose != null) {
                fields.get(name).setRobotPose(est.pose);
                publishers.get(name).set(est.pose);

                SmartDashboard.putNumber("Limelight/" + name + "/pose_x", est.pose.getX());
                SmartDashboard.putNumber("Limelight/" + name + "/pose_y", est.pose.getY());
                SmartDashboard.putNumber("Limelight/" + name + "/pose_rot_degrees", est.pose.getRotation().getDegrees());
                SmartDashboard.putNumber("Limelight/" + name + "/pose_timestamp", est.timestampSeconds);
                SmartDashboard.putNumber("Limelight/" + name + "/tagCount", est.tagCount);
                SmartDashboard.putNumber("Limelight/" + name + "/tagSpan", est.tagSpan);
                SmartDashboard.putNumber("Limelight/" + name + "/tagAvgDist", est.avgTagDist);
                SmartDashboard.putNumber("Limelight/" + name + "/tagAvgArea", est.avgTagArea);
                SmartDashboard.putString("Limelight/" + name + "/pose_status", "yes_pose");


                // Vision-to-odometry is handled by the Limelight subsystem itself to avoid
                // duplicate application. LimelightDebug only publishes telemetry now.
            } else {
                SmartDashboard.putString("Limelight/" + name + "/pose_status", "no_pose");
            }
        }
    }

    /**
     * Start a Notifier to call periodic at the requested interval (seconds).
     * Call stop() to cancel.
     */
    public void startPeriodic(double periodSeconds) {
        if (notifier != null) {
            notifier.stop();
        }
        notifier = new Notifier(this::periodic);
        notifier.startPeriodic(periodSeconds);
    }

    public void stop() {
        if (notifier != null) {
            notifier.stop();
            notifier = null;
        }
    }
}

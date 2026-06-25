package frc.robot.subsystems.limelight;


import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.helpers.TagValues;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.*;

import static frc.robot.RobotContainer.getCachedAlliance;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;


public class Limelight extends SubsystemBase {
    public enum LED_MODE {
        PIPELINE, OFF, BLINK, ON
    }

    public enum STREAM_MODE {
        STANDARD, PIP_MAIN, PIP_SECOND
    }

    private boolean ignoreAllLimes = false;
    public LimelightHelpers.PoseEstimate estimate;
    public static int FrontTag, BackTag, blTag, brTag;
    private final String limelightName;
    // public static final String[] limes = new String[] {"limelight-fr" /*, "limelight-fl", "limelight-bl", "limelight-br" */};

    private Field2d limelightField = new Field2d();

    public Field2d getField2d() {
        return limelightField;
    }

    private final Swerve swerve;
    private volatile boolean useMegaTag2 = false;
    // private volatile boolean calibrating = false;

    public Limelight(String limelightName, Swerve swerve) {
        this.limelightName = limelightName;
        this.swerve = swerve;
        initialize();
    }

    public void setUseMegaTag2(boolean use) {
        this.useMegaTag2 = use;
    }

    public boolean isUsingMegaTag2() {
        return this.useMegaTag2;
    }

    public void initialize() {

    }
    

    /**
     * Sets the stream mode of the limelight
     *
     * @param limelightName The name of the limelight
     * @param mode          {@link STREAM_MODE} of the limelight
     */
    public void setStreamMode(String limelightName, STREAM_MODE mode) {
        switch (mode) {
            case STANDARD -> LimelightHelpers.setStreamMode_Standard(limelightName);
            case PIP_MAIN -> LimelightHelpers.setStreamMode_PiPMain(limelightName);
            case PIP_SECOND -> LimelightHelpers.setStreamMode_PiPSecondary(limelightName);
        }
    }

    public void setStreamMode(STREAM_MODE mode) {
        setStreamMode(this.limelightName, mode);
    }

    /**
     * Sets the pipeline of the limelight
     *
     * @param limelightName The name of the limelight
     * @param pipeline      The pipeline index
     */
    public void setPipeline(String limelightName, int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    public void setPipeline(int pipeline) {
        setPipeline(this.limelightName, pipeline);
    }

    /**
     * Gets the tag ID of the front limelight
     *
     * @return The tag ID of the front limelight
     */
    public static int getTag(String limelightName) {
        return (int) (LimelightHelpers.getFiducialID(limelightName));
    }

    /**
     * Update the estimate of front limelight
     */
    public void updateEstimate() {
        if (this.useMegaTag2) {
            this.estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
        } else {
            this.estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
        }
    }

    /**
     * A lightweight/"lazy" vision update that only applies XY corrections and
     * leaves heading/theta untouched. This forces a vision measurement into the
     * drivetrain with a very large theta standard deviation so the estimator
     * effectively ignores heading.
     */
    
    public void updateSwervePoseLazy() {
        if (this.estimate == null || this.estimate.pose == null || this.estimate.tagCount == 0) {
            return;
        }

        double rotationRate = Math.abs(this.swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        if (rotationRate > 720) {
            return;
        }

        double xyStdDev = 0.3;
        this.swerve.addVisionMeasurement(this.estimate.pose, this.estimate.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, 999999.0));
    }

    public void updateSwervePose() {
        if (this.estimate == null || this.estimate.pose == null || this.estimate.tagCount == 0) {
            return;
        }

        double rotationRate = Math.abs(this.swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        if (rotationRate > 720) {
            return;
        }

        double visionAccuracy = calculateVisionTrust();
        // SmartDashboard.putNumber("Limelight/" + this.limelightName + "/visionAccuracy", visionAccuracy);
        // SmartDashboard.putNumber("Swerve/currentPoseAccuracy", this.swerve.getCurrentPoseAccuracy());

        double acceptanceMargin = 0.05; // 5% margin

        if (visionAccuracy > this.swerve.getCurrentPoseAccuracy() - acceptanceMargin) {
            double confidenceScale = 1.0 - visionAccuracy;
            double xyStdDev = 0.01 + (confidenceScale * confidenceScale);
            
            this.swerve.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStdDev, xyStdDev, 999999.0)
            );

            this.swerve.addVisionMeasurement(this.estimate.pose, this.estimate.timestampSeconds);
            this.swerve.setPoseAccuracy(visionAccuracy);
        }
    }

    /**
     * Update the swerve pose including heading, but only when at least 2 fiducial
     * tags are visible. This provides a more accurate heading estimate from
     * MT1 (or MT2 if configured) and should only be used when the reading is
     * confident (>=2 tags).
     */
    public void disabledUpdate() {
        updateEstimate();

        if (this.estimate == null || this.estimate.pose == null || this.estimate.tagCount < 2) {
            return;
        }

        LimelightHelpers.SetRobotOrientation(RobotContainer.limelightBACK.getLimelightName(), this.estimate.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(RobotContainer.limelightFRONT.getLimelightName(), this.estimate.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        RobotContainer.drivetrain.seedFieldCentric(getCachedAlliance().get().equals(Alliance.Blue) ? this.estimate.pose.getRotation() : this.estimate.pose.getRotation().plus(Rotation2d.k180deg));
    }

    private double calculateVisionTrust() {
        if (this.estimate == null || this.estimate.rawFiducials == null || this.estimate.rawFiducials.length == 0) {
            return 0.0;
        }

        LimelightHelpers.RawFiducial primary = null;
        for (var fiducial : this.estimate.rawFiducials) {
            if (fiducial == null) continue;
            primary = fiducial;
            break;
        }

        if (primary == null) return 0.0;
        if (primary.distToRobot <= 0 || primary.ta <= 0) return 0.0;

        var tagData = TagValues.getTagTrust((int) primary.id);
        double baseWeight = (tagData.hubAccuracy() * 0.8) + (tagData.climbAccuracy() * 0.2);
        double distancePenalty = Math.max(0, 1.0 - (primary.distToRobot / 8.0));
        double areaBonus = Math.min(1.0, primary.ta / 4.0);

        return (baseWeight * 0.5) + (distancePenalty * 0.4) + (areaBonus * 0.1);
    }

    @Override
    public void periodic() {
        updateEstimate();
        // updateSwervePose();
        updateSwervePoseLazy();
        // SmartDashboard.putBoolean("Limelight/" + this.limelightName + "/calibrating", this.calibrating);
        // SmartDashboard.putBoolean("Limelight/" + this.limelightName + "/calibrating", this.calibrating);
    }

    

    public boolean hasTarget() {
        return LimelightHelpers.getTV(this.limelightName);
    }

    public double getTx() {
        return LimelightHelpers.getTX(this.limelightName);
    }

    public double getTy() {
        return LimelightHelpers.getTY(this.limelightName);
    }

    public double getTa() {
        return LimelightHelpers.getTA(this.limelightName);
    }

    public boolean getTv() {
        return LimelightHelpers.getTV(this.limelightName);
    }

    public int getPipelineIndex() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(this.limelightName);
    }

    public String getPipelineType() {
        return LimelightHelpers.getCurrentPipelineType(this.limelightName);
    }

    public double getLatencyPipelineMs() {
        return LimelightHelpers.getLatency_Pipeline(this.limelightName);
    }

    public double getLatencyCaptureMs() {
        return LimelightHelpers.getLatency_Capture(this.limelightName);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate() {
        return this.estimate;
    }

    public String getLimelightName() {
        return this.limelightName;
    }

    public Swerve getSwerve() {
        return this.swerve;
    }

    public boolean isIgnored() {
        return this.ignoreAllLimes;
    }

    public void setLEDMode(String limelightName, LED_MODE mode) {
        switch (mode) {
            case PIPELINE -> LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            case OFF -> LimelightHelpers.setLEDMode_ForceOff(limelightName);
            case BLINK -> LimelightHelpers.setLEDMode_ForceBlink(limelightName);
            case ON -> LimelightHelpers.setLEDMode_ForceOn(limelightName);
        }
    }

    public void ignoreAll(boolean ignore) {
        this.ignoreAllLimes = ignore;
    }
}

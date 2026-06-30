package frc.robot.AutoTune;

public record TuningDataPoint(
    double timestamp,
    double position,
    double statorCurrent,
    double appliedVolts,
    double angularVelocityRotPerSec,
    double anngularAccRotPerSecPerSec
){
}

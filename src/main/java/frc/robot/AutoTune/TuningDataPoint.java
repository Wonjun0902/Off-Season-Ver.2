package frc.robot.AutoTune;

public record TuningDataPoint(
    double timestamp,
    double posRad,
    double posRot,
    double statorCurrent,
    double appliedVolts,
    double angularVelocityRadPerSec,
    double angularVelocityRotPerSec
){
}

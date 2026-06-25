package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
public class ThroatIOSim implements ThroatIO {
    public ThroatIOSim() {
       
    }
    @Override
    public void setSpeed(AngularVelocity targetVel) {
    }


    @Override
    public void stop() {
    }


    
    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(0.0);
    }


}

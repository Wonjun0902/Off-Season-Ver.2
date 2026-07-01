package frc.robot.AutoTune;

import frc.robot.AutoTune.HardWare.TunableMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.units.Units.*;

public class MotorExecute {

    private final TunableMotor m_motor;
    private final double m_gearRatio;
    private final boolean m_isPositionMechanism;

    //For position mechanism subsystems
    private final Angle m_forwardSoftLimit;
    private final Angle m_reverseSoftLimit;
    private final DigitalInput m_hardLimitSwitch;

    public MotorExecute(TunableMotor motor, double gearRatio, boolean isPositionMechanism, Angle forwardLimit, Angle reverseLimit, DigitalInput hardLimitSwitch){
        this.m_motor = motor;
        this.m_gearRatio = gearRatio;
        this.m_isPositionMechanism = isPositionMechanism;
        this.m_forwardSoftLimit = forwardLimit;
        this.m_reverseSoftLimit = reverseLimit;
        this.m_hardLimitSwitch = hardLimitSwitch;
    }

    public MotorExecute(TunableMotor motor, double gearRatio){
        this(motor, gearRatio, false, Rotations.of(Double.POSITIVE_INFINITY), Rotations.of(Double.NEGATIVE_INFINITY), (DigitalInput) null);
    }

    public MotorExecute(TunableMotor motor, double gearRatio, Angle forwardLimit, Angle reverseLimit, DigitalInput hardLimitSwitch){
        this(motor, gearRatio, true, forwardLimit, reverseLimit, hardLimitSwitch);
    }

    /**
     * I'll have all the motor controls to be new here. 
     * I want to do that because I want to have all motor controls to have forward and reverse soft limits
     * So there will be new methods that consider the mechanism of the motor: free spin or position mechanism 
     */

}
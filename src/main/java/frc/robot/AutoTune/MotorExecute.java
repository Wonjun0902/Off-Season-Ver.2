package frc.robot.AutoTune;

import frc.robot.AutoTune.HardWare.TunableMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

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
     * I'll have all the motor controls to be new here, seperate from the TunableMotor Classes 
     * I want to do that because I want to have all motor controls to have forward and reverse soft limits
     * So there will be new methods that consider the mechanism of the motor: free spin or position mechanism 
     */

    // Voltage Control for Position Mechanism Subsystem
    public void setMotorVoltagePOS(double volts){
        //1. Check if the hard limit switch is on
        if(m_hardLimitSwitch != null && m_hardLimitSwitch.get()){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond limits: ", "HARD LIMIT");
            throw new IllegalStateException("Motor Stopped: over hard limit");
        }
        //2. Gets the current motor position 
        Angle currentPos = m_motor.getMotorPosition();

        //3. Check if the motor is beyond the soft limits 
        if(volts > 0 && currentPos.gt(m_forwardSoftLimit)){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond the limits: ", "forward soft limit");
            return;
        }

        if(volts < 0 && currentPos.lt(m_reverseSoftLimit)){
            m_motor.stopMotor();
            SmartDashboard.putString("The motor is beyond the limits: ", "reverse soft limit");
            return;
        }

        //4. Run the motor with given voltage
        m_motor.setMotorVoltage(volts);
    }
}
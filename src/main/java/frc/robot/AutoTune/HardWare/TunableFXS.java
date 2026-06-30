package frc.robot.AutoTune.HardWare;

import frc.lib.LazyFXS;
import frc.lib.LazyFXSBuilder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;

public class TunableFXS implements TunableMotor{

    private LazyFXS m_motor;

    public TunableFXS(LazyFXS motor){
        this.m_motor = motor;
    }

    @Override
    public void setMotorVoltage(double volts){
        m_motor.setVoltageControl(Volts.of(volts));
    }

    //Note for MySelf 6/30
    //I just added a method for VoltageOut Controls. I added one method in LazyCTRE so that I can do VoltageOut controls. 
    //I need to figure out how to do current controls for Torque Current FOC 
    //Add methods to LazyCTRE, LazyFXS, and LazyTalon if you figure out how to do Current controls 
}

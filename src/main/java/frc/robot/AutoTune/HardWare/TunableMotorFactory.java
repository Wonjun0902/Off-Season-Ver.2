package frc.robot.AutoTune.HardWare;

import frc.lib.LazyFXS;
import frc.lib.LazyTalon;

public class TunableMotorFactory {

    //Creates a Tunable TalonFXS
    public static TunableMotor createFXSMotor(LazyFXS FXSMotor){
        return new TunableFXS(FXSMotor);
    }

    //Creates a Tunable TalonFX
    public static TunableMotor createTalonMotor(LazyTalon talonMotor){
        return new TunableTalon(talonMotor);
    }

    //Creates a Tunable Sim Motor
    public static TunableMotor createSimMotor(){
        return new TunableMotorSim();
    }
}

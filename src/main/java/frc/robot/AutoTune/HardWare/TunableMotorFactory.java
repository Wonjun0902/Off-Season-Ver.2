package frc.robot.AutoTune.HardWare;

import frc.lib.LazyFXS;
import frc.lib.LazyTalon;

public class TunableMotorFactory {

    //Creates a Tunable Pheonix Motor -> TalonFX or TalonFXS
    public static TunableMotor createFXSMotor(LazyFXS FXSMotor){
        return new TunablePheonix(FXSMotor);
    }

    //Creates a Tunable Sim Motor
    public static TunableMotor createSimMotor(){
        return new TunableMotorSim();
    }
}

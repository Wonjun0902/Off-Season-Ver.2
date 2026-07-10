package frc.robot.AutoTune;

import frc.robot.AutoTune.Commands.StandardPID.kDTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kPTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kSTuningCommand;
import frc.robot.AutoTune.Commands.StandardPID.kVTuningCommand;
import frc.robot.AutoTune.HardWare.LazyTunableFXS;
import frc.robot.AutoTune.HardWare.LazyTunableTalon;
import frc.robot.AutoTune.HardWare.TunableMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus; 

public class AutoCalibratorCombiner extends SubsystemBase{

    CANBus defaultBus = new CANBus();

    private final kSTuningCommand kSTuningCommand = new kSTuningCommand();
    private final kVTuningCommand kVTuningCommand = new kVTuningCommand();
    private final kPTuningCommand kPTuningCommand = new kPTuningCommand();
    private final kDTuningCommand kDTuningCommand = new kDTuningCommand();

    //Implement it with real motors 
    private final MotorExecute motor1;
    private final MotorExecute motor2;
    private final MotorExecute motor3;

    //Implement with Actual Values 
    public AutoCalibratorCombiner(){
        //Initialize Motors 
        motor1 = new MotorExecute(new LazyTunableFXS(0, 0)); 
        motor2 = new MotorExecute(new LazyTunableTalon(0, 0));
        motor3 = new MotorExecute(new LazyTunableTalon(0, 0));
    }

    /**
     * Command for Seqeunce for the master tuning Command 
     * @param motor motorExecute 
     * @param targetVelocity setUp target Velocity for each subsystem -> SET UP DIFFERENTLY 
     * @param gearRatio DIFFERENT GEAR RATIOS FOR ALL SUBSYSTEMS
     */
    public Command tuningSeqeunce(MotorExecute motor, AngularVelocity targetVelocity, double gearRatio){
        double targetSpeedRAD = targetVelocity.in(RotationsPerSecond)*2*Math.PI;

        return Commands.sequence(
            //1. Tune kS
            kSTuningCommand.kSTuningCommand(targetSpeedRAD, gearRatio, motor),
            Commands.waitSeconds(1), //TODO: Change to real values after testing

            //2. Tune kV
            kVTuningCommand.kVTuningCommand(targetVelocity, 0.05, gearRatio, motor, kSTuningCommand),
            Commands.waitSeconds(1), //TODO: Change to real values after testing 

            //3. Tune kP 
            kPTuningCommand.kPTuningCommand(0.01, targetSpeedRAD, 2, 4, gearRatio, motor, kSTuningCommand, kVTuningCommand), //TODO: Change to Real Values after testing 
            Commands.waitSeconds(1), //TODO: Change to real values after testing 
            
            //4. Tune kD
            kDTuningCommand.kDTuningCommand(0.01, targetSpeedRAD, 2, 1, gearRatio, kPTuningCommand, kSTuningCommand, kVTuningCommand, motor), //TODO: Change to real values after testing 
            Commands.waitSeconds(1)//TODO: Change to real values after testing 

        );
    }

    /**
     * THE ULTIMATE tuning command 
     * Runs the tuning sequence for each motor 
     * @return
     */
    public Command tuneCommand(){

        return Commands.sequence( 

        //1. Tune motor1
        tuningSeqeunce(motor1, RotationsPerSecond.of(0.0), 0),
        Commands.waitSeconds(1.5),

        //2. Tune motor2
        tuningSeqeunce(motor1, RotationsPerSecond.of(0.0), 0),
        Commands.waitSeconds(1.5),

        //3. Tune motor3
        tuningSeqeunce(motor1, null, 0)
        )
        .finallyDo(
            (interrupted) -> {
                motor1.stopMotor();
                motor2.stopMotor();
                motor3.stopMotor();
            }
        );
    }
}

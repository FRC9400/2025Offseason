package frc.robot.Subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants.pivotConstants;

public class Pivot extends SubsystemBase {
    private final PivotIO pivotIO;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public Pivot(PivotIO pivotIO){
        this.pivotIO = pivotIO;
    }
    

    @Override
    public void periodic(){
        pivotIO.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public void requestMotionMagic(double deg){
        pivotIO.requestMotionMagic(deg);
    }

    public void requestVoltage(double volts){
        pivotIO.requestVoltage(volts);
    }
    
}

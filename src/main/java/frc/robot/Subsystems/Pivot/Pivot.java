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
    private final SysIdRoutine pivotRoutine;


    public Pivot(PivotIO pivotIO){
        this.pivotIO = pivotIO;
        pivotRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> pivotIO.requestVoltage(volts.in(Volts)), null,
                    this));
    }

    public Command pivotSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            pivotRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> Math.abs(inputs.positionDeg[0]) > 35), 
            this.runOnce(() -> pivotIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            pivotRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> inputs.positionDeg[0] < 5), 
            this.runOnce(() -> pivotIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            pivotRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> Math.abs(inputs.positionDeg[0]) > 35),
            this.runOnce(() -> pivotIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            pivotRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> inputs.positionDeg[0] < 5), //Keep in mind the max height is around 0.6
            this.runOnce(() -> pivotIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop()));
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

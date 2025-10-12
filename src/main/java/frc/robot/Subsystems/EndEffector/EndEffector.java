package frc.robot.Subsystems.EndEffector;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO endEffectorIO;
    private EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private final SysIdRoutine pivotSysID;
    /*private double algaeSetpointVolts = 0;
    private double coralSetpointVolts = 0;
    private double pivotSetpointVolts = 0;
    private double pivotSetpointDeg = 0;

    public enum EndEffectorStates{
        IDLE,
        INTAKE,
        SCORE,
        HOLD_ALGAE
    }*/

    public EndEffector(EndEffectorIO endEffectorIO){
        this.endEffectorIO = endEffectorIO;
        pivotSysID = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(3), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> endEffectorIO.requestPivotVoltage(volts.in(Volts)), null, this));
    }

    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                pivotSysID
                        .quasistatic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 100),
                this.runOnce(() -> endEffectorIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                pivotSysID
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> endEffectorIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 100),
                this.runOnce(() -> endEffectorIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> endEffectorIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    }

    @Override
    public void periodic(){
        // update inputs, voltage setpoint for states
        endEffectorIO.updateInputs(inputs);
        Logger.processInputs("End Effector", inputs);
    }

    // Control Requests
    public void requestAlgaeVoltage(double voltage){
        endEffectorIO.requestAlgaeVoltage(voltage);
    }

    public void requestCoralVoltage(double voltage){
        endEffectorIO.requestCoralVoltage(voltage);
    }

    public void requestPivotVoltage(double voltage){
        endEffectorIO.requestPivotVoltage(voltage);
    }

    public void requestPivotMotionMagic(double degrees){
        endEffectorIO.requestPivotMotionMagic(degrees);
    }

}
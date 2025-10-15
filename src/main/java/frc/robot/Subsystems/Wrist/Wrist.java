package frc.robot.Subsystems.Wrist;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Wrist extends SubsystemBase {
    private final WristIO wristIO;
    private WristInputsAutoLogged inputs = new WristInputsAutoLogged();
    private WristStates wristState = WristStates.IDLE;
    private final SysIdRoutine pivotSysID;
    private double pivotSetpointVolts = 0;
    private double pivotSetpointDeg = 0;

    public enum WristStates{
        IDLE,
        SETPOINT
    }

    public Wrist(WristIO wristIO){
        this.wristIO = wristIO;
        pivotSysID = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(3), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> wristIO.requestPivotVoltage(volts.in(Volts)), null, this));
    }

    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                pivotSysID
                        .quasistatic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 100),
                this.runOnce(() -> wristIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                pivotSysID
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> wristIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 100),
                this.runOnce(() -> wristIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> wristIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    }

    //state machine
    @Override
    public void periodic(){
        wristIO.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        Logger.recordOutput("Wrist State", this.wristState);

        switch(wristState){
            case IDLE:
                wristIO.requestPivotVoltage(0);
                break;
            case SETPOINT:
                wristIO.requestPivotMotionMagic(pivotSetpointDeg);
                break;
            default:
                break;
        }
    }

    // Control Requests
    public void requestPivotVoltage(double voltage){
        pivotSetpointVolts = voltage;
    }

    public void requestPivotMotionMagic(double degrees){
        pivotSetpointDeg = degrees;
    }

    // states
    public void setState(WristStates nextState){
        wristState = nextState;
    }

    public WristStates getState(){
        return wristState;
    }

    public void requestIdle(){
        setState(WristStates.IDLE);
    }

    public void requestSetpoint(){
        setState(WristStates.SETPOINT);
    }
}
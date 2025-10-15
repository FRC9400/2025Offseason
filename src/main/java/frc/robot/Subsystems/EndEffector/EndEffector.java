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
    private EndEffectorStates endEffectorState = EndEffectorStates.IDLE;
    private final SysIdRoutine pivotSysID;
    private double algaeSetpointVolts = 0;
    private double coralSetpointVolts = 0;
    private double pivotSetpointVolts = 0;
    private double pivotSetpointDeg = 0;

    public enum EndEffectorStates{
        IDLE,
        INTAKE,
        SCORE,
        HOLD_ALGAE
    }

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

    //state machine
    @Override
    public void periodic(){
        endEffectorIO.updateInputs(inputs);
        Logger.processInputs("End Effector", inputs);
        Logger.recordOutput("End Effector State", this.endEffectorState);

        switch(endEffectorState){
            case IDLE:
                endEffectorIO.setAlgaeVoltage(0);
                endEffectorIO.setCoralVoltage(0);
                endEffectorIO.setPivotVoltage(0);
                break;
            case INTAKE:
                endEffectorIO.setAlgaeVoltage(0);
                endEffectorIO.setCoralVoltage(coralSetpointVolts);
                endEffectorIO.setPivotMotionMagic(pivotSetpointDeg);
                break;
            case SCORE:
                endEffectorIO.setAlgaeVoltage(0);
                endEffectorIO.setCoralVoltage(coralSetpointVolts);
                endEffectorIO.setPivotMotionMagic(pivotSetpointDeg);
                break;
            case HOLD_ALGAE:
                endEffectorIO.setAlgaeVoltage(algaeSetpointVolts);
                endEffectorIO.setCoralVoltage(0);
                endEffectorIO.setPivotMotionMagic(pivotSetpointDeg);
                break;
            default:
                break;
        }
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

    // states
    public void setState(EndEffectorStates nextState){
        endEffectorState = nextState;
    }

    public EndEffectorStates returnState(){
        return endEffectorState;
    }

    public void requestIdle(){
        setState(EndEffectorStates.IDLE);
    }

    public void requestIntake(){
        setState(EndEffectorStates.INTAKE);
    }

    public void requestScore(){
        setState(EndEffectorStates.SCORE);
    }

    public void requestHoldAlgae(){
        setState(EndEffectorStates.HOLD_ALGAE);
    }

}
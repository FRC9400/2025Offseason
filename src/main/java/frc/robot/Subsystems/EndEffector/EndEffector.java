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
    private double algaeSetpointVolts = 0;
    private double coralSetpointVolts = 0;

    public enum EndEffectorStates{
        IDLE,
        INTAKE,
        SCORE,
        HOLD_ALGAE
    }

    public EndEffector(EndEffectorIO endEffectorIO){
        this.endEffectorIO = endEffectorIO;
    }

    //state machine
    @Override
    public void periodic(){
        endEffectorIO.updateInputs(inputs);
        Logger.processInputs("End Effector", inputs);
        Logger.recordOutput("End Effector State", this.endEffectorState);

        switch(endEffectorState){
            case IDLE:
                endEffectorIO.requestAlgaeVoltage(0);
                endEffectorIO.requestCoralVoltage(0);
                break;
            case INTAKE:
                endEffectorIO.requestAlgaeVoltage(0);
                endEffectorIO.requestCoralVoltage(coralSetpointVolts);
                break;
            case SCORE:
                endEffectorIO.requestAlgaeVoltage(0);
                endEffectorIO.requestCoralVoltage(coralSetpointVolts);
                break;
            case HOLD_ALGAE:
                endEffectorIO.requestAlgaeVoltage(algaeSetpointVolts);
                endEffectorIO.requestCoralVoltage(0);
                break;
            default:
                break;
        }
    }

    // Control Requests
    public void requestAlgaeVoltage(double voltage){
        algaeSetpointVolts = voltage;
    }

    public void requestCoralVoltage(double voltage){
        coralSetpointVolts = voltage;
    }

    // states
    public void setState(EndEffectorStates nextState){
        endEffectorState = nextState;
    }

    public EndEffectorStates getState(){
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
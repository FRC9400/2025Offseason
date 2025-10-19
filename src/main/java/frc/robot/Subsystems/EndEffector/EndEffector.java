package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

import frc.commons.LoggedTunableNumber;

public class EndEffector {
    private final EndEffectorIO endEffectorIO;
    private EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private EndEffectorStates endEffectorState = EndEffectorStates.IDLE;
    private double algaeSetpointVolts = 0;
    private double coralSetpointVolts = 0;

    public boolean hasAlgae = false;
    LoggedTunableNumber algaeVoltage = new LoggedTunableNumber("Elevator/Hold Algae Voltage", 2);

    public enum EndEffectorStates{
        IDLE,
        INTAKE_CORAL,
        OUTTAKE_CORAL,
        DEALGAE,
        BARGE,
        PROCESSOR
    }

    public EndEffector(EndEffectorIO endEffectorIO){
        this.endEffectorIO = endEffectorIO;
    }

    public void Loop(){
        endEffectorIO.updateInputs(inputs);
        Logger.processInputs("End Effector", inputs);
        Logger.recordOutput("End Effector State", this.endEffectorState);
        Logger.recordOutput("Coral Setpoint", coralSetpointVolts);
        Logger.recordOutput("Algae Setpoint", algaeSetpointVolts);


        switch(endEffectorState){
            case IDLE:
                endEffectorIO.requestCoralVoltage(0);
                if (hasAlgae){
                    endEffectorIO.requestAlgaeVoltage(algaeVoltage.get());
                } else {
                    endEffectorIO.requestAlgaeVoltage(0);
                }
                break;
            case INTAKE_CORAL:
                endEffectorIO.requestCoralVoltage(coralSetpointVolts);
                if (hasAlgae){
                    endEffectorIO.requestAlgaeVoltage(algaeVoltage.get());
                } else {
                    endEffectorIO.requestAlgaeVoltage(0);
                }
                break;
            case OUTTAKE_CORAL:
                endEffectorIO.requestCoralVoltage(coralSetpointVolts);
                if (hasAlgae){
                    endEffectorIO.requestAlgaeVoltage(algaeVoltage.get());
                } else {
                    endEffectorIO.requestAlgaeVoltage(0);
                }
                break;
            case DEALGAE:
                endEffectorIO.requestAlgaeVoltage(algaeSetpointVolts);
                endEffectorIO.requestCoralVoltage(0);
                break;
            case BARGE:
                endEffectorIO.requestAlgaeVoltage(algaeSetpointVolts);
                endEffectorIO.requestCoralVoltage(0);
                break;
            case PROCESSOR:
                endEffectorIO.requestAlgaeVoltage(algaeSetpointVolts);
                endEffectorIO.requestCoralVoltage(0);
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(EndEffectorStates.IDLE);
    }

    public void requestCoralIntake(double voltage){
        coralSetpointVolts = voltage;
        setState(EndEffectorStates.INTAKE_CORAL);
    }

    public void requestCoralOuttake(double voltage){
        coralSetpointVolts = voltage;
        setState(EndEffectorStates.OUTTAKE_CORAL);
    }

    public void requestDealgae(double voltage){
        algaeSetpointVolts = voltage;
        setState(EndEffectorStates.DEALGAE);
    }

    public void requestBarge(double voltage){
        algaeSetpointVolts = voltage;
        setState(EndEffectorStates.BARGE);
    }

    public void requestProcessor(double voltage){
        algaeSetpointVolts = voltage;
        setState(EndEffectorStates.PROCESSOR);
    }

    public double getAlgaeCurrent(){
        return inputs.algaeCurrent;
    }

    public double getCoralCurrent(){
        return inputs.coralCurrent;
    }

    public void setState(EndEffectorStates nextState){
        this.endEffectorState = nextState;
    }

    public EndEffectorStates getEndEffectorState(){
        return this.endEffectorState;
    }
}
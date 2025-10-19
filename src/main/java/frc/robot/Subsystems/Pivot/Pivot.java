package frc.robot.Subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

public class Pivot {
    private final PivotIO pivotIO;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private PivotStates pivotState = PivotStates.IDLE;
    private double pivotSetpoint = 0; 

    public enum PivotStates{
        IDLE,
        SETPOINT, 
        ZERO_SENSOR,
        HOLD
    }

    public Pivot(PivotIO pivotIO){
        this.pivotIO = pivotIO;
    }

    public void Loop(){
        pivotIO.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        Logger.recordOutput("Pivot State", pivotState);
        Logger.recordOutput("Pivot Setpoint", pivotSetpoint);

        switch(pivotState){
            case IDLE:
                pivotIO.requestVoltage(0);
                break;
            case SETPOINT:
                pivotIO.requestMotionMagic(pivotSetpoint);
                break;
            case ZERO_SENSOR:
                pivotIO.setPosition(0);
                pivotIO.requestVoltage(0);
                break;
            case HOLD:
                pivotIO.requestMotionMagic(pivotSetpoint);
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(PivotStates.IDLE);
    }

    public void requestSetpoint(double degrees){
        pivotSetpoint = degrees;
        setState(PivotStates.SETPOINT);
    }

    public void requestHold(){
        setState(PivotStates.HOLD);
    }

    public void zeroSensor(){
        setState(PivotStates.ZERO_SENSOR);
    }

    public boolean atSetpoint(){
        return Math.abs(inputs.positionDeg[0] - pivotSetpoint) < 2;
    }

    public void setState(PivotStates nextState){
        this.pivotState = nextState;
    }

    public PivotStates getPivotStates(){
        return this.pivotState;
    }
    
}

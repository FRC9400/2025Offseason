package frc.robot.Subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

public class Wrist {
    private final WristIO wristIO;
    private WristInputsAutoLogged inputs = new WristInputsAutoLogged();
    private WristStates wristState = WristStates.IDLE;
    private double wristSetpoint = 0;


    public enum WristStates{
        IDLE,
        SETPOINT, 
        ZERO_SENSOR,
        HOLD
    }

    public Wrist(WristIO wristIO){
        this.wristIO = wristIO;
    }

    public void Loop(){
        wristIO.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        Logger.recordOutput("Wrist Setpoint", wristSetpoint);
        Logger.recordOutput("Wrist State", wristState);
        
        switch(wristState){
                case IDLE:
                        wristIO.requestVoltage(0);
                        break;
                case SETPOINT:
                        wristIO.requestMotionMagic(wristSetpoint);
                        break;
                case ZERO_SENSOR:
                        wristIO.setPosition(0);
                        wristIO.requestVoltage(0);
                        break;
                case HOLD:
                        wristIO.requestMotionMagic(wristSetpoint);
                        break;
                default:
                        break;
        }
    }

    public void requestIdle(){
        setState(WristStates.IDLE);
    }

    public void requestSetpoint(double degrees){
        wristSetpoint = degrees;
        setState(WristStates.SETPOINT);
    }

    public void requestHold(){
        setState(WristStates.HOLD);
    }

    public void zeroSensor(){
        setState(WristStates.ZERO_SENSOR);
    }

    public boolean atSetpoint(){
        return Math.abs(inputs.pivotPosDeg - wristSetpoint) < 2;
    }

    public void setState(WristStates nextState){
        this.wristState = nextState;
    }

    public WristStates getWristState(){
        return this.wristState;
    }

}
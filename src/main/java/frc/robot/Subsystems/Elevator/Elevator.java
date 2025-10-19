package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;

public class Elevator {
    private final ElevatorIO elevatorIO;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorStates elevatorState = ElevatorStates.IDLE;
    private double elevatorSetpoint = 0; 

    public enum ElevatorStates{
        IDLE,
        SETPOINT,
        ZERO
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void Loop(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator Setpoint", elevatorSetpoint);
        Logger.recordOutput("ElevatorState", elevatorState);

        switch(elevatorState){
            case IDLE:
                elevatorIO.requestVoltage(0);
                break;
            case SETPOINT:
                elevatorIO.requestMotionMagic(elevatorSetpoint);
                break;
            case ZERO:
                elevatorIO.requestMotionMagic(0);
                break;
        }
    }

    public void requestIdle(){
        setState(ElevatorStates.IDLE);
    }

    public void requestZero(){
        elevatorSetpoint = 0;
        setState(ElevatorStates.ZERO);
    }

    public void requestSetpoint(double setpointMeters){
        elevatorSetpoint = setpointMeters;
        setState(ElevatorStates.SETPOINT);
    }

    public boolean atSetpoint(){
        return Math.abs(inputs.elevatorHeightMeters - elevatorSetpoint) < Units.inchesToMeters(0.3);
    }

    public void setState(ElevatorStates nextState){
        this.elevatorState = nextState;
    }

    public ElevatorStates getElevatorState(){
        return this.elevatorState;
    }
}

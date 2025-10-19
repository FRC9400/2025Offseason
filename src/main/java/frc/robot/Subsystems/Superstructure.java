package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.Pivot.Pivot;
import frc.robot.Subsystems.Pivot.PivotIO;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristIO;
import frc.commons.ArmPosition;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.armPositionConstants;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private Elevator s_elevator;
    private EndEffector s_endeffector;
    private Pivot s_pivot;
    private Wrist s_wrist;

    private String selectedHeight = "L1";

    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    LoggedTunableNumber coralIntakeVoltage = new LoggedTunableNumber("Superstructure/Coral Intake Voltage", 3);
    LoggedTunableNumber coralIntakeCurrent = new LoggedTunableNumber("Superstructure/Coral Intake Current Limit", 25);
    LoggedTunableNumber coralScoreVoltage = new LoggedTunableNumber("Superstructure/Coral Score Voltage", -3);

    LoggedTunableNumber dealgaeVoltage = new LoggedTunableNumber("Superstructure/Algae Dealgae Voltage", 3);
    LoggedTunableNumber dealgaeCurrent = new LoggedTunableNumber("Superstructure/Algae Dealgae Current", 25);
    LoggedTunableNumber bargeVoltage = new LoggedTunableNumber("Superstructure/Algae Barge Voltage", -3);
    LoggedTunableNumber processorVoltage = new LoggedTunableNumber("Superstructure/Algae Processor Voltage", -3);

    LoggedTunableNumber elevatorPos = new LoggedTunableNumber("Superstructure/Elevator Position", 0);
    LoggedTunableNumber pivotPos = new LoggedTunableNumber("Superstructure/Pivot Degrees", 0);
    LoggedTunableNumber wristPos = new LoggedTunableNumber("Superstructure/Wrist degrees", 0);


    public Superstructure(ElevatorIO elevatorIO, EndEffectorIO endEffectorIO, PivotIO pivotIO, WristIO wristIO){
        this.s_elevator = new Elevator(elevatorIO);
        this.s_endeffector = new EndEffector(endEffectorIO);
        this.s_pivot = new Pivot(pivotIO);
        this.s_wrist = new Wrist(wristIO);
    }

    public enum SuperstructureStates{
        IDLE,
        ZERO,
        ELEVATOR_DOWN,
        KICKSTAND,
        STATION_INTAKE_A,
        STATION_INTAKE_B,
        GROUND_INTAKE_A,
        GROUND_INTAKE_B,
        SCORE_CORAL_A,
        SCORE_CORAL_B,
        SCORE_CORAL_C,
        DEALGAE_A,
        DEALGAE_B,
        DEALGAE_C,
        BARGE_A,
        BARGE_B,
        BARGE_C,
        PROCESSOR_A,
        PROCESSOR_B,
        TEST_CORAL_INTAKE,
        TEST_CORAL_SCORE,
        TEST_DEALGAE,
        TEST_ARM_POSITION,
        ZERO_WRIST
    }

    @Override
    public void periodic(){
        s_elevator.Loop();
        s_endeffector.Loop();
        s_pivot.Loop();
        s_wrist.Loop();
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                s_elevator.requestIdle();
                s_endeffector.requestIdle();
                s_pivot.requestIdle();
                s_wrist.requestIdle();
                break;
            case ZERO:
                s_elevator.requestZero();
                s_endeffector.requestIdle();
                s_pivot.requestSetpoint(0);
                s_wrist.requestSetpoint(0);
                break;
            case ELEVATOR_DOWN:
                s_elevator.requestZero();
                s_endeffector.requestIdle();
                s_pivot.requestHold();
                s_wrist.requestHold();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.KICKSTAND);
                }
                break;
            case KICKSTAND:
                requestArmPosition(armPositionConstants.kickstandPosition);
                s_endeffector.requestIdle();
                break;
            case STATION_INTAKE_A:
                requestArmPosition(armPositionConstants.stationIntake);
                s_endeffector.requestIdle();
                if (atSetpoint()){
                    setState(SuperstructureStates.STATION_INTAKE_B);
                }
                break;
            case STATION_INTAKE_B:
                requestArmPosition(armPositionConstants.stationIntake);
                s_endeffector.requestCoralIntake(coralIntakeVoltage.get());
                if (s_endeffector.getCoralCurrent() > coralIntakeCurrent.get() && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.KICKSTAND);
                }
                break;
            case GROUND_INTAKE_A:
                requestArmPosition(armPositionConstants.groundIntake);
                s_endeffector.requestIdle();
                if (atSetpoint()){
                    setState(SuperstructureStates.GROUND_INTAKE_B);
                }
                break;
            case GROUND_INTAKE_B:
                requestArmPosition(armPositionConstants.groundIntake);
                s_endeffector.requestCoralIntake(coralIntakeVoltage.get());
                if (s_endeffector.getCoralCurrent() > coralIntakeCurrent.get() && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.KICKSTAND);
                }
                break;
            case SCORE_CORAL_A:
                s_elevator.requestZero();
                s_pivot.requestSetpoint(getArmPosition().getPivotDegrees());
                s_wrist.requestHold();
                s_endeffector.requestIdle();
                if (s_pivot.atSetpoint()){
                    setState(SuperstructureStates.SCORE_CORAL_B);
                }
                break;
            case SCORE_CORAL_B:
                requestArmPosition(getArmPosition());
                s_endeffector.requestIdle();
                if (atSetpoint()){
                    setState(SuperstructureStates.SCORE_CORAL_C);
                }
                break;
            case SCORE_CORAL_C:
                requestArmPosition(getArmPosition());
                s_endeffector.requestCoralOuttake(coralScoreVoltage.get());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 3){
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case DEALGAE_A:
                s_elevator.requestZero();
                s_pivot.requestSetpoint(getArmPosition().getPivotDegrees());
                s_wrist.requestHold();
                s_endeffector.requestIdle();
                if (s_pivot.atSetpoint()){
                    setState(SuperstructureStates.DEALGAE_B);
                }
                break;
            case DEALGAE_B:
                requestArmPosition(getArmPosition());
                s_endeffector.requestIdle();
                if (atSetpoint()){
                    setState(SuperstructureStates.DEALGAE_C);
                }
                break;
            case DEALGAE_C:
                requestArmPosition(getArmPosition());
                s_endeffector.requestDealgae(dealgaeVoltage.get());
                if (s_endeffector.getAlgaeCurrent() > dealgaeCurrent.get() && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                   // s_endeffector.hasAlgae = true; // this is to be used for holding the algae
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case BARGE_A:
                s_elevator.requestZero();
                s_pivot.requestSetpoint(armPositionConstants.barge.getPivotDegrees());
                s_wrist.requestHold();
                s_endeffector.requestIdle();
                if (s_pivot.atSetpoint()){
                    setState(SuperstructureStates.BARGE_B);
                }
                break;
            case BARGE_B:
                requestArmPosition(armPositionConstants.barge);
                s_endeffector.requestIdle();
                if (atSetpoint()){
                    setState(SuperstructureStates.BARGE_C);
                }
                break;
            case BARGE_C:
                requestArmPosition(armPositionConstants.barge);
                s_endeffector.requestBarge(bargeVoltage.get());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 3){
                    s_endeffector.hasAlgae = false;
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case PROCESSOR_A:
                requestArmPosition(armPositionConstants.processor);
                s_endeffector.requestIdle();
                if (atSetpoint()){
                    setState(SuperstructureStates.PROCESSOR_B);
                }
                break;
            case PROCESSOR_B:
                requestArmPosition(armPositionConstants.processor);
                s_endeffector.requestProcessor(processorVoltage.get());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 3){
                    s_endeffector.hasAlgae = false;
                    setState(SuperstructureStates.KICKSTAND);
                }
                break;
            case TEST_ARM_POSITION:
                s_elevator.requestSetpoint(elevatorPos.get());
                s_endeffector.requestIdle();
                s_pivot.requestSetpoint(pivotPos.get());
                s_wrist.requestSetpoint(wristPos.get());
                break;
            case TEST_CORAL_INTAKE:
                s_elevator.requestZero();
                s_endeffector.requestCoralIntake(coralIntakeVoltage.get());
                s_pivot.requestHold();
                s_wrist.requestHold();
                if (s_endeffector.getCoralCurrent() > coralIntakeCurrent.get() && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.KICKSTAND);
                }
                break;
            case TEST_CORAL_SCORE:
                s_elevator.requestZero();
                s_endeffector.requestCoralOuttake(coralScoreVoltage.get());
                s_pivot.requestHold();
                s_wrist.requestHold();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 3){
                    setState(SuperstructureStates.KICKSTAND);
                }
                break;
            case TEST_DEALGAE:
                s_elevator.requestZero();
                s_endeffector.requestDealgae(dealgaeVoltage.get());
                s_pivot.requestHold();
                s_wrist.requestHold();
                if (s_endeffector.getAlgaeCurrent() > dealgaeCurrent.get() && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    // s_endeffector.hasAlgae = true; // this is to be used for holding the algae
                     setState(SuperstructureStates.KICKSTAND);
                }
                break;
            case ZERO_WRIST:
                s_elevator.requestIdle();
                s_endeffector.requestIdle();
                s_pivot.requestIdle();
                s_wrist.zeroSensor();
                break;
            default:
                break;
        }
    }

    /* PRIVATE ARM METHODS */

    private boolean atSetpoint(){
        // Returns true only if the elevator, pivot, and wrist are at setpoint
        return s_elevator.atSetpoint() && s_pivot.atSetpoint() && s_wrist.atSetpoint();
    }

    private void requestArmPosition(ArmPosition position){
        s_elevator.requestSetpoint(position.getElevatorHeight());
        s_pivot.requestSetpoint(position.getPivotDegrees());
        s_wrist.requestSetpoint(position.getWristDegrees());
    }

    private ArmPosition getArmPosition(){
        if (selectedHeight == "L1"){
            return armPositionConstants.L1;
        } else if (selectedHeight == "L2"){
            return armPositionConstants.L2;
        } else if (selectedHeight == "L3"){
            return armPositionConstants.L3;
        } else if (selectedHeight == "L4"){
            return armPositionConstants.L4;
        } else if (selectedHeight == "Low"){
            return armPositionConstants.dealgaeLow;
        } else if (selectedHeight == "High"){
            return armPositionConstants.dealgaeHigh;
        }
        return armPositionConstants.kickstandPosition;
    }

    /* PUBLIC METHODS */

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
    }

    public void requestZero(){
        setState(SuperstructureStates.ZERO);
    }

    public void requestKickstand(){
        setState(SuperstructureStates.KICKSTAND);
    }

    public void requestStationIntake(){
        setState(SuperstructureStates.STATION_INTAKE_A);
    }

    public void requestGroundIntake(){
        setState(SuperstructureStates.GROUND_INTAKE_A);
    }

    public void requestL1Coral(){
        selectedHeight = "L1";
        setState(SuperstructureStates.SCORE_CORAL_A);
    }

    public void requestL2Coral(){
        selectedHeight = "L2";
        setState(SuperstructureStates.SCORE_CORAL_A);
    }

    public void requestL3Coral(){
        selectedHeight = "L3";
        setState(SuperstructureStates.SCORE_CORAL_A);
    }

    public void requestL4Coral(){
        selectedHeight = "L4";
        setState(SuperstructureStates.SCORE_CORAL_A);
    }

    public void requestDealgaeLow(){
        selectedHeight = "Low";
        setState(SuperstructureStates.DEALGAE_A);
    }

    public void requestDealgaeHigh(){
        selectedHeight = "High";
        setState(SuperstructureStates.DEALGAE_A);
    }

    public void requestBarge(){
        setState(SuperstructureStates.BARGE_A);
    }

    public void requestProcessor(){
        setState(SuperstructureStates.PROCESSOR_A);
    }

    /* TESTS */

    public void requestTestCoralIntake(){
        setState(SuperstructureStates.TEST_CORAL_INTAKE);
    }

    public void requestTestCoralScore(){
        setState(SuperstructureStates.TEST_CORAL_SCORE);
    }

    public void requestTestDealgae(){
        setState(SuperstructureStates.TEST_DEALGAE);
    }

    public void requestTestArmPosition(){
        setState(SuperstructureStates.TEST_ARM_POSITION);
    }

    public void zeroWrist(){
        setState(SuperstructureStates.ZERO_WRIST);
    }
    
    private void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1.0E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
    
}

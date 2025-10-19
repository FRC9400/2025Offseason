package frc.robot.Subsystems.Wrist;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.commons.LoggedTunableNumber;

public class Wrist extends SubsystemBase {
    private final WristIO wristIO;
    private WristInputsAutoLogged inputs = new WristInputsAutoLogged();
    private final SysIdRoutine wristRoutine;
    private double pivotSetpointVolts = 0;
    private double pivotSetpointDeg = 0;

    public Wrist(WristIO wristIO){
        this.wristIO = wristIO;
        wristRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(3), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> wristIO.requestPivotVoltage(volts.in(Volts)), null, this));
    }

    public Command wristSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            wristRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> Math.abs(inputs.pivotPosDeg) > 90), 
            this.runOnce(() -> wristIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),
            wristRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> inputs.pivotPosDeg < 5), 
            this.runOnce(() -> wristIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),
            wristRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> Math.abs(inputs.pivotPosDeg) > 90),
            this.runOnce(() -> wristIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),

            wristRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> inputs.pivotPosDeg < 5), //Keep in mind the max height is around 0.6
            this.runOnce(() -> wristIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop()));
    }

    //state machine
    @Override
    public void periodic(){
        wristIO.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        Logger.recordOutput("Wrist Setpoint", pivotSetpointDeg);
    }

    // Control Requests
    public void requestPivotVoltage(double voltage){
        pivotSetpointVolts = voltage;
        wristIO.requestPivotVoltage(voltage);
    }

    public void requestPivotMotionMagic(double degrees){
        pivotSetpointDeg = degrees;
        wristIO.requestPivotMotionMagic(degrees);
    }
}
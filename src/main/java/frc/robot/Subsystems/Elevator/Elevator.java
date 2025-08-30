package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.elevatorConstants;


public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine elevatorRoutine;

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> elevatorIO.requestVoltage(volts.in(Volts)), null,
                        this));
    }

    public Command elevatorSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            elevatorRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.2), //Keep in mind the max height is around 0.6
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            elevatorRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> inputs.elevatorHeightMeters < 0.1), //Keep in mind the max height is around 0.6
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            elevatorRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.2), //Keep in mind the max height is around 0.6
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            elevatorRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> inputs.elevatorHeightMeters < 0.1), //Keep in mind the max height is around 0.6
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop()));
    }

    public void setHeight(double meters, double radians){
        elevatorIO.requestPosition(meters, radians);
    }

    @Override
    public void periodic(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }
}

package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO{
    TalonFX leftMotor = new TalonFX(canIDConstants.elevatorMotor1, "canivore");
    TalonFX rightMotor = new TalonFX(canIDConstants.elevatorMotor2, "canivore");
    TalonFXConfiguration config = new TalonFXConfiguration();

    private MotionMagicVoltage motionMagicVolts = new MotionMagicVoltage(0).withEnableFOC(true);
    private VoltageOut voltageReq = new VoltageOut(0).withEnableFOC(true);
    private double setpoint = 0;

    LoggedTunableNumber kGTunable = new LoggedTunableNumber("Elevator/kG", 0.5);
    LoggedTunableNumber kCTunable = new LoggedTunableNumber("Elevator/kC", -0.18);
    LoggedTunableNumber angle = new LoggedTunableNumber("Elevator/Angle", 0);

    public double kG = 0.5;//placeholder values
    public double kC = -0.18;

    public ElevatorIOTalonFX() {
        config.MotionMagic.MotionMagicCruiseVelocity = elevatorConstants.CruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = elevatorConstants.Acceleration;
        config.MotionMagic.MotionMagicJerk = elevatorConstants.Jerk;

        config.Slot0.kP = 1;//placeholders
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.01;
        config.Slot0.kS = 0.09;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;

        config.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        config.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;

        leftMotor.getConfigurator().apply(config);
        rightMotor.setControl(new Follower(canIDConstants.elevatorMotor1, false));
        
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    public void updateInputs(ElevatorIOInputs inputs){
        inputs.appliedVolts = voltageReq.Output;
        inputs.setpointMeters = setpoint;
        inputs.elevatorHeightMeters = Conversions.RotationsToMeters(leftMotor.getPosition().getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.velocityRPS = new double[] {leftMotor.getVelocity().getValueAsDouble(), rightMotor.getVelocity().getValueAsDouble()};
        inputs.velocityMPS =  new double[] {Conversions.RPStoMPS(leftMotor.getVelocity().getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio), Conversions.RPStoMPS(rightMotor.getVelocity().getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)};
        inputs.currentAmps = new double[] {leftMotor.getStatorCurrent().getValueAsDouble(), rightMotor.getStatorCurrent().getValueAsDouble()};
        inputs.tempFahrenheit = new double[] {leftMotor.getDeviceTemp().getValueAsDouble(), rightMotor.getDeviceTemp().getValueAsDouble()};
    }

    public void requestPosition(double meters, double radians){
        kG = kGTunable.getAsDouble();
        kC = kCTunable.getAsDouble();
        setpoint = meters;
        leftMotor.setControl(motionMagicVolts.withPosition(meters/elevatorConstants.wheelCircumferenceMeters).withFeedForward(kG*Math.sin(angle.getAsDouble())+kC));//placeholder value
    }

    public void requestVoltage(double volts){
        leftMotor.setControl(voltageReq.withOutput(volts));
    }
}
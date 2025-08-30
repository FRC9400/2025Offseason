package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO{
    TalonFX leftMotor = new TalonFX(canIDConstants.leftElevatorMotor, "rio");
    TalonFX rightMotor = new TalonFX(canIDConstants.rightElevatorMotor, "rio");
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    private MotionMagicVoltage motionMagicVolts = new MotionMagicVoltage(0).withEnableFOC(true);
    private VoltageOut voltageReq = new VoltageOut(0).withEnableFOC(true);
    private double setpoint = 0;

    public double kG = 0.5;//placeholder values
    public double kC = -0.18;

    public ElevatorIOTalonFX() {
        leftConfig.MotionMagic.MotionMagicCruiseVelocity = elevatorConstants.CruiseVelocity;
        leftConfig.MotionMagic.MotionMagicAcceleration = elevatorConstants.Acceleration;
        leftConfig.MotionMagic.MotionMagicJerk = elevatorConstants.Jerk;

        leftConfig.Slot0.kP = 1;//placeholders
        leftConfig.Slot0.kI = 0;
        leftConfig.Slot0.kD = 0.01;
        leftConfig.Slot0.kS = 0.09;
        leftConfig.Slot0.kV = 0;
        leftConfig.Slot0.kA = 0;

        leftConfig.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfig.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leftConfig.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        leftConfig.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;
        rightConfig.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        rightConfig.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;

        leftMotor.getConfigurator().apply(leftConfig);
        rightMotor.getConfigurator().apply(rightConfig);
        rightMotor.setControl(new Follower(canIDConstants.leftElevatorMotor, false));
        
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
        setpoint = meters;
        leftMotor.setControl(motionMagicVolts.withPosition(meters/elevatorConstants.wheelCircumferenceMeters).withFeedForward(kG*Math.sin(radians)+kC));//placeholder value
    }

    public void requestVoltage(double volts){
        leftMotor.setControl(voltageReq.withOutput(volts));
    }
}
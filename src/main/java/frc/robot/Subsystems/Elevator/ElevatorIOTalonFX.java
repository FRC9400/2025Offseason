package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO{
    /* Motor Objects */
    TalonFX leftMotor = new TalonFX(canIDConstants.elevatorMotor1, "rio");
    TalonFX rightMotor = new TalonFX(canIDConstants.elevatorMotor2, "rio");
    TalonFXConfiguration config = new TalonFXConfiguration();

    /* Control Requests */
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private VoltageOut voltageReq = new VoltageOut(0).withEnableFOC(true);
    
    /* Doubles */
    private double setpointMeters = 0;
    private double setpointVolts = 0;

    /* Status Signals */
    private final StatusSignal<Current> leftElevatorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> rightElevatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftElevatorTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Temperature> rightElevatorTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> leftElevatorAngularVelocity = leftMotor.getRotorVelocity();
    private final StatusSignal<AngularVelocity> rightElevatorAngularVelocity = rightMotor.getRotorVelocity();
    private final StatusSignal<Voltage> leftVoltage = leftMotor.getMotorVoltage();
    private final StatusSignal<Voltage> rightVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Angle> leftElevatorPos = leftMotor.getRotorPosition(); 

    public ElevatorIOTalonFX() {
        config.MotionMagic.MotionMagicCruiseVelocity = elevatorConstants.CruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = elevatorConstants.Acceleration;
        config.MotionMagic.MotionMagicJerk = elevatorConstants.Jerk;

        config.Slot0.kP = 6.0235;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.15516;
        config.Slot0.kS = 0.034291;
        config.Slot0.kV = 0.12709;
        config.Slot0.kA = 0.0030174;
        config.Slot0.kG = 0.45551;

        config.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        config.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;

        leftMotor.setPosition(0);

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorAngularVelocity,
            rightElevatorAngularVelocity,
            leftVoltage,
            rightVoltage,
            leftElevatorPos);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    public void updateInputs(ElevatorIOInputs inputs){
        /* Refresh Status Signals */
        BaseStatusSignal.refreshAll(
           leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorAngularVelocity,
            rightElevatorAngularVelocity,
            leftVoltage,
            rightVoltage,
            leftElevatorPos 
        );

        inputs.voltage = new double[] {leftVoltage.getValueAsDouble(), rightVoltage.getValueAsDouble()};
        inputs.appliedVolts = voltageReq.Output;
        inputs.appliedMeters = motionMagicRequest.Position;
        inputs.setpointVolts = setpointVolts;
        inputs.setpointMeters = setpointMeters;
        
        inputs.elevatorHeightMeters = Conversions.RotationsToMeters(leftElevatorPos.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.velocityRPS = new double[] {leftElevatorAngularVelocity.getValueAsDouble(), rightElevatorAngularVelocity.getValueAsDouble()};
        inputs.velocityMPS =  new double[] {Conversions.RPStoMPS(leftElevatorAngularVelocity.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio), Conversions.RPStoMPS(rightElevatorAngularVelocity.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)};
        inputs.currentAmps = new double[] {leftElevatorCurrent.getValueAsDouble(), rightElevatorCurrent.getValueAsDouble()};
        inputs.tempFahrenheit = new double[] {leftElevatorTemp.getValueAsDouble(), rightElevatorTemp.getValueAsDouble()};
        inputs.elevatorHeightRotations = leftElevatorPos.getValueAsDouble();
    }

    public void requestMotionMagic(double meters){
        setpointMeters = meters;
        leftMotor.setControl(motionMagicRequest.withPosition(Conversions.metersToRotations(meters, elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)));
    }

    public void requestVoltage(double volts){
        setpointVolts = volts;
        leftMotor.setControl(voltageReq.withOutput(volts));
    }
}
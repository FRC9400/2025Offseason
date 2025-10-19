package frc.robot.Subsystems.Pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.pivotConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOTalonFX implements PivotIO {
    /* Motor Objects */
    private final TalonFX leftMotor1;
    private final TalonFX leftMotor2;
    private final TalonFX rightMotor1;
    private final TalonFX rightMotor2;
    private TalonFXConfiguration leftMotor1Configs;

    /* Status Signals */
    private final StatusSignal<Current> leftPivot1Current;
    private final StatusSignal<Current> leftPivot2Current;
    private final StatusSignal<Current> rightPivot1Current;
    private final StatusSignal<Current> rightPivot2Current;

    private final StatusSignal<Temperature> leftPivot1Temp;
    private final StatusSignal<Temperature> leftPivot2Temp;
    private final StatusSignal<Temperature> rightPivot1Temp;
    private final StatusSignal<Temperature> rightPivot2Temp;

    private final StatusSignal<AngularVelocity> leftPivot1AngularVelocity;
    private final StatusSignal<AngularVelocity> leftPivot2AngularVelocity;
    private final StatusSignal<AngularVelocity> rightPivot1AngularVelocity;
    private final StatusSignal<AngularVelocity> rightPivot2AngularVelocity;

    private final StatusSignal<Voltage> leftPivot1Voltage;
    private final StatusSignal<Voltage> leftPivot2Voltage;
    private final StatusSignal<Voltage> rightPivot1Voltage;
    private final StatusSignal<Voltage> rightPivot2Voltage;

    private final StatusSignal<Angle> leftPivot1Position;
    private final StatusSignal<Angle> leftPivot2Position;
    private final StatusSignal<Angle> rightPivot1Position;
    private final StatusSignal<Angle> rightPivot2Position;

    /* Control Requests */
    private MotionMagicVoltage motionMagicRequest;
    private VoltageOut voltageOutRequest;

    /* Doubles */
    private double setpointVolts;
    private double setpointDeg;

    public PivotIOTalonFX(){
        /* Motor Objects */
        leftMotor1 = new TalonFX(canIDConstants.leftPivotMotor1, "rio");
        leftMotor2 = new TalonFX(canIDConstants.leftPivotMotor2, "rio");
        rightMotor1 = new TalonFX(canIDConstants.rightPivotMotor1, "rio");
        rightMotor2 = new TalonFX(canIDConstants.rightPivotMotor2, "rio");
        leftMotor1Configs = new TalonFXConfiguration();

        /* Status Signals */
        leftPivot1Current = leftMotor1.getStatorCurrent();
        leftPivot2Current = leftMotor2.getStatorCurrent();
        rightPivot1Current = rightMotor1.getStatorCurrent();
        rightPivot2Current = rightMotor2.getStatorCurrent();

        leftPivot1Temp = leftMotor1.getDeviceTemp();
        leftPivot2Temp = leftMotor2.getDeviceTemp();
        rightPivot1Temp = rightMotor1.getDeviceTemp();
        rightPivot2Temp = rightMotor2.getDeviceTemp();

        leftPivot1AngularVelocity = leftMotor1.getRotorVelocity();
        leftPivot2AngularVelocity = leftMotor2.getRotorVelocity();
        rightPivot1AngularVelocity = rightMotor1.getRotorVelocity();
        rightPivot2AngularVelocity = rightMotor2.getRotorVelocity();

        leftPivot1Voltage = leftMotor1.getMotorVoltage();
        leftPivot2Voltage = leftMotor2.getMotorVoltage();
        rightPivot1Voltage = rightMotor1.getMotorVoltage();
        rightPivot2Voltage = rightMotor2.getMotorVoltage();

        leftPivot1Position = leftMotor1.getRotorPosition();
        leftPivot2Position = leftMotor2.getRotorPosition();
        rightPivot1Position = rightMotor1.getRotorPosition();
        rightPivot2Position = rightMotor2.getRotorPosition();

        /* Control Requests */
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        setpointVolts = 0.0;
        setpointDeg = 0.0;

        /* Current Limit Configuration */
        leftMotor1Configs.CurrentLimits.StatorCurrentLimit = pivotConstants.pivotCurrentLimit;
        leftMotor1Configs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        leftMotor1Configs.MotorOutput.NeutralMode = pivotConstants.pivotNeutralMode;
        leftMotor1Configs.MotorOutput.Inverted = pivotConstants.leftPivotMotor1Invert;

        /* Slot 0 Configuration */
        leftMotor1Configs.Slot0.kP = 18.283;
        leftMotor1Configs.Slot0.kI = 0;
        leftMotor1Configs.Slot0.kD = 0.3557;
        leftMotor1Configs.Slot0.kS = 0.1716;
        leftMotor1Configs.Slot0.kV = 0.12787;
        leftMotor1Configs.Slot0.kA = 0.005205;
        leftMotor1Configs.Slot0.kG = 0.68031;
        leftMotor1Configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        /* Motion Magic Configuration */
        leftMotor1Configs.MotionMagic.MotionMagicCruiseVelocity = 75;
        leftMotor1Configs.MotionMagic.MotionMagicAcceleration = 120;
        leftMotor1Configs.MotionMagic.MotionMagicJerk = 10000;

        /* Feedback Configuration */
        leftMotor1Configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        /* Zero Motor */
        leftMotor1.setPosition(0);

        /* Configure Configs */
        leftMotor1.getConfigurator().apply(leftMotor1Configs);
        leftMotor2.getConfigurator().apply(leftMotor1Configs);
        leftMotor2.setControl(new Follower(leftMotor1.getDeviceID(), false));
        rightMotor1.setControl(new Follower(leftMotor1.getDeviceID(), true));
        rightMotor2.setControl(new Follower(leftMotor1.getDeviceID(), true));

        /* Set Frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftPivot1Current,
            leftPivot2Current,
            rightPivot1Current,
            rightPivot2Current,
            leftPivot1Temp,
            leftPivot2Temp,
            rightPivot1Temp,
            rightPivot2Temp,
            leftPivot1AngularVelocity,
            leftPivot2AngularVelocity,
            rightPivot1AngularVelocity,
            rightPivot2AngularVelocity,
            leftPivot1Voltage,
            leftPivot2Voltage,
            rightPivot1Voltage,
            rightPivot2Voltage,
            leftPivot1Position,
            leftPivot2Position,
            rightPivot1Position,
            rightPivot2Position
        );

        /* Optimize Bus Utilization */
        leftMotor1.optimizeBusUtilization();
        leftMotor2.optimizeBusUtilization();
        rightMotor1.optimizeBusUtilization();
        rightMotor2.optimizeBusUtilization();
    }

    public void updateInputs(PivotIOInputs inputs){
        /* Refresh Status Signals */
        BaseStatusSignal.refreshAll(
            leftPivot1Current,
            leftPivot2Current,
            rightPivot1Current,
            rightPivot2Current,
            leftPivot1Temp,
            leftPivot2Temp,
            rightPivot1Temp,
            rightPivot2Temp,
            leftPivot1AngularVelocity,
            leftPivot2AngularVelocity,
            rightPivot1AngularVelocity,
            rightPivot2AngularVelocity,
            leftPivot1Voltage,
            leftPivot2Voltage,
            rightPivot1Voltage,
            rightPivot2Voltage,
            leftPivot1Position,
            leftPivot2Position,
            rightPivot1Position,
            rightPivot2Position
        );

        /* Refresh Inputs */        
        inputs.appliedPosition = motionMagicRequest.Position;
        inputs.appliedVolts = voltageOutRequest.Output;

        inputs.setpointVolts = setpointVolts;
        inputs.setpointRot = Conversions.DegreesToRotations(setpointVolts, pivotConstants.gearRatio);
        inputs.setpointDeg = setpointDeg;
        inputs.positionRot = new double [] {leftPivot1Position.getValueAsDouble(),leftPivot2Position.getValueAsDouble(), rightPivot1Position.getValueAsDouble(), rightPivot2Position.getValueAsDouble()};
        inputs.positionDeg = new double [] {Conversions.RotationsToDegrees(leftPivot1Position.getValueAsDouble(), pivotConstants.gearRatio), Conversions.RotationsToDegrees(leftPivot2Position.getValueAsDouble(), pivotConstants.gearRatio), Conversions.RotationsToDegrees(rightPivot1Position.getValueAsDouble(), pivotConstants.gearRatio), Conversions.RotationsToDegrees(rightPivot2Position.getValueAsDouble(), pivotConstants.gearRatio)};
        
        inputs.voltage = new double[] {leftPivot1Voltage.getValueAsDouble(), leftPivot1Voltage.getValueAsDouble(), rightPivot1Voltage.getValueAsDouble(), rightPivot2Voltage.getValueAsDouble()};
        inputs.currentAmps = new double[] {leftPivot1Current.getValueAsDouble(), leftPivot2Current.getValueAsDouble(), rightPivot1Current.getValueAsDouble(), rightPivot2Current.getValueAsDouble()};
        inputs.tempFahrenheit = new double[] {leftPivot1Temp.getValueAsDouble(), leftPivot2Temp.getValueAsDouble(), rightPivot1Temp.getValueAsDouble(), rightPivot2Temp.getValueAsDouble()};
        inputs.velocityRPS = new double[] {leftPivot1AngularVelocity.getValueAsDouble(), leftPivot2AngularVelocity.getValueAsDouble(), rightPivot1AngularVelocity.getValueAsDouble(), rightPivot2AngularVelocity.getValueAsDouble()};
    }

    public void requestVoltage(double volts){
        setpointVolts = volts;
        leftMotor1.setControl(voltageOutRequest.withOutput(volts));
    }

    public void requestMotionMagic(double deg){
        setpointDeg = deg;
        leftMotor1.setControl(motionMagicRequest.withPosition(Conversions.DegreesToRotations(deg, pivotConstants.gearRatio)));
    }

    public void setPosition(double degrees){
        leftMotor1.setPosition(degrees);
    }

}

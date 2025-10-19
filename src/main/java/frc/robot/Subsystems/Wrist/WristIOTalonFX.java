package frc.robot.Subsystems.Wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.wristConstants;
import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;

public class WristIOTalonFX implements WristIO {
    // Motors + Configs
    private final TalonFX pivot = new TalonFX(canIDConstants.pivotMotor, "rio");
    private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    // Control Requests
    private VoltageOut pivotVoltageRequest;
    private MotionMagicVoltage pivotMotionMagicRequest;

    // Setpoint Doubles
    private double pivotSetpointVolts;
    private double pivotSetpointDeg;
    private double pivotSetpointRot;

    // Status Signals
    private final StatusSignal<Current> pivotCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivot.getDeviceTemp();
    private final StatusSignal<AngularVelocity> pivotRPS = pivot.getRotorVelocity();
    private final StatusSignal<Angle> pivotPos = pivot.getRotorPosition();
    private final StatusSignal<Voltage> pivotVolts = pivot.getMotorVoltage();

    public WristIOTalonFX(){
        // Control Requests
        pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        // Current Limits
        pivotConfigs.CurrentLimits.StatorCurrentLimit = wristConstants.pivotCurrentLimit;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        // Motor Output Invert
        pivotConfigs.MotorOutput.Inverted = wristConstants.pivotMotorInvert;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        pivot.setPosition(0);

        // Pivot PID Vals
        pivotConfigs.Slot0.kP = 10;
        pivotConfigs.Slot0.kI = 0;
        pivotConfigs.Slot0.kD = 0.023;
        pivotConfigs.Slot0.kS = 0.20502;
        pivotConfigs.Slot0.kV = 0.027833;
        pivotConfigs.Slot0.kA = 0.02;
        pivotConfigs.Slot0.kG = 1.6;
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        // Motion Magic Configs for Pivot
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 75;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = 120;
        pivotConfigs.MotionMagic.MotionMagicJerk = 10000;

        pivot.getConfigurator().apply(pivotConfigs);

        // Frequency Update
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotCurrent,
            pivotTemp,
            pivotRPS,
            pivotPos,
            pivotVolts
        );

        // Bus Utilization
        pivot.optimizeBusUtilization();
    }

    // Input Update + Refresh
    public void updateInputs(WristInputs inputs){
        BaseStatusSignal.refreshAll(
            pivotCurrent,
            pivotTemp,
            pivotRPS,
            pivotPos,
            pivotVolts
        );

        inputs.pivotAppliedVolts = pivotVoltageRequest.Output;
        inputs.pivotAppliedDeg = Conversions.RotationsToDegrees(pivotMotionMagicRequest.Position, wristConstants.pivotGearRatio);
        inputs.pivotSetpointVolts = pivotSetpointVolts;
        inputs.pivotSetpointDeg = pivotSetpointDeg;
        inputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpointDeg, wristConstants.pivotGearRatio);
        inputs.pivotPosRot = pivotPos.getValueAsDouble();
        inputs.pivotPosDeg = Conversions.RotationsToDegrees(pivotPos.getValueAsDouble(), wristConstants.pivotGearRatio); 
    }

    // Set Voltage + Position  
    public void requestPivotVoltage(double voltage){
        this.pivotSetpointVolts = voltage;
        pivot.setControl(pivotVoltageRequest.withOutput(pivotSetpointVolts));
    }

    public void requestPivotMotionMagic(double degrees){
        this.pivotSetpointDeg = degrees;
        pivotSetpointRot = Conversions.DegreesToRotations(degrees, wristConstants.pivotGearRatio);
        pivot.setControl(pivotMotionMagicRequest.withPosition(pivotSetpointRot));
    }

    public void zeroPosition(){
        pivot.setPosition(0);
    }
}
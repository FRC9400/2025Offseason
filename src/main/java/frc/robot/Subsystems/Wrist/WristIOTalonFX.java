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
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.wristConstants;
import frc.commons.Conversions;

public class WristIOTalonFX implements WristIO{
    // Motors + Configs, UPDATE CANBUS*****
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

        // Apply Configs
        pivot.getConfigurator().apply(pivotConfigs);

        // Pivot PID Vals
        pivotConfigs.Slot0.kP = 8;
        pivotConfigs.Slot0.kI = 0;
        pivotConfigs.Slot0.kD = 0;
        pivotConfigs.Slot0.kS = 0;
        pivotConfigs.Slot0.kV = 0;
        pivotConfigs.Slot0.kA = 0;
        pivotConfigs.Slot0.kG = 0;
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        // Motion Magic Configs for Pivot
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 0;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = 0;
        pivotConfigs.MotionMagic.MotionMagicJerk = 0;

        // Frequency Update
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotCurrent,
            pivotTemp,
            pivotRPS,
            pivotPos
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
            pivotPos
        );

        inputs.pivotAppliedVolts = pivotVoltageRequest.Output;
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
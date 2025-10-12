package frc.robot.Subsystems.EndEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.endEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO{
    // Motors + Configs, UPDATE CANBUS*****
    private final TalonFX algaeMotor = new TalonFX(canIDConstants.algaeMotor, "rio");
    private final TalonFX coral = new TalonFX(canIDConstants.coralMotor, "rio");
    private final TalonFX pivot = new TalonFX(canIDConstants.pivotMotor, "rio");
    
    private final TalonFXConfiguration algaeConfigs = new TalonFXConfiguration();
    private final TalonFXConfiguration coralConfigs = new TalonFXConfiguration();
    private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    // Control Requests
    private VoltageOut algaeVoltageRequest;
    private VoltageOut coralVoltageRequest;
    private VoltageOut pivotVoltageRequest;
    private MotionMagicVoltage pivotMotionMagicRequest;

    // Setpoint Doubles
    private double algaeSetpointVolts;
    private double coralSetpointVolts;
    private double pivotSetpointVolts;
    private double pivotSetpointDeg;
    private double pivotSetpointRot;

    // Status Signals
    private final StatusSignal<Current> algaeCurrent = algaeMotor.getStatorCurrent();
    private final StatusSignal<Temperature> algaeTemp = algaeMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> algaeRPS = algaeMotor.getRotorVelocity();

    private final StatusSignal<Current> coralCurrent = coral.getStatorCurrent();
    private final StatusSignal<Temperature> coralTemp = coral.getDeviceTemp();
    private final StatusSignal<AngularVelocity> coralRPS = coral.getRotorVelocity();

    private final StatusSignal<Current> pivotCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivot.getDeviceTemp();
    private final StatusSignal<AngularVelocity> pivotRPS = pivot.getRotorVelocity();
    private final StatusSignal<Angle> pivotPos = pivot.getRotorPosition();

    public EndEffectorIOTalonFX(){
        // Control Requests
        algaeVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        coralVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        // Current Limits
        algaeConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.algaeCurrentLimit;
        algaeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        coralConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.coralCurrentLimit;
        coralConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.pivotCurrentLimit;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        // Motor Output Invert
        algaeConfigs.MotorOutput.Inverted = endEffectorConstants.algaeMotorInvert;
        coralConfigs.MotorOutput.Inverted = endEffectorConstants.coralMotorInvert;
        pivotConfigs.MotorOutput.Inverted = endEffectorConstants.pivotMotorInvert;
        
        // Apply Configs
        algaeMotor.getConfigurator().apply(algaeConfigs);
        coral.getConfigurator().apply(coralConfigs);
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
            algaeCurrent,
            algaeTemp,
            algaeRPS,
            coralCurrent,
            coralTemp,
            coralRPS,
            pivotCurrent,
            pivotTemp,
            pivotRPS,
            pivotPos
        );

        // Bus Utilization
        algaeMotor.optimizeBusUtilization();
        coral.optimizeBusUtilization();
    }

    // Input Update + Refresh
    public void updateInputs(EndEffectorInputs inputs){
        BaseStatusSignal.refreshAll(
            algaeCurrent,
            algaeTemp,
            algaeRPS,
            coralCurrent,
            coralTemp,
            coralRPS,
            pivotCurrent,
            pivotTemp,
            pivotRPS,
            pivotPos
        );

        inputs.algaeAppliedVolts = algaeVoltageRequest.Output;
        inputs.algaeSetpointVolts = algaeSetpointVolts;
        inputs.algaeCurrent = algaeCurrent.getValueAsDouble();
        inputs.algaeRPS = algaeRPS.getValueAsDouble();
        inputs.algaeTemp = algaeTemp.getValueAsDouble();

        inputs.coralAppliedVolts = coralVoltageRequest.Output;
        inputs.coralSetpointVolts = coralSetpointVolts;
        inputs.coralCurrent = coralCurrent.getValueAsDouble();
        inputs.coralRPS = coralRPS.getValueAsDouble();
        inputs.coralTemp = coralTemp.getValueAsDouble();

        inputs.pivotAppliedVolts = pivotVoltageRequest.Output;
        inputs.pivotSetpointVolts = pivotSetpointVolts;
        inputs.pivotSetpointDeg = pivotSetpointDeg;
        inputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpointDeg, endEffectorConstants.pivotGearRatio);
        inputs.pivotPosRot = pivotPos.getValueAsDouble();
        inputs.pivotPosDeg = Conversions.RotationsToDegrees(pivotPos.getValueAsDouble(), endEffectorConstants.pivotGearRatio); 
        inputs.pivotCurrent = algaeCurrent.getValueAsDouble();
        inputs.pivotRPS = algaeRPS.getValueAsDouble();
        inputs.pivotTemp = algaeTemp.getValueAsDouble();
    }

    // Set Voltage + Position  
    public void requestAlgaeVoltage(double voltage){
        this.algaeSetpointVolts = voltage;
        algaeMotor.setControl(algaeVoltageRequest.withOutput(algaeSetpointVolts));
    }

    public void requestCoralVoltage(double voltage){
        this.coralSetpointVolts = voltage;
        coral.setControl(coralVoltageRequest.withOutput(coralSetpointVolts));
    }

    public void requestPivotVoltage(double voltage){
        this.pivotSetpointVolts = voltage;
        pivot.setControl(pivotVoltageRequest.withOutput(pivotSetpointVolts));
    }

    public void requestPivotMotionMagic(double degrees){
        this.pivotSetpointDeg = degrees;
        pivotSetpointRot = Conversions.DegreesToRotations(degrees, endEffectorConstants.pivotGearRatio);
        pivot.setControl(pivotMotionMagicRequest.withPosition(pivotSetpointRot));
    }

    public void zeroPosition(){
        pivot.setPosition(0);
    }
}
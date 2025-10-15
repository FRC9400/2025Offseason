package frc.robot.Subsystems.EndEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.endEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO{
    // Motors + Configs, UPDATE CANBUS*****
    private final TalonFX algaeMotor = new TalonFX(canIDConstants.algaeMotor, "rio");
    private final TalonFX coral = new TalonFX(canIDConstants.coralMotor, "rio");
    
    private final TalonFXConfiguration algaeConfigs = new TalonFXConfiguration();
    private final TalonFXConfiguration coralConfigs = new TalonFXConfiguration();

    // Control Requests
    private VoltageOut algaeVoltageRequest;
    private VoltageOut coralVoltageRequest;

    // Setpoint Doubles
    private double algaeSetpointVolts;
    private double coralSetpointVolts;

    // Status Signals
    private final StatusSignal<Current> algaeCurrent = algaeMotor.getStatorCurrent();
    private final StatusSignal<Temperature> algaeTemp = algaeMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> algaeRPS = algaeMotor.getRotorVelocity();

    private final StatusSignal<Current> coralCurrent = coral.getStatorCurrent();
    private final StatusSignal<Temperature> coralTemp = coral.getDeviceTemp();
    private final StatusSignal<AngularVelocity> coralRPS = coral.getRotorVelocity();

    public EndEffectorIOTalonFX(){
        // Control Requests
        algaeVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        coralVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        
        // Current Limits
        algaeConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.algaeCurrentLimit;
        algaeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        coralConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.coralCurrentLimit;
        coralConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Motor Output Invert
        algaeConfigs.MotorOutput.Inverted = endEffectorConstants.algaeMotorInvert;
        coralConfigs.MotorOutput.Inverted = endEffectorConstants.coralMotorInvert;
        
        // Apply Configs
        algaeMotor.getConfigurator().apply(algaeConfigs);
        coral.getConfigurator().apply(coralConfigs);
        
        // Frequency Update
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            algaeCurrent,
            algaeTemp,
            algaeRPS,
            coralCurrent,
            coralTemp,
            coralRPS
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
            coralRPS
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
}
package frc.robot.Subsystems.Pivot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.canIDConstants;

public class PivotIOTalonFX implements PivotIO {
    /* Motor Objects */
    private final TalonFX leftMotor1;
    private final TalonFX leftMotor2;
    private final TalonFX rightMotor1;
    private final TalonFX rightMotor2;
    private TalonFXConfiguration leftMotor1Configs;

    public PivotIOTalonFX(){
        /* Motor Objects */
        leftMotor1 = new TalonFX(canIDConstants.leftPivotMotor1, "rio");
        leftMotor2 = new TalonFX(canIDConstants.leftPivotMotor2, "rio");
        rightMotor1 = new TalonFX(canIDConstants.rightPivotMotor1, "rio");
        rightMotor2 = new TalonFX(canIDConstants.rightPivotMotor2, "rio");
        leftMotor1Configs = new TalonFXConfiguration();
    }

}

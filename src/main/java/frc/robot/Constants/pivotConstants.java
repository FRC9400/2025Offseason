package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class pivotConstants {
    /* Invert */
    public static final InvertedValue leftPivotMotor1Invert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue leftPivotMotor2Invert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightPivotMotor1Invert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightPivotMotor2Invert = InvertedValue.Clockwise_Positive;
    
    /* Current Limits */
    public static final double pivotCurrentLimit = 80;

    /* Gear Ratio */
    public static double gearRatio = 138.89;
}

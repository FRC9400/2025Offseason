package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class endEffectorConstants {
    /* Inverts */
    public static final InvertedValue algaeMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue coralMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue pivotMotorInvert = InvertedValue.Clockwise_Positive;

    /* Current Limits */
    public static final double algaeCurrentLimit = 50;
    public static final double coralCurrentLimit = 50;
    public static final double pivotCurrentLimit = 50;

    /* Gear Ratios */
    public static final double gearRatio = 23.625;
}
package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class endEffectorConstants {
    /* Inverts */
    public static final InvertedValue algaeMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue coralMotorInvert = InvertedValue.Clockwise_Positive;

    /* Current Limits */
    public static final double algaeCurrentLimit = 50;
    public static final double coralCurrentLimit = 50;


    /* Gear Ratios */
    public static final double algaeGearRatio = 6.111;
    public static final double coralGearRatio = 7.666;
    

}
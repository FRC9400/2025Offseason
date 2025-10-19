package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class wristConstants{
    // invert
    public static final InvertedValue pivotMotorInvert = InvertedValue.CounterClockwise_Positive;

    // current limit
    public static final double pivotCurrentLimit = 50;

    // gear ratio
    public static final double pivotGearRatio = 28.78571428;
}
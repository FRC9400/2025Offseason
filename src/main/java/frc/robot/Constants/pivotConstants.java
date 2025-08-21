package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class pivotConstants {
    /* Invert */
    public static final InvertedValue leftPivotMotor1Invert = InvertedValue.Clockwise_Positive;
    
    /* Current Limits */
    public static final double pivotCurrentLimit = 80;

    /* Neutral Modes */
    public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

    /* Gear Ratio */
    public static double gearRatio = 138.89;
}

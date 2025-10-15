package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.commons.ArmPosition;

public class armPositionConstants {
    public static final ArmPosition zero = new ArmPosition(0.0, Rotation2d.kZero, Rotation2d.kZero);
    public static final ArmPosition kickstandPosition = new ArmPosition(0, Rotation2d.fromDegrees(42), Rotation2d.fromDegrees(0.0));
    public static final ArmPosition L2 = new ArmPosition(0.0, Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0));


}

package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.commons.ArmPosition;
import frc.commons.LoggedTunableNumber;

public class armPositionConstants {

    public static final ArmPosition zero = new ArmPosition(0.0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));
    public static final ArmPosition kickstandPosition = new ArmPosition(0, Rotation2d.fromDegrees(42), Rotation2d.fromDegrees(0.0));
    public static final ArmPosition L1 = new ArmPosition(0.0, Rotation2d.fromDegrees(23.627), Rotation2d.fromDegrees(0));
    public static final ArmPosition L2 = new ArmPosition(0.0, Rotation2d.fromDegrees(23.627), Rotation2d.fromDegrees(0));
    public static final ArmPosition L3 = new ArmPosition(Units.inchesToMeters(4.38), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0));
    public static final ArmPosition L4 = new ArmPosition(Units.inchesToMeters(25), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0));
    public static final ArmPosition groundIntake = new ArmPosition(0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(125.643));
    public static final ArmPosition dealgaeLow = new ArmPosition(0, Rotation2d.fromDegrees(107), Rotation2d.fromDegrees(24));
    public static final ArmPosition dealgaeHigh = new ArmPosition(Units.inchesToMeters(6), Rotation2d.fromDegrees(107), Rotation2d.fromDegrees(37));
    public static final ArmPosition stationIntake = new ArmPosition(0, Rotation2d.fromDegrees(66.5), Rotation2d.fromDegrees(156));
    public static final ArmPosition barge = new ArmPosition(Units.inchesToMeters(39.75), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(101));
    public static final ArmPosition processor = new ArmPosition(0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(95));
}

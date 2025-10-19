package frc.commons;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmPosition {
    private double extensionLengthMeters = 0;
    private Rotation2d pivotAngleRot2d = new Rotation2d();
    private Rotation2d wristAngleRot2d = new Rotation2d();

    public ArmPosition(double extensionLengthMeters, Rotation2d pivotAngleRot2d, Rotation2d wristAngleRot2d) {
        this.extensionLengthMeters = extensionLengthMeters;
        this.pivotAngleRot2d = pivotAngleRot2d;
        this.wristAngleRot2d = wristAngleRot2d;
    }

    public double getPivotDegrees(){
        return pivotAngleRot2d.getDegrees();
    }

    public double getWristDegrees(){
        return wristAngleRot2d.getDegrees();
    }

    public double getElevatorHeight() {
        return extensionLengthMeters;
    }
}
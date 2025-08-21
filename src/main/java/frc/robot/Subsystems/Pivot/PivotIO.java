package frc.robot.Subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double pivotAppliedVolts = 0.0;

        public double pivotSetpointDeg = 0.0;
        public double pivotSetpointRot = 0.0;
        public double[] pivotPosDeg = new double[] {};
        public double[] pivotPosRot = new double[] {};

        public double[] pivotCurrent = new double[] {};
        public double[] pivotTemp = new double[] {};
        public double[] pivotRPS = new double[] {};
    }
}

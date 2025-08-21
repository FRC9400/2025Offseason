package frc.robot.Subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double appliedVolts = 0.0;
        public double appliedPosition = 0.0;

        public double setpointVolts = 0.0;
        public double setpointDeg = 0.0;
        public double setpointRot = 0.0;
        public double[] positionDeg = new double[] {};
        public double[] positionRot = new double[] {};

        public double[] voltage = new double [] {};
        public double[] currentAmps = new double[] {};
        public double[] tempFahrenheit = new double[] {};
        public double[] velocityRPS = new double[] {};
    }

    public default void updateInputs(PivotIOInputs inputs){}
    
    public default void requestVoltage(double volts){}

    public default void requestMotionMagic(double deg){}
}

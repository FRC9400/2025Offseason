package frc.robot.Subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

// end effector pivot, motion magic voltage
public interface WristIO {
    @AutoLog
    public static class WristInputs{
        public double pivotSetpointVolts = 0;
        public double pivotSetpointDeg = 0;
        public double pivotSetpointRot = 0;
        public double pivotAppliedVolts = 0;
        public double pivotAppliedDeg = 0;
        public double pivotCurrent = 0;
        public double pivotRPS = 0;
        public double pivotTemp = 0;
        public double pivotPosDeg = 0;
        public double pivotPosRot = 0;
    }

    public default void updateInputs(WristInputs inputs){}

    public default void zeroPosition(){}
    
    public default void requestPivotVoltage(double voltage){}

    public default void requestPivotMotionMagic(double degrees){}
}
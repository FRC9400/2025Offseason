package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

// 3 motors; 1) algae, 2) coral, 3) small pivot
// 1, 2 -> voltage out, 3 -> motion magic voltage
public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorInputs{
        public double algaeSetpointVolts = 0;
        public double algaeAppliedVolts = 0;
        public double algaeCurrent = 0;
        public double algaeRPS = 0;        
        public double algaeTemp = 0;

        public double coralSetpointVolts = 0;
        public double coralAppliedVolts = 0;
        public double coralCurrent = 0;
        public double coralRPS = 0;
        public double coralTemp = 0;

        public double pivotSetpointVolts = 0;
        public double pivotSetpointDeg = 0;
        public double pivotSetpointRot = 0;
        public double pivotAppliedVolts = 0;
        public double pivotCurrent = 0;
        public double pivotRPS = 0;
        public double pivotTemp = 0;
        public double pivotPosDeg = 0;
        public double pivotPosRot = 0;
    }

    public default void updateInputs(EndEffectorInputs inputs){}

    public default void zeroPosition(){}

    public default void setAlgaeVoltage(double voltage){}
    public default void setCoralVoltage(double voltage){}
    public default void setPivotVoltage(double voltage){}
    public default void setPivotMotionMagic(double degrees){}
}
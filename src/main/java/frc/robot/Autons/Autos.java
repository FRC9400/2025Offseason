/*package frc.robot.Autons;

import choreo.auto.AutoFactory;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {
    private final Swerve swerve;
    private final Superstructure superstructure;
    private final AutoFactory autoFactory;

    public Autos(Swerve swerve, Superstructure superstructure){
        this.swerve=swerve;
        this.superstructure=superstructure;
        autoFactory = new AutoFactory(swerve::getPoseRaw, swerve::resetPose, swerve::followChoreoTraj, trur, swerve);
    }
    public AutoFactory getFactory(){
        return autoFactory;
    }
}
*/
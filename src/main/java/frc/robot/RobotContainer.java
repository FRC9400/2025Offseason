// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.EndEffector.EndEffectorIOTalonFX;
import frc.robot.Subsystems.Pivot.PivotIO;
import frc.robot.Subsystems.Pivot.PivotIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.WristIO;
import frc.robot.Subsystems.Wrist.WristIOTalonFX;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1); 
    private final PivotIO s_pivot = new PivotIOTalonFX();
    private final EndEffectorIO s_endeffector = new EndEffectorIOTalonFX();
    private final ElevatorIO s_elevator = new ElevatorIOTalonFX();
    private final WristIO s_wrist = new WristIOTalonFX(); 
    private final Superstructure superstructure = new Superstructure(s_elevator,s_endeffector,s_pivot,s_wrist);
    private final Swerve swerve = new Swerve();
  
    public RobotContainer() {
        swerve.zeroGyro();
        swerve.zeroWheels();
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> -driver.getRawAxis(XboxController.Axis.kRightX.value)
            
            )
        );

        configureBindings();
    }

    private void configureBindings() {

        driver.a().onTrue(new InstantCommand(() -> swerve.zeroWheels()));

        driver.b().onTrue(new InstantCommand(() -> superstructure.requestKickstand()));

        driver.x().onTrue(new InstantCommand(() -> superstructure.requestIdle()));

        // driver.y().onTrue(new InstantCommand(() -> superstructure.requestGroundIntake()));

        // driver.leftBumper().onTrue(new InstantCommand(() -> superstructure.requestStationIntake()));

        driver.rightBumper().onTrue(new InstantCommand(() -> superstructure.requestTestCoralIntake()));

        driver.leftTrigger().onTrue(new InstantCommand(() -> superstructure.requestTestCoralScore()));

        driver.rightTrigger().onTrue(new InstantCommand(() -> superstructure.requestTestDealgae()));
    }

    public Swerve getSwerve(){
        return swerve;
    }

    public Superstructure getSuperstructure(){
        return superstructure;
    }

}
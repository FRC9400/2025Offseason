// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIOTalonFX;
import frc.robot.Subsystems.Pivot.Pivot;
import frc.robot.Subsystems.Pivot.PivotIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristIOTalonFX;
import frc.commons.LoggedTunableNumber;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Constants.elevatorConstants;

public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);    
    private final Pivot pivot = new Pivot(new PivotIOTalonFX());
    private final Elevator elevator = new Elevator(new ElevatorIOTalonFX());
    private final Swerve swerve = new Swerve();
    private final Wrist wrist = new Wrist(new WristIOTalonFX());
    private final EndEffector endEffector = new EndEffector(new EndEffectorIOTalonFX());
  
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
        driver.a().whileTrue(new RunCommand(() -> swerve.zeroWheels()));
        driver.leftBumper().whileTrue(new RunCommand(() -> pivot.requestVoltage(0)));
        driver.rightBumper().whileTrue(new RunCommand(() -> endEffector.requestCoralVoltage(1)));
        driver.rightTrigger().whileTrue(new RunCommand(() -> endEffector.requestCoralVoltage(0)));

        operator.a().whileTrue(new RunCommand(() -> wrist.requestPivotMotionMagic(20)));
        operator.b().whileTrue(new RunCommand(() -> wrist.requestPivotMotionMagic(0)));
        operator.x().whileTrue(new RunCommand(() -> wrist.requestPivotMotionMagic(10)));
        operator.y().whileTrue(new RunCommand(() -> wrist.requestPivotMotionMagic(50)));
        operator.leftBumper().whileTrue(wrist.wristSysIdCmd());
        operator.leftTrigger().whileTrue(new RunCommand(() -> pivot.requestVoltage(1)));
        operator.rightBumper().whileTrue(new RunCommand(() -> pivot.requestVoltage(0)));
        
    }

    public Swerve getSwerve(){
        return swerve;
    }

}
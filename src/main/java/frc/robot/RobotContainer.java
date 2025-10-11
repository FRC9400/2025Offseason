// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Pivot.Pivot;
import frc.robot.Subsystems.Pivot.PivotIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.commons.LoggedTunableNumber;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(0);    
    private final Pivot pivot = new Pivot(new PivotIOTalonFX());
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
        driver.a().whileTrue(new RunCommand(() -> pivot.requestVoltage(-2)));
        driver.b().whileTrue(new RunCommand(() -> pivot.requestVoltage(0)));
        driver.y().whileTrue(new RunCommand(() ->
        pivot.requestVoltage(2)));

    }

    public Swerve getSwerve(){
        return swerve;
    }
    
    

}

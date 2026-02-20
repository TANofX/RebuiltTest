// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.command.RPMTest;
import frc.robot.command.RotationTest;
import frc.robot.subsystem.ShooterMotorTest;
import frc.robot.subsystem.TurretMotorTest;

public class RobotContainer {
  private final XboxControllerWrapper driverController = new XboxControllerWrapper(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverController.LB().onTrue(new RotationTest(new TurretMotorTest("Test Turret", 37, 0), driverController));
    driverController.A().onTrue(new RPMTest(new ShooterMotorTest("Top Left", 30)));
    driverController.B().onTrue(new RPMTest(new ShooterMotorTest("Bottom Left", 31)));
    driverController.X().onTrue(new RPMTest(new ShooterMotorTest("Top Right",32)));
    driverController.Y().onTrue(new RPMTest(new ShooterMotorTest("Bottom Right", 33)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

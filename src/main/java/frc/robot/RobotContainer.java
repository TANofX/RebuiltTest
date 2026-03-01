// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.command.JoystickDirectDrive;
import frc.robot.command.JoystickRPMDrive;
import frc.robot.command.LiveTuneShooterCommand;
import frc.robot.command.SetRPM;
import frc.robot.subsystem.TunableShooterSubsystem;

public class RobotContainer {
  private final XboxControllerWrapper driverController = new XboxControllerWrapper(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // driverController.LB().whileTrue(new RotationTest(new TurretMotorTest("Test Turret", 50, 0), driverController));
    // driverController.A().whileTrue(new RPMTest(new ShooterMotorTest("Top Left", 30)));
    // driverController.B().whileTrue(new RPMTest(new ShooterMotorTest("Bottom Left", 31)));
    // driverController.X().whileTrue(new RPMTest(new ShooterMotorTest("Top Right",32)));
    // driverController.Y().whileTrue(new RPMTest(new ShooterMotorTest("Bottom Right", 33)));

    // Example wiring for the live-tune subsystem (commented out by default).
    // To enable live tuning, uncomment the lines below and adjust motor IDs.
    TunableShooterSubsystem tunable = new TunableShooterSubsystem(30, 32, 0.0019764, 0.002, 0.0064, 0.00025, 0.00000012, 0.00004, "top", false);
    TunableShooterSubsystem bottomTunable = new TunableShooterSubsystem(31, 33, 0.0021294, 0.0022, 0.0038, 0.00015, 0.00000012, 0.00004, "bottom", false);
    TunableShooterSubsystem middleTunable = new TunableShooterSubsystem(34,  0.0021294, 0.0022, 0.0038, 0.00015, 0.00000012, 0.00004, "middle", true);
    TunableShooterSubsystem indexer = new TunableShooterSubsystem(40, 0.0021294, 0.0022, 0.0038, 0.00015, 0.00000012, 0.00004, "indexer", true);
    // LiveTuneShuffleboard.setup(tunable); // creates NT keys and simple displays
    // To start immediately (for testing) call: live.schedule();
    // Or bind to a controller button in this method when you're ready.
    driverController.A().toggleOnTrue(new JoystickRPMDrive(indexer, "indexer/", driverController));
    driverController.B().toggleOnTrue(new JoystickDirectDrive(bottomTunable, driverController));
    driverController.X().toggleOnTrue(Commands.parallel(new JoystickRPMDrive(bottomTunable, "bottom/", driverController), new JoystickRPMDrive(tunable, "top/", driverController), new JoystickRPMDrive(middleTunable, "middle/", driverController)));
    driverController.Y().toggleOnTrue(new SetRPM(tunable, bottomTunable));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

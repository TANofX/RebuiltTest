// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.subsystem.TunableMotorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JoystickRPMDrive extends Command {
  private XboxControllerWrapper controller;
  private TunableMotorSubsystem subsystem;
  private String subName;
  private double targetRPM = 900;
  private Timer waitTime = new Timer();

  /** Creates a new JoystickDirectDrive. */
  public JoystickRPMDrive(TunableMotorSubsystem shooter, XboxControllerWrapper wrapper) {
    controller = wrapper;
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = shooter;
    subName = shooter.getName();
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setDirectOutput(0);
    subsystem.setTargetRPM(targetRPM);
    subsystem.enable();
    SmartDashboard.putNumber (subName + "/TargetRPM", targetRPM);
    waitTime.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // subsystem.setTargetRPM(6000 * controller.getLeftY());
    if (controller.DDown().getAsBoolean() && waitTime.advanceIfElapsed(2)) {
      targetRPM -= 100;
      subsystem.setTargetRPM(targetRPM);
      SmartDashboard.putNumber(subName + "/TargetRPM", targetRPM);
    } else if (controller.DUp().getAsBoolean() && waitTime.advanceIfElapsed(2)) {
      targetRPM += 100;
      subsystem.setTargetRPM(targetRPM);
      SmartDashboard.putNumber(subName + "/TargetRPM", targetRPM);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.disable();
    subsystem.setDirectOutput(0.0);
    SmartDashboard.putNumber(subName + "/TargetRPM", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
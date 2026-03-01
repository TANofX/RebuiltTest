// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.TunableShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRPM extends Command {
  private TunableShooterSubsystem top;
  private TunableShooterSubsystem bottom;

  private double target;
  private NetworkTableEntry targetRPMEntry = NetworkTableInstance.getDefault().getTable("SetRPM").getEntry("Target");
  /** Creates a new SetRPM. */
  public SetRPM(TunableShooterSubsystem topSubsystem, TunableShooterSubsystem bottomSubsystem) {
    top = topSubsystem;
    bottom = bottomSubsystem;
    addRequirements(top, bottom);
  }

  public void setRPM(double rpm) {
      target = rpm;
      top.setTargetRPM(target);
      bottom.setTargetRPM(target);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = 0;
    //SmartDashboard.putNumber("TargetRPM", target);
    targetRPMEntry.setDouble(0.0);
    top.enable();
    bottom.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double newTarget = SmartDashboard.getNumber("TargetRPM", 1000);
    double newTarget = targetRPMEntry.getDouble(0.0);

    // SmartDashboard.updateValues();
    if (newTarget != target) {
      target = newTarget;
      setRPM(target);
    }
    else {
      setRPM(newTarget);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

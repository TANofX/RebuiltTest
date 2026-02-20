// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterMotorTest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RPMTest extends Command {
  private ShooterMotorTest shooterMotorTest;
  private Timer increaseTimer = new Timer();
  private double currentOutput = 0.0;
  private boolean isIncreasing = true;

  /** Creates a new RPMTest. */
  public RPMTest(ShooterMotorTest shooterMotorTest) {
    this.shooterMotorTest = shooterMotorTest;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterMotorTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    increaseTimer.restart();
    currentOutput = 0.0;
    isIncreasing = true;
    shooterMotorTest.setMotorOutput(currentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (increaseTimer.advanceIfElapsed(0.1)) {
      if (isIncreasing) {
        currentOutput += 0.01;
        if (currentOutput >= 1.0) {
          currentOutput = 1.0;
          isIncreasing = false;
        }
      } else {
        currentOutput -= 0.01;
        if (currentOutput <= -1.0) {
          currentOutput = -1.0;
          isIncreasing = true;
        }
      }
      shooterMotorTest.setMotorOutput(currentOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterMotorTest.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

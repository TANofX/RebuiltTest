// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.lib.input.controllers.rumble.RumbleAnimation;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.input.controllers.rumble.RumbleOn;
import frc.robot.subsystem.TurretMotorTest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotationTest extends Command {
  private final TurretMotorTest testMotor;
  private double currentOutput = 0.0;
  private boolean isIncreasing = true;
  private Timer increaseTimer = new Timer();
  private final XboxControllerWrapper controller;
  private static final RumbleAnimation rumbleOn = new RumbleOn();
  private static final RumbleAnimation rumbleOff = new RumbleOff();
  
  /** Creates a new RotationTest. */
  public RotationTest(TurretMotorTest motorToTest, XboxControllerWrapper controller) {
    testMotor = motorToTest;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(testMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      currentOutput = 0.0;
      isIncreasing = true;
      increaseTimer.restart();
      testMotor.setMotorOutput(currentOutput);
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
      testMotor.setMotorOutput(currentOutput);
    }

    if (testMotor.isLimitSwitchPressed()) {
      controller.setRumbleAnimation(rumbleOn);
    } else {
      controller.setRumbleAnimation(rumbleOff); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testMotor.stopMotor();
    controller.setRumbleAnimation(rumbleOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

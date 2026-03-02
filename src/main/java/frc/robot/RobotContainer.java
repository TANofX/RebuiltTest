// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.command.JoystickDirectDrive;
import frc.robot.command.JoystickRPMDrive;
import frc.robot.subsystem.TunableMotorSubsystem;

public class RobotContainer {
  private final XboxControllerWrapper driverController = new XboxControllerWrapper(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final SendableChooser<TunableMotorSubsystem> liveTuneChooser = new SendableChooser<TunableMotorSubsystem>();
  private final TreeMap<String, TunableMotorSubsystem> tunableSubsystems = new TreeMap<>();

  public RobotContainer() {
    tunableSubsystems.put("Top Tunable", new TunableMotorSubsystem(30, 32, 0.0019764, 0.002, 0.0064, 0.00025, 0.00000012, 0.00004, "top", false, true));
    tunableSubsystems.put("Bottom Tunable", new TunableMotorSubsystem(31, 33, 0.0021294, 0.0022, 0.0038, 0.00015, 0.00000012, 0.00004, "bottom", false, true));
    tunableSubsystems.put("Middle Tunable", new TunableMotorSubsystem(34,0.0021294, 0.0022, 0.0038, 0.00015, 0.00000012, 0.00004, "middle", true));
    tunableSubsystems.put("Indexer Tunable", new TunableMotorSubsystem(40, 0.0021294, 0.0022, 0.0038, 0.0015, 1e-7, 4e-5, "indexer", true));  
    tunableSubsystems.put("Intake Tunable", new TunableMotorSubsystem(51, 0.0021294, 0.0022, 0.0038, 0.0015, 1e-7, 4e-5, "intake", false));

    configureBindings();
  }

  private void configureBindings() {
    int index = 0;
    for (String key : tunableSubsystems.keySet()) {
      TunableMotorSubsystem subsystem = tunableSubsystems.get(key);
      if (index == 0) {
        liveTuneChooser.setDefaultOption(key, subsystem);
        autoChooser.setDefaultOption(key + " Quasistatic Forward", subsystem.sysIdQuasistatic(Direction.kForward));
        autoChooser.addOption(key + " Quasistatic Reverse", subsystem.sysIdQuasistatic(Direction.kReverse));
        autoChooser.addOption(key + " Dynamic Forward", subsystem.sysIdDynamic(Direction.kForward));
        autoChooser.addOption(key + " Dynamic Reverse", subsystem.sysIdDynamic(Direction.kReverse));
      } else {
        liveTuneChooser.addOption(key, subsystem);
        autoChooser.addOption(key + " Quasistatic Forward", subsystem.sysIdQuasistatic(Direction.kForward));
        autoChooser.addOption(key + " Quasistatic Reverse", subsystem.sysIdQuasistatic(Direction.kReverse));
        autoChooser.addOption(key + " Dynamic Forward", subsystem.sysIdDynamic(Direction.kForward));
        autoChooser.addOption(key + " Dynamic Reverse", subsystem.sysIdDynamic(Direction.kReverse));
      }

      index++;
    }

    liveTuneChooser.onChange(this::configureTunableButtonBindings);

    driverController.X().toggleOnTrue(Commands.parallel(
      new JoystickRPMDrive(tunableSubsystems.get("Top Tunable"), driverController), 
      new JoystickRPMDrive(tunableSubsystems.get("Bottom Tunable"), driverController), 
      new JoystickRPMDrive(tunableSubsystems.get("Middle Tunable"), driverController))
    );
    driverController.Y().toggleOnTrue(
      new JoystickDirectDrive(tunableSubsystems.get("Indexer Tunable"), () -> driverController.getLeftY())
    );
    driverController.RB().toggleOnTrue(
      new JoystickDirectDrive(tunableSubsystems.get("Intake Tunable"), () -> driverController.getLeftX())
    );

    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Live Tune Subsystem", liveTuneChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureTunableButtonBindings(TunableMotorSubsystem subsystem) {
    driverController.A().toggleOnTrue(new JoystickRPMDrive(subsystem, driverController));
    driverController.B().toggleOnTrue(new JoystickDirectDrive(subsystem, () -> driverController.getLeftY()));
  }
}

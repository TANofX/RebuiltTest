// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.command.JoystickDirectDrive;
import frc.robot.command.JoystickRPMDrive;
import frc.robot.command.RPMTest;
import frc.robot.subsystem.TunableMotorSubsystem;
import frc.robot.subsystem.TunableSparkMAXSubsystem;
import frc.robot.util.TuningButtonHolder;

public class RobotContainer {
  private final XboxControllerWrapper driverController = new XboxControllerWrapper(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final SendableChooser<TuningButtonHolder> liveTuneChooser = new SendableChooser<TuningButtonHolder>();
  private final TreeMap<String, TunableMotorSubsystem> tunableSubsystems = new TreeMap<>();
  private TuningButtonHolder activeHolder = null;

  

  public RobotContainer() {
    tunableSubsystems.put("Top Tunable", new TunableMotorSubsystem(30, 32, 0.002199, 0.0022147, 0.97509, 0.0002, 0.0000015, 0.0001, "top", false, true));
    tunableSubsystems.put("Bottom Tunable", new TunableMotorSubsystem(31, 33, 0.001, 0.00021225, 0.25114, 0.0002, 0.000001, 0.0003, "bottom", false, true));
    tunableSubsystems.put("Middle Tunable", new TunableMotorSubsystem(34,0.0018718, 0.00015177, 0.28045, 0.0001, 0.000001, 0.0005, "middle", true));
    tunableSubsystems.put("Indexer Tunable", new TunableMotorSubsystem(40, 0.0019755, 0.00056382, 0.089733, 0.00003, 0.000001, 0.00001, "indexer", true));  
    tunableSubsystems.put("Intake Tunable", new TunableMotorSubsystem(20, 0.0018096, 0.0022961, .3012, 0.00005, 0.0, 0.0, "intake", true));
    tunableSubsystems.put("Turret Tunable", new TunableSparkMAXSubsystem(50, 0.0010522, 0.00010721, 0.17925, 0.00024301, 0.0, 0.0, "turret", false));

    configureBindings();
  }

  private void configureBindings() {
    int index = 0;
    for (String key : tunableSubsystems.keySet()) {
      TunableMotorSubsystem subsystem = tunableSubsystems.get(key);
      if (index == 0) {
        activeHolder = new TuningButtonHolder( driverController, 
                                                                      new JoystickRPMDrive(subsystem, driverController), 
                                                                      new JoystickDirectDrive(subsystem, () -> driverController.getLeftY()));
        activeHolder.setActive(true);
        liveTuneChooser.setDefaultOption(key, activeHolder);
        autoChooser.setDefaultOption(key + " Quasistatic Forward", subsystem.sysIdQuasistatic(Direction.kForward));
        autoChooser.addOption(key + " Quasistatic Reverse", subsystem.sysIdQuasistatic(Direction.kReverse));
        autoChooser.addOption(key + " Dynamic Forward", subsystem.sysIdDynamic(Direction.kForward));
        autoChooser.addOption(key + " Dynamic Reverse", subsystem.sysIdDynamic(Direction.kReverse));
      } else {
        liveTuneChooser.setDefaultOption(key, new TuningButtonHolder( driverController, 
                                                                      new JoystickRPMDrive(subsystem, driverController), 
                                                                      new JoystickDirectDrive(subsystem, () -> driverController.getLeftY())));
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

  private void configureTunableButtonBindings(TuningButtonHolder newHolder) {
    activeHolder.setActive(false);
    activeHolder = newHolder;
    activeHolder.setActive(true);
  }
}

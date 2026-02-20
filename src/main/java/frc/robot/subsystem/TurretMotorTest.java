// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretMotorTest extends SubsystemBase {
  private final String name;
  private SparkFlex turretMotor;
  private SparkFlexConfig config = new SparkFlexConfig();
  private RelativeEncoder turretEncoder;
  private final DoublePublisher appliedOutputPublisher;
  private final DoublePublisher rpmPublisher;
  private final BooleanPublisher limitSwitchPublisher;
  private final DoublePublisher positionPublisher;
  private final DoublePublisher voltagePublisher;
  private DigitalInput limitSwitch;

  /** Creates a new TurretMotorTest. */
  public TurretMotorTest(String turretName, int motorID, int dioChannel) {
    name = turretName;
    turretMotor = new SparkFlex(motorID, SparkFlex.MotorType.kBrushless);

    config.smartCurrentLimit(40, 20);
    config.idleMode(SparkFlexConfig.IdleMode.kBrake);
    config.voltageCompensation(11.0);
    config.voltageCompensation(10.0);

    turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretEncoder = turretMotor.getEncoder();

    appliedOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Turret/" + name + "/Applied Output").publish();
    rpmPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Turret/" + name + "/RPM").publish();
    limitSwitchPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/Turret/" + name + "/Limit Switch").publish();
    positionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Turret/" + name + "/Position").publish();
    voltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Turret/" + name + "/Voltage").publish();

    limitSwitch = new DigitalInput(dioChannel);
  }

  public void setMotorOutput(double output) {
    turretMotor.set(output);
  }

  public void stopMotor() {
    turretMotor.stopMotor();
  }

  public boolean isLimitSwitchPressed() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    appliedOutputPublisher.set(turretMotor.getAppliedOutput());
    rpmPublisher.set(turretEncoder.getVelocity());
    limitSwitchPublisher.set(limitSwitch.get());
    positionPublisher.set(turretEncoder.getPosition());
    voltagePublisher.set(turretMotor.getBusVoltage() * turretMotor.getAppliedOutput());
  }
}

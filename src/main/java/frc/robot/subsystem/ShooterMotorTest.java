// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterMotorTest extends SubsystemBase {
  private final String name;
  private final SparkFlex shooterMotor;
  private final RelativeEncoder shooterEncoder;
  private final DoublePublisher appliedOutputPublisher;
  private final DoublePublisher rpmPublisher;
  private final DoublePublisher voltagePublisher;
  private SparkFlexConfig config = new SparkFlexConfig();

  /** Creates a new ShooterMotorTest. */
  public ShooterMotorTest(String motorName, int motorID) {
    // Initialize your motor controller here using the provided name and motorID
    // For example, if you're using a TalonFX motor controller:
    // TalonFX shooterMotor = new TalonFX(motorID);
    shooterMotor = new SparkFlex(motorID, SparkFlex.MotorType.kBrushless);
    this.name = motorName;
    this.shooterEncoder = shooterMotor.getEncoder();

    config.smartCurrentLimit(100, 100);
    config.idleMode(IdleMode.kCoast);
    config.voltageCompensation(11.0);

    shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    appliedOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/Applied Output").publish();
    rpmPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/RPM").publish();
    voltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/Voltage").publish();
  }

  public void setMotorOutput(double output) {
    shooterMotor.set(output);
  }

  public void stopMotor() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    appliedOutputPublisher.set(shooterMotor.getAppliedOutput());
    rpmPublisher.set(shooterEncoder.getVelocity());
    voltagePublisher.set(shooterMotor.getBusVoltage() * shooterMotor.getAppliedOutput());
  }
}

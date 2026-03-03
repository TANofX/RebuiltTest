// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class TunableSparkMAXSubsystem extends TunableMotorSubsystem {

public TunableSparkMAXSubsystem(int leaderId, double kV, double kA, double kS, double p, double i, double d, String name, boolean inverted) {
    super(leaderId, kV, kA, kS, p, i, d, name, inverted);
  }

  public TunableSparkMAXSubsystem(int leaderId, int followerId, double kV, double kA, double kS, double p, double i, double d, String name, boolean inverted, boolean invertedFollower) {
    super(leaderId, followerId, kV, kA, kS, p, i, d, name, inverted, invertedFollower);
  }

    @Override
    protected SparkBaseConfig createConfig() {
        // TODO Auto-generated method stub
        return new SparkMaxConfig();
    }

    @Override
    protected SparkBase createMotor(int deviceId) {
        // TODO Auto-generated method stub
        return new SparkMax(deviceId, MotorType.kBrushless);
    }

    @Override
    protected SparkSim createSim(SparkBase motor) {
        // TODO Auto-generated method stub
        return new SparkMaxSim((SparkMax)motor, DCMotor.getNEO(1));
    }

}

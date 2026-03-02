package frc.robot.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.TunableMotorSubsystem;

/**
 * Command that reads tuning values from NetworkTables and applies them to
 * the TunableShooterSubsystem. This command is intentionally not scheduled
 * anywhere by default — add it to your RobotContainer for manual activation
 * once you're ready to tune.
 *
 * Expected NetworkTable keys (under table "Shooter/TunableShooter/LiveTune"):
 * - TargetRPM (double)
 * - kP, kI, kD, kF, kS, kV, kA (doubles)
 * - Enabled (boolean) — when true the subsystem is enabled and will accept setpoints
 */
public class LiveTuneCommand extends Command {
  private final TunableMotorSubsystem shooter;


  public LiveTuneCommand(TunableMotorSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Populate NT defaults from current subsystem state so UI shows live values
    SmartDashboard.putNumber("Shooter/LiveTune/TargetRPM", shooter.getTargetRPM());
    SmartDashboard.putNumber("Shooter/LiveTune/kP",shooter.kP());
    SmartDashboard.putNumber("Shooter/LiveTune/kI",shooter.kI());
    SmartDashboard.putNumber("Shooter/LiveTune/kD",shooter.kD());
    SmartDashboard.putNumber("Shooter/LiveTune/kF",shooter.kF);
    SmartDashboard.putNumber("Shooter/LiveTune/kS",shooter.getKS());
    SmartDashboard.putNumber("Shooter/LiveTune/kV",shooter.getKV());
    SmartDashboard.putNumber("Shooter/LiveTune/kA",shooter.getKA());
    SmartDashboard.putBoolean("Shooter/LiveTune/Enabled",false);
  }

  @Override
  public void execute() {
    // Read values (use current subsystem values as fallbacks)
    double target = SmartDashboard.getNumber("Shooter/LiveTune/TargetRPM", shooter.getTargetRPM());
    double p = SmartDashboard.getNumber("Shooter/LiveTune/kP",shooter.kP());
    double i = SmartDashboard.getNumber("Shooter/LiveTune/kI",shooter.kI());


    double d = SmartDashboard.getNumber("Shooter/LiveTune/kD",shooter.kD());
    double kf = SmartDashboard.getNumber("Shooter/LiveTune/kF",shooter.kF);
    double ks = SmartDashboard.getNumber("Shooter/LiveTune/kS",shooter.getKS());
    double kv = SmartDashboard.getNumber("Shooter/LiveTune/kV",shooter.getKV());
    double ka = SmartDashboard.getNumber("Shooter/LiveTune/kA",shooter.getKA());
    boolean en =  SmartDashboard.getBoolean("Shooter/LiveTune/Enabled",false);

    // Apply tuning values
    shooter.setTargetRPM(target);
    shooter.setPID(p, i, d);
    shooter.setKF(kf);
    shooter.setFeedforward(ks, kv, ka);

    if (en) {
      shooter.enable();
    } else {
      shooter.disable();
    }

    // Populate NT defaults from current subsystem state so UI shows live values
    SmartDashboard.putNumber("Shooter/LiveTune/TargetRPM", shooter.getTargetRPM());
    SmartDashboard.putNumber("Shooter/LiveTune/kP",shooter.kP());
    SmartDashboard.putNumber("Shooter/LiveTune/kI",shooter.kI());
    SmartDashboard.putNumber("Shooter/LiveTune/kD",shooter.kD());
    SmartDashboard.putNumber("Shooter/LiveTune/kF",shooter.kF);
    SmartDashboard.putNumber("Shooter/LiveTune/kS",shooter.getKS());
    SmartDashboard.putNumber("Shooter/LiveTune/kV",shooter.getKV());
    SmartDashboard.putNumber("Shooter/LiveTune/kA",shooter.getKA());
    SmartDashboard.putBoolean("Shooter/LiveTune/Enabled",false);
  }

  @Override
  public boolean isFinished() {
    return false; // runs until explicitly canceled
  }

  @Override
  public void end(boolean interrupted) {
    shooter.disable();
  }
}

package frc.robot.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.TunableShooterSubsystem;

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
public class LiveTuneShooterCommand extends Command {
  private final TunableShooterSubsystem shooter;
  private final NetworkTable table;

  private final NetworkTableEntry targetEntry;
  private final NetworkTableEntry pEntry;
  private final NetworkTableEntry iEntry;
  private final NetworkTableEntry dEntry;
  private final NetworkTableEntry kfEntry;
  private final NetworkTableEntry ksEntry;
  private final NetworkTableEntry kvEntry;
  private final NetworkTableEntry kaEntry;
  private final NetworkTableEntry enabledEntry;

  public LiveTuneShooterCommand(TunableShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);

    table = NetworkTableInstance.getDefault().getTable("Shooter").getSubTable("TunableShooter").getSubTable("LiveTune");

    targetEntry = table.getEntry("TargetRPM");
    pEntry = table.getEntry("kP");
    iEntry = table.getEntry("kI");
    dEntry = table.getEntry("kD");
    kfEntry = table.getEntry("kF");
    ksEntry = table.getEntry("kS");
    kvEntry = table.getEntry("kV");
    kaEntry = table.getEntry("kA");
    enabledEntry = table.getEntry("Enabled");
  }

  @Override
  public void initialize() {
    // Populate NT defaults from current subsystem state so UI shows live values
    targetEntry.setDouble(shooter.getTargetRPM());
    pEntry.setDouble(shooter.kP());
    iEntry.setDouble(shooter.kI());
    dEntry.setDouble(shooter.kD());
    kfEntry.setDouble(shooter.kF);
    ksEntry.setDouble(shooter.getKS());
    kvEntry.setDouble(shooter.getKV());
    kaEntry.setDouble(shooter.getKA());
    enabledEntry.setBoolean(false);
  }

  @Override
  public void execute() {
    // Read values (use current subsystem values as fallbacks)
    double target = targetEntry.getDouble(shooter.getTargetRPM());
    double p = pEntry.getDouble(shooter.kP());
    double i = iEntry.getDouble(shooter.kI());
    double d = dEntry.getDouble(shooter.kD());
    double kf = kfEntry.getDouble(shooter.kF);
    double ks = ksEntry.getDouble(shooter.getKS());
    double kv = kvEntry.getDouble(shooter.getKV());
    double ka = kaEntry.getDouble(shooter.getKA());
    boolean en = enabledEntry.getBoolean(false);

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

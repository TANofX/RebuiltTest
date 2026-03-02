package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// BuiltInWidgets intentionally unused here; widgets created with default types.
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystem.TunableMotorSubsystem;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.networktables.GenericEntry;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Helper to create NetworkTable entries and a small Shuffleboard layout for
 * the live-tuning keys used by LiveTuneShooterCommand.
 *
 * This creates the expected subtable at: Shooter -> TunableShooter -> LiveTune
 * and populates default values. It also creates simple display widgets on the
 * "Shooter" tab so you can monitor values; interactive widgets can be added
 * in the Shuffleboard UI and bound to the same NT keys if desired.
 */
public final class LiveTuneShuffleboard {
  private LiveTuneShuffleboard() {}

  public static void setup(TunableMotorSubsystem shooter) {
    NetworkTable live = NetworkTableInstance.getDefault().getTable("Shooter")
        .getSubTable("TunableShooter").getSubTable("LiveTune");

    // Initialize entries with current subsystem state
    live.getEntry("TargetRPM").setDouble(shooter.getTargetRPM());
    live.getEntry("kP").setDouble(shooter.kP());
    live.getEntry("kI").setDouble(shooter.kI());
    live.getEntry("kD").setDouble(shooter.kD());
    live.getEntry("kF").setDouble(shooter.kF);
    live.getEntry("kS").setDouble(shooter.getKS());
    live.getEntry("kV").setDouble(shooter.getKV());
    live.getEntry("kA").setDouble(shooter.getKA());
    live.getEntry("Enabled").setBoolean(false);

        // Create interactive widgets on the Shooter tab and synchronize them with
        // the LiveTune NetworkTable subtable. We use a small poller to copy widget
        // values into the LiveTune entries and vice-versa so Shuffleboard edits are
        // immediately reflected where LiveTuneShooterCommand reads them.
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

        // Helper lists to keep widget and live entries paired
        List<GenericEntry> widgetEntries = new ArrayList<>();
        List<NetworkTableEntry> liveEntries = new ArrayList<>();

        // Keys we want interactive widgets for
        String[] keys = new String[] {"TargetRPM", "kP", "kI", "kD", "kF", "kS", "kV", "kA", "Enabled"};

        for (String key : keys) {
            NetworkTableEntry liveEntry = live.getEntry(key);
            // Create a user-visible widget with the same label. Use a sensible default
            // from the subsystem or the live entry.
            double defaultDouble = 0.0;
            if (key.equals("TargetRPM")) defaultDouble = shooter.getTargetRPM();
            else if (key.equals("kP")) defaultDouble = shooter.kP();
            else if (key.equals("kI")) defaultDouble = shooter.kI();
            else if (key.equals("kD")) defaultDouble = shooter.kD();
            else if (key.equals("kF")) defaultDouble = shooter.kF;
            else if (key.equals("kS")) defaultDouble = shooter.getKS();
            else if (key.equals("kV")) defaultDouble = shooter.getKV();
            else if (key.equals("kA")) defaultDouble = shooter.getKA();

            // Initialize live entry if missing
            liveEntry.setDouble(liveEntry.getDouble(defaultDouble));

            // Create an interactive widget and grab its entry so we can read user changes
            GenericEntry widgetEntry;
                    if (key.equals("Enabled")) {
                        widgetEntry = tab.add("LiveTune " + key, liveEntry.getBoolean(false)).getEntry();
                    } else {
                        widgetEntry = tab.add("LiveTune " + key, liveEntry.getDouble(defaultDouble)).getEntry();
                    }

            // Ensure widget starts with the live value
            if (key.equals("Enabled")) widgetEntry.setBoolean(liveEntry.getBoolean(false));
            else widgetEntry.setDouble(liveEntry.getDouble(defaultDouble));

            widgetEntries.add(widgetEntry);
            liveEntries.add(liveEntry);
        }

        // Poller: synchronize widget -> live and live -> widget every 100ms.
        Timer poller = new Timer("LiveTuneSync", true);
        poller.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                        for (int i = 0; i < keys.length; i++) {
                    String key = keys[i];
                            GenericEntry w = widgetEntries.get(i);
                    NetworkTableEntry l = liveEntries.get(i);
                    try {
                        if (key.equals("Enabled")) {
                            boolean wb = w.getBoolean(l.getBoolean(false));
                            if (wb != l.getBoolean(false)) l.setBoolean(wb);
                            // reflect live->widget if external change
                            boolean lb = l.getBoolean(false);
                            if (lb != w.getBoolean(false)) w.setBoolean(lb);
                        } else {
                            double wd = w.getDouble(l.getDouble(0.0));
                            if (Double.isFinite(wd) && wd != l.getDouble(0.0)) l.setDouble(wd);
                            // reflect live->widget if external change
                            double ld = l.getDouble(wd);
                            if (Double.isFinite(ld) && ld != w.getDouble(wd)) w.setDouble(ld);
                        }
                    } catch (Exception ex) {
                        // Ignore transient NT exceptions; poller continues
                    }
                }
            }
        }, 0, 100);

        // --- Status widgets (polished layout) ---
        // Show whether follower hardware-follow is active and whether the internal
        // closed-loop controller is available, plus live telemetry values.
        tab.addBoolean("HW Follow Active", () -> shooter.isHardwareFollowConfigured())
            .withPosition(0, 2)
            .withSize(1, 1);

        tab.addBoolean("ClosedLoop Available", () -> shooter.isClosedLoopAvailable())
            .withPosition(1, 2)
            .withSize(1, 1);

        tab.addNumber("Measured RPM", () -> shooter.getCurrentRPM())
            .withPosition(2, 2)
            .withSize(2, 1);

        tab.addNumber("Applied Output", () -> shooter.getAppliedOutput())
            .withPosition(0, 3)
            .withSize(2, 1);

        tab.addNumber("Bus Voltage", () -> shooter.getBusVoltage())
            .withPosition(2, 3)
            .withSize(1, 1);
  }
}

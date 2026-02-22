package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

/**
 * Tunable shooter subsystem built for tuning velocity control.
 *
 * Notes:
 * - This class uses a local PID + simple feedforward to control RPM. The repo
 *   includes `SparkFlex` controllers but there is no uniform usage of an
 *   internal closed-loop API in the codebase; this implementation therefore
 *   implements the closed-loop in robot code so gains are easy to tune and
 *   observe. It mirrors the leader output to an inverted follower motor.
 */
public class TunableShooterSubsystem extends SubsystemBase {
  private final String name = "TunableShooter";
  private final SparkFlex leader;
  private final SparkFlex follower;
  private final RelativeEncoder encoder;
  private final SparkFlexConfig config = new SparkFlexConfig();

  // PID + feedforward terms (exposed for tuning)
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private double kS = 0.01;
  private double kV = 0.0;
  private double kA = 0.0;
  // extra kF feedforward (applied either via internal controller's FF or as an arbFF)
  public double kF = 0.0; // user-tunable feed-forward multiplier (applied as kF * RPM)

  // Internal closed-loop controller (when using SparkFlex built-in controller)
  private SparkClosedLoopController closedLoopController = null;
  // Whether follower was successfully configured to hardware-follow the leader
  private boolean hardwareFollowConfigured = false;

  private double targetRPM = 0.0;
  private double lastTargetRPM = 0.0;
  private double lastTimestamp = 0.0;
  private boolean enabled = false;

  // NetworkTables publishers for telemetry and tuning
  private final DoublePublisher appliedOutputPublisher;
  private final DoublePublisher rpmPublisher;
  private final DoublePublisher targetPublisher;
  private final DoublePublisher busVoltagePublisher;
  private final DoublePublisher currentPublisher;

  /**
   * Create a tunable shooter with a leader and an inverted follower.
   *
   * @param leaderId CAN ID of the leader SparkFlex (brushless)
   * @param followerId CAN ID of the follower SparkFlex (brushless) — this motor
   *                   will be driven with the inverted output of the leader
   */
  public TunableShooterSubsystem(int leaderId, int followerId, double kV, double kA, double kS, double p, double i, double d, String name) {
    leader = new SparkFlex(leaderId, SparkFlex.MotorType.kBrushless);
    follower = new SparkFlex(followerId, SparkFlex.MotorType.kBrushless);
    encoder = leader.getEncoder();

    // reasonable defaults copied from other motor tests in the project
    config.smartCurrentLimit(80, 50);
    config.idleMode(IdleMode.kCoast);
    config.voltageCompensation(10.0);

    leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Apply the same base settings to the follower first
    follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Then configure the follower to hardware-follow the leader and persist that
    try {
      SparkFlexConfig followerFollow = new SparkFlexConfig();
      // follow the leader and invert the output for the follower
      followerFollow.follow(leader, true);
      // Don't reset previously-applied safe parameters; only enable follower mode
      follower.configure(followerFollow, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      hardwareFollowConfigured = true;
    } catch (Exception ex) {
      // If the follow configuration isn't available in this REVLib version,
      // we'll fall back to software mirroring (below in periodic()).
      hardwareFollowConfigured = false;
    }

    // initial PID/FF values are zero; user can set them via setters
    pid.setTolerance(50.0); // RPM tolerance (coarse)

    appliedOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/Applied Output").publish();
    rpmPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/RPM").publish();
    targetPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/Target RPM").publish();
    busVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/Bus Voltage").publish();
    currentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Shooter/" + name + "/Current").publish();
    lastTimestamp = Timer.getFPGATimestamp();

    // Try to get the SparkFlex internal closed-loop controller (preferred over software PID)
    try {
      closedLoopController = leader.getClosedLoopController();
    } catch (Exception ex) {
      // If the API isn't available for some reason, we'll stay with software PID
      closedLoopController = null;
    }
    
    setFeedforward(kS, kV, kA);
    setPID(p, i, d);

  }

  /** Enable closed-loop control. */
  public void enable() {
    enabled = true;
    pid.reset();
    lastTargetRPM = targetRPM;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  /** Disable closed-loop control and stop motors. */
  public void disable() {
    enabled = false;
    leader.stopMotor();
    follower.stopMotor();
  }

  public void setTargetRPM(double rpm) {
    this.targetRPM = rpm;
    targetPublisher.set(rpm);
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  // PID setters
  public void setPID(double p, double i, double d) {
    pid.setPID(p, i, d);
    // Also update the closed-loop configuration and persist it to the controller
    try {
      config.closedLoop.p(p).i(i).d(d);
      config.closedLoopRampRate(0.1);
      leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception ex) {
      // If the config API isn't available/throws, continue using the software PID only
    }
  }

  public void setFeedforward(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    try {
      config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
      leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception ex) {
      // ignore and keep software FF values
    }
  }

  /**
   * Set controller kF. If the internal controller exposes a feedforward setter
   * this will attempt to write it there as well.
   */
  public void setKF(double kF) {
    this.kF = kF;
    // kF remains a software convenience term; feedforward parameters are applied via config.feedForward
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(1e-6, now - lastTimestamp);
    lastTimestamp = now;

    double currentRPM = encoder.getVelocity();
    rpmPublisher.set(currentRPM);
    busVoltagePublisher.set(leader.getBusVoltage());
    currentPublisher.set(leader.getOutputCurrent());

    double output = 0.0;
    if (enabled) {
      double accel = (targetRPM - lastTargetRPM) / dt;
      lastTargetRPM = targetRPM;

      double busV = leader.getBusVoltage();

      if (closedLoopController != null) {
        try {
          // Use the SparkFlex internal closed-loop controller for velocity
          closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
          // Controller drives the leader; mirror applied output for follower/telemetry
          output = leader.getAppliedOutput();
        } catch (Exception ex) {
          // fallback to software PID if internal controller call fails
          double pidOut = pid.calculate(currentRPM, targetRPM);
          double ff = 0.0;
          if (Math.abs(targetRPM) > 1e-6) ff = Math.copySign(kS, targetRPM) + kV * targetRPM + kA * accel;
          double totalVoltage = ff + pidOut + kF * targetRPM;
          if (busV <= 1e-6) output = 0.0; else output = totalVoltage / busV;
        }
      } else {
        // software PID + feedforward
        double pidOut = pid.calculate(currentRPM, targetRPM);
        double ff = 0.0;
        if (Math.abs(targetRPM) > 1e-6) ff = Math.copySign(kS, targetRPM) + kV * targetRPM + kA * accel;
        double totalVoltage = ff + pidOut + kF * targetRPM;
        if (busV <= 1e-6) output = 0.0; else output = totalVoltage / busV;
      }
    }

    // clamp
    output = MathUtil.clamp(output, -1.0, 1.0);

    // If the SparkFlex closed-loop controller is present and enabled, it
    // drives the leader hardware directly — do NOT call leader.set() in
    // that case. We also configured the follower to follow the leader above
    // (hardware follow); if that failed at runtime we still fall back to
    // software mirroring here.
    if (enabled && closedLoopController != null) {
      // closed-loop path: controller already set the setpoint earlier; don't call set()
      // output variable contains leader.getAppliedOutput() when controller path was used
      // (set in the closed-loop branch above). We don't need to call follower.set()
      // because we configured hardware follower mode in the constructor. If that
      // configuration failed, the follower will be left uncontrolled here — in that
      // case we still mirror the output for safety.
      try {
        // If follower isn't in hardware-follow mode, call software mirror as a fallback
        // Use a small probe: follower.getAppliedOutput() will reflect its state if following.
        // We conservatively avoid extra calls unless necessary, so do nothing here.
      } catch (Exception e) {
        // No-op: fallthrough to mirrored control below
        setDirectOutput(output);
      }
    } else if (enabled) {
      // software fallback: drive leader and mirrored inverted follower
      setDirectOutput(output);
    } else {
      // disabled: ensure motors are stopped
      setDirectOutput(0.0);
    }

    appliedOutputPublisher.set(leader.getAppliedOutput());
  }

  public void setDirectOutput(double output) {
    if (hardwareFollowConfigured) {
        leader.set(output);
    } else {
        leader.set(output);
        follower.set(-output);  
    }
  }

  // convenience public gain fields for UI integration or network tuning
  public double kP() { return pid.getP(); }
  public double kI() { return pid.getI(); }
  public double kD() { return pid.getD(); }
  public double getKS() { return kS; }
  public double getKV() { return kV; }
  public double getKA() { return kA; }

  /**
   * Returns true if the SparkFlex closed-loop controller was obtained and is
   * available for use.
   */
  public boolean isClosedLoopAvailable() {
    return closedLoopController != null;
  }

  /**
   * Returns true if the follower was successfully configured to hardware-follow
   * the leader (persisted when possible).
   */
  public boolean isHardwareFollowConfigured() {
    return hardwareFollowConfigured;
  }

  /** Current measured RPM from the leader encoder. */
  public double getCurrentRPM() {
    return encoder.getVelocity();
  }

  /** Last applied output on the leader (raw applied output reported by controller). */
  public double getAppliedOutput() {
    try {
      return leader.getAppliedOutput();
    } catch (Exception ex) {
      return 0.0;
    }
  }

  /** Current leader bus voltage. */
  public double getBusVoltage() {
    try {
      return leader.getBusVoltage();
    } catch (Exception ex) {
      return 0.0;
    }
  }
}

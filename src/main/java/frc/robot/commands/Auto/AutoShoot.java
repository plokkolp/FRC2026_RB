package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterLookup;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends Command {

  private final Shooter shooter;

  private static final double kYawP = 0.0045;
  private static final double kYawD = 0.0010;
  private static final double kYawMaxOut = 0.2;
  private static final double kYawTolDeg = 1.8;
  private static final double kMaxValidTxDeg = 30.0;

  private static final double kTrainDuty = -0.7;
  private static final double kIntakeDuty = -0.7;

  private static final boolean kGateByRPM = false;
  private static final double kRpmTol = 120.0;
  private static final double kSpinupMinTime = 0.2;

  private double startTime;

  private double lastTargetRpm = 2200.0;
  private double lastTargetPitchRot = -0.45;

  private double lastTx = 0.0;
  private double lastTime = 0.0;

  public AutoShoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    lastTime = Timer.getFPGATimestamp();
    lastTx = 0.0;

    double dist = shooter.getlong();
    boolean distValid = Double.isFinite(dist) && dist > 0.05 && dist < 10.0;
    if (distValid) {
      ShooterLookup.Point sp = ShooterLookup.sample(dist);
      lastTargetRpm = sp.rpm;
      lastTargetPitchRot = sp.pitchRot;
    }
  }

  @Override
  public void execute() {
    boolean hasTarget = shooter.hasLLTarget();
    double tx = shooter.getLLTx();
    double dist = shooter.getlong();

    boolean distValid = Double.isFinite(dist) && dist > 0.05 && dist < 10.0;
    if (hasTarget && distValid) {
      ShooterLookup.Point sp = ShooterLookup.sample(dist);
      lastTargetRpm = sp.rpm;
      lastTargetPitchRot = sp.pitchRot;
    }

    shooter.setPitchPosition(lastTargetPitchRot);
    shooter.setShooterRPM(lastTargetRpm);

    double yawCmd = 0.0;
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;

    if (hasTarget) {
      if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
        double error = tx;

        if (Math.abs(error) > kYawTolDeg) {
          double errorRate = 0.0;
          if (dt > 1e-4) {
            errorRate = (error - lastTx) / dt;
          }

          yawCmd = error * kYawP + errorRate * kYawD;
          yawCmd = MathUtil.clamp(yawCmd, -kYawMaxOut, kYawMaxOut);
        }
      }
    }

    shooter.setYawSpeed(yawCmd);

    if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
      lastTx = tx;
    }
    lastTime = now;

    boolean trigger = shooter.getLeftShooterRPM() > 2000.0;

    boolean rpmReady = Math.abs(shooter.getShooterRPM() - lastTargetRpm) <= kRpmTol;
    boolean timeReady = (Timer.getFPGATimestamp() - startTime) > kSpinupMinTime;

    boolean allowFeed = trigger;
    if (kGateByRPM) {
      allowFeed = trigger && rpmReady && timeReady;
    }

    if (allowFeed) {
      shooter.setTrainSpeed(kTrainDuty);
      shooter.setIntaketrainSpeed(kIntakeDuty);
    } else {
      shooter.setTrainSpeed(0.0);
      shooter.setIntaketrainSpeed(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0.0);
    shooter.setTrainSpeed(0.0);
    shooter.setIntaketrainSpeed(0.0);
    shooter.stopPitch();
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

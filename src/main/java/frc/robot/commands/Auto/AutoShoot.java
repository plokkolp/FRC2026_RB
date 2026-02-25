package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.ShooterLookup;

public class AutoShoot extends Command {

  private final Shooter shooter;
  

  private static final double kYawP = 0.01;
  private static final double kYawMaxOut = 0.3;
  private static final double kYawTolDeg = 1.0;
  private static final double kMaxValidTxDeg = 30.0;

  private static final double kTrainDuty = -0.7;
  private static final double kIntakeDuty = 0.85;
  private static final double kFireTrig = 0.3;

  private static final boolean kGateByRPM = false;
  private static final double kRpmTol = 120.0;
  private static final double kSpinupMinTime = 0.2;

  private double startTime;

  private double lastTargetRpm = 2200.0;
  private double lastTargetPitchRot = -0.45;

  public AutoShoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

    boolean hasTarget = shooter.hasLLTarget();
    double dist = shooter.getlong();

    boolean distValid = Double.isFinite(dist) && dist > 0.05 && dist < 10.0; 
    if (hasTarget && distValid) {
      ShooterLookup.Point sp = ShooterLookup.sample(dist);
      lastTargetRpm = sp.rpm;
      lastTargetPitchRot = sp.pitchRot;
      SmartDashboard.putString("Auto/setpointMode", "LIVE");
    } else {
      SmartDashboard.putString("Auto/setpointMode", "HOLD");
    }

    shooter.setPitchPosition(lastTargetPitchRot);
    shooter.setShooterRPM(lastTargetRpm);

    if (hasTarget) {
      double tx = shooter.getLLTx();
      if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
        if (Math.abs(tx) > kYawTolDeg) {
          double yawCmd = tx * kYawP;
          yawCmd = MathUtil.clamp(yawCmd, -kYawMaxOut, kYawMaxOut);
          shooter.setYawSpeed(yawCmd);
          SmartDashboard.putNumber("Auto/yawCmd", yawCmd);
        } else {
          shooter.setYawSpeed(0);
          SmartDashboard.putNumber("Auto/yawCmd", 0);
        }
      } else {
        shooter.setYawSpeed(0);
        SmartDashboard.putNumber("Auto/yawCmd", 0);
      }
      SmartDashboard.putNumber("Auto/tx", tx);
    } else {
      shooter.setYawSpeed(0);
      SmartDashboard.putNumber("Auto/tx", 999);
      SmartDashboard.putNumber("Auto/yawCmd", 0);
    }

    boolean trigger = shooter.getLeftShooterRPM() > 2000;

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
      shooter.setTrainSpeed(0);
      shooter.setIntaketrainSpeed(0);
    }

    // ===== Debug =====
    // SmartDashboard.putBoolean("Auto/hasTarget", hasTarget);
    // SmartDashboard.putNumber("Auto/dist", dist);

    // SmartDashboard.putNumber("Auto/RPM_target", lastTargetRpm);
    // SmartDashboard.putNumber("Auto/RPM_now", shooter.getShooterRPM());

    // SmartDashboard.putNumber("Auto/Pitch_targetRot", lastTargetPitchRot);
    // SmartDashboard.putNumber("Auto/Pitch_nowRot", shooter.getPitchPositionRot());

    // SmartDashboard.putBoolean("Auto/trigger", trigger);
    // SmartDashboard.putBoolean("Auto/rpmReady", rpmReady);
    // SmartDashboard.putBoolean("Auto/allowFeed", allowFeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0);
    shooter.setTrainSpeed(0);
    shooter.setIntaketrainSpeed(0);
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
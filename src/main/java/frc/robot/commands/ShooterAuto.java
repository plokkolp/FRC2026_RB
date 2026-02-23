package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.ShooterLookup;

public class ShooterAuto extends Command {

  private final Shooter shooter;
  private final XboxController controller;

  private static final double kYawP = 0.01;
  private static final double kYawMaxOut = 0.35;
  private static final double kYawTolDeg = 1.0;
  private static final double kMaxValidTxDeg = 30.0;

  private static final double kTrainDuty = -0.7;
  private static final double kIntakeDuty = 0.85;
  private static final double kFireTrig = 0.9;

  private static final boolean kGateByRPM = true;
  private static final double kRpmTol = 120.0;
  private static final double kSpinupMinTime = 0.2;

  private double startTime;

  public ShooterAuto(Shooter shooter, XboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

    // 沒目標：至少不要送彈，Yaw停
    if (!shooter.hasLLTarget()) {
      shooter.setYawSpeed(0);
      shooter.setTrainSpeed(0);
      shooter.setIntaketrainSpeed(0);
      SmartDashboard.putString("Auto/state", "NO_TARGET");
      return;
    }

    // ===== 1) 距離查表 -> 直接取得 pitchRot（你給的就是 pitch 角度/rot）=====
    double dist = shooter.getlong();
    ShooterLookup.Point sp = ShooterLookup.sample(dist);

    // Pitch 用「位置」控制（閉迴路），目標就是表的第三欄
    shooter.setPitchPosition(sp.pitchRot);

    // 飛輪轉速用 RPM（閉迴路）
    shooter.setShooterRPM(sp.rpm);

    // ===== 2) Yaw 追 tx（維持你原本 ShooterMaster）=====
    double tx = shooter.getLLTx();

    if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
      if (Math.abs(tx) > kYawTolDeg) {
        double yawCmd = -tx * kYawP;
        yawCmd = MathUtil.clamp(yawCmd, -kYawMaxOut, kYawMaxOut);
        shooter.setYawSpeed(yawCmd);
      } else {
        shooter.setYawSpeed(0);
      }
    } else {
      shooter.setYawSpeed(0);
    }

    // ===== 3) Trigger 才送彈 =====
    boolean trigger = controller.getRightTriggerAxis() > kFireTrig;

    boolean rpmReady =
        Math.abs(shooter.getShooterRPM() - sp.rpm) <= kRpmTol;

    boolean timeReady =
        (Timer.getFPGATimestamp() - startTime) > kSpinupMinTime;

    boolean allowFeed = trigger;
    if (kGateByRPM) {
      allowFeed = trigger && rpmReady && timeReady;
    }

    if (allowFeed) {
      shooter.setTrainSpeed(kTrainDuty);
      shooter.setIntaketrainSpeed(kIntakeDuty);
      SmartDashboard.putString("Auto/state", "FEEDING");
    } else {
      shooter.setTrainSpeed(0);
      shooter.setIntaketrainSpeed(0);
      SmartDashboard.putString("Auto/state", "AIMING");
    }

    // ===== 4) Debug =====
    SmartDashboard.putNumber("Auto/dist", dist);
    SmartDashboard.putNumber("Auto/tx", tx);

    SmartDashboard.putNumber("Auto/RPM_target", sp.rpm);
    SmartDashboard.putNumber("Auto/RPM_now", shooter.getShooterRPM());

    SmartDashboard.putNumber("Auto/Pitch_targetRot", sp.pitchRot);
    SmartDashboard.putNumber("Auto/Pitch_nowRot", shooter.getPitchPositionRot());

    SmartDashboard.putBoolean("Auto/trigger", trigger);
    SmartDashboard.putBoolean("Auto/rpmReady", rpmReady);
    SmartDashboard.putBoolean("Auto/allowFeed", allowFeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0);
    shooter.setTrainSpeed(0);
    shooter.setIntaketrainSpeed(0);
    shooter.stopShooter();
    // Pitch 通常保持位置比較好；你要停就加 shooter.stopPitch();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
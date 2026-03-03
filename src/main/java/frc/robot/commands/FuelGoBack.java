package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class FUELGOBACK extends Command {

  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;

  // ===== 只在 ±60° 內啟用 =====
  private static final double kEnableRangeDeg = 60.0;

  // ===== 砲台校正參數 =====
  private static final double kCenterRot = -0.28857421875; // TODO: 你實測中心
  private static final double kRotPerDeg = 0.0171;  // rot / deg

  // ===== 機械極限 =====
  private static final double kMinRot = -1.3525390625; // TODO: 你實測最左
  private static final double kMaxRot = -0.20; // TODO: 你實測最右

  // ===== 控制參數 =====
  private static final double kP = 0.8;
  private static final double kMaxOut = 0.25;
  private static final double kTolRot = 0.01;

  public FUELGOBACK(Shooter shooter, CommandSwerveDrivetrain drivetrain) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    addRequirements(shooter);
  }

  @Override
  public void execute() {

    // 1️⃣ 底盤場地角度
    double poseDeg = drivetrain.getState().Pose.getRotation().getDegrees();
    poseDeg = MathUtil.inputModulus(poseDeg, -180.0, 180.0);

    // 2️⃣ 啟用條件：只在 ±60° 內
    boolean enabled = Math.abs(poseDeg) <= kEnableRangeDeg;

    double curRot = shooter.getYawMotorPositionRot();
    double desiredDeg = 0.0;
    double targetRot = curRot;
    double errRot = 0.0;
    double out = 0.0;

    if (enabled) {

      // 鎖場地 0°
      desiredDeg = poseDeg;

      targetRot = kCenterRot + desiredDeg * kRotPerDeg;

      // 機械限制
      targetRot = MathUtil.clamp(targetRot, kMinRot, kMaxRot);

      errRot = targetRot - curRot;

      if (Math.abs(errRot) < kTolRot) {
        out = 0.0;
      } else {
        out = MathUtil.clamp(errRot * kP, -kMaxOut, kMaxOut);
      }

      shooter.setYawSpeed(out);

    } else {
      shooter.setYawSpeed(0.0);
    }

    // ===== Dashboard =====
    SmartDashboard.putBoolean("FUELGOBACK/Enabled", enabled);
    SmartDashboard.putNumber("FUELGOBACK/PoseDeg", poseDeg);
    SmartDashboard.putNumber("FUELGOBACK/DesiredDeg", desiredDeg);
    SmartDashboard.putNumber("FUELGOBACK/TurretCurRot", curRot);
    SmartDashboard.putNumber("FUELGOBACK/TurretTargetRot", targetRot);
    SmartDashboard.putNumber("FUELGOBACK/ErrRot", errRot);
    SmartDashboard.putNumber("FUELGOBACK/Output", out);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0.0);
  }
}
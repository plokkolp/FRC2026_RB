package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class AlignRightUntilTxZero extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;

  private static final double kTxToleranceDeg = 2.0;

  private static final double kRightSpeed = 1.2; // m/s（robot-relative；往右是負Y）

  private static final double kHeadingTargetDeg = 180;

  private static final double kHeadingRangeDeg = -5.0;

  private static final double kHeadingKp = 0.1;     // 你目前用 1（OK，但通常會偏大）
  private static final double kMaxOmegaRad = 2;   // 最大角速度 rad/s
  private static final double kMinOmegaRad = 0.25;  // 最小角速度 rad/s（避免卡住不轉）

  private final SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();

  private boolean headingInRange = false;

  public AlignRightUntilTxZero(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    headingInRange = false;
  }

  @Override
  public void execute() {
    double yawDeg = drivetrain.getTeleopHeading().getDegrees();
    double errDeg = MathUtil.inputModulus(kHeadingTargetDeg - yawDeg, -180.0, 180.0);

    // ===== 角度控制 omega（rad/s）=====
    double omegaRadPerSec = kHeadingKp * Math.toRadians(errDeg);

    // 如果還沒進 range，給它最小轉速避免磨蹭不動
    if (Math.abs(errDeg) > kHeadingRangeDeg) {
      omegaRadPerSec =
          Math.copySign(Math.max(Math.abs(omegaRadPerSec), kMinOmegaRad), omegaRadPerSec);
    } else {
      headingInRange = true;
    }

    omegaRadPerSec = MathUtil.clamp(omegaRadPerSec, -kMaxOmegaRad, +kMaxOmegaRad);

    double vy = 0.0;
    if (headingInRange) {
      vy = -kRightSpeed; // 往右
    }

    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds(
        0.0,  // vx
        vy,   // vy
        omegaRadPerSec
    )));
  }

  @Override
  public boolean isFinished() {
    // 只有在「已經進入 heading range」後，才判斷 tx 是否到位
    if (!headingInRange) return false;
    return Math.abs(shooter.getLLTx()) <= kTxToleranceDeg;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
  }
}
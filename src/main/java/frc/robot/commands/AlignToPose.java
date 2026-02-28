// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class AlignToPose extends Command {

//   private final CommandSwerveDrivetrain drivetrain;

//   private static final Pose2d kTargetBlue =
//       new Pose2d(15.041, 4.321, Rotation2d.fromDegrees(180.0));

//   private final PIDController xPid = new PIDController(2.2, 0.0, 0.0);
//   private final PIDController yPid = new PIDController(2.2, 0.0, 0.0);
//   private final PIDController tPid = new PIDController(4.0, 0.0, 0.15);

//   private static final double kMaxV = 0.8;
//   private static final double kMaxOmega = 1.0;

//   private static final double kTolXY = 0.05;
//   private static final double kTolDeg = 2.0;

//   private static final double kTimeoutSec = 1.5;

//   private static final double kMinV = 0.03;
//   private static final double kMinOmega = 0.05;

//   // 🔧 修正：Y 反向
//   private static final boolean kInvertX = true;
//   private static final boolean kInvertY = true;
//   private static final boolean kInvertOmega = false;

//   private double startTs;
//   private Pose2d targetField;

//   private final SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();

//   public AlignToPose(CommandSwerveDrivetrain drivetrain) {
//     this.drivetrain = drivetrain;
//     addRequirements(drivetrain);
//     tPid.enableContinuousInput(-Math.PI, Math.PI);
//   }

//   @Override
//   public void initialize() {
//     startTs = Timer.getFPGATimestamp();

//     boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
//     targetField = isRed ? kTargetBlue : kTargetBlue;

//     xPid.reset();
//     yPid.reset();
//     tPid.reset();

//     xPid.setSetpoint(targetField.getX());
//     yPid.setSetpoint(targetField.getY());
//     tPid.setSetpoint(targetField.getRotation().getRadians());
//   }

//   @Override
//   public void execute() {
//     Pose2d cur = drivetrain.getPose();

//     double vxField = xPid.calculate(cur.getX());
//     double vyField = yPid.calculate(cur.getY());
//     double om = -tPid.calculate(cur.getRotation().getRadians());

//     if (kInvertX) vxField = -vxField;
//     if (kInvertY) vyField = -vyField;
//     if (kInvertOmega) om = -om;

//     vxField = MathUtil.clamp(vxField, -kMaxV, kMaxV);
//     vyField = MathUtil.clamp(vyField, -kMaxV, kMaxV);
//     om = MathUtil.clamp(om, -kMaxOmega, kMaxOmega);

//     vxField = applyDeadband(vxField, kMinV);
//     vyField = applyDeadband(vyField, kMinV);

//     // 🔥 角度 ±2° 內直接停轉
//     double angleErrorDeg =
//         targetField.getRotation().minus(cur.getRotation()).getDegrees();

//     if (Math.abs(angleErrorDeg) < kTolDeg) {
//       om = 0.0;
//     } else {
//       om = applyDeadband(om, kMinOmega);
//     }

//     Rotation2d heading = drivetrain.getTeleopHeading();

//     ChassisSpeeds robotRel =
//         ChassisSpeeds.fromFieldRelativeSpeeds(
//             vxField, vyField, om, heading);

//     drivetrain.setControl(req.withSpeeds(robotRel));

//     SmartDashboard.putNumber("Align/ErrDeg", angleErrorDeg);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.setControl(req.withSpeeds(new ChassisSpeeds()));
//   }

//   @Override
//   public boolean isFinished() {
//     Pose2d cur = drivetrain.getPose();

//     double dx = targetField.getX() - cur.getX();
//     double dy = targetField.getY() - cur.getY();
//     double dxy = Math.hypot(dx, dy);

//     double ddeg =
//         Math.abs(targetField.getRotation().minus(cur.getRotation()).getDegrees());

//     boolean done = (dxy < kTolXY) && (ddeg < kTolDeg);
//     boolean timeout = (Timer.getFPGATimestamp() - startTs) > kTimeoutSec;

//     return done || timeout;
//   }

//   private static double applyDeadband(double v, double minAbs) {
//     return (Math.abs(v) < minAbs) ? 0.0 : v;
//   }
// }
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToPose extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  private static final Pose2d kTargetBlue =
      new Pose2d(15.041, 4.321, Rotation2d.fromDegrees(180.0));

  private final PIDController xPid = new PIDController(2.2, 0.0, 0.0);
  private final PIDController yPid = new PIDController(2.2, 0.0, 0.0);
  private final PIDController tPid = new PIDController(4.0, 0.0, 0.15);

  private static final double kMaxV = 0.8;
  private static final double kMaxOmega = 1.0;

  private static final double kTolXY = 0.05;
  private static final double kTolDeg = 2.0;

  private static final double kBackDistance = 0.50; // ★ 20cm

  private double startTs;
  private Pose2d targetField;

  private Pose2d phase2StartPose;
  private boolean phase2 = false;

  private final SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();

  public AlignToPose(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    tPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    startTs = Timer.getFPGATimestamp();

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    targetField = isRed ? kTargetBlue : kTargetBlue;

    xPid.reset();
    yPid.reset();
    tPid.reset();

    xPid.setSetpoint(targetField.getX());
    yPid.setSetpoint(targetField.getY());
    tPid.setSetpoint(targetField.getRotation().getRadians());

    phase2 = false;
  }

  @Override
  public void execute() {

    Pose2d cur = drivetrain.getPose();

    // ========================
    // Phase 1：對位
    // ========================
    if (!phase2) {

      double vxField = -xPid.calculate(cur.getX());
      double vyField = -yPid.calculate(cur.getY());
      double om = -tPid.calculate(cur.getRotation().getRadians());

      vxField = MathUtil.clamp(vxField, -kMaxV, kMaxV);
      vyField = MathUtil.clamp(vyField, -kMaxV, kMaxV);
      om = MathUtil.clamp(om, -kMaxOmega, kMaxOmega);

      Rotation2d heading = drivetrain.getTeleopHeading();

      ChassisSpeeds robotRel =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              vxField, vyField, om, heading);

      drivetrain.setControl(req.withSpeeds(robotRel));

      double dx = targetField.getX() - cur.getX();
      double dy = targetField.getY() - cur.getY();
      double dxy = Math.hypot(dx, dy);
      double ddeg =
          Math.abs(targetField.getRotation().minus(cur.getRotation()).getDegrees());

      // 進入 Phase 2 條件
      if (dxy < kTolXY && ddeg < kTolDeg) {
        phase2 = true;
        phase2StartPose = cur;
      }

      return;
    }

    // ========================
    // Phase 2：後退 20cm（robot-relative）
    // ========================

    double traveled =
        cur.getTranslation().getDistance(phase2StartPose.getTranslation());

    if (traveled < kBackDistance) {
      // 往機器人後方（-X robot frame）
      drivetrain.setControl(
          req.withSpeeds(new ChassisSpeeds(-0.4, 0.0, 0.0)));
    } else {
      drivetrain.setControl(
          req.withSpeeds(new ChassisSpeeds()));
    }
  }

  @Override
  public boolean isFinished() {
    if (!phase2) return false;

    Pose2d cur = drivetrain.getPose();
    double traveled =
        cur.getTranslation().getDistance(phase2StartPose.getTranslation());

    return traveled >= kBackDistance;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds()));
  }
}
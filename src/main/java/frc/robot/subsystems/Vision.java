package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

  private static final String kLLName = "limelight-down";

  private static final int kMinTagsForMultiTag = 2;
  private static final double kSingleTagMaxDistM = 3.5;
  private static final double kRejectAvgDist = 7.0;

  private static final double kSingleTagMaxJumpM = 1.5;
  private static final double kSingleTagMaxYawDeltaDeg = 45.0;

  private Pose2d emaPose = null;
  private double lastEmaTs = 0.0;

  private static final double kEmaTauSecMulti = 0.15;  // 高改
  private static final double kEmaTauSecSingle = 0.30; 

  private static final double kAlphaMin = 0.02;
  private static final double kAlphaMax = 0.40;

  private boolean hasHardSeededPose = false;

  private final CommandSwerveDrivetrain drivetrain;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {

    boolean tv =
        NetworkTableInstance.getDefault()
            .getTable(kLLName)
            .getEntry("tv")
            .getDouble(0.0) == 1.0;

    double tid =
        NetworkTableInstance.getDefault()
            .getTable(kLLName)
            .getEntry("tid")
            .getDouble(-1.0);

    SmartDashboard.putBoolean("Vision/LL/HasTarget", tv);
    SmartDashboard.putNumber("Vision/LL/TagID", tid);

    Pose2d odo = drivetrain.getPose();
    Optional<Candidate> candOpt = getCandidate();

    if (candOpt.isEmpty()) {
      SmartDashboard.putString("Vision/Gate", "NO_CANDIDATE");
      SmartDashboard.putBoolean("Vision/WillUpdatePose", false);
      return;
    }

    Candidate c = candOpt.get();

    double jumpM = odo.getTranslation().getDistance(c.pose.getTranslation());
    double yawDeltaDeg =
        Math.abs(odo.getRotation().minus(c.pose.getRotation()).getDegrees());

    SmartDashboard.putNumber("Vision/TagCount", c.tagCount);
    SmartDashboard.putNumber("Vision/AvgDist", c.avgDist);
    SmartDashboard.putNumber("Vision/JumpM", jumpM);
    SmartDashboard.putNumber("Vision/YawDeltaDeg", yawDeltaDeg);
    SmartDashboard.putBoolean("Vision/HardSeeded", hasHardSeededPose);

    // 規則 1：尚未 hard seed 時，只要 MultiTag 就硬重設一次 + 融合
    if (!hasHardSeededPose && c.tagCount >= kMinTagsForMultiTag) {
      SmartDashboard.putString("Vision/Gate", "HARDSEED_MULTITAG");
      hardSeedOnce(c);
      fuse(c);
      SmartDashboard.putBoolean("Vision/WillUpdatePose", true);
      return;
    }

    // 規則 2：hard seed 後
    // 2A) MultiTag 永遠直接更新
    if (c.tagCount >= kMinTagsForMultiTag) {
      SmartDashboard.putString("Vision/Gate", "MULTITAG_UPDATE");
      fuse(c);
      SmartDashboard.putBoolean("Vision/WillUpdatePose", true);
      return;
    }

    // 2B) SingleTag：距離門檻 + jump < 1.5m + yawDelta < 45deg 才更新
    if (isSingleTagOK(c)
        && hasHardSeededPose
        && jumpM < kSingleTagMaxJumpM
        && yawDeltaDeg < kSingleTagMaxYawDeltaDeg) {

      SmartDashboard.putString("Vision/Gate", "SINGLETAG_UPDATE_OK");
      fuse(c);
      SmartDashboard.putBoolean("Vision/WillUpdatePose", true);
      return;
    }

    SmartDashboard.putString("Vision/Gate", "REJECT");
    SmartDashboard.putBoolean("Vision/WillUpdatePose", false);
  }

  private static class Candidate {
    final Pose2d pose;
    final double timestamp;
    final int tagCount;
    final double avgDist;

    Candidate(Pose2d pose, double timestamp, int tagCount, double avgDist) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.tagCount = tagCount;
      this.avgDist = avgDist;
    }
  }

  private Optional<Candidate> getCandidate() {

    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    // 你原本的寫法我先照留（如果你發現紅藍對調，再把兩個呼叫交換）
    PoseEstimate pe =
        isRed
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue(kLLName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed(kLLName);

    if (pe == null || pe.tagCount <= 0) return Optional.empty();

    Pose2d pose = pe.pose;
    double ts = pe.timestampSeconds;
    double avgDist = pe.avgTagDist;

    if (avgDist > kRejectAvgDist) return Optional.empty();

    return Optional.of(new Candidate(pose, ts, pe.tagCount, avgDist));
  }

  private boolean isSingleTagOK(Candidate c) {
    return c.tagCount == 1 && c.avgDist < kSingleTagMaxDistM;
  }

  private void hardSeedOnce(Candidate c) {
    if (hasHardSeededPose) return;

    drivetrain.resetPose(c.pose);
    hasHardSeededPose = true;

    // hard seed 當下把 EMA 也初始化成同一筆，避免 EMA 把你拉回去
    emaPose = c.pose;
    lastEmaTs = c.timestamp;

    SmartDashboard.putNumber("Vision/HardSeedTs", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Vision/HardSeedX", c.pose.getX());
    SmartDashboard.putNumber("Vision/HardSeedY", c.pose.getY());
    SmartDashboard.putNumber("Vision/HardSeedDeg", c.pose.getRotation().getDegrees());
  }

  private void fuse(Candidate c) {

    // ===== EMA 綠波：先把 vision pose 平滑後再丟給 estimator =====
    Pose2d filteredPose = emaFilterPose(c.pose, c.timestamp, c.tagCount);

    double sx = 0.05;
    double sy = 0.05;
    double st = Math.toRadians(3.0);

    if (c.tagCount <= 1) {
      sx *= 3.0;
      sy *= 3.0;
      st *= 3.0;
    }

    double distScale = 1.0 + 2.0 * clamp(c.avgDist, 0.5, 8.0) / 8.0;

    sx = clamp(sx * distScale, 0.03, 0.70);
    sy = clamp(sy * distScale, 0.03, 0.70);
    st = clamp(st * distScale, Math.toRadians(1.0), Math.toRadians(25.0));

    Matrix<N3, N1> stdDevs = VecBuilder.fill(sx, sy, st);

    drivetrain.addVisionMeasurement(filteredPose, c.timestamp, stdDevs);

    SmartDashboard.putNumber("Vision/FuseStdX", sx);
    SmartDashboard.putNumber("Vision/FuseStdY", sy);
    SmartDashboard.putNumber("Vision/FuseStdDeg", Math.toDegrees(st));
  }

  private Pose2d emaFilterPose(Pose2d measurement, double ts, int tagCount) {
    if (measurement == null) return measurement;

    // 第一筆直接吃
    if (emaPose == null) {
      emaPose = measurement;
      lastEmaTs = ts;

      SmartDashboard.putNumber("Vision/EMA/Alpha", 1.0);
      SmartDashboard.putNumber("Vision/EMA/X", emaPose.getX());
      SmartDashboard.putNumber("Vision/EMA/Y", emaPose.getY());
      SmartDashboard.putNumber("Vision/EMA/Deg", emaPose.getRotation().getDegrees());
      return emaPose;
    }

    double dt = ts - lastEmaTs;
    if (dt <= 0.0 || dt > 0.5) dt = 0.02; // 保底，避免 timestamp 跳太大

    double tau = (tagCount >= kMinTagsForMultiTag) ? kEmaTauSecMulti : kEmaTauSecSingle;

    // alpha = dt / (tau + dt)
    double alpha = dt / (tau + dt);
    alpha = clamp(alpha, kAlphaMin, kAlphaMax);

    double x = emaPose.getX() + alpha * (measurement.getX() - emaPose.getX());
    double y = emaPose.getY() + alpha * (measurement.getY() - emaPose.getY());

    Rotation2d r = emaPose.getRotation().interpolate(measurement.getRotation(), alpha);

    emaPose = new Pose2d(x, y, r);
    lastEmaTs = ts;

    SmartDashboard.putNumber("Vision/EMA/Alpha", alpha);
    SmartDashboard.putNumber("Vision/EMA/X", emaPose.getX());
    SmartDashboard.putNumber("Vision/EMA/Y", emaPose.getY());
    SmartDashboard.putNumber("Vision/EMA/Deg", emaPose.getRotation().getDegrees());

    return emaPose;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private static final String kLeftName = "Left";
  private static final String kRightName = "Right";

  // +X 前 +Y 左 +Z 上
  private static final Transform3d kRobotToLeftCam =
      new Transform3d(
          new Translation3d(0.75, 0.16, 0.30),
          new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(0)));

  private static final Transform3d kRobotToRightCam =
      new Transform3d(
          new Translation3d(0, 0, 0),
          new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(0)));

  // ===== Gate 參數 =====
  private static final int kMinTagsForMultiTag = 2;

  private static final double kDualSameTagMaxDtSec = 0.06;
  private static final double kDualMaxPoseDeltaM = 0.35;
  private static final double kDualMaxYawDeltaDeg = 12.0;

  // 1-tag（你要單鏡頭單Tag也更新：所以只保留品質門檻，不再當成 fallback）
  private static final double kSingleTagMaxDistM = 3.5; // 原本 3.0，稍微放寬一點避免太容易斷
  private static final double kSingleTagMaxAmb = 0.15;  // 原本 0.08，稍微放寬；太嚴會很常拒絕

  // Candidate 基本品質拒絕（只看 vision 本身品質，不看跟里程計差多少）
  private static final double kRejectAvgDist = 7.0;
  private static final double kRejectAvgAmb = 0.40;

  private final PhotonCamera leftCam = new PhotonCamera(kLeftName);
  private final PhotonCamera rightCam = new PhotonCamera(kRightName);

  private final PhotonPoseEstimator leftEstimator;
  private final PhotonPoseEstimator rightEstimator;

  private final CommandSwerveDrivetrain drivetrain;

  // ===== 第一次雙Tag：無條件硬切 Pose =====
  private boolean hasHardSeededPose = false;
  private double hardSeedTs = -1.0;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    AprilTagFieldLayout layout = null;
    try {
      layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    } catch (Exception e) {
      DriverStation.reportError("Vision: field layout load failed", e.getStackTrace());
    }

    if (layout == null) {
      leftEstimator = null;
      rightEstimator = null;
      return;
    }

    leftEstimator =
        new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToLeftCam);

    rightEstimator =
        new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToRightCam);

    leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    if (leftEstimator == null || rightEstimator == null) {
      SmartDashboard.putString("Vision/Gate", "NO_ESTIMATOR");
      return;
    }

    // SmartDashboard.putBoolean("Vision/LeftConnected", leftCam.isConnected());
    // SmartDashboard.putBoolean("Vision/RightConnected", rightCam.isConnected());

    Pose2d odo = drivetrain.getPose();
    Pose3d ref = new Pose3d(odo);

    leftEstimator.setReferencePose(ref);
    rightEstimator.setReferencePose(ref);

    Optional<Candidate> left = getCandidate(leftCam, leftEstimator, odo);
    Optional<Candidate> right = getCandidate(rightCam, rightEstimator, odo);

    // 1) MultiTag：任一鏡頭達標就更新（第一次雙Tag無條件 resetPose）
    if (left.isPresent() && left.get().tagCount >= kMinTagsForMultiTag) {
      hardSeedPoseOnce(left.get(), "HARDSEED_LEFT_MULTITAG");
      fuse(left.get(), "LEFT_MULTITAG");
      return;
    }
    if (right.isPresent() && right.get().tagCount >= kMinTagsForMultiTag) {
      hardSeedPoseOnce(right.get(), "HARDSEED_RIGHT_MULTITAG");
      fuse(right.get(), "RIGHT_MULTITAG");
      return;
    }

    // 2) 雙鏡頭同一Tag：兩邊都要有（保留你原本的保護條件）
    if (left.isPresent() && right.isPresent()) {
      Candidate L = left.get();
      Candidate R = right.get();

      boolean sameTag = (L.bestFid >= 0) && (L.bestFid == R.bestFid);
      double dt = Math.abs(L.timestamp - R.timestamp);
      double posDelta = L.pose.getTranslation().getDistance(R.pose.getTranslation());
      double yawDeltaDeg =
          Math.abs(L.pose.getRotation().minus(R.pose.getRotation()).getDegrees());

      // SmartDashboard.putBoolean("Vision/Dual/sameTag", sameTag);
      // SmartDashboard.putNumber("Vision/Dual/dt", dt);
      // SmartDashboard.putNumber("Vision/Dual/posDelta", posDelta);
      // SmartDashboard.putNumber("Vision/Dual/yawDeltaDeg", yawDeltaDeg);

      if (sameTag
          && dt <= kDualSameTagMaxDtSec
          && posDelta <= kDualMaxPoseDeltaM
          && yawDeltaDeg <= kDualMaxYawDeltaDeg) {

        Candidate chosen = chooseBetter(L, R);
        fuse(chosen, "DUAL_SAME_TAG");
        return;
      }
    }

    // 3) ★你要的：單鏡頭單Tag也更新（不再當 fallback，而是正常更新）
    //    規則：只要 candidate 有、且單Tag品質 OK，就直接 fuse
    if (left.isPresent() && isSingleTagOK(left.get())) {
      fuse(left.get(), "LEFT_SINGLE_TAG");
      return;
    }
    if (right.isPresent() && isSingleTagOK(right.get())) {
      fuse(right.get(), "RIGHT_SINGLE_TAG");
      return;
    }

    // SmartDashboard.putString("Vision/Gate", "REJECT");
  }

  private static class Candidate {
    final String camName;
    final Pose2d pose;
    final double timestamp; // seconds
    final int tagCount;
    final int bestFid;
    final double avgAmb;
    final double avgDist;
    final double jumpMeters; // debug only

    Candidate(
        String camName,
        Pose2d pose,
        double ts,
        int tagCount,
        int bestFid,
        double avgAmb,
        double avgDist,
        double jumpMeters) {
      this.camName = camName;
      this.pose = pose;
      this.timestamp = ts;
      this.tagCount = tagCount;
      this.bestFid = bestFid;
      this.avgAmb = avgAmb;
      this.avgDist = avgDist;
      this.jumpMeters = jumpMeters;
    }
  }

  private Optional<Candidate> getCandidate(
      PhotonCamera cam, PhotonPoseEstimator estimator, Pose2d odoPose) {

    PhotonPipelineResult result = cam.getLatestResult();
    if (!result.hasTargets()) {
      // SmartDashboard.putString("Vision/" + cam.getName(), "NO_TARGETS");
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> opt = estimator.update(result);
    if (opt.isEmpty()) {
      // SmartDashboard.putString("Vision/" + cam.getName(), "NO_POSE");
      return Optional.empty();
    }

    EstimatedRobotPose erp = opt.get();
    Pose2d pose = erp.estimatedPose.toPose2d();
    double ts = result.getTimestampSeconds();

    int tagCount = erp.targetsUsed.size();
    int bestFid = -1;
    double avgAmb = 0.0;
    double avgDist = 0.0;

    if (tagCount > 0) {
      double bestAmb = 999.0;
      for (var t : erp.targetsUsed) {
        double amb = t.getPoseAmbiguity();
        if (amb < bestAmb) {
          bestAmb = amb;
          bestFid = t.getFiducialId();
        }
        avgAmb += amb;
        avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
      }
      avgAmb /= tagCount;
      avgDist /= tagCount;
    }

    double jumpMeters = odoPose.getTranslation().getDistance(pose.getTranslation());

    // 只看 vision 自己的品質：太遠 / 太不確定就拒絕
    if (avgDist > kRejectAvgDist || avgAmb > kRejectAvgAmb) {
      // SmartDashboard.putString("Vision/" + cam.getName(), "CAND_REJECT_QUALITY");
      // SmartDashboard.putNumber("Vision/" + cam.getName() + "/jumpM", jumpMeters);
      // SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgAmb", avgAmb);
      // SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgDist", avgDist);
      return Optional.empty();
    }

    // SmartDashboard.putString("Vision/" + cam.getName(), "CAND_OK");
    // SmartDashboard.putNumber("Vision/" + cam.getName() + "/tags", tagCount);
    // SmartDashboard.putNumber("Vision/" + cam.getName() + "/bestFid", bestFid);
    // SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgAmb", avgAmb);
    // SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgDist", avgDist);
    // SmartDashboard.putNumber("Vision/" + cam.getName() + "/jumpM", jumpMeters);
    // SmartDashboard.putNumber("Vision/" + cam.getName() + "/tsAge", Timer.getFPGATimestamp() - ts);

    return Optional.of(
        new Candidate(cam.getName(), pose, ts, tagCount, bestFid, avgAmb, avgDist, jumpMeters));
  }

  // ★單Tag也更新：這就是單Tag的品質門檻
  private boolean isSingleTagOK(Candidate c) {
    if (c.tagCount != 1) return false;
    if (c.avgDist >= kSingleTagMaxDistM) return false;
    if (c.avgAmb >= kSingleTagMaxAmb) return false;
    return true;
  }

  private Candidate chooseBetter(Candidate a, Candidate b) {
    if (a.tagCount != b.tagCount) return (a.tagCount > b.tagCount) ? a : b;
    if (Math.abs(a.avgAmb - b.avgAmb) > 1e-6) return (a.avgAmb < b.avgAmb) ? a : b;
    return (a.avgDist < b.avgDist) ? a : b;
  }

  // 第一次雙Tag無條件硬切 pose（只做一次）
  private void hardSeedPoseOnce(Candidate c, String reason) {
    if (hasHardSeededPose) return;

    drivetrain.resetPose(c.pose);

    hasHardSeededPose = true;
    hardSeedTs = Timer.getFPGATimestamp();

    // SmartDashboard.putString("Vision/HardSeed/Reason", reason);
    // SmartDashboard.putNumber("Vision/HardSeed/ts", hardSeedTs);
    // SmartDashboard.putNumber("Vision/HardSeed/X", c.pose.getX());
    // SmartDashboard.putNumber("Vision/HardSeed/Y", c.pose.getY());
    // SmartDashboard.putNumber("Vision/HardSeed/Deg", c.pose.getRotation().getDegrees());
  }

  private void fuse(Candidate c, String reason) {

    // 多Tag更信任，單Tag更保守（但仍然更新）
    double sx = 0.05;
    double sy = 0.05;
    double st = Math.toRadians(3.0);

    if (c.tagCount <= 1) {
      sx *= 3.0;
      sy *= 3.0;
      st *= 3.0;
    }

    double ambScale = 1.0 + 2.0 * clamp(c.avgAmb, 0.0, 0.6) / 0.6;
    double distScale = 1.0 + 2.0 * clamp(c.avgDist, 0.5, 8.0) / 8.0;

    sx = clamp(sx * ambScale * distScale, 0.03, 0.70);
    sy = clamp(sy * ambScale * distScale, 0.03, 0.70);
    st = clamp(st * ambScale * distScale, Math.toRadians(1.0), Math.toRadians(25.0));

    Matrix<N3, N1> stdDevs = VecBuilder.fill(sx, sy, st);

    drivetrain.addVisionMeasurement(c.pose, c.timestamp, stdDevs);

    // SmartDashboard.putString("Vision/Gate", reason);
    // SmartDashboard.putString("Vision/FusedCam", c.camName);
    // SmartDashboard.putNumber("Vision/Fused/bestFid", c.bestFid);
    // SmartDashboard.putNumber("Vision/Fused/timestampAge", Timer.getFPGATimestamp() - c.timestamp);
    // SmartDashboard.putNumber("Vision/Fused/stdX", sx);
    // SmartDashboard.putNumber("Vision/Fused/stdY", sy);
    // SmartDashboard.putNumber("Vision/Fused/stdThetaDeg", Math.toDegrees(st));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
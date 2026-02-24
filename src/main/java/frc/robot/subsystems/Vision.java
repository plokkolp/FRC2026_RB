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

  //+X 前 +Y 左 +Z 上
  private static final Transform3d kRobotToLeftCam =
      new Transform3d(
          new Translation3d(0.25, 0.20, 0.30),
          new Rotation3d(0.0, Math.toRadians(-10), Math.toRadians(0)));

  private static final Transform3d kRobotToRightCam =
      new Transform3d(
          new Translation3d(0.25, -0.20, 0.30),
          new Rotation3d(0.0, Math.toRadians(-10), Math.toRadians(0)));

  // ===== Gate 參數 =====
  private static final int kMinTagsForMultiTag = 2;

  private static final double kDualSameTagMaxDtSec = 0.06;
  private static final double kDualMaxPoseDeltaM = 0.35;
  private static final double kDualMaxYawDeltaDeg = 12.0;

  //1tag
  private static final double kSingleTagMaxDistM = 3.0;
  private static final double kSingleTagMaxAmb = 0.08;
  private static final double kSingleTagMaxJumpM = 0.25;

  private static final double kRejectJumpMeters = 1.5;
  private static final double kRejectAvgDist = 7.0;
  private static final double kRejectAvgAmb = 0.40;

  private final PhotonCamera leftCam = new PhotonCamera(kLeftName);
  private final PhotonCamera rightCam = new PhotonCamera(kRightName);

  private final PhotonPoseEstimator leftEstimator;
  private final PhotonPoseEstimator rightEstimator;

  private final CommandSwerveDrivetrain drivetrain;

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

    // 連線狀態：只接一顆時會很有用
    SmartDashboard.putBoolean("Vision/LeftConnected", leftCam.isConnected());
    SmartDashboard.putBoolean("Vision/RightConnected", rightCam.isConnected());

    Pose2d odo = drivetrain.getPose();
    Pose3d ref = new Pose3d(odo);

    leftEstimator.setReferencePose(ref);
    rightEstimator.setReferencePose(ref);

    Optional<Candidate> left = getCandidate(leftCam, leftEstimator, odo);
    Optional<Candidate> right = getCandidate(rightCam, rightEstimator, odo);

    // 1) MultiTag：任一鏡頭達標就融合
    if (left.isPresent() && left.get().tagCount >= kMinTagsForMultiTag) {
      fuse(left.get(), "LEFT_MULTITAG");
      return;
    }
    if (right.isPresent() && right.get().tagCount >= kMinTagsForMultiTag) {
      fuse(right.get(), "RIGHT_MULTITAG");
      return;
    }

    // 2) 雙鏡頭同一Tag：兩邊都要有
    if (left.isPresent() && right.isPresent()) {
      Candidate L = left.get();
      Candidate R = right.get();

      boolean sameTag = (L.bestFid >= 0) && (L.bestFid == R.bestFid);
      double dt = Math.abs(L.timestamp - R.timestamp);
      double posDelta = L.pose.getTranslation().getDistance(R.pose.getTranslation());
      double yawDeltaDeg =
          Math.abs(L.pose.getRotation().minus(R.pose.getRotation()).getDegrees());

      SmartDashboard.putBoolean("Vision/Dual/sameTag", sameTag);
      SmartDashboard.putNumber("Vision/Dual/dt", dt);
      SmartDashboard.putNumber("Vision/Dual/posDelta", posDelta);
      SmartDashboard.putNumber("Vision/Dual/yawDeltaDeg", yawDeltaDeg);

      if (sameTag
          && dt <= kDualSameTagMaxDtSec
          && posDelta <= kDualMaxPoseDeltaM
          && yawDeltaDeg <= kDualMaxYawDeltaDeg) {

        Candidate chosen = chooseBetter(L, R);
        fuse(chosen, "DUAL_SAME_TAG");
        return;
      }
    }

    // 3) 單鏡頭 1-tag fallback（保守）
    if (left.isPresent() && isSingleTagFallbackOK(left.get())) {
      fuse(left.get(), "LEFT_SINGLE_FALLBACK");
      return;
    }
    if (right.isPresent() && isSingleTagFallbackOK(right.get())) {
      fuse(right.get(), "RIGHT_SINGLE_FALLBACK");
      return;
    }

    SmartDashboard.putString("Vision/Gate", "REJECT");
  }

  private static class Candidate {
    final String camName;
    final Pose2d pose;
    final double timestamp; // seconds
    final int tagCount;
    final int bestFid;
    final double avgAmb;
    final double avgDist;
    final double jumpMeters;

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
      SmartDashboard.putString("Vision/" + cam.getName(), "NO_TARGETS");
      return Optional.empty();
    }

    // ★ 你的版本：update(PhotonPipelineResult)
    Optional<EstimatedRobotPose> opt = estimator.update(result);
    if (opt.isEmpty()) {
      SmartDashboard.putString("Vision/" + cam.getName(), "NO_POSE");
      return Optional.empty();
    }

    EstimatedRobotPose erp = opt.get();
    Pose2d pose = erp.estimatedPose.toPose2d();

    // ★ 你的版本：result timestamp 可用
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

    // 基本拒絕：太遠 / 太跳 / 太不確定
    if (jumpMeters > kRejectJumpMeters || avgDist > kRejectAvgDist || avgAmb > kRejectAvgAmb) {
      SmartDashboard.putString("Vision/" + cam.getName(), "CAND_REJECT_BASIC");
      SmartDashboard.putNumber("Vision/" + cam.getName() + "/jumpM", jumpMeters);
      SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgAmb", avgAmb);
      SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgDist", avgDist);
      return Optional.empty();
    }

    // Debug
    SmartDashboard.putString("Vision/" + cam.getName(), "CAND_OK");
    SmartDashboard.putNumber("Vision/" + cam.getName() + "/tags", tagCount);
    SmartDashboard.putNumber("Vision/" + cam.getName() + "/bestFid", bestFid);
    SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgAmb", avgAmb);
    SmartDashboard.putNumber("Vision/" + cam.getName() + "/avgDist", avgDist);
    SmartDashboard.putNumber("Vision/" + cam.getName() + "/jumpM", jumpMeters);
    SmartDashboard.putNumber("Vision/" + cam.getName() + "/tsAge", Timer.getFPGATimestamp() - ts);

    return Optional.of(
        new Candidate(cam.getName(), pose, ts, tagCount, bestFid, avgAmb, avgDist, jumpMeters));
  }

  private boolean isSingleTagFallbackOK(Candidate c) {
    if (c.tagCount != 1) return false;
    if (c.avgDist >= kSingleTagMaxDistM) return false;
    if (c.avgAmb >= kSingleTagMaxAmb) return false;
    if (c.jumpMeters >= kSingleTagMaxJumpM) return false;
    return true;
  }

  private Candidate chooseBetter(Candidate a, Candidate b) {
    if (a.tagCount != b.tagCount) return (a.tagCount > b.tagCount) ? a : b;
    if (Math.abs(a.avgAmb - b.avgAmb) > 1e-6) return (a.avgAmb < b.avgAmb) ? a : b;
    return (a.avgDist < b.avgDist) ? a : b;
  }

  private void fuse(Candidate c, String reason) {

    double sx = 0.07;
    double sy = 0.07;
    double st = Math.toRadians(4.0);

    if (c.tagCount <= 1) {
      sx *= 2.5;
      sy *= 2.5;
      st *= 2.5;
    }

    double ambScale = 1.0 + 2.0 * clamp(c.avgAmb, 0.0, 0.6) / 0.6;
    double distScale = 1.0 + 2.0 * clamp(c.avgDist, 0.5, 8.0) / 8.0;

    sx = clamp(sx * ambScale * distScale, 0.03, 0.70);
    sy = clamp(sy * ambScale * distScale, 0.03, 0.70);
    st = clamp(st * ambScale * distScale, Math.toRadians(1.0), Math.toRadians(25.0));

    Matrix<N3, N1> stdDevs = VecBuilder.fill(sx, sy, st);

    drivetrain.addVisionMeasurement(c.pose, c.timestamp, stdDevs);

    SmartDashboard.putString("Vision/Gate", reason);
    SmartDashboard.putString("Vision/FusedCam", c.camName);
    SmartDashboard.putNumber("Vision/Fused/bestFid", c.bestFid);
    SmartDashboard.putNumber("Vision/Fused/timestampAge", Timer.getFPGATimestamp() - c.timestamp);
    SmartDashboard.putNumber("Vision/Fused/stdX", sx);
    SmartDashboard.putNumber("Vision/Fused/stdY", sy);
    SmartDashboard.putNumber("Vision/Fused/stdY", sy);
    SmartDashboard.putNumber("Vision/Fused/stdThetaDeg", Math.toDegrees(st));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
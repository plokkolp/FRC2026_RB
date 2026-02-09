package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class LL4 extends SubsystemBase {


  private final String name;

  private static final double GOAL_BACK_METERS = 0.58;
  private static final double GOAL_UP_METERS   = 0.70;

  public LL4() {
    this("limelight-shoot");
  }

  public LL4(String name) {
    if (name == null || name.isEmpty()) {
      throw new IllegalArgumentException("Limelight name CANNOT be empty");
    }
    this.name = name;
  }

  public boolean hasTag(int tagId) {
    RawFiducial[] tags = getRawFiducials();
    if (tags == null) return false;
    for (RawFiducial t : tags) {
      if (t != null && t.id == tagId) return true;
    }
    return false;
  }

  public boolean hasSpeakerTag() {
    return hasTag(10) || hasTag(26);
  }
  public double getBestTagDistanceMeters() {
    RawFiducial[] tags = getRawFiducials();
    if (tags == null || tags.length == 0) return Double.NaN;

    RawFiducial best = null;

    for (RawFiducial t : tags) {
      if (t == null) continue;
      if (!Double.isFinite(t.distToRobot) || t.distToRobot <= 0.0) continue;

      if (best == null) {
        best = t;
        continue;
      }

      if (t.ambiguity < best.ambiguity - 1e-6) {
        best = t;
      } else if (Math.abs(t.ambiguity - best.ambiguity) <= 1e-6 && t.ta > best.ta) {
        best = t;
      }
    }

    return (best == null) ? Double.NaN : best.distToRobot;
  }

  public double getTagDistanceMeters(int tagId) {
    RawFiducial[] tags = getRawFiducials();
    if (tags == null || tags.length == 0) return Double.NaN;

    for (RawFiducial t : tags) {
      if (t == null) continue;
      if (t.id == tagId && Double.isFinite(t.distToRobot) && t.distToRobot > 0.0) {
        return t.distToRobot;
      }
    }
    return Double.NaN;
  }

  private Pose3d getTagPoseInCameraSpaceById(int tagId) {
    LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
    if (results == null || results.targets_Fiducials == null) return null;

    LimelightHelpers.LimelightTarget_Fiducial best = null;

    for (LimelightHelpers.LimelightTarget_Fiducial t : results.targets_Fiducials) {
      if (t == null) continue;
      if ((int) t.fiducialID != tagId) continue;

      if (best == null || t.ta > best.ta) best = t;
    }

    if (best == null) return null;

    return best.getTargetPose_CameraSpace();
  }

  private Pose3d getGoalPoseInCameraSpace(int tagId) {
    Pose3d tagInCam = getTagPoseInCameraSpaceById(tagId);
    if (tagInCam == null) return null;

    Transform3d tagToGoal = new Transform3d(
        new Translation3d(-GOAL_BACK_METERS, 0.0, GOAL_UP_METERS),
        new Rotation3d()
    );

    return tagInCam.transformBy(tagToGoal);
  }

  public double getGoalYawDeg(int tagId) {
    Pose3d goalInCam = getGoalPoseInCameraSpace(tagId);
    if (goalInCam == null) return Double.NaN;

    double x = goalInCam.getTranslation().getX(); // forward
    double y = goalInCam.getTranslation().getY(); // left

    if (!Double.isFinite(x) || !Double.isFinite(y)) return Double.NaN;
    if (Math.abs(x) < 1e-9) return Double.NaN;

    return Math.toDegrees(Math.atan2(-y, x));
  }

  //直線距離
  public double getGoalDistanceMeters(int tagId) {
    Pose3d goalInCam = getGoalPoseInCameraSpace(tagId);
    if (goalInCam == null) return Double.NaN;

    double dist = goalInCam.getTranslation().getNorm();
    return Double.isFinite(dist) ? dist : Double.NaN;
  }


  public boolean hasTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getTX() { return LimelightHelpers.getTX(name); }
  public double getTY() { return LimelightHelpers.getTY(name); }
  public double getTA() { return LimelightHelpers.getTA(name); }

  public double getTXNC() { return LimelightHelpers.getTXNC(name); }
  public double getTYNC() { return LimelightHelpers.getTYNC(name); }

  public double getTagID() {
    return LimelightHelpers.getFiducialID(name);
  }

  public double getHeartbeat() {
    return LimelightHelpers.getHeartbeat(name);
  }

  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(name, index);
  }

  public void ledPipelineControl() { LimelightHelpers.setLEDMode_PipelineControl(name); }
  public void ledForceOn()         { LimelightHelpers.setLEDMode_ForceOn(name); }
  public void ledForceOff()        { LimelightHelpers.setLEDMode_ForceOff(name); }
  public void ledForceBlink()      { LimelightHelpers.setLEDMode_ForceBlink(name); }

  // ===================== Pose Estimation =====================
  public PoseEstimate getPoseEstimateBlue_MegaTag1() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
  }

  public PoseEstimate getPoseEstimateBlue_MegaTag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  public void setRobotOrientationDeg(
      double yawDeg,
      double yawRateDegPerSec,
      double pitchDeg,
      double pitchRateDegPerSec,
      double rollDeg,
      double rollRateDegPerSec
  ) {
    LimelightHelpers.SetRobotOrientation(
        name,
        yawDeg, yawRateDegPerSec,
        pitchDeg, pitchRateDegPerSec,
        rollDeg, rollRateDegPerSec
    );
  }

  public RawFiducial[] getRawFiducials() {
    return LimelightHelpers.getRawFiducials(name);
  }

  public RawDetection[] getRawDetections() {
    return LimelightHelpers.getRawDetections(name);
  }

  public LimelightHelpers.IMUData getIMU() {
    return LimelightHelpers.getIMUData(name);
  }

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  public void setIMUAssistAlpha(double alpha) {
    LimelightHelpers.SetIMUAssistAlpha(name, alpha);
  }
  /** 取目前畫面中「最適合」的一顆 fiducial 的 id（用 ta 最大），沒有回 -1 */
public int getBestFiducialId() {
  var results = LimelightHelpers.getLatestResults(name);
  if (results == null || results.targets_Fiducials == null) return -1;

  LimelightHelpers.LimelightTarget_Fiducial best = null;
  for (var t : results.targets_Fiducials) {
    if (best == null || t.ta > best.ta) best = t;
  }
  return (best == null) ? -1 : (int) best.fiducialID;
}

/** 直接回「目前最佳 tag」對應的框框 yaw 誤差（deg），抓不到回 NaN */
public double getBestGoalYawDeg() {
  int id = getBestFiducialId();
  if (id < 0) return Double.NaN;
  return getGoalYawDeg(id);
}


  @Override
  public void periodic() {
    SmartDashboard.putString("LL4/Name", name);

    SmartDashboard.putBoolean("LL4/HasTarget", hasTarget());
    SmartDashboard.putNumber("LL4/tx", getTX());
    SmartDashboard.putNumber("LL4/ty", getTY());
    SmartDashboard.putNumber("LL4/ta", getTA());
    SmartDashboard.putNumber("LL4/txnc", getTXNC());
    SmartDashboard.putNumber("LL4/tync", getTYNC());
    SmartDashboard.putNumber("LL4/tagID", getTagID());
    SmartDashboard.putNumber("LL4/heartbeat", getHeartbeat());

    PoseEstimate p1 = getPoseEstimateBlue_MegaTag1();
    SmartDashboard.putNumber("LL4/MT1_tagCount", p1.tagCount);
    SmartDashboard.putNumber("LL4/MT1_ts", p1.timestampSeconds);

    PoseEstimate p2 = getPoseEstimateBlue_MegaTag2();
    SmartDashboard.putNumber("LL4/MT2_tagCount", p2.tagCount);
    SmartDashboard.putNumber("LL4/MT2_ts", p2.timestampSeconds);

    var imu = getIMU();
    SmartDashboard.putNumber("LL4/IMU_robotYaw", imu.robotYaw);
    SmartDashboard.putNumber("LL4/IMU_roll", imu.Roll);
    SmartDashboard.putNumber("LL4/IMU_pitch", imu.Pitch);
    SmartDashboard.putNumber("LL4/IMU_rawYaw", imu.Yaw);

    // 只認 10 / 26 的狀態
    SmartDashboard.putBoolean("LL4/HasTag10", hasTag(10));
    SmartDashboard.putBoolean("LL4/HasTag26", hasTag(26));
    SmartDashboard.putBoolean("LL4/HasSpeakerTag", hasSpeakerTag());

    // 方便你立即驗證：用 10 / 26 看 GoalYaw 是否合理
    SmartDashboard.putNumber("LL4/GoalYawDeg(tag10)", getGoalYawDeg(10));
    SmartDashboard.putNumber("LL4/GoalDistM(tag10)", getGoalDistanceMeters(10));
    SmartDashboard.putNumber("LL4/GoalYawDeg(tag26)", getGoalYawDeg(26));
    SmartDashboard.putNumber("LL4/GoalDistM(tag26)", getGoalDistanceMeters(26));

    var r = LimelightHelpers.getLatestResults(name);
int fidCount = (r == null || r.targets_Fiducials == null) ? 0 : r.targets_Fiducials.length;

SmartDashboard.putNumber("LL4/debug/fidCount", fidCount);

if (fidCount > 0) {
  var t = r.targets_Fiducials[0];
  SmartDashboard.putNumber("LL4/debug/fid0_id", t.fiducialID);
  SmartDashboard.putNumber("LL4/debug/fid0_ta", t.ta);

  var pose = t.getTargetPose_CameraSpace();
  SmartDashboard.putNumber("LL4/debug/cam_x", pose.getX());
  SmartDashboard.putNumber("LL4/debug/cam_y", pose.getY());
  SmartDashboard.putNumber("LL4/debug/cam_z", pose.getZ());
}

  }
}

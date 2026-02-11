package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class LL4 extends SubsystemBase {

  private final String name;

  public LL4() {
    this("limelight-shoot");
  }

  public LL4(String name) {
    if (name == null || name.isEmpty()) {
      throw new IllegalArgumentException("Limelight name CANNOT be empty");
    }
    this.name = name;
  }


  public boolean hasTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getTX() {
    return LimelightHelpers.getTX(name);
  }

  public double getTY() {
    return LimelightHelpers.getTY(name);
  }

  public double getTA() {
    return LimelightHelpers.getTA(name);
  }

  public double getTXNC() {
    return LimelightHelpers.getTXNC(name);
  }

  public double getTYNC() {
    return LimelightHelpers.getTYNC(name);
  }

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


  public RawFiducial[] getRawFiducials() {
    return LimelightHelpers.getRawFiducials(name);
  }

  public RawDetection[] getRawDetections() {
    return LimelightHelpers.getRawDetections(name);
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

  public int getBestFiducialId() {
    var results = LimelightHelpers.getLatestResults(name);
    if (results == null || results.targets_Fiducials == null) return -1;

    LimelightHelpers.LimelightTarget_Fiducial best = null;
    for (var t : results.targets_Fiducials) {
      if (best == null || t.ta > best.ta) best = t;
    }
    return (best == null) ? -1 : (int) best.fiducialID;
  }

  public double getBestGoalYawDeg() {
    if (!hasTarget()) return Double.NaN;
    return getTX(); 
  }

  public boolean hasLLTarget() {
    return hasTarget();
  }

  //

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

  public LimelightHelpers.IMUData getIMU() {
    return LimelightHelpers.getIMUData(name);
  }

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  public void setIMUAssistAlpha(double alpha) {
    LimelightHelpers.SetIMUAssistAlpha(name, alpha);
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
    SmartDashboard.putNumber("LL4/bestYawErrDeg(A)", getBestGoalYawDeg());
  }
}

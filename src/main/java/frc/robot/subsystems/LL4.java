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
    this("");
  }

  public LL4(String name) {
    this.name = (name == null) ? "" : name;
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getTX() { return LimelightHelpers.getTX(name); }
  public double getTY() { return LimelightHelpers.getTY(name); }
  public double getTA() { return LimelightHelpers.getTA(name); }

  public double getTXNC() { return LimelightHelpers.getTXNC(name); }
  public double getTYNC() { return LimelightHelpers.getTYNC(name); }

  public double getTagID() { return LimelightHelpers.getFiducialID(name); }

  public double getHeartbeat() { return LimelightHelpers.getHeartbeat(name); }

  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(name, index);
  }

  public void ledPipelineControl() { LimelightHelpers.setLEDMode_PipelineControl(name); }
  public void ledForceOn()         { LimelightHelpers.setLEDMode_ForceOn(name); }
  public void ledForceOff()        { LimelightHelpers.setLEDMode_ForceOff(name); }
  public void ledForceBlink()      { LimelightHelpers.setLEDMode_ForceBlink(name); }

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
    LimelightHelpers.SetRobotOrientation(name, yawDeg, yawRateDegPerSec,
        pitchDeg, pitchRateDegPerSec, rollDeg, rollRateDegPerSec);
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

  public void setRewindEnabled(boolean enabled) {
    LimelightHelpers.setRewindEnabled(name, enabled);
  }

  public void triggerRewindCapture(double seconds) {
    LimelightHelpers.triggerRewindCapture(name, seconds);
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
  }
}
